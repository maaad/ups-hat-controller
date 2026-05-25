use crate::config::UpsHatConfig;
use crate::driver::{BatteryData, CellVoltages, ChargingStatus, UpsDevice, VbusData};
use log::{error, info, warn};
use std::fs;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const THERMAL_REFRESH_PERIOD: Duration = Duration::from_secs(10);
const BATTERY_HEARTBEAT_PERIOD: Duration = Duration::from_secs(60);
const READ_ERROR_LOG_PERIOD: Duration = Duration::from_secs(30);
const MIN_RATE_HZ: f64 = 0.1;
const MAX_RATE_HZ: f64 = 20.0;

#[derive(Clone, Copy)]
struct ThermalSnapshot {
    cooling_state: Option<i32>,
    cpu_temp_c: Option<f64>,
}

pub struct UpsHatMonitor<T: UpsDevice> {
    config: UpsHatConfig,
    driver: T,
    stop_flag: Arc<AtomicBool>,
    on_mains: bool,
    power_loss_time: Option<Instant>,
    low_voltage_count: i32,
    last_battery_status_log: Option<Instant>,
    thermal_cache: ThermalSnapshot,
    last_thermal_update: Option<Instant>,
    consecutive_read_errors: u32,
    last_read_error_log: Option<Instant>,
    host_shutdown_triggered: bool,
    dry_run_shutdown_reason_latched: Option<&'static str>,
    last_mains_soc: Option<u16>,
    last_mains_charge_state: Option<u8>,
}

impl<T: UpsDevice> UpsHatMonitor<T> {
    pub fn new(config: UpsHatConfig, driver: T, stop_flag: Arc<AtomicBool>) -> Self {
        Self {
            config,
            driver,
            stop_flag,
            on_mains: true,
            power_loss_time: None,
            low_voltage_count: 0,
            last_battery_status_log: None,
            thermal_cache: ThermalSnapshot {
                cooling_state: None,
                cpu_temp_c: None,
            },
            last_thermal_update: None,
            consecutive_read_errors: 0,
            last_read_error_log: None,
            host_shutdown_triggered: false,
            dry_run_shutdown_reason_latched: None,
            last_mains_soc: None,
            last_mains_charge_state: None,
        }
    }

    pub fn run(&mut self) {
        let rate = self.config.publish_rate_hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ);
        let period = Duration::from_secs_f64(1.0 / rate);
        let mut next_tick = Instant::now();

        info!(
            "monitor_start i2c_bus={} i2c_addr={} rate_hz={:.3} dry_run={} shutdown_delay_sec={} low_voltage_threshold_mv={} low_voltage_threshold_count={}",
            self.config.i2c_bus,
            self.config.i2c_addr,
            rate,
            self.config.dry_run,
            self.config.shutdown_delay_sec,
            self.config.low_voltage_threshold_mv,
            self.config.low_voltage_threshold_count
        );

        let mut prev_on_mains = true;
        let mut prev_charge_state = u8::MAX;

        while !self.stop_flag.load(Ordering::SeqCst) && !self.host_shutdown_triggered {
            let loop_start = Instant::now();
            self.refresh_thermal_if_needed(loop_start);

            match self.read_and_process(loop_start, &mut prev_on_mains, &mut prev_charge_state) {
                Ok(()) => {
                    if self.consecutive_read_errors > 0 {
                        info!(
                            "ups_read_recovered consecutive_errors={}",
                            self.consecutive_read_errors
                        );
                        self.consecutive_read_errors = 0;
                        self.last_read_error_log = None;
                    }
                }
                Err(e) => self.handle_read_error(loop_start, &e),
            }

            next_tick += period;
            let now = Instant::now();
            if now < next_tick {
                thread::sleep(next_tick - now);
            } else {
                let lag = now.duration_since(next_tick);
                if lag > period {
                    warn!(
                        "monitor_loop_lag lag_ms={} period_ms={}",
                        lag.as_millis(),
                        period.as_millis()
                    );
                }
                next_tick = now;
            }
        }

        info!(
            "monitor_stop host_shutdown_triggered={} dry_run_shutdown_latched={}",
            self.host_shutdown_triggered,
            self.dry_run_shutdown_reason_latched.is_some()
        );
    }

    fn read_and_process(
        &mut self,
        now: Instant,
        prev_on_mains: &mut bool,
        prev_charge_state: &mut u8,
    ) -> Result<(), String> {
        let charging_status = self.driver.read_charging_status()?;
        let vbus_data = self.driver.read_vbus()?;
        let battery_data = self.driver.read_battery()?;
        let cell_voltages = self.driver.read_cells()?;

        let on_mains_now = charging_status.charging
            || charging_status.fast_charging
            || charging_status.vbus_powered;

        if on_mains_now != self.on_mains {
            self.on_mains = on_mains_now;
            if !self.on_mains {
                self.handle_power_loss(now);
            } else {
                self.handle_power_restored();
            }
        }

        self.process_low_voltage(cell_voltages, battery_data.current_ma);
        self.process_power_loss_timeout(now);

        self.log_state_changes(
            *prev_on_mains,
            *prev_charge_state,
            charging_status,
            vbus_data,
            battery_data,
        );

        *prev_on_mains = self.on_mains;
        *prev_charge_state = charging_status.charge_state;

        self.log_periodic_battery_status(
            now,
            charging_status,
            vbus_data,
            battery_data,
            cell_voltages,
        );
        self.log_mains_heartbeat_on_change(charging_status, vbus_data, battery_data);

        Ok(())
    }

    fn process_low_voltage(&mut self, cells: CellVoltages, current_ma: i16) {
        if !self.is_low_voltage(cells, current_ma) {
            if self.low_voltage_count > 0 {
                info!(
                    "low_voltage_recovered consecutive_count={}",
                    self.low_voltage_count
                );
                self.clear_dry_run_latch_if("low_voltage_threshold_reached");
            }
            self.low_voltage_count = 0;
            return;
        }

        self.low_voltage_count += 1;
        let remaining = (self.config.low_voltage_threshold_count - self.low_voltage_count).max(0);

        warn!(
            "low_voltage_detected count={} threshold_count={} threshold_mv={} current_ma={}",
            self.low_voltage_count,
            self.config.low_voltage_threshold_count,
            self.config.low_voltage_threshold_mv,
            current_ma
        );

        if self.low_voltage_count >= self.config.low_voltage_threshold_count {
            self.initiate_shutdown("low_voltage_threshold_reached");
        } else if remaining == 1 || remaining % 5 == 0 {
            warn!("low_voltage_countdown remaining_cycles={remaining}");
        }
    }

    fn process_power_loss_timeout(&mut self, now: Instant) {
        if self.on_mains {
            return;
        }
        if let Some(start) = self.power_loss_time {
            let elapsed = now.duration_since(start).as_secs() as i64;
            let remaining = self.config.shutdown_delay_sec - elapsed;
            if remaining <= 0 {
                self.initiate_shutdown("power_loss_timeout");
            }
        }
    }

    fn is_low_voltage(&self, c: CellVoltages, current_ma: i16) -> bool {
        if current_ma >= 50 {
            return false;
        }
        c.cell1_mv < self.config.low_voltage_threshold_mv
            || c.cell2_mv < self.config.low_voltage_threshold_mv
            || c.cell3_mv < self.config.low_voltage_threshold_mv
            || c.cell4_mv < self.config.low_voltage_threshold_mv
    }

    fn handle_power_loss(&mut self, now: Instant) {
        self.power_loss_time = Some(now);
        warn!(
            "power_lost shutdown_in_sec={} dry_run={}",
            self.config.shutdown_delay_sec, self.config.dry_run
        );

        let wall_msg = format!(
            "UPS: mains power lost, shutdown scheduled in {} sec{}",
            self.config.shutdown_delay_sec,
            if self.config.dry_run {
                " (dry-run: host shutdown suppressed)"
            } else {
                ""
            }
        );

        if let Err(e) = Command::new("sh")
            .arg("-c")
            .arg(format!("echo \"{}\" | wall", wall_msg))
            .status()
        {
            warn!("wall_notify_failed error={e}");
        }
    }

    fn handle_power_restored(&mut self) {
        let elapsed_sec = self
            .power_loss_time
            .map(|t| t.elapsed().as_secs())
            .unwrap_or_default();
        self.power_loss_time = None;
        self.low_voltage_count = 0;
        self.last_battery_status_log = None;
        self.clear_dry_run_latch_if("power_loss_timeout");
        self.last_mains_soc = None;
        self.last_mains_charge_state = None;
        info!("power_restored outage_sec={elapsed_sec}");
    }

    fn initiate_shutdown(&mut self, reason: &'static str) {
        if self.host_shutdown_triggered {
            return;
        }

        if self.config.dry_run {
            if self.dry_run_shutdown_reason_latched != Some(reason) {
                error!(
                    "shutdown_intent reason={} dry_run={} command=\"{}\"",
                    reason, self.config.dry_run, self.config.shutdown_command
                );
                warn!("shutdown_suppressed reason={} mode=dry_run", reason);
                self.dry_run_shutdown_reason_latched = Some(reason);
            }
            return;
        }

        self.host_shutdown_triggered = true;
        error!(
            "shutdown_intent reason={} dry_run={} command=\"{}\"",
            reason, self.config.dry_run, self.config.shutdown_command
        );

        if let Err(e) = self.driver.shutdown() {
            error!("ups_shutdown_command_failed error={e}");
        } else {
            info!("ups_shutdown_command_sent register=0x01 value=0x55");
        }

        match Command::new("sh")
            .arg("-c")
            .arg(&self.config.shutdown_command)
            .status()
        {
            Ok(status) => {
                if !status.success() {
                    warn!("host_shutdown_command_nonzero status={status}");
                }
            }
            Err(e) => warn!("host_shutdown_command_failed error={e}"),
        }
    }

    fn log_state_changes(
        &self,
        prev_on_mains: bool,
        prev_charge_state: u8,
        charging: ChargingStatus,
        vbus: VbusData,
        batt: BatteryData,
    ) {
        let power_changed = prev_on_mains != self.on_mains;
        let charge_changed = prev_charge_state != charging.charge_state;

        if !power_changed && !charge_changed {
            return;
        }

        let power = if self.on_mains { "mains" } else { "battery" };
        let prev_power = if prev_on_mains { "mains" } else { "battery" };
        let charge = charge_state_name(charging.charge_state);

        if prev_charge_state == u8::MAX {
            info!(
                "state_change: power {} -> {} | charge {}({}) | {}",
                prev_power,
                power,
                charge,
                charging.charge_state,
                format_brief_metrics(vbus, batt),
            );
        } else {
            info!(
                "state_change: power {} -> {} | charge {}({}) -> {}({}) | {}",
                prev_power,
                power,
                charge_state_name(prev_charge_state),
                prev_charge_state,
                charge,
                charging.charge_state,
                format_brief_metrics(vbus, batt),
            );
        }
    }

    fn log_periodic_battery_status(
        &mut self,
        now: Instant,
        charging: ChargingStatus,
        vbus: VbusData,
        batt: BatteryData,
        cells: CellVoltages,
    ) {
        if self.on_mains {
            self.last_battery_status_log = None;
            return;
        }

        let should_log = match self.last_battery_status_log {
            None => true,
            Some(t0) => now.duration_since(t0) >= BATTERY_HEARTBEAT_PERIOD,
        };

        if !should_log {
            return;
        }

        self.last_battery_status_log = Some(now);

        let cooling = format_optional_int(self.thermal_cache.cooling_state);
        let cpu = format_optional_temp(self.thermal_cache.cpu_temp_c);
        let outage_sec = self
            .power_loss_time
            .map(|t| t.elapsed().as_secs())
            .unwrap_or_default();

        info!(
            "battery_heartbeat: outage={}s | charge {}({}) | {} | cells {} {} {} {} mV | cap={}mAh | cooling={} | cpu={}C",
            outage_sec,
            charge_state_name(charging.charge_state),
            charging.charge_state,
            format_brief_metrics(vbus, batt),
            cells.cell1_mv,
            cells.cell2_mv,
            cells.cell3_mv,
            cells.cell4_mv,
            batt.remaining_capacity_mah,
            cooling,
            cpu
        );
    }

    fn log_mains_heartbeat_on_change(
        &mut self,
        charging: ChargingStatus,
        vbus: VbusData,
        batt: BatteryData,
    ) {
        if !self.on_mains {
            self.last_mains_soc = None;
            self.last_mains_charge_state = None;
            return;
        }

        let soc_changed = self.last_mains_soc != Some(batt.percent);
        let charge_changed = self.last_mains_charge_state != Some(charging.charge_state);
        if !(soc_changed || charge_changed) {
            return;
        }

        let mut reason = Vec::new();
        if soc_changed {
            reason.push("soc_changed");
        }
        if charge_changed {
            reason.push("charge_state_changed");
        }

        info!(
            "mains_heartbeat: {} | charge {}({}) | {}",
            reason.join(","),
            charge_state_name(charging.charge_state),
            charging.charge_state,
            format_brief_metrics(vbus, batt),
        );

        self.last_mains_soc = Some(batt.percent);
        self.last_mains_charge_state = Some(charging.charge_state);
    }

    fn refresh_thermal_if_needed(&mut self, now: Instant) {
        let should_refresh = match self.last_thermal_update {
            None => true,
            Some(t0) => now.duration_since(t0) >= THERMAL_REFRESH_PERIOD,
        };
        if !should_refresh {
            return;
        }

        self.last_thermal_update = Some(now);
        self.thermal_cache = ThermalSnapshot {
            cooling_state: read_sysfs_int("/sys/class/thermal/cooling_device0/cur_state"),
            cpu_temp_c: read_sysfs_double("/sys/class/hwmon/hwmon0/temp1_input", 0.001),
        };
    }

    fn handle_read_error(&mut self, now: Instant, error_message: &str) {
        self.consecutive_read_errors += 1;
        let should_log = match self.last_read_error_log {
            None => true,
            Some(t0) => now.duration_since(t0) >= READ_ERROR_LOG_PERIOD,
        };

        if should_log {
            self.last_read_error_log = Some(now);
            error!(
                "ups_read_failed consecutive_errors={} message=\"{}\"",
                self.consecutive_read_errors, error_message
            );
        }
    }

    fn clear_dry_run_latch_if(&mut self, reason: &'static str) {
        if self.dry_run_shutdown_reason_latched == Some(reason) {
            self.dry_run_shutdown_reason_latched = None;
        }
    }
}

fn format_brief_metrics(vbus: VbusData, batt: BatteryData) -> String {
    format!(
        "VBUS {:.2}V {}mA {}mW | BAT {:.2}V {:+}mA SOC {}% | ETA dis={}m chg={}m",
        vbus.voltage_mv as f64 / 1000.0,
        vbus.current_ma,
        vbus.power_mw,
        batt.voltage_mv as f64 / 1000.0,
        batt.current_ma,
        batt.percent,
        batt.discharge_time_min,
        batt.charge_time_min
    )
}

fn charge_state_name(state: u8) -> &'static str {
    match state {
        0 => "Standby",
        1 => "Trickle Charge",
        2 => "Constant Current Charge",
        3 => "Constant Voltage Charge",
        4 => "Charging Pending",
        5 => "Full State",
        6 => "Charge Timeout",
        _ => "Unknown",
    }
}

fn read_sysfs_int(path: &str) -> Option<i32> {
    fs::read_to_string(path).ok()?.trim().parse::<i32>().ok()
}

fn read_sysfs_double(path: &str, scale: f64) -> Option<f64> {
    read_sysfs_int(path).map(|v| v as f64 * scale)
}

fn format_optional_int(v: Option<i32>) -> String {
    v.map(|x| x.to_string()).unwrap_or_else(|| "na".to_string())
}

fn format_optional_temp(v: Option<f64>) -> String {
    v.map(|x| format!("{x:.1}"))
        .unwrap_or_else(|| "na".to_string())
}
