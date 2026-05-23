use crate::config::UpsHatConfig;
use crate::driver::{BatteryData, CellVoltages, ChargingStatus, UpsDevice, VbusData};
use log::{error, info, warn};
use std::fs;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub struct UpsHatMonitor<T: UpsDevice> {
    config: UpsHatConfig,
    driver: T,
    stop_flag: Arc<AtomicBool>,
    on_mains: bool,
    power_loss_time: Option<Instant>,
    last_battery_status_log: Option<Instant>,
    low_voltage_count: i32,
}

impl<T: UpsDevice> UpsHatMonitor<T> {
    pub fn new(config: UpsHatConfig, driver: T, stop_flag: Arc<AtomicBool>) -> Self {
        Self {
            config,
            driver,
            stop_flag,
            on_mains: true,
            power_loss_time: None,
            last_battery_status_log: None,
            low_voltage_count: 0,
        }
    }

    pub fn run(&mut self) {
        let period = Duration::from_millis((1000.0 / self.config.publish_rate_hz.max(0.1)) as u64);
        let mut prev_on_mains = true;
        let mut prev_charge_state = u8::MAX;

        info!("Starting UPS HAT monitoring loop");

        while !self.stop_flag.load(Ordering::SeqCst) {
            match self.read_and_process(&mut prev_on_mains, &mut prev_charge_state) {
                Ok(should_break) => {
                    if should_break {
                        break;
                    }
                }
                Err(e) => error!("UPS read failed: {e}"),
            }

            thread::sleep(period);
        }

        info!("UPS HAT monitoring loop stopped");
    }

    fn read_and_process(
        &mut self,
        prev_on_mains: &mut bool,
        prev_charge_state: &mut u8,
    ) -> Result<bool, String> {
        let charging_status = self.driver.read_charging_status()?;
        let vbus_data = self.driver.read_vbus()?;
        let battery_data = self.driver.read_battery()?;
        let cell_voltages = self.driver.read_cells()?;

        let on_mains_now = charging_status.charging || charging_status.fast_charging || charging_status.vbus_powered;

        if on_mains_now != self.on_mains {
            self.on_mains = on_mains_now;
            if !self.on_mains {
                self.handle_power_loss();
            } else {
                self.handle_power_restored();
            }
        }

        if self.check_low_voltage(cell_voltages, battery_data.current_ma) {
            self.handle_low_voltage();
        } else {
            self.low_voltage_count = 0;
        }

        if !self.on_mains {
            if let Some(t0) = self.power_loss_time {
                if t0.elapsed().as_secs() as i64 >= self.config.shutdown_delay_sec {
                    error!("Power loss persisted. Shutting down system now.");
                    self.shutdown_system();
                    return Ok(true);
                }
            }
        }

        self.log_state_changes(
            *prev_on_mains,
            *prev_charge_state,
            charging_status,
            vbus_data,
            battery_data,
        );

        if *prev_on_mains != self.on_mains {
            *prev_on_mains = self.on_mains;
        }
        if *prev_charge_state != charging_status.charge_state {
            *prev_charge_state = charging_status.charge_state;
        }

        self.log_periodic_battery_status(charging_status, vbus_data, battery_data, cell_voltages);

        Ok(false)
    }

    fn check_low_voltage(&self, c: CellVoltages, current_ma: i16) -> bool {
        if current_ma >= 50 {
            return false;
        }
        c.cell1_mv < self.config.low_voltage_threshold_mv
            || c.cell2_mv < self.config.low_voltage_threshold_mv
            || c.cell3_mv < self.config.low_voltage_threshold_mv
            || c.cell4_mv < self.config.low_voltage_threshold_mv
    }

    fn handle_power_loss(&mut self) {
        self.power_loss_time = Some(Instant::now());
        let msg = format!(
            "Power loss detected. Scheduling shutdown in {} sec",
            self.config.shutdown_delay_sec
        );
        warn!("{msg}");
        let _ = Command::new("sh")
            .arg("-c")
            .arg(format!("echo \"{}\" | wall", msg))
            .status();
    }

    fn handle_power_restored(&mut self) {
        self.power_loss_time = None;
        info!("Mains power restored. Cancelling pending shutdown");
    }

    fn handle_low_voltage(&mut self) {
        self.low_voltage_count += 1;
        if self.low_voltage_count >= self.config.low_voltage_threshold_count {
            error!("Low voltage detected. Shutting down system now.");
            self.shutdown_system();
        } else {
            let remaining_time = 60 - (2 * self.low_voltage_count);
            warn!(
                "Voltage Low, please charge in time, otherwise it will shut down in {} s",
                remaining_time
            );
        }
    }

    fn shutdown_system(&self) {
        if let Err(e) = self.driver.shutdown() {
            error!("Failed to send shutdown command to UPS HAT: {e}");
        }
        let status = Command::new("sh")
            .arg("-c")
            .arg(&self.config.shutdown_command)
            .status();
        if let Err(e) = status {
            warn!("Shutdown command failed: {e}");
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

        let mut parts = Vec::new();
        if power_changed {
            let prev = if prev_on_mains { "mains" } else { "battery" };
            let now = if self.on_mains { "mains" } else { "battery" };
            parts.push(format!("Power: {prev} -> {now}"));
        }
        if charge_changed {
            if prev_charge_state == u8::MAX {
                parts.push(format!(
                    "Charge: {} ({})",
                    charge_state_name(charging.charge_state),
                    charging.charge_state
                ));
            } else {
                parts.push(format!(
                    "Charge: {} ({}) -> {} ({})",
                    charge_state_name(prev_charge_state),
                    prev_charge_state,
                    charge_state_name(charging.charge_state),
                    charging.charge_state
                ));
            }
        }
        parts.push(format_brief_metrics(vbus, batt));
        info!("{}", parts.join(" | "));
    }

    fn log_periodic_battery_status(
        &mut self,
        charging: ChargingStatus,
        vbus: VbusData,
        batt: BatteryData,
        cells: CellVoltages,
    ) {
        if !self.on_mains {
            let now = Instant::now();
            let should_log = match self.last_battery_status_log {
                None => true,
                Some(t0) => now.duration_since(t0).as_secs() >= 60,
            };

            if should_log {
                self.last_battery_status_log = Some(now);
                let cooling = read_sysfs_int("/sys/class/thermal/cooling_device0/cur_state");
                let cpu = read_sysfs_double("/sys/class/hwmon/hwmon0/temp1_input", 0.001);

                let mut msg = format!(
                    "Battery Status: VBUS={:.2}V {}mA, BAT={:.2}V {:+}mA {}%, Cells={:.3}V/{:.3}V/{:.3}V/{:.3}V, Capacity={}mAh, State={} ({})",
                    vbus.voltage_mv as f64 / 1000.0,
                    vbus.current_ma,
                    batt.voltage_mv as f64 / 1000.0,
                    batt.current_ma,
                    batt.percent,
                    cells.cell1_mv as f64 / 1000.0,
                    cells.cell2_mv as f64 / 1000.0,
                    cells.cell3_mv as f64 / 1000.0,
                    cells.cell4_mv as f64 / 1000.0,
                    batt.remaining_capacity_mah,
                    charge_state_name(charging.charge_state),
                    charging.charge_state
                );
                if let Some(v) = cooling {
                    msg.push_str(&format!(", Cooling={v}"));
                }
                if let Some(v) = cpu {
                    msg.push_str(&format!(", CPU={v:.1}°C"));
                }
                info!("{msg}");
            }
        } else {
            self.last_battery_status_log = None;
        }
    }
}

fn format_brief_metrics(vbus: VbusData, batt: BatteryData) -> String {
    format!(
        "VBUS={:.2}V {}mA, BAT={:.2}V {:+}mA {}%",
        vbus.voltage_mv as f64 / 1000.0,
        vbus.current_ma,
        batt.voltage_mv as f64 / 1000.0,
        batt.current_ma,
        batt.percent
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
