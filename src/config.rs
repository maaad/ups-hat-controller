use std::collections::HashMap;
use std::env;
use std::fs;

#[derive(Debug, Clone)]
pub struct UpsHatConfig {
    pub i2c_bus: i32,
    pub i2c_addr: u8,
    pub publish_rate_hz: f64,
    pub shutdown_delay_sec: i64,
    pub shutdown_command: String,
    pub low_voltage_threshold_mv: u16,
    pub low_voltage_threshold_count: i32,
    pub enable_syslog: bool,
}

impl Default for UpsHatConfig {
    fn default() -> Self {
        Self {
            i2c_bus: 1,
            i2c_addr: 0x2d,
            publish_rate_hz: 1.0,
            shutdown_delay_sec: 60,
            shutdown_command: "systemctl poweroff".to_string(),
            low_voltage_threshold_mv: 3150,
            low_voltage_threshold_count: 30,
            enable_syslog: true,
        }
    }
}

pub fn parse_environment() -> UpsHatConfig {
    let mut cfg = UpsHatConfig::default();
    apply_map(&mut cfg, &env::vars().collect());
    cfg
}

pub fn parse_config_file(path: &str) -> UpsHatConfig {
    let mut cfg = UpsHatConfig::default();
    if let Ok(content) = fs::read_to_string(path) {
        let kv = parse_key_value(&content);
        apply_map(&mut cfg, &kv);
    }
    cfg
}

fn parse_key_value(content: &str) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for raw_line in content.lines() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with(';') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            let key = k.trim();
            let val = v.trim();
            if !key.is_empty() && !val.is_empty() {
                map.insert(key.to_string(), val.to_string());
            }
        }
    }
    map
}

fn parse_bool(v: &str) -> bool {
    matches!(v.to_ascii_lowercase().as_str(), "true" | "1" | "yes")
}

fn apply_map(cfg: &mut UpsHatConfig, map: &HashMap<String, String>) {
    if let Some(v) = map.get("I2C_BUS") {
        if let Ok(n) = v.parse::<i32>() {
            cfg.i2c_bus = n;
        }
    }
    if let Some(v) = map.get("I2C_ADDR") {
        if let Ok(n) = v.parse::<u8>() {
            cfg.i2c_addr = n;
        }
    }
    if let Some(v) = map.get("PUBLISH_RATE_HZ") {
        if let Ok(n) = v.parse::<f64>() {
            cfg.publish_rate_hz = n;
        }
    }
    if let Some(v) = map.get("SHUTDOWN_DELAY_SEC") {
        if let Ok(n) = v.parse::<i64>() {
            cfg.shutdown_delay_sec = n;
        }
    }
    if let Some(v) = map.get("SHUTDOWN_COMMAND") {
        cfg.shutdown_command = v.to_string();
    }
    if let Some(v) = map.get("LOW_VOLTAGE_THRESHOLD") {
        if let Ok(n) = v.parse::<u16>() {
            cfg.low_voltage_threshold_mv = n;
        }
    }
    if let Some(v) = map.get("LOW_VOLTAGE_THRESHOLD_COUNT") {
        if let Ok(n) = v.parse::<i32>() {
            cfg.low_voltage_threshold_count = n;
        }
    }
    if let Some(v) = map.get("ENABLE_SYSLOG") {
        cfg.enable_syslog = parse_bool(v);
    }
}
