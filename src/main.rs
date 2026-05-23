mod config;
mod driver;
mod monitor;

use config::{parse_config_file, parse_environment};
use driver::UpsHatDriver;
use monitor::UpsHatMonitor;
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;
use std::env;
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use syslog::Formatter3164;

fn print_help(program: &str) {
    println!("Usage: {program} [--config <path>] [--help]");
    println!("Configuration file: /etc/ups-hat-controller/ups-hat-controller.conf");
    println!("Environment variables: I2C_BUS, I2C_ADDR, PUBLISH_RATE_HZ, etc.");
}

fn main() {
    let args: Vec<String> = env::args().collect();

    for arg in &args {
        if arg == "--help" || arg == "-h" {
            print_help(&args[0]);
            return;
        }
    }

    let default_path = "/etc/ups-hat-controller/ups-hat-controller.conf";
    let mut cfg = if Path::new(default_path).is_file() {
        parse_config_file(default_path)
    } else {
        parse_environment()
    };

    let mut i = 1;
    while i < args.len() {
        if args[i] == "--config" && i + 1 < args.len() {
            cfg = parse_config_file(&args[i + 1]);
            i += 1;
        }
        i += 1;
    }

    if cfg.enable_syslog {
        let formatter = Formatter3164 {
            facility: syslog::Facility::LOG_DAEMON,
            hostname: None,
            process: "ups-hat-controller".to_string(),
            pid: 0,
        };
        match syslog::unix(formatter) {
            Ok(logger) => {
                let _ = log::set_boxed_logger(Box::new(syslog::BasicLogger::new(logger)))
                    .map(|()| log::set_max_level(log::LevelFilter::Info));
            }
            Err(_) => {
                eprintln!("Failed to initialize syslog, continuing without logger");
            }
        }
    }

    let driver = match UpsHatDriver::initialize(cfg.i2c_bus, cfg.i2c_addr) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("Failed to initialize I2C driver: {e}");
            return;
        }
    };

    let stop_flag = Arc::new(AtomicBool::new(false));
    let _ = flag::register(SIGINT, Arc::clone(&stop_flag));
    let _ = flag::register(SIGTERM, Arc::clone(&stop_flag));

    let mut monitor = UpsHatMonitor::new(cfg, driver, stop_flag);
    monitor.run();
}
