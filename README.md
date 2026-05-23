# UPS HAT Controller

Standalone Rust application for monitoring [Waveshare UPS HAT (E)](https://www.waveshare.com/wiki/UPS_HAT_(E)) and performing safe system shutdown on power loss or low battery voltage.

## Features

- I2C communication with Waveshare UPS HAT (E) (default I2C address: `0x2D`)
- Real-time monitoring of battery and charging state
- Automatic shutdown on power loss (configurable delay)
- Low-voltage protection with configurable threshold/count
- Thermal monitoring from sysfs
- Syslog/journald logging
- Systemd service integration

## Build

```bash
cargo build --release
```

Binary:

```bash
./target/release/ups_hat_controller
```

## Configuration

Priority order:

1. Config file (`/etc/ups-hat-controller/ups-hat-controller.conf`)
2. Environment variables
3. Built-in defaults

CLI options:

```bash
ups_hat_controller --config /path/to/config.conf
ups_hat_controller --help
```

Supported keys (same as previous C++ version):

- `I2C_BUS` (default: `1`)
- `I2C_ADDR` (default: `45`)
- `PUBLISH_RATE_HZ` (default: `1.0`)
- `SHUTDOWN_DELAY_SEC` (default: `60`)
- `SHUTDOWN_COMMAND` (default: `systemctl poweroff`)
- `LOW_VOLTAGE_THRESHOLD` (default: `3150`)
- `LOW_VOLTAGE_THRESHOLD_COUNT` (default: `30`)
- `ENABLE_SYSLOG` (default: `true`)
- `DRY_RUN` (default: `false`) - when `true`, monitoring and alerting continue but host shutdown is suppressed

## Install

Create symlinks in system paths (recommended for development):

```bash
make install-links
```

This creates links:
- `/etc/ups-hat-controller/ups-hat-controller.env` -> `config/ups-hat-controller.env`
- `/usr/local/bin/ups_hat_controller` -> `target/release/ups_hat_controller`
- `/etc/systemd/system/ups-hat-controller.service` -> `systemd/ups-hat-controller.service`

Useful commands:

```bash
make status
make uninstall-links
sudo systemctl enable --now ups-hat-controller.service
```

## Logs

```bash
sudo journalctl -u ups-hat-controller.service -f
```

## License

MIT, see [LICENSE](LICENSE).
