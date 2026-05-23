# UPS HAT Controller

Rust daemon for monitoring [Waveshare UPS HAT (E)](https://www.waveshare.com/wiki/UPS_HAT_(E)) on Linux (Raspberry Pi) and triggering safe shutdown logic on power loss or low battery voltage.

## Features

- Native Rust implementation (no C++/CMake)
- I2C communication with UPS HAT (default address `0x2D` / decimal `45`)
- Power source transition detection (`mains` <-> `battery`)
- Shutdown on prolonged power loss (configurable delay)
- Shutdown on repeated low-cell-voltage condition (configurable threshold/count)
- `DRY_RUN` mode: full monitoring and alarms without host shutdown
- Syslog/journald integration
- systemd service unit
- Makefile targets for build/check/format and symlink-based installation

## Runtime Behavior

Monitoring loop:

- Polling interval is derived from `PUBLISH_RATE_HZ`
- Loop timing is stabilized to reduce drift over long uptime
- Thermal data is cached and refreshed periodically to reduce sysfs I/O

Shutdown triggers:

1. `power_loss_timeout`: mains power is lost and not restored before `SHUTDOWN_DELAY_SEC`
2. `low_voltage_threshold_reached`: low-voltage condition repeated `LOW_VOLTAGE_THRESHOLD_COUNT` times

Shutdown sequence when `DRY_RUN=false`:

1. Send UPS shutdown command (`register 0x01 <- 0x55`)
2. Execute `SHUTDOWN_COMMAND` (default `systemctl poweroff`)

When `DRY_RUN=true`, shutdown intent is logged but real UPS/host shutdown actions are suppressed.

## Logging Model

The daemon logs structured operational events into journald/syslog.

Typical event types:

- `monitor_start` / `monitor_stop`
- `state_change`
- `power_lost` / `power_restored`
- `low_voltage_detected` / `low_voltage_countdown` / `low_voltage_recovered`
- `battery_heartbeat`
- `shutdown_intent` / `shutdown_suppressed`
- `ups_read_failed` / `ups_read_recovered`

View logs:

```bash
sudo journalctl -u ups-hat-controller.service -f
```

## Build

```bash
cargo build --release
```

Binary:

```bash
./target/release/ups_hat_controller
```

## Configuration

Priority:

1. Config file (`/etc/ups-hat-controller/ups-hat-controller.conf`)
2. Environment variables (`EnvironmentFile` for systemd)
3. Built-in defaults

CLI:

```bash
ups_hat_controller --config /path/to/config.conf
ups_hat_controller --help
```

Supported config keys:

- `I2C_BUS` (default: `1`)
- `I2C_ADDR` (default: `45`)
- `PUBLISH_RATE_HZ` (default: `1.0`)
- `SHUTDOWN_DELAY_SEC` (default: `60`)
- `SHUTDOWN_COMMAND` (default: `systemctl poweroff`)
- `LOW_VOLTAGE_THRESHOLD` (default: `3150` mV)
- `LOW_VOLTAGE_THRESHOLD_COUNT` (default: `30`)
- `ENABLE_SYSLOG` (default: `true`)
- `DRY_RUN` (default: `false`)

Config files in repo:

- Main config template: `config/ups-hat-controller.conf`
- Optional env overrides template: `config/ups-hat-controller.env`

## Makefile Targets

- `make build` -> `cargo build`
- `make release` -> `cargo build --release`
- `make check` -> `cargo check`
- `make fmt` -> `cargo fmt`
- `make install-links` -> create/update symlinks in system paths
- `make uninstall-links` -> remove created symlinks
- `make status` -> show current symlink state

## Symlink-based Install (Recommended for Development)

```bash
make install-links
```

Creates/updates:

- `/etc/ups-hat-controller/ups-hat-controller.env` -> `config/ups-hat-controller.env`
- `/usr/local/bin/ups_hat_controller` -> `target/release/ups_hat_controller`
- `/etc/systemd/system/ups-hat-controller.service` -> `systemd/ups-hat-controller.service`

Then enable service:

```bash
sudo systemctl enable --now ups-hat-controller.service
```

Check status:

```bash
make status
sudo systemctl status ups-hat-controller.service
```

Remove symlinks:

```bash
make uninstall-links
```

## Permissions

The service needs access to `/dev/i2c-*`.

- Run as root, or
- Run as a user in `i2c` group with corresponding unit hardening/device rules

## License

MIT, see [LICENSE](LICENSE).
