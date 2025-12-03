#ifndef UPS_HAT_DRIVER_HPP
#define UPS_HAT_DRIVER_HPP

/**
 * @file ups_hat_driver.hpp
 * @brief I2C driver for Waveshare UPS HAT (E) communication
 *
 * Implements low-level I2C communication with UPS HAT (E) according to
 * https://www.waveshare.com/wiki/UPS_HAT_(E)_Register
 */

#include <cstdint>
#include <string>
#include <vector>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// Charging status from register 0x02
struct ChargingStatus {
    bool charging;
    bool fast_charging;
    bool vbus_powered;
    uint8_t charge_state;
};

// VBUS (Type-C) data from registers 0x10-0x15
struct VbusData {
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint16_t power_mw;
};

// Battery data from registers 0x20-0x2b
struct BatteryData {
    int16_t voltage_mv;
    int16_t current_ma;
    uint16_t percent;
    uint16_t remaining_capacity_mah;
    uint16_t discharge_time_min;
    uint16_t charge_time_min;
};

// Individual cell voltages from registers 0x30-0x37
struct CellVoltages {
    uint16_t cell1_mv;
    uint16_t cell2_mv;
    uint16_t cell3_mv;
    uint16_t cell4_mv;
};

/**
 * @brief I2C driver for Waveshare UPS HAT (E)
 *
 * Handles low-level I2C communication with the UPS HAT (E) module.
 * Default I2C address is 0x2D as per Waveshare documentation.
 */
class UpsHatDriver {
public:
    UpsHatDriver(int i2c_bus = 1, uint8_t i2c_addr = 0x2d);
    ~UpsHatDriver();

    bool initialize();
    void close();

    ChargingStatus readChargingStatus();
    VbusData readVbus();
    BatteryData readBattery();
    CellVoltages readCells();
    void shutdown();

    bool isOpen() const { return fd_ >= 0; }

private:
    // Register addresses per Waveshare UPS HAT (E) Register documentation
    static constexpr uint8_t REG_SHUTDOWN = 0x01;      // Shutdown control register
    static constexpr uint8_t REG_CHG_STATUS = 0x02;    // Charging status register
    static constexpr uint8_t REG_VBUS_BASE = 0x10;     // VBUS voltage/current/power base
    static constexpr uint8_t REG_BATT_BASE = 0x20;     // Battery data base
    static constexpr uint8_t REG_CELL_BASE = 0x30;     // Cell voltages base

    int fd_;
    int i2c_bus_;
    uint8_t i2c_addr_;
    std::string device_path_;

    std::vector<uint8_t> readRegister(uint8_t register_addr, size_t length);
    void writeRegister(uint8_t register_addr, uint8_t value);
    std::string getDevicePath() const;
};

#endif // UPS_HAT_DRIVER_HPP

