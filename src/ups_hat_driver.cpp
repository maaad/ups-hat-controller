/**
 * @file ups_hat_driver.cpp
 * @brief Implementation of I2C driver for Waveshare UPS HAT (E)
 */

#include "ups_hat_driver.hpp"
#include <stdexcept>
#include <sstream>
#include <cstring>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

UpsHatDriver::UpsHatDriver(int i2c_bus, uint8_t i2c_addr)
    : fd_(-1), i2c_bus_(i2c_bus), i2c_addr_(i2c_addr) {
    device_path_ = getDevicePath();
}

UpsHatDriver::~UpsHatDriver() {
    close();
}

std::string UpsHatDriver::getDevicePath() const {
    std::ostringstream oss;
    oss << "/dev/i2c-" << i2c_bus_;
    return oss.str();
}

bool UpsHatDriver::initialize() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = open(device_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void UpsHatDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::vector<uint8_t> UpsHatDriver::readRegister(uint8_t register_addr, size_t length) {
    if (fd_ < 0) {
        throw std::runtime_error("I2C device not initialized");
    }

    // Use I2C_RDWR ioctl for block read (similar to SMBus read_i2c_block_data)
    struct i2c_rdwr_ioctl_data ioctl_data;
    struct i2c_msg msgs[2];
    uint8_t reg_addr = register_addr;
    std::vector<uint8_t> data(length);

    // First message: write register address
    msgs[0].addr = i2c_addr_;
    msgs[0].flags = 0; // write
    msgs[0].len = 1;
    msgs[0].buf = &reg_addr;

    // Second message: read data
    msgs[1].addr = i2c_addr_;
    msgs[1].flags = I2C_M_RD; // read
    msgs[1].len = length;
    msgs[1].buf = data.data();

    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        throw std::runtime_error("Failed to read I2C register block");
    }

    return data;
}

void UpsHatDriver::writeRegister(uint8_t register_addr, uint8_t value) {
    if (fd_ < 0) {
        throw std::runtime_error("I2C device not initialized");
    }

    uint8_t buffer[2] = {register_addr, value};
    if (write(fd_, buffer, 2) != 2) {
        throw std::runtime_error("Failed to write to I2C register");
    }
}

ChargingStatus UpsHatDriver::readChargingStatus() {
    // Read register 0x02: BIT7=charging, BIT6=fast_charging, BIT5=VBUS_powered, BIT2-0=charge_state
    auto data = readRegister(REG_CHG_STATUS, 1);
    uint8_t status = data[0];

    ChargingStatus result;
    result.charging = (status & 0x80) != 0;        // BIT7
    result.fast_charging = (status & 0x40) != 0;   // BIT6
    result.vbus_powered = (status & 0x20) != 0;    // BIT5
    result.charge_state = status & 0x07;           // BIT2-0: 0=standby, 1=trickle, 2=CC, 3=CV, 4=pending, 5=full, 6=timeout

    return result;
}

VbusData UpsHatDriver::readVbus() {
    // Read registers 0x10-0x15: voltage (0x10-0x11), current (0x12-0x13), power (0x14-0x15)
    auto data = readRegister(REG_VBUS_BASE, 6);

    VbusData result;
    result.voltage_mv = data[0] | (data[1] << 8);  // Little-endian 16-bit
    result.current_ma = data[2] | (data[3] << 8);
    result.power_mw = data[4] | (data[5] << 8);

    return result;
}

BatteryData UpsHatDriver::readBattery() {
    // Read registers 0x20-0x2b: voltage, current (signed), percent, capacity, times
    auto data = readRegister(REG_BATT_BASE, 12);

    BatteryData result;
    result.voltage_mv = data[0] | (data[1] << 8);

    // Current is signed 16-bit: positive=charging, negative=discharging
    uint16_t current_raw = data[2] | (data[3] << 8);
    if (current_raw > 0x7FFF) {
        current_raw -= 0x10000;  // Convert to signed
    }
    result.current_ma = static_cast<int16_t>(current_raw);

    result.percent = data[4] | (data[5] << 8);
    result.remaining_capacity_mah = data[6] | (data[7] << 8);
    result.discharge_time_min = data[8] | (data[9] << 8);
    result.charge_time_min = data[10] | (data[11] << 8);

    return result;
}

CellVoltages UpsHatDriver::readCells() {
    auto data = readRegister(REG_CELL_BASE, 8);

    CellVoltages result;
    result.cell1_mv = data[0] | (data[1] << 8);
    result.cell2_mv = data[2] | (data[3] << 8);
    result.cell3_mv = data[4] | (data[5] << 8);
    result.cell4_mv = data[6] | (data[7] << 8);

    return result;
}

void UpsHatDriver::shutdown() {
    // Write 0x55 to register 0x01 to initiate shutdown sequence
    // UPS HAT will disconnect power after 30s and check for charging after 60s
    writeRegister(REG_SHUTDOWN, 0x55);
}

