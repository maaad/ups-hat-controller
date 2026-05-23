use libc::{c_ulong, ioctl, write};
use std::fs::OpenOptions;
use std::os::fd::{AsRawFd, FromRawFd, IntoRawFd, OwnedFd};
use std::path::PathBuf;

const I2C_SLAVE: c_ulong = 0x0703;
const I2C_RDWR: c_ulong = 0x0707;
const I2C_M_RD: u16 = 0x0001;

const REG_SHUTDOWN: u8 = 0x01;
const REG_CHG_STATUS: u8 = 0x02;
const REG_VBUS_BASE: u8 = 0x10;
const REG_BATT_BASE: u8 = 0x20;
const REG_CELL_BASE: u8 = 0x30;

#[repr(C)]
struct I2cMsg {
    addr: u16,
    flags: u16,
    len: u16,
    buf: *mut u8,
}

#[repr(C)]
struct I2cRdwrIoctlData {
    msgs: *mut I2cMsg,
    nmsgs: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct ChargingStatus {
    pub charging: bool,
    pub fast_charging: bool,
    pub vbus_powered: bool,
    pub charge_state: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct VbusData {
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub power_mw: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct BatteryData {
    pub voltage_mv: i16,
    pub current_ma: i16,
    pub percent: u16,
    pub remaining_capacity_mah: u16,
    pub discharge_time_min: u16,
    pub charge_time_min: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct CellVoltages {
    pub cell1_mv: u16,
    pub cell2_mv: u16,
    pub cell3_mv: u16,
    pub cell4_mv: u16,
}

pub trait UpsDevice {
    fn read_charging_status(&self) -> Result<ChargingStatus, String>;
    fn read_vbus(&self) -> Result<VbusData, String>;
    fn read_battery(&self) -> Result<BatteryData, String>;
    fn read_cells(&self) -> Result<CellVoltages, String>;
    fn shutdown(&self) -> Result<(), String>;
}

pub struct UpsHatDriver {
    fd: OwnedFd,
    i2c_addr: u8,
}

impl UpsHatDriver {
    pub fn initialize(i2c_bus: i32, i2c_addr: u8) -> Result<Self, String> {
        let path = PathBuf::from(format!("/dev/i2c-{i2c_bus}"));
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| format!("failed to open i2c device: {e}"))?;

        let fd = file.as_raw_fd();
        // SAFETY: ioctl called with valid fd and I2C_SLAVE request expects integer address.
        let rc = unsafe { ioctl(fd, I2C_SLAVE, i2c_addr as c_ulong) };
        if rc < 0 {
            return Err("failed to set I2C slave address".to_string());
        }

        // SAFETY: take ownership of raw fd; file will not be used afterwards.
        let owned_fd = unsafe { OwnedFd::from_raw_fd(file.into_raw_fd()) };

        Ok(Self { fd: owned_fd, i2c_addr })
    }

    fn read_register(&self, register_addr: u8, len: usize) -> Result<Vec<u8>, String> {
        let mut reg = [register_addr];
        let mut data = vec![0_u8; len];

        let mut msgs = [
            I2cMsg {
                addr: self.i2c_addr as u16,
                flags: 0,
                len: 1,
                buf: reg.as_mut_ptr(),
            },
            I2cMsg {
                addr: self.i2c_addr as u16,
                flags: I2C_M_RD,
                len: len as u16,
                buf: data.as_mut_ptr(),
            },
        ];

        let mut ioctl_data = I2cRdwrIoctlData {
            msgs: msgs.as_mut_ptr(),
            nmsgs: 2,
        };

        // SAFETY: structures match Linux i2c-dev ABI and pointers live across call.
        let rc = unsafe { ioctl(self.fd.as_raw_fd(), I2C_RDWR, &mut ioctl_data) };
        if rc < 0 {
            return Err("failed to read I2C register block".to_string());
        }

        Ok(data)
    }

    fn write_register(&self, register_addr: u8, value: u8) -> Result<(), String> {
        let buf = [register_addr, value];
        // SAFETY: write called with valid fd and pointer to fixed-size buffer.
        let rc = unsafe { write(self.fd.as_raw_fd(), buf.as_ptr().cast(), 2) };
        if rc != 2 {
            return Err("failed to write I2C register".to_string());
        }
        Ok(())
    }
}

impl UpsDevice for UpsHatDriver {
    fn read_charging_status(&self) -> Result<ChargingStatus, String> {
        let data = self.read_register(REG_CHG_STATUS, 1)?;
        let status = data[0];
        Ok(ChargingStatus {
            charging: status & 0x80 != 0,
            fast_charging: status & 0x40 != 0,
            vbus_powered: status & 0x20 != 0,
            charge_state: status & 0x07,
        })
    }

    fn read_vbus(&self) -> Result<VbusData, String> {
        let d = self.read_register(REG_VBUS_BASE, 6)?;
        Ok(VbusData {
            voltage_mv: u16::from_le_bytes([d[0], d[1]]),
            current_ma: u16::from_le_bytes([d[2], d[3]]),
            power_mw: u16::from_le_bytes([d[4], d[5]]),
        })
    }

    fn read_battery(&self) -> Result<BatteryData, String> {
        let d = self.read_register(REG_BATT_BASE, 12)?;
        Ok(BatteryData {
            voltage_mv: i16::from_le_bytes([d[0], d[1]]),
            current_ma: i16::from_le_bytes([d[2], d[3]]),
            percent: u16::from_le_bytes([d[4], d[5]]),
            remaining_capacity_mah: u16::from_le_bytes([d[6], d[7]]),
            discharge_time_min: u16::from_le_bytes([d[8], d[9]]),
            charge_time_min: u16::from_le_bytes([d[10], d[11]]),
        })
    }

    fn read_cells(&self) -> Result<CellVoltages, String> {
        let d = self.read_register(REG_CELL_BASE, 8)?;
        Ok(CellVoltages {
            cell1_mv: u16::from_le_bytes([d[0], d[1]]),
            cell2_mv: u16::from_le_bytes([d[2], d[3]]),
            cell3_mv: u16::from_le_bytes([d[4], d[5]]),
            cell4_mv: u16::from_le_bytes([d[6], d[7]]),
        })
    }

    fn shutdown(&self) -> Result<(), String> {
        self.write_register(REG_SHUTDOWN, 0x55)
    }
}
