/*!
  Registers that are not reset as long as Vbat or Vdd has power.

  The registers retain their values during wakes from standby mode or system resets. They also
  retain their value when Vdd is switched off as long as V_BAT is powered.

  The backup domain also contains tamper protection and writes to it must be enabled in order
  to use the real time clock (RTC).

  Write access to the backup domain is enabled in RCC using the `rcc::Rcc::BKP::constrain()`
  function.

  Only the RTC functionality is currently implemented.
*/

use crate::pac::BKP;

/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is aquired by calling `constrain` on `rcc::Rcc::BKP`
*/
pub struct BackupDomain {
    pub(crate) _regs: BKP,
}

macro_rules! write_drx {
    ($self:ident, $drx:ident, $idx:expr, $new:expr) => {
        $self._regs.$drx[$idx].write(|w| w.d().bits($new));
    };
}

macro_rules! read_drx {
    ($self:ident, $drx:ident, $idx:expr) => {
        $self._regs.$drx[$idx].read().d().bits();
    };
}

impl BackupDomain {
    /// Read a 16-bit value from one of the DR1 to DR10 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR1, up to 9 for DR10. Providing a number above 9
    /// will panic.
    pub fn read_data_register_low(&self, register: usize) -> u16 {
        read_drx!(self, dr, register)
    }

    /// Read a 16-bit value from one of the DR11 to DR42 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR11, up to 31 for DR42. Providing a number above 31
    /// will panic.
    /// NOTE: not available on medium- and low-density devices!
    #[cfg(feature = "high")]
    pub fn read_data_register_high(&self, register: usize) -> u16 {
        read_drx!(self, bkp_dr, register)
    }

    /// Write a 16-bit value to one of the DR1 to DR10 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR1, up to 9 for DR10. Providing a number above 9
    /// will panic.
    pub fn write_data_register_low(&self, register: usize, data: u16) {
        write_drx!(self, dr, register, data)
    }

    /// Write a 16-bit value to one of the DR11 to DR42 registers part of the
    /// Backup Data Register. The register argument is a zero based index to the
    /// DRx registers: 0 is DR11, up to 31 for DR42. Providing a number above 31
    /// will panic.
    /// NOTE: not available on medium- and low-density devices!
    #[cfg(feature = "high")]
    pub fn write_data_register_high(&self, register: usize, data: u16) {
        write_drx!(self, bkp_dr, register, data)
    }
}
