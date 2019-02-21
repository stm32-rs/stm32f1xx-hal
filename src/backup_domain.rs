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
