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

use stm32::{BKP, rcc, RCC};

use crate::rcc::{LSE};

/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is aquired by calling `constrain` on `rcc::Rcc::BKP`
*/
pub struct BackupDomain {
    pub(crate) _regs: BKP,
}

impl BackupDomain {
    /**
      Enables the RTC device with the lse as the clock

      Takes ownership over LSE in order to freeze it
    */
    pub(crate) fn enable_rtc(&mut self, lse: LSE) {
        // NOTE: Safe RCC access because we are only accessing bdcr
        let rcc = unsafe { &*RCC::ptr() };
        rcc.bdcr.modify(|_, w| {
            w
                .lseon().set_bit()
                // Enable the RTC
                .rtcen().set_bit()
                // Set the source of the RTC to LSE
                .rtcsel().lse()
        })
    }
}
