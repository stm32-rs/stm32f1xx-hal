use crate::rtc::Rtc;
use crate::rcc::Clocks;

use stm32::RTC;


/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is aquired by calling the `enable_backup_domain`
  function on `rcc::CFGR`
*/
pub struct BackupDomain {
    pub(crate) _0: ()
}

impl BackupDomain {
    /// Initialise the RTC
    pub fn rtc(&self, regs: RTC, clocks: &Clocks) -> Rtc {
        Rtc::rtc(regs, clocks)
    }
}



