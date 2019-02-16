use crate::rtc::Rtc;
use crate::rcc::Clocks;

use stm32::{BKP, RTC};


/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is aquired by calling the `enable_backup_domain`
  function on `rcc::CFGR`
*/
pub struct BackupDomain {
    pub(crate) regs: BKP
}

impl BackupDomain {
}
