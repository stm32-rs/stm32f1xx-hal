use crate::rtc::Rtc;
use crate::rcc::Clocks;

use stm32::RTC;


pub struct BackupDomain {
    pub(crate) _0: ()
}

impl BackupDomain {
    pub fn rtc(&self, regs: RTC, _clocks: Clocks) -> Rtc {
        Rtc::rtc(regs)
    }
}



