//! Outputs the current time in seconds to hstdout using the real time clock

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, rtc::Rtc};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut pwr = p.PWR;
    let rcc = p.RCC.constrain();
    let mut backup_domain = rcc.bkp.constrain(p.BKP, &mut pwr);

    let rtc = Rtc::rtc(p.RTC, &mut backup_domain);

    loop {
        hprintln!("time: {}", rtc.current_time()).unwrap();
    }
}
