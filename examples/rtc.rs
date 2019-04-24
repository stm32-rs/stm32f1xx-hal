//! Outputs the current time in seconds to hstdout using the real time clock

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    rtc::Rtc,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut pwr = p.PWR;
    let mut rcc = p.RCC.constrain();
    let mut backup_domain = rcc.bkp.constrain(p.BKP, &mut rcc.apb1, &mut pwr);

    let rtc = Rtc::rtc(p.RTC, &mut backup_domain);

    loop {
        hprintln!("time: {}", rtc.seconds()).unwrap();
    }
}
