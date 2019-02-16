//! Outputs the current time in seconds to hstdout using the real time clock

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use hal::prelude::*;
use rt::{entry, exception, ExceptionFrame};

use sh::hio;

use core::fmt::Write;

use hal::rtc::Rtc;

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();

    let p = hal::stm32::Peripherals::take().unwrap();

    let mut pwr = p.PWR;
    let mut rcc = p.RCC.constrain();
    let backup_domain = rcc.bkp.constrain(p.BKP, &mut rcc.apb1, &mut pwr);
    let lse = rcc.lse.freeze(&backup_domain);

    let rtc = Rtc::rtc(p.RTC, lse, &backup_domain);

    loop {
        writeln!(hstdout, "time: {}", rtc.read_seconds()).unwrap();
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
