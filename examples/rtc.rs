//! Blinks an LED

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

#[entry]
fn main() -> ! {
    let mut hstdout = hio::hstdout().unwrap();

    let p = hal::stm32::Peripherals::take().unwrap();

    let mut pwr = p.PWR;
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let backup_domain = rcc.cfgr.enable_backup_domain(&mut pwr);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let rtc = backup_domain.rtc(p.RTC, clocks);

    loop {
        writeln!(hstdout, "time: {}", rtc.read()).unwrap();
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
