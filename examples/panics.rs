//! Prints "Hello, world" on the OpenOCD console

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_semihosting as _;
//use panic_itm as _;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal as _;

use cortex_m_rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!");
    loop {}
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
