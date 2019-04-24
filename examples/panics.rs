//! Prints "Hello, world" on the OpenOCD console

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;
//use panic_itm as _;
use stm32f1xx_hal as _;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!").unwrap();
    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
