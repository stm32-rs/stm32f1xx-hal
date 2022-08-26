//! Prints "Hello, world" on the OpenOCD console

#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;
use stm32f1xx_hal as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!");
    loop {}
}
