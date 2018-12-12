//! Prints "Hello, world" on the OpenOCD console

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;
extern crate stm32f1xx_hal;

use core::fmt::Write;
use rt::entry;

#[entry]
fn main() -> ! {
    let mut hstdout = sh::hio::hstdout().unwrap();
    writeln!(hstdout, "Hello, world!").unwrap();
    loop {}
}
