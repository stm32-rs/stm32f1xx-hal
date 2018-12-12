#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
#[macro_use]
extern crate cortex_m;
extern crate panic_itm;
extern crate stm32f1xx_hal;

use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let mut itm = p.ITM;

    iprintln!(&mut itm.stim[0], "Hello, world!");

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
