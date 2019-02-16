//! Turns the user LED on

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    #[cfg(feature = "stm32f100")]
    gpioc.pc9.into_push_pull_output(&mut gpioc.crh).set_high();

    #[cfg(feature = "stm32f103")]
    gpioc.pc13.into_push_pull_output(&mut gpioc.crh).set_high();

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
