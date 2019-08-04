//! Turns the user LED on
//!
//! If compiled for the stm32f103, this assumes that an active low LED is connected to pc13 as
//! is the case on the blue pill board.
//!
//! If compiled for the stm32f100, this assumes that an active high LED is connected to pc9
//!
//! Note: Without additional hardware, PC13 should not be used to drive a LED, see
//! section 5.1.2 of the reference manaual for an explanation.
//! This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    #[cfg(feature = "stm32f100")]
    gpioc.pc9.into_push_pull_output(&mut gpioc.crh).set_high().unwrap();

    #[cfg(feature = "stm32f101")]
    gpioc.pc9.into_push_pull_output(&mut gpioc.crh).set_high().unwrap();

    #[cfg(feature = "stm32f103")]
    gpioc.pc13.into_push_pull_output(&mut gpioc.crh).set_low().unwrap();

    loop {}
}
