//! Turns the user LED on
//!
//! If compiled for the stm32f103, this assumes that an active low LED is connected to pc13 as
//! is the case on the blue pill board.
//!
//! If compiled for the stm32f100, this assumes that an active high LED is connected to pc9
//!
//! Note: Without additional hardware, PC13 should not be used to drive a LED, see
//! section 5.1.2 of the reference manual for an explanation.
//! This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut gpioc = p.GPIOC.split();

    #[cfg(feature = "stm32f100")]
    gpioc.pc9.into_push_pull_output(&mut gpioc.crh).set_high();

    #[cfg(feature = "stm32f101")]
    gpioc.pc9.into_push_pull_output(&mut gpioc.crh).set_high();

    #[cfg(any(feature = "stm32f103", feature = "stm32f105", feature = "stm32f107"))]
    gpioc.pc13.into_push_pull_output(&mut gpioc.crh).set_low();

    loop {}
}
