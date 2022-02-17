//! Demonstrate the use of a blocking `Delay` using TIM2 general-purpose timer.

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use stm32f1xx_hal as hal;

use crate::hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(_cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let mut flash = dp.FLASH.constrain();
        // Set up the LED. On the BluePill it's connected to pin PC13.
        let mut gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .freeze(&mut flash.acr);

        // Create a delay abstraction based on general-pupose 32-bit timer TIM2

        //let mut delay = hal::timer::FTimerUs::new(dp.TIM2, &clocks).delay();
        // or
        let mut delay = dp.TIM2.delay_us(&clocks);

        loop {
            // On for 1s, off for 3s.
            led.set_high();
            // Use `embedded_hal::DelayMs` trait
            delay.delay_ms(1000_u32);
            led.set_low();
            // or use `fugit` duration units
            delay.delay(3.secs());
        }
    }

    loop {}
}
