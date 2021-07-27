//! Blinks several LEDs stored in an array

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIO peripherals
    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());

    // Create an array of LEDS to blink
    let mut leds = [
        gpioc.pc13.into_push_pull_output(&mut gpioc.crh).erase(),
        gpioa.pa1.into_push_pull_output(&mut gpioa.crl).erase(),
    ];

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        for led in leds.iter_mut() {
            led.set_high();
        }
        block!(timer.wait()).unwrap();
        for led in leds.iter_mut() {
            led.set_low();
        }
    }
}
