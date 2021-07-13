//! "Blinky" using delays instead of a timer

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = dp.GPIOC.split();

    #[cfg(feature = "stm32f100")]
    let mut led = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);

    #[cfg(feature = "stm32f101")]
    let mut led = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);

    #[cfg(any(feature = "stm32f103", feature = "stm32f105", feature = "stm32f107"))]
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high();
        delay.delay_ms(1_000_u16);
        led.set_low();
        delay.delay_ms(1_000_u16);
    }
}
