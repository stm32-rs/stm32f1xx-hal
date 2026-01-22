#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{gpio::PinState, pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc);

    let mut pin = gpioc.pc13.into_floating_input(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &rcc.clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        hprintln!("{}", pin.is_high());
        pin.as_push_pull_output(&mut gpioc.crh, |out| {
            out.set_high();
            block!(timer.wait()).unwrap();
            out.set_low();
            block!(timer.wait()).unwrap();
        });
        pin.as_push_pull_output_with_state(&mut gpioc.crh, PinState::High, |out| {
            block!(timer.wait()).unwrap();
            out.set_low();
            block!(timer.wait()).unwrap();
        });
    }
}
