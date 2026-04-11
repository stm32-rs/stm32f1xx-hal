//! Digital to Analog Converter (DAC) example
//!
//! This example demonstrates basic usage of the DAC peripheral on STM32F1xx devices
//! that have a DAC (e.g. STM32F103xC/D/E high/xl-density devices).
//!
//! DAC_OUT1 is on PA4, DAC_OUT2 is on PA5.
//!
//! The example cycles through the following tests using a 1Hz timer:
//!   - Full scale output (4095 ≈ VREF, typically 3.3V)
//!   - Half scale output (2048 ≈ VREF/2)
//!   - Zero output (0V)
//!
//! # Hardware requirements
//! - A device with DAC peripheral (High-density: STM32F103xC/D/E,
//!   XL-density: STM32F103xF/G)
//! - Optional: oscilloscope or voltmeter on PA4 / PA5 to observe output

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    dac::{DacExt, DacOut, DacPin},
    pac,
    prelude::*,
    timer::Timer,
};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership of RCC and freeze clock configuration
    let mut rcc = dp.RCC.constrain();

    // Configure PA4 and PA5 as analog (required for DAC output pins)
    // PA4 = DAC_OUT1, PA5 = DAC_OUT2
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let pa4 = gpioa.pa4.into_analog(&mut gpioa.crl);
    let pa5 = gpioa.pa5.into_analog(&mut gpioa.crl);

    // Initialize DAC with both channels
    // Alternatively, use a single channel: dp.DAC.constrain(pa4, &mut rcc)
    let (mut ch1, mut ch2) = dp.DAC.constrain((pa4, pa5), &mut rcc);

    // Enable DAC output channels
    ch1.enable();
    ch2.enable();

    // Configure SysTick timer to trigger at 1 Hz
    let mut timer = Timer::syst(cp.SYST, &rcc.clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    // DAC is 12-bit: values range from 0 (0V) to 4095 (≈VREF)
    // Step through: full → half → zero, repeating every 3 seconds
    let steps: [u16; 3] = [
        4095, // Full scale  ≈ 3.3V (when VREF = 3.3V)
        2048, // Half scale  ≈ 1.65V
        0,    // Zero        = 0V
    ];

    loop {
        for &value in steps.iter() {
            // Set both channels to the same value for this example
            ch1.set_value(value);
            ch2.set_value(value);

            // Verify readback matches written value (reads DORx output register)
            debug_assert_eq!(ch1.get_value(), value);
            debug_assert_eq!(ch2.get_value(), value);

            block!(timer.wait()).unwrap();
        }
    }
}
