//! Reads data from a BME280 over i2c
//!
//! This assumes that a BME280 is connected with clk on PB6 and data on PB7.
//!
//! For the Adafruit breakout boards PB6 should be connected to SCK and PB7 to SDI
//!
//! This program writes the sensor values to the debug output provided by semihosting
//! you must enable semihosting in gdb with `monitor arm semihosting enable` I have it
//! added to my `.gdbinit`. Then the debug infomation will be printed in your openocd
//! terminal.
//!
//! This program dose not fit on my blue pill unless compiled in release mode
//! eg. `cargo run --example i2c-bme280 --features "stm32f103 bme280 rt" --release`
//! However as noted above the debug output with the read values will be in the openocd
//! terminal.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_semihosting::hprintln;
use panic_semihosting as _;

use bme280::i2c::BME280;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    i2c::{DutyCycle, I2cExt, Mode},
    pac,
    prelude::*,
    rcc,
};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        if 1 == 1 {
            // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
            // `clocks`
            rcc::Config::hse(8.MHz())
        } else {
            // My blue pill with a stm32f103 clone dose not seem to respect rcc so will not compensate its pulse legths
            // with a faster clock like this. And so the sensor dose not have time to respond to the START pulse.
            // I would be interested if others with real stm32f103's can use this program with the faster clocks.
            rcc::Config::hse(8.MHz()).sysclk(48.MHz()).pclk1(6.MHz())
        },
        &mut flash.acr,
    );

    //let mut afio = dp.AFIO.constrain(&mut rcc);  // add this if want to use PB8, PB9 instead

    // Acquire the GPIOB peripheral
    let gpiob = dp.GPIOB.split(&mut rcc);

    let scl = gpiob.pb6;
    let sda = gpiob.pb7;

    let i2c = dp
        .I2C1
        //.remap(&mut afio.mapr) // add this if want to use PB8, PB9 instead
        .i2c((scl, sda),
         Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            }
        , &mut rcc)
        .blocking(
            1000,
            10,
            1000,
            1000,
            &rcc.clocks
        );

    // The Adafruit boards have address 0x77 without closing the jumper on the back, the BME280 lib connects to 0x77 with `new_secondary`, use
    // `new_primary` for 0x76 if you close the jumper/solder bridge.
    let mut bme280 = BME280::new_secondary(i2c, cp.SYST.delay(&rcc.clocks));
    bme280
        .init()
        .map_err(|error| {
            hprintln!("Could not initialize bme280, Error: {:?}", error);
            panic!();
        })
        .unwrap();
    loop {
        match bme280.measure() {
            Ok(measurements) => {
                hprintln!("Relative Humidity = {}%", measurements.humidity);
                hprintln!("Temperature = {} deg C", measurements.temperature);
                hprintln!("Pressure = {} pascals", measurements.pressure)
            }
            Err(error) => {
                hprintln!("Could not read bme280 due to error: {:?}", error);
            }
        }
    }
}
