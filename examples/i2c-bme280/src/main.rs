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

use bme280::BME280;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
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
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = if 1 == 1 {
        rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr)
    } else {
        // My blue pill with a stm32f103 clone dose not seem to respect rcc so will not compensate its pulse legths
        // with a faster clock like this. And so the sensor dose not have time to respond to the START pulse.
        // I would be interested if others with real stm32f103's can use this program with the faster clocks.
        rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(6.mhz())
            .freeze(&mut flash.acr)
    };

    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio16to9,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    // The Adafruit boards have address 0x77 without closing the jumper on the back, the BME280 lib connects to 0x77 with `new_secondary`, use
    // `new_primary` for 0x76 if you close the jumper/solder bridge.
    let mut bme280 = BME280::new_secondary(i2c, Delay::new(cp.SYST, clocks));
    bme280
        .init()
        .map_err(|error| {
            hprintln!("Could not initialize bme280, Error: {:?}", error).unwrap();
            panic!();
        })
        .unwrap();
    loop {
        match bme280.measure() {
            Ok(measurements) => {
                hprintln!("Relative Humidity = {}%", measurements.humidity).unwrap();
                hprintln!("Temperature = {} deg C", measurements.temperature).unwrap();
                hprintln!("Pressure = {} pascals", measurements.pressure).unwrap();
            }
            Err(error) => {
                hprintln!("Could not read bme280 due to error: {:?}", error).unwrap();
            }
        }
    }
}
