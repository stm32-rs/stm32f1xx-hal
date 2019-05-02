//! # HAL for the STM32F1 family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32F1 family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//!
//! # Usage
//!
//! ## Building an application (binary crate)
//!
//! Follow the [cortex-m-quickstart] instructions, add this crate as a dependency
//! and make sure you enable the "rt" Cargo feature of this crate. Also select which
//! microcontroller you will be using by using the corresponding feature. The currently
//! supported microcontrollers are:
//!
//! - stm32f103
//! - stm32f100
//!
//! ```toml
//! [dependencies.stm32f1xx-hal]
//! version = "0.2.1"
//! features = ["stm32f103", "rt"]
//! ```
//!
//! [cortex-m-quickstart]: https://docs.rs/cortex-m-quickstart/0.3.1
//!
//! ## Usage example
//!
//! The following example blinks an LED connected to PC13 which is where the LED is connected on the
//! [blue_pill] board. If you are testing on a different breakout board, you may need
//! to change the pin accordingly.
//!
//!
//! ```rust
//! #![no_std]
//! #![no_main]
//! 
//! extern crate panic_halt;
//! 
//! use nb::block;
//! 
//! use stm32f1xx_hal::{
//!     prelude::*,
//!     pac,
//!     timer::Timer,
//! };
//! use cortex_m_rt::entry;
//! 
//! #[entry]
//! fn main() -> ! {
//!     // Get access to the core peripherals from the cortex-m crate
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!     // Get access to the device specific peripherals from the peripheral access crate
//!     let dp = pac::Peripherals::take().unwrap();
//! 
//!     // Take ownership over the raw flash and rcc devices and convert them
//!     // into the corresponding HAL structs
//!     let mut flash = dp.FLASH.constrain();
//!     let mut rcc = dp.RCC.constrain();
//! 
//!     // Freeze the configuration of all the clocks in the system and store
//!     // the frozen frequencies in `clocks`
//!     let clocks = rcc.cfgr.freeze(&mut flash.acr);
//! 
//!     // Acquire the GPIOC peripheral
//!     let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
//! 
//!     // Configure gpio C pin 13 as a push-pull output. The `crh` register is
//!     // passed to the function in order to configure the port. For pins 0-7,
//!     // crl should be passed instead.
//!     let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
//!     // Configure the syst timer to trigger an update every second
//!     let mut timer = Timer::syst(cp.SYST, 1.hz(), clocks);
//! 
//!     // Wait for the timer to trigger an update and change the state of the LED
//!     loop {
//!         block!(timer.wait()).unwrap();
//!         led.set_high();
//!         block!(timer.wait()).unwrap();
//!         led.set_low();
//!     }
//! }
//! ```
//!
//! [blue_pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill
//!
//! # More examples
//!
//! See the [examples] folder.
//!
//! [examples]: https://github.com/stm32-rs/stm32f1xx-hal/tree/master/examples

#![no_std]

#[cfg(feature = "device-selected")]
use embedded_hal as hal;

#[cfg(feature = "stm32f100")]
pub use stm32f1::stm32f100 as pac;

#[cfg(feature = "stm32f101")]
pub use stm32f1::stm32f101 as pac;

#[cfg(feature = "stm32f103")]
pub use stm32f1::stm32f103 as pac;

#[cfg(feature = "device-selected")]
pub use crate::pac as device;

#[cfg(feature = "device-selected")]
pub use crate::pac as stm32;

#[cfg(feature = "device-selected")]
pub mod adc;
#[cfg(feature = "device-selected")]
pub mod afio;
#[cfg(feature = "device-selected")]
pub mod backup_domain;
#[cfg(feature = "device-selected")]
pub mod bb;
#[cfg(feature = "device-selected")]
pub mod delay;
#[cfg(feature = "device-selected")]
pub mod dma;
#[cfg(feature = "device-selected")]
pub mod flash;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod pwm;
#[cfg(feature = "device-selected")]
pub mod pwm_input;
#[cfg(feature = "device-selected")]
pub mod qei;
#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(feature = "device-selected")]
pub mod rtc;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
#[cfg(feature = "device-selected")]
pub mod watchdog;
