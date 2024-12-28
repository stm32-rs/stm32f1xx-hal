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
//! A detailed usage guide can be found in the [README]
//!
//! supported microcontrollers are:
//!
//! - stm32f103
//! - stm32f101
//! - stm32f100
//! - stm32f105
//! - stm32f107
//!
//! ## Usage
//!
//! This crate supports multiple microcontrollers in the
//! stm32f1 family. Which specific microcontroller you want to build for has to be
//! specified with a feature, for example `stm32f103`.
//!
//! If no microcontroller is specified, the crate will not compile.
//!
//! The currently supported variants are
//!
//! - `stm32f100`
//! - `stm32f101`
//! - `stm32f103`
//! - `stm32f105`
//! - `stm32f107`
//!
//! You may also need to specify the density of the device with `medium`, `high` or `xl`
//! to enable certain peripherals. Generally the density can be determined by the 2nd character
//! after the number in the device name (i.e. For STM32F103C6U, the 6 indicates a low-density
//! device) but check the datasheet or CubeMX to be sure.
//! * 4, 6 => low density, no feature required
//! * 8, B => `medium` feature
//! * C, D, E => `high` feature
//! * F, G => `xl` feature
//!
//! ## Commonly used setup
//! Almost all peripherals require references to some registers in `RCC` and `AFIO`. The following
//! code shows how to set up those registers
//!
//! ```rust
//! // Get access to the device specific peripherals from the peripheral access crate
//! let dp = pac::Peripherals::take().unwrap();
//!
//! // Take ownership over the raw flash and rcc devices and convert them into the corresponding
//! // HAL structs
//! let mut flash = dp.FLASH.constrain();
//! let mut rcc = dp.RCC.constrain();
//!
//! // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
//! // `clocks`
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//!
//! // Prepare the alternate function I/O registers
//! let mut afio = dp.AFIO.constrain();
//! ```
//!
//! ## Usage examples
//!
//! See the [examples] folder.
//!
//! Most of the examples require the following additional dependencies
//! ```toml
//! [dependencies]
//! embedded-hal = "0.2.3"
//! nb = "0.1.2"
//! cortex-m = "0.6.2"
//! cortex-m-rt = "0.6.11"
//! # Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
//! panic-halt = "0.2.0"
//! ```
//!
//! [examples]: https://github.com/stm32-rs/stm32f1xx-hal/tree/v0.7.0/examples
//! [README]: https://github.com/stm32-rs/stm32f1xx-hal/tree/v0.7.0

#![no_std]
#![deny(rustdoc::broken_intra_doc_links)]

// If no target specified, print error message.
#[cfg(not(any(
    feature = "stm32f100",
    feature = "stm32f101",
    feature = "stm32f103",
    feature = "stm32f105",
    feature = "stm32f107",
)))]
compile_error!("Target not found. A `--features <target-name>` is required.");

// If any two or more targets are specified, print error message.
#[cfg(any(
    all(feature = "stm32f100", feature = "stm32f101"),
    all(feature = "stm32f100", feature = "stm32f103"),
    all(feature = "stm32f100", feature = "stm32f105"),
    all(feature = "stm32f100", feature = "stm32f107"),
    all(feature = "stm32f101", feature = "stm32f103"),
    all(feature = "stm32f101", feature = "stm32f105"),
    all(feature = "stm32f101", feature = "stm32f107"),
    all(feature = "stm32f103", feature = "stm32f105"),
    all(feature = "stm32f103", feature = "stm32f107"),
    all(feature = "stm32f105", feature = "stm32f107"),
))]
compile_error!(
    "Multiple targets specified. Only a single `--features <target-name>` can be specified."
);

pub use embedded_hal as hal;
pub use embedded_hal_02 as hal_02;

#[cfg(feature = "stm32f100")]
pub use stm32f1::stm32f100 as pac;

#[cfg(feature = "stm32f101")]
pub use stm32f1::stm32f101 as pac;

#[cfg(feature = "stm32f103")]
pub use stm32f1::stm32f103 as pac;

#[cfg(any(feature = "stm32f105", feature = "stm32f107"))]
pub use stm32f1::stm32f107 as pac;

pub mod adc;
pub mod afio;
pub mod backup_domain;
pub mod bb;
#[cfg(feature = "has-can")]
pub mod can;
pub mod crc;
#[cfg(feature = "has-dac")]
pub mod dac;
pub mod dma;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod rcc;
pub mod rtc;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
#[cfg(feature = "stm32-usbd")]
#[cfg(feature = "stm32f103")]
pub mod usb;
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}
use sealed::Sealed;

pub trait Steal {
    /// Steal an instance of this peripheral
    ///
    /// # Safety
    ///
    /// Ensure that the new instance of the peripheral cannot be used in a way
    /// that may race with any existing instances, for example by only
    /// accessing read-only or write-only registers, or by consuming the
    /// original peripheral and using critical sections to coordinate
    /// access between multiple new instances.
    ///
    /// Additionally the HAL may rely on only one
    /// peripheral instance existing to ensure memory safety; ensure
    /// no stolen instances are passed to such software.
    unsafe fn steal() -> Self;
}
