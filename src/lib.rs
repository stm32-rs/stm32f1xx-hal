//! # HAL for the STM32F1 family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32F1 family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//!
//! # Usage
//!
//! - Trying out the examples
//!
//! ``` text
//! $ git clone https://github.com/stm32-rs/stm32f1xx-hal
//!
//! # on another terminal
//! $ openocd -f interface/$INTERFACE.cfg -f target/stm32f1x.cfg
//!
//! # flash and debug the "Hello, world" example
//! # NOTE examples assume 64KB of Flash and 20KB of RAM; you can tweak layout in memory.x
//! $ cd stm32f1xx-hal
//! $ rustup target add thumbv7m-none-eabi
//! $ cargo run --example hello
//! ```
//!
//! - Building an application (binary crate)
//!
//! Follow the [cortex-m-quickstart] instructions and add this crate as a dependency
//! and make sure you enable the "rt" Cargo feature of this crate.
//!
//! [cortex-m-quickstart]: https://docs.rs/cortex-m-quickstart/~0.2.3
//!
//! # Examples
//!
//! See the [examples] folder.
//!
//! [examples]: https://github.com/stm32-rs/stm32f1xx-hal/tree/master/examples

#![no_std]

use embedded_hal as hal;

#[cfg(feature = "stm32f100")]
pub use stm32f1::stm32f100 as pac;

#[cfg(feature = "stm32f103")]
pub use stm32f1::stm32f103 as pac;

pub use crate::pac as device;
pub use crate::pac as stm32;

pub mod afio;
pub mod backup_domain;
pub mod bb;
pub mod delay;
pub mod dma;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod prelude;
#[cfg(not(feature = "stm32f100"))]
pub mod pwm;
pub mod pwm_input;
pub mod qei;
pub mod rcc;
pub mod rtc;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
