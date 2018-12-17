//! HAL for the STM32F1 family of microcontrollers
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
//! See the [examples] module.
//!
//! [examples]: examples/index.html

#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
extern crate void;

pub extern crate stm32f1;

#[cfg(feature = "stm32f103")]
pub use stm32f1::stm32f103 as stm32;

pub mod afio;
pub mod bb;
pub mod delay;
pub mod dma;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod pwm;
pub mod qei;
pub mod rcc;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
