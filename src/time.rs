//! Time units
//!
//! See [`Hertz`], [`KiloHertz`] and [`MegaHertz`] for creating increasingly higher frequencies.
//!
//! The [`U32Ext`] trait adds various methods like `.hz()`, `.mhz()`, etc to the `u32` primitive type,
//! allowing it to be converted into frequencies.
//!
//! # Examples
//!
//! ## Create a 2 MHz frequency
//!
//! This example demonstrates various ways of creating a 2 MHz (2_000_000 Hz) frequency. They are
//! all equivalent, however the `2.mhz()` variant should be preferred for readability.
//!
//! ```rust
//! use stm32f1xx_hal::{
//!     time::Hertz,
//!     // Imports U32Ext trait
//!     prelude::*,
//! };
//!
//! let freq_hz = 2_000_000.hz();
//! let freq_khz = 2_000.khz();
//! let freq_mhz = 2.mhz();
//!
//! assert_eq!(freq_hz, freq_khz);
//! assert_eq!(freq_khz, freq_mhz);
//! ```

use core::ops;
use cortex_m::peripheral::DWT;

use crate::rcc::Clocks;

/// Bits per second
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Bps(pub u32);

/// Hertz
///
/// Create a frequency specified in [Hertz](https://en.wikipedia.org/wiki/Hertz).
///
/// See also [`KiloHertz`] and [`MegaHertz`] for semantically correct ways of creating higher
/// frequencies.
///
/// # Examples
///
/// ## Create an 60 Hz frequency
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 60.hz();
/// ```
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Hertz(pub u32);

/// Kilohertz
///
/// Create a frequency specified in kilohertz.
///
/// See also [`Hertz`] and [`MegaHertz`] for semantically correct ways of creating lower or higher
/// frequencies.
///
/// # Examples
///
/// ## Create a 100 Khz frequency
///
/// This example creates a 100 KHz frequency. This could be used to set an I2C data rate or PWM
/// frequency, etc.
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 100.khz();
/// ```
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct KiloHertz(pub u32);

/// Megahertz
///
/// Create a frequency specified in kilohertz.
///
/// See also [`Hertz`] and [`KiloHertz`] for semantically correct ways of creating lower
/// frequencies.
///
/// # Examples
///
/// ## Create a an 8 MHz frequency
///
/// This example creates an 8 MHz frequency that could be used to configure an SPI peripheral, etc.
///
/// ```rust
/// use stm32f1xx_hal::time::Hertz;
///
/// let freq = 8.mhz();
/// ```
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct MegaHertz(pub u32);

/// Time unit
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliSeconds(pub u32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MicroSeconds(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;

    /// Wrap in `MilliSeconds`
    fn ms(self) -> MilliSeconds;

    /// Wrap in `MicroSeconds`
    fn us(self) -> MicroSeconds;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }

    fn ms(self) -> MilliSeconds {
        MilliSeconds(self)
    }

    fn us(self) -> MicroSeconds {
        MicroSeconds(self)
    }
}

impl From<KiloHertz> for Hertz {
    fn from(val: KiloHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

impl Into<Hertz> for MilliSeconds {
    fn into(self) -> Hertz {
        Hertz(1_000 / self.0)
    }
}

impl Into<Hertz> for MicroSeconds {
    fn into(self) -> Hertz {
        Hertz(1_000_000 / self.0)
    }
}

/// Macro to implement arithmetic operations (e.g. multiplication, division)
/// for wrapper types.
macro_rules! impl_arithmetic {
    ($wrapper:ty, $wrapped:ty) => {
        impl ops::Mul<$wrapped> for $wrapper {
            type Output = Self;
            fn mul(self, rhs: $wrapped) -> Self {
                Self(self.0 * rhs)
            }
        }

        impl ops::MulAssign<$wrapped> for $wrapper {
            fn mul_assign(&mut self, rhs: $wrapped) {
                self.0 *= rhs;
            }
        }

        impl ops::Div<$wrapped> for $wrapper {
            type Output = Self;
            fn div(self, rhs: $wrapped) -> Self {
                Self(self.0 / rhs)
            }
        }

        impl ops::Div<$wrapper> for $wrapper {
            type Output = $wrapped;
            fn div(self, rhs: $wrapper) -> $wrapped {
                self.0 / rhs.0
            }
        }

        impl ops::DivAssign<$wrapped> for $wrapper {
            fn div_assign(&mut self, rhs: $wrapped) {
                self.0 /= rhs;
            }
        }
    }
}

impl_arithmetic!(Hertz, u32);
impl_arithmetic!(KiloHertz, u32);
impl_arithmetic!(MegaHertz, u32);
impl_arithmetic!(Bps, u32);

/// A monotonic non-decreasing timer
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, clocks: Clocks) -> Self {
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or reset
        drop(dwt);

        MonoTimer {
            frequency: clocks.sysclk(),
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(&self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(&self) -> Instant {
        Instant {
            now: DWT::get_cycle_count(),
        }
    }
}

/// A measurement of a monotonically non-decreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}
