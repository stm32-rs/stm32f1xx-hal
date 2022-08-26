//! Time units
//!
//! See [`Hertz`], [`KiloHertz`] and [`MegaHertz`] for creating increasingly higher frequencies.
//!
//! The [`fugit::ExtU32`] [`U32Ext`] trait adds various methods like `.Hz()`, `.MHz()`, etc to the `u32` primitive type,
//! allowing it to be converted into frequencies.
//!
//! # Examples
//!
//! ## Create a 2 MHz frequency
//!
//! This example demonstrates various ways of creating a 2 MHz (2_000_000 Hz) frequency. They are
//! all equivalent, however the `2.MHz()` variant should be preferred for readability.
//!
//! ```rust
//! use stm32f1xx_hal::{
//!     time::Hertz,
//!     // Imports U32Ext trait
//!     prelude::*,
//! };
//!
//! let freq_hz = 2_000_000.Hz();
//! let freq_khz = 2_000.kHz();
//! let freq_mhz = 2.MHz();
//!
//! assert_eq!(freq_hz, freq_khz);
//! assert_eq!(freq_khz, freq_mhz);
//! ```

#![allow(non_snake_case)]

use core::ops;
use cortex_m::peripheral::{DCB, DWT};

use crate::rcc::Clocks;

/// Bits per second
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Debug)]
pub struct Bps(pub u32);

pub use fugit::{
    HertzU32 as Hertz, KilohertzU32 as KiloHertz, MegahertzU32 as MegaHertz,
    MicrosDurationU32 as MicroSeconds, MillisDurationU32 as MilliSeconds,
};

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }
}

pub const fn Hz(val: u32) -> Hertz {
    Hertz::from_raw(val)
}

pub const fn kHz(val: u32) -> KiloHertz {
    KiloHertz::from_raw(val)
}

pub const fn MHz(val: u32) -> MegaHertz {
    MegaHertz::from_raw(val)
}

pub const fn ms(val: u32) -> MilliSeconds {
    MilliSeconds::from_ticks(val)
}

pub const fn us(val: u32) -> MicroSeconds {
    MicroSeconds::from_ticks(val)
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
    };
}

impl_arithmetic!(Bps, u32);

/// A monotonic non-decreasing timer
///
/// This uses the timer in the debug watch trace peripheral. This means, that if the
/// core is stopped, the timer does not count up. This may be relevant if you are using
/// cortex_m_semihosting::hprintln for debugging in which case the timer will be stopped
/// while printing
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, mut dcb: DCB, clocks: Clocks) -> Self {
        dcb.enable_trace();
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or reset

        MonoTimer {
            frequency: clocks.hclk(),
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
        Instant {
            now: DWT::cycle_count(),
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
    pub fn elapsed(self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}
