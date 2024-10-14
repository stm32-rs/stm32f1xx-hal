//! Inter-Integrated Circuit (I2C) bus
//!
//! ## Alternate function remapping
//!
//! ### I2C1
//!
//! | Function \ Remap | 0 (default) | 1   |
//! |------------------|-------------|-----|
//! | SCL (A-OD)       | PB6         | PB8 |
//! | SDA (A-OD)       | PB7         | PB9 |
//!
//! ### I2C2
//!
//! | Function   |      |
//! |------------|------|
//! | SCL (A-OD) | PB10 |
//! | SDA (A-OD) | PB11 |

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::afio::{self, RInto, Rmp};
use crate::pac::{self, DWT, RCC};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{kHz, Hertz};
use core::ops::Deref;

pub mod blocking;
pub use blocking::BlockingI2c;

mod hal_02;
mod hal_1;

pub use embedded_hal::i2c::NoAcknowledgeSource;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    ArbitrationLoss,
    /// No ack received
    NoAcknowledge(NoAcknowledgeSource),
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    Timeout,
    // Alert, // SMBUS mode only
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard(frequency: Hertz) -> Self {
        Mode::Standard { frequency }
    }

    pub fn fast(frequency: Hertz, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency,
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

impl From<Hertz> for Mode {
    fn from(frequency: Hertz) -> Self {
        if frequency <= kHz(100) {
            Self::Standard { frequency }
        } else {
            Self::Fast {
                frequency,
                duty_cycle: DutyCycle::Ratio2to1,
            }
        }
    }
}

pub trait I2cExt: Sized + Instance {
    fn i2c(
        self,
        pins: (impl RInto<Self::Scl, 0>, impl RInto<Self::Sda, 0>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> I2c<Self>;

    #[allow(clippy::too_many_arguments)]
    fn blocking_i2c(
        self,
        pins: (impl RInto<Self::Scl, 0>, impl RInto<Self::Sda, 0>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> BlockingI2c<Self> {
        Self::i2c(self, pins, mode, clocks).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            clocks,
        )
    }
}

impl<I2C: Instance> I2cExt for I2C {
    fn i2c(
        self,
        pins: (impl RInto<Self::Scl, 0>, impl RInto<Self::Sda, 0>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> I2c<Self> {
        I2c::new(self, pins, mode, clocks)
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C: Instance> {
    i2c: I2C,
    pins: (I2C::Scl, I2C::Sda),
    mode: Mode,
    pclk1: Hertz,
}

pub trait Instance:
    crate::Sealed
    + Deref<Target = crate::pac::i2c1::RegisterBlock>
    + Enable
    + Reset
    + BusClock
    + afio::I2cCommon
{
}

impl Instance for pac::I2C1 {}
impl Instance for pac::I2C2 {}

impl<I2C: Instance> I2c<I2C> {
    /// Creates a generic I2C object
    pub fn new(
        i2c: I2C,
        pins: (impl RInto<I2C::Scl, 0>, impl RInto<I2C::Sda, 0>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> Self {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let pclk1 = I2C::clock(clocks);

        assert!(mode.get_frequency() <= kHz(400));

        let mut i2c = I2c {
            i2c,
            pins: (pins.0.rinto(), pins.1.rinto()),
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }
}

impl<I2C: Instance, const R: u8> Rmp<I2C, R> {
    /// Creates a generic I2C object
    pub fn i2c(
        self,
        pins: (impl RInto<I2C::Scl, R>, impl RInto<I2C::Sda, R>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> I2c<I2C> {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let pclk1 = I2C::clock(clocks);

        assert!(mode.get_frequency() <= kHz(400));

        let mut i2c = I2c {
            i2c: self.0,
            pins: (pins.0.rinto(), pins.1.rinto()),
            mode,
            pclk1,
        };
        i2c.init();
        i2c
    }

    #[allow(clippy::too_many_arguments)]
    pub fn blocking_i2c(
        self,
        pins: (impl RInto<I2C::Scl, R>, impl RInto<I2C::Sda, R>),
        mode: impl Into<Mode>,
        clocks: &Clocks,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> BlockingI2c<I2C> {
        self.i2c(pins, mode, clocks).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            clocks,
        )
    }
}

impl<I2C: Instance> I2c<I2C> {
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = self.pclk1.to_MHz() as u16;

        self.i2c
            .cr2()
            .write(|w| unsafe { w.freq().bits(pclk1_mhz as u8) });
        self.i2c.cr1().write(|w| w.pe().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .trise()
                    .write(|w| w.trise().set((pclk1_mhz + 1) as u8));
                self.i2c
                    .ccr()
                    .write(|w| unsafe { w.ccr().bits(((self.pclk1 / (freq * 2)) as u16).max(4)) });
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .trise()
                    .write(|w| w.trise().set((pclk1_mhz * 300 / 1000 + 1) as u8));

                self.i2c.ccr().write(|w| {
                    let (freq, duty) = match duty_cycle {
                        DutyCycle::Ratio2to1 => (((self.pclk1 / (freq * 3)) as u16).max(1), false),
                        DutyCycle::Ratio16to9 => (((self.pclk1 / (freq * 25)) as u16).max(1), true),
                    };

                    unsafe { w.ccr().bits(freq).duty().bit(duty).f_s().set_bit() }
                });
            }
        };

        self.i2c.cr1().modify(|_, w| w.pe().set_bit());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c.cr1().write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1().reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.cr1().modify(|_, w| w.start().set_bit());
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .dr()
            .write(|w| w.dr().set(addr << 1 | (u8::from(read))));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.cr1().modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn release(self) -> (I2C, (I2C::Scl, I2C::Sda)) {
        (self.i2c, self.pins)
    }
}
