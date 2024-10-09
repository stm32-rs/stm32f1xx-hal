//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::afio::{Remap, MAPR};
use crate::gpio::{self, Alternate, Cr, OpenDrain};
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

pub struct Pins<SCL, SDA> {
    pub scl: SCL,
    pub sda: SDA,
}

impl<SCL, SDA> From<(SCL, SDA)> for Pins<SCL, SDA> {
    fn from(value: (SCL, SDA)) -> Self {
        Self {
            scl: value.0,
            sda: value.1,
        }
    }
}

pub mod i2c1 {
    use super::*;

    remap! {
        pac::I2C1: [
            Scl: PB6, Sda: PB7 => MAPR: 0;
            Scl: PB8, Sda: PB9 => MAPR: 1;
        ]
    }
}
pub mod i2c2 {
    use super::*;

    remap! {
        pac::I2C2: [
            Scl: PB10, Sda: PB11;
        ]
    }
}

macro_rules! remap {
    ($PER:ty: [
        $(Scl: $SCL:ident, Sda: $SDA:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        pub enum Scl {
            $(
                $SCL(gpio::$SCL<Alternate<OpenDrain>>),
            )+
        }
        pub enum Sda {
            $(
                $SDA(gpio::$SDA<Alternate<OpenDrain>>),
            )+
        }

        $(
            impl From<(gpio::$SCL<Alternate<OpenDrain>>, gpio::$SDA<Alternate<OpenDrain>> $(, &mut $MAPR)?)> for Pins<Scl, Sda> {
                fn from(p: (gpio::$SCL<Alternate<OpenDrain>>, gpio::$SDA<Alternate<OpenDrain>> $(, &mut $MAPR)?)) -> Self {
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { scl: Scl::$SCL(p.0), sda: Sda::$SDA(p.1) }
                }
            }

            impl From<(gpio::$SCL, gpio::$SDA $(, &mut $MAPR)?)> for Pins<Scl, Sda> {
                fn from(p: (gpio::$SCL, gpio::$SDA $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let scl = p.0.into_mode(&mut cr);
                    let sda = p.1.into_mode(&mut cr);
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { scl: Scl::$SCL(scl), sda: Sda::$SDA(sda) }
                }
            }
        )+
    }
}
use remap;

pub trait I2cExt: Sized + Instance {
    fn i2c(
        self,
        pins: impl Into<Pins<Self::Scl, Self::Sda>>,
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> I2c<Self>;

    #[allow(clippy::too_many_arguments)]
    fn blocking_i2c(
        self,
        pins: impl Into<Pins<Self::Scl, Self::Sda>>,
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
        pins: impl Into<Pins<Self::Scl, Self::Sda>>,
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
    crate::Sealed + Deref<Target = crate::pac::i2c1::RegisterBlock> + Enable + Reset + BusClock
{
    type Scl;
    type Sda;
}

impl Instance for pac::I2C1 {
    type Scl = i2c1::Scl;
    type Sda = i2c1::Sda;
}
impl Instance for pac::I2C2 {
    type Scl = i2c2::Scl;
    type Sda = i2c2::Sda;
}

impl<I2C: Instance> I2c<I2C> {
    /// Creates a generic I2C object
    pub fn new(
        i2c: I2C,
        pins: impl Into<Pins<I2C::Scl, I2C::Sda>>,
        mode: impl Into<Mode>,
        clocks: &Clocks,
    ) -> Self {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let pclk1 = I2C::clock(clocks);

        assert!(mode.get_frequency() <= kHz(400));
        let pins = pins.into();

        let mut i2c = I2c {
            i2c,
            pins: (pins.scl, pins.sda),
            mode,
            pclk1,
        };
        i2c.init();
        i2c
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
