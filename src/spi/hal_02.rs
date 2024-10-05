use super::*;

pub use embedded_hal_02::spi::{Mode, Phase, Polarity};
use embedded_hal_02::{blocking::spi as blocking, spi};

impl From<Polarity> for super::Polarity {
    fn from(p: Polarity) -> Self {
        match p {
            Polarity::IdleLow => Self::IdleLow,
            Polarity::IdleHigh => Self::IdleHigh,
        }
    }
}

impl From<Phase> for super::Phase {
    fn from(p: Phase) -> Self {
        match p {
            Phase::CaptureOnFirstTransition => Self::CaptureOnFirstTransition,
            Phase::CaptureOnSecondTransition => Self::CaptureOnSecondTransition,
        }
    }
}

impl From<Mode> for super::Mode {
    fn from(m: Mode) -> Self {
        Self {
            polarity: m.polarity.into(),
            phase: m.phase.into(),
        }
    }
}

impl<SPI, REMAP, PINS, W, OP> spi::FullDuplex<W> for Spi<SPI, REMAP, PINS, W, OP>
where
    SPI: Instance,
    W: Copy,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<W, Error> {
        self.read_nonblocking()
    }

    fn send(&mut self, data: W) -> nb::Result<(), Error> {
        self.write_nonblocking(data)
    }
}

impl<SPI, REMAP, PINS, W, OP> blocking::transfer::Default<W> for Spi<SPI, REMAP, PINS, W, OP>
where
    SPI: Instance,
    W: Copy,
{
}

impl<SPI: Instance, REMAP, PINS, OP> blocking::Write<u8> for Spi<SPI, REMAP, PINS, u8, OP> {
    type Error = Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.write(words)
    }
}

impl<SPI: Instance, REMAP, PINS, OP> blocking::Write<u16> for Spi<SPI, REMAP, PINS, u16, OP> {
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Error> {
        self.write(words)
    }
}
