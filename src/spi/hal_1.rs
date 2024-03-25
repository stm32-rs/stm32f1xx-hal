use super::*;
pub use embedded_hal::spi::{ErrorKind, ErrorType, Mode, Phase, Polarity};

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

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Overrun => ErrorKind::Overrun,
            Self::ModeFault => ErrorKind::ModeFault,
            Self::Crc => ErrorKind::Other,
        }
    }
}

impl<SPI: Instance, REMAP, PINS, W, OP> ErrorType for Spi<SPI, REMAP, PINS, W, OP> {
    type Error = Error;
}

mod nb {
    use super::{Error, Instance, Spi};
    use embedded_hal_nb::spi::FullDuplex;

    impl<SPI, REMAP, PINS, W, OP> FullDuplex<W> for Spi<SPI, REMAP, PINS, W, OP>
    where
        SPI: Instance,
        W: Copy,
    {
        fn read(&mut self) -> nb::Result<W, Error> {
            self.read_nonblocking()
        }

        fn write(&mut self, data: W) -> nb::Result<(), Error> {
            self.write_nonblocking(data)
        }
    }
}

mod blocking {
    use super::super::{Instance, Spi};
    use embedded_hal::spi::SpiBus;

    impl<SPI: Instance, REMAP, PINS, W, OP> SpiBus<W> for Spi<SPI, REMAP, PINS, W, OP>
    where
        SPI: Instance,
        W: Copy + 'static,
    {
        fn transfer_in_place(&mut self, _words: &mut [W]) -> Result<(), Self::Error> {
            todo!()
        }

        fn transfer(&mut self, _buff: &mut [W], _data: &[W]) -> Result<(), Self::Error> {
            todo!()
        }

        fn read(&mut self, _words: &mut [W]) -> Result<(), Self::Error> {
            todo!()
        }

        fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
            self.write(words)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}
