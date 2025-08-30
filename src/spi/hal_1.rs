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

impl<SPI: Instance, W, PULL> ErrorType for Spi<SPI, W, PULL> {
    type Error = Error;
}

mod nb {
    use super::{Error, FrameSize, Instance, Spi};
    use embedded_hal_nb::spi::FullDuplex;

    impl<SPI, W, PULL> FullDuplex<W> for Spi<SPI, W, PULL>
    where
        SPI: Instance,
        W: FrameSize,
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
    use super::super::{FrameSize, Instance, Spi};
    use core::ops::DerefMut;
    use embedded_hal::spi::SpiBus;

    impl<SPI: Instance, W, PULL> SpiBus<W> for Spi<SPI, W, PULL>
    where
        SPI: Instance,
        W: FrameSize + 'static,
    {
        fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
            self.deref_mut().transfer_in_place(words)
        }

        fn transfer(&mut self, buff: &mut [W], data: &[W]) -> Result<(), Self::Error> {
            self.deref_mut().transfer(buff, data)
        }

        fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
            self.deref_mut().read(words)
        }

        fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
            self.deref_mut().write(words)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}
