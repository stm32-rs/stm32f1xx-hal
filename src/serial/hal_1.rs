use super::*;

mod nb {
    use super::{Error, Instance, Rx, Serial, Tx};
    use embedded_hal_nb::serial::ErrorKind;
    use embedded_hal_nb::{serial, serial::ErrorType};

    impl embedded_hal_nb::serial::Error for Error {
        fn kind(&self) -> ErrorKind {
            match self {
                Error::Overrun => ErrorKind::Overrun,
                Error::FrameFormat => ErrorKind::FrameFormat,
                Error::Parity => ErrorKind::Parity,
                Error::Noise => ErrorKind::Noise,
                Error::Other => ErrorKind::Other,
            }
        }
    }

    impl<USART: Instance> ErrorType for Tx<USART> {
        type Error = Error;
    }

    impl<USART: Instance> ErrorType for Rx<USART> {
        type Error = Error;
    }

    impl<USART: Instance, Otype, PULL> ErrorType for Serial<USART, Otype, PULL> {
        type Error = Error;
    }

    impl<USART: Instance> serial::Write<u8> for Tx<USART> {
        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.write_u8(word)?;
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.flush()
        }
    }

    impl<USART: Instance> serial::Write<u16> for Tx<USART> {
        fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
            self.write_u16(word)
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.flush()
        }
    }

    impl<USART: Instance> serial::Read<u8> for Rx<USART> {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.read()
        }
    }

    impl<USART: Instance> serial::Read<u16> for Rx<USART> {
        fn read(&mut self) -> nb::Result<u16, Self::Error> {
            self.read_u16()
        }
    }

    impl<USART: Instance, Otype, PULL> serial::Write<u8> for Serial<USART, Otype, PULL> {
        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.tx.write_u8(word).unwrap();
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.tx.flush().unwrap();
            Ok(())
        }
    }

    impl<USART: Instance, Otype, PULL> serial::Write<u16> for Serial<USART, Otype, PULL> {
        fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
            self.tx.write_u16(word).unwrap();
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.tx.flush().unwrap();
            Ok(())
        }
    }

    impl<USART: Instance, Otype, PULL> serial::Read<u8> for Serial<USART, Otype, PULL> {
        fn read(&mut self) -> nb::Result<u8, Error> {
            self.rx.read()
        }
    }

    impl<USART: Instance, Otype, PULL> serial::Read<u16> for Serial<USART, Otype, PULL> {
        fn read(&mut self) -> nb::Result<u16, Error> {
            self.rx.read_u16()
        }
    }
}

mod io {
    use super::super::{Error, Instance, Rx, Serial, Tx};
    use embedded_io::Write;

    impl embedded_io::Error for Error {
        // TODO: fix error conversion
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }

    impl<USART: Instance, Otype, PULL> embedded_io::ErrorType for Serial<USART, Otype, PULL> {
        type Error = Error;
    }

    impl<USART: Instance> embedded_io::ErrorType for Tx<USART> {
        type Error = Error;
    }

    impl<USART: Instance> embedded_io::ErrorType for Rx<USART> {
        type Error = Error;
    }

    impl<USART: Instance> Write for Tx<USART> {
        fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            let mut i = 0;
            for byte in bytes.iter() {
                match self.write_u8(*byte) {
                    Ok(_) => {
                        i += 1;
                    }
                    Err(nb::Error::WouldBlock) => {
                        return Ok(i);
                    }
                    Err(nb::Error::Other(e)) => {
                        return Err(e);
                    }
                }
            }
            Ok(i)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.bflush()?;
            Ok(())
        }
    }

    impl<USART: Instance, Otype, PULL> Write for Serial<USART, Otype, PULL>
    where
        Tx<USART>: Write<Error = Error>,
    {
        fn write(&mut self, bytes: &[u8]) -> Result<usize, Self::Error> {
            self.tx.write(bytes)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Write::flush(&mut self.tx)
        }
    }
}
