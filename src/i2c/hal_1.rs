use embedded_hal::i2c::{Error, ErrorKind, ErrorType};

impl Error for super::Error {
    fn kind(&self) -> ErrorKind {
        match *self {
            Self::Overrun => ErrorKind::Overrun,
            Self::Bus => ErrorKind::Bus,
            Self::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            Self::NoAcknowledge(nack) => ErrorKind::NoAcknowledge(nack),
            Self::Timeout => ErrorKind::Other,
        }
    }
}

impl<I2C: super::Instance, PINS> ErrorType for super::BlockingI2c<I2C, PINS> {
    type Error = super::Error;
}

mod blocking {
    use super::super::{BlockingI2c, Instance};
    use embedded_hal::i2c::Operation;

    impl<I2C: Instance, PINS> embedded_hal::i2c::I2c for BlockingI2c<I2C, PINS> {
        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.read(addr, buffer)
        }

        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.write(addr, bytes)
        }

        fn write_read(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write_read(addr, bytes, buffer)
        }

        fn transaction(
            &mut self,
            addr: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            self.transaction_slice(addr, operations)
        }
    }
}
