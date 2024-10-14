use super::*;
use embedded_hal_02::{blocking::serial as blocking, serial};

impl<USART: Instance> serial::Write<u8> for Tx<USART> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_u8(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush()
    }
}

impl<USART: Instance> serial::Write<u16> for Tx<USART> {
    type Error = Error;

    fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        self.write_u16(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush()
    }
}

impl<USART: Instance> serial::Read<u8> for Rx<USART> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read()
    }
}

impl<USART: Instance> serial::Read<u16> for Rx<USART> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        self.read_u16()
    }
}

impl<USART: Instance, Otype, PULL> serial::Write<u8> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write_u8(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<USART: Instance, Otype, PULL> serial::Write<u16> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        self.tx.write_u16(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<USART: Instance, Otype, PULL> serial::Read<u8> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.rx.read()
    }
}

impl<USART: Instance, Otype, PULL> serial::Read<u16> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        self.rx.read_u16()
    }
}

// Blocking

impl<USART: Instance> blocking::Write<u8> for Tx<USART> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.bwrite_all_u8(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.bflush()
    }
}

impl<USART: Instance> blocking::Write<u16> for Tx<USART> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        self.bwrite_all_u16(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.bflush()
    }
}

impl<USART: Instance, Otype, PULL> blocking::Write<u8> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.tx.bwrite_all_u8(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}

impl<USART: Instance, Otype, PULL> blocking::Write<u16> for Serial<USART, Otype, PULL> {
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        self.tx.bwrite_all_u16(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}
