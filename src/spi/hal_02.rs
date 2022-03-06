use super::*;

use embedded_hal::{blocking::spi as blocking, spi};

impl<SPI, REMAP, PINS, FrameSize, OP> spi::FullDuplex<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
    FrameSize: Copy,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<FrameSize, Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            Error::Overrun.into()
        } else if sr.modf().bit_is_set() {
            Error::ModeFault.into()
        } else if sr.crcerr().bit_is_set() {
            Error::Crc.into()
        } else if sr.rxne().bit_is_set() {
            // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
            // reading a half-word)
            return Ok(self.read_data_reg());
        } else {
            nb::Error::WouldBlock
        })
    }

    fn send(&mut self, data: FrameSize) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        // NOTE: Error::Overrun was deleted in #408. Need check
        Err(if sr.modf().bit_is_set() {
            Error::ModeFault.into()
        } else if sr.crcerr().bit_is_set() {
            Error::Crc.into()
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            self.write_data_reg(data);
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<SPI, REMAP, PINS, FrameSize, OP> blocking::transfer::Default<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
    FrameSize: Copy,
{
}

impl<SPI: Instance, REMAP, PINS, OP> blocking::Write<u8> for Spi<SPI, REMAP, PINS, u8, OP> {
    type Error = Error;

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.spi_write(words)
    }
}

impl<SPI: Instance, REMAP, PINS, OP> blocking::Write<u16> for Spi<SPI, REMAP, PINS, u16, OP> {
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Error> {
        self.spi_write(words)
    }
}
