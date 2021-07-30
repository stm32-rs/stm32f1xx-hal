//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::afio::MAPR;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::{Alternate, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::pac::{self, DWT, I2C1, I2C2, RCC};
use crate::rcc::{Clocks, Enable, GetBusFreq, Reset};
use crate::time::Hertz;
use core::ops::Deref;

pub mod blocking;
pub use blocking::BlockingI2c;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq)]
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
    pub fn standard<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::Standard {
            frequency: frequency.into(),
        }
    }

    pub fn fast<F: Into<Hertz>>(frequency: F, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency: frequency.into(),
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

impl<F> From<F> for Mode
where
    F: Into<Hertz>,
{
    fn from(frequency: F) -> Self {
        let frequency: Hertz = frequency.into();
        if frequency.0 <= 100_000 {
            Self::Standard { frequency }
        } else {
            Self::Fast {
                frequency,
                duty_cycle: DutyCycle::Ratio2to1,
            }
        }
    }
}

/// Helper trait to ensure that the correct I2C pins are used for the corresponding interface
pub trait Pins<I2C> {
    const REMAP: bool;
}

impl Pins<I2C1> for (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>) {
    const REMAP: bool = false;
}

impl Pins<I2C1> for (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>) {
    const REMAP: bool = true;
}

impl Pins<I2C2> for (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>) {
    const REMAP: bool = false;
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    mode: Mode,
    clk: Hertz,
    start_retries: u8,
}

pub trait Instance:
    crate::Sealed + Deref<Target = pac::i2c1::RegisterBlock> + Enable + Reset + GetBusFreq
{
}

impl Instance for I2C1 {}
impl Instance for I2C2 {}

impl<PINS> I2c<I2C1, PINS> {
    /// Creates a generic I2C1 object on pins PB6 and PB7 or PB8 and PB9 (if remapped)
    pub fn i2c1<M: Into<Mode>>(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: M,
        start_retries: u8,
        clocks: Clocks,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        I2c::<I2C1, _>::_i2c(i2c, pins, mode, start_retries, clocks)
    }
}

impl<PINS> I2c<I2C2, PINS> {
    /// Creates a generic I2C2 object on pins PB10 and PB11 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c2<M: Into<Mode>>(
        i2c: I2C2,
        pins: PINS,
        mode: M,
        start_retries: u8,
        clocks: Clocks,
    ) -> Self
    where
        PINS: Pins<I2C2>,
    {
        I2c::<I2C2, _>::_i2c(i2c, pins, mode, start_retries, clocks)
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Configures the I2C peripheral to work in master mode
    fn _i2c<M: Into<Mode>>(
        i2c: I2C,
        pins: PINS,
        mode: M,
        start_retries: u8,
        clocks: Clocks,
    ) -> Self {
        let mode = mode.into();
        let rcc = unsafe { &(*RCC::ptr()) };
        I2C::enable(rcc);
        I2C::reset(rcc);

        let clk = I2C::get_frequency(&clocks);

        assert!(mode.get_frequency().0 <= 400_000);

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            clk,
            start_retries,
        };
        i2c.init();
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self) {
        let freq = self.mode.get_frequency();
        let clk_mhz = (self.clk.0 / 1000000) as u16;

        self.i2c
            .cr2
            .write(|w| unsafe { w.freq().bits(clk_mhz as u8) });
        self.i2c.cr1.write(|w| w.pe().clear_bit());

        match self.mode {
            Mode::Standard { .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((clk_mhz + 1) as u8));
                self.i2c.ccr.write(|w| unsafe {
                    w.ccr().bits(((self.clk.0 / (freq.0 * 2)) as u16).max(4))
                });
            }
            Mode::Fast { ref duty_cycle, .. } => {
                self.i2c
                    .trise
                    .write(|w| w.trise().bits((clk_mhz * 300 / 1000 + 1) as u8));

                self.i2c.ccr.write(|w| {
                    let (freq, duty) = match duty_cycle {
                        DutyCycle::Ratio2to1 => {
                            (((self.clk.0 / (freq.0 * 3)) as u16).max(1), false)
                        }
                        DutyCycle::Ratio16to9 => {
                            (((self.clk.0 / (freq.0 * 25)) as u16).max(1), true)
                        }
                    };

                    unsafe { w.ccr().bits(freq).duty().bit(duty).f_s().set_bit() }
                });
            }
        };

        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Perform an I2C software reset
    fn reset(&mut self) {
        self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1.reset();
        self.init();
    }

    /// Generate START condition
    fn send_start(&mut self) {
        // Clear all pending error bits
        // NOTE(unsafe): Writing 0 clears the r/w bits and has no effect on the r bits
        self.i2c.sr1.reset();
        self.i2c.cr1.modify(|_, w| w.start().set_bit());
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .dr
            .write(|w| w.dr().bits(addr << 1 | (if read { 1 } else { 0 })));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
    }

    /// Clears the I2C ADDR pending flag
    fn clear_addr_flag(&self) {
        self.i2c.sr1.read();
        self.i2c.sr2.read();
    }

    /// Releases the I2C peripheral and associated pins
    pub fn release(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }

    #[deprecated(since = "0.7.1", note = "Please use release instead")]
    pub fn free(self) -> (I2C, PINS) {
        self.release()
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Instance,
{
    fn check_and_clear_error_flags(&self) -> Result<pac::i2c1::sr1::R, Error> {
        let sr1 = self.i2c.sr1.read();
        if sr1.bits() != 0 {
            // Writing 1s in order to only clear the flag we spotted even
            // if the register gets modified externally
            // NOTE(unsafe): Writing 1 to registers which are cleared by 0 has no effect.
            //               Similarly, writing to read-only registers has no effect
            if sr1.berr().bit_is_set() {
                self.i2c
                    .sr1
                    .write(|w| unsafe { w.bits(0xffff).berr().clear_bit() });
                return Err(Error::Bus);
            } else if sr1.arlo().bit_is_set() {
                self.i2c
                    .sr1
                    .write(|w| unsafe { w.bits(0xffff).arlo().clear_bit() });
                return Err(Error::Arbitration);
            } else if sr1.af().bit_is_set() {
                self.i2c
                    .sr1
                    .write(|w| unsafe { w.bits(0xffff).af().clear_bit() });
                return Err(Error::Acknowledge);
            } else if sr1.ovr().bit_is_set() {
                self.i2c
                    .sr1
                    .write(|w| unsafe { w.bits(0xffff).ovr().clear_bit() });
                return Err(Error::Overrun);
            }
        }
        Ok(sr1)
    }

    /// Check if STOP condition is generated
    fn wait_for_stop(&mut self) {
        while self.i2c.cr1.read().stop().is_stop() {}
    }

    fn send_start_and_wait(&mut self) -> Result<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries.max(1);
        loop {
            retries_left -= 1;
            self.send_start();
            loop {
                match self.check_and_clear_error_flags() {
                    Ok(sr1) => {
                        // Check if START condition is generated
                        if sr1.sb().bit_is_set() {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        if retries_left > 0 {
                            self.reset();
                            break;
                        } else {
                            return Err(e);
                        }
                    }
                }
            }
        }
    }

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> Result<(), Error> {
        self.i2c.sr1.read();
        self.send_addr(addr, read);
        loop {
            match self.check_and_clear_error_flags() {
                Ok(sr1) => {
                    if sr1.addr().bit_is_set() {
                        break Ok(());
                    }
                }
                Err(Error::Acknowledge) => {
                    self.send_stop();
                    break Err(Error::Acknowledge);
                }
                Err(e) => break Err(e),
            }
        }
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.clear_addr_flag();

        self.i2c.dr.write(|w| w.dr().bits(bytes[0]));

        for byte in &bytes[1..] {
            while self.check_and_clear_error_flags()?.tx_e().bit_is_clear() {}
            self.i2c.dr.write(|w| w.dr().bits(*byte));
        }
        while self.check_and_clear_error_flags()?.btf().bit_is_clear() {}

        Ok(())
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;

        let ret = self.write_bytes_and_wait(bytes);
        if ret == Err(Error::Acknowledge) {
            self.send_stop();
        }
        ret
    }
}

impl<I2C, PINS> Write for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.send_stop();
        self.wait_for_stop();

        Ok(())
    }
}

impl<I2C, PINS> Read for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;

        match buffer.len() {
            1 => {
                self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                self.clear_addr_flag();
                self.send_stop();

                while self.check_and_clear_error_flags()?.rx_ne().bit_is_clear() {}
                buffer[0] = self.i2c.dr.read().dr().bits();

                self.wait_for_stop();
                self.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            2 => {
                self.i2c
                    .cr1
                    .modify(|_, w| w.pos().set_bit().ack().set_bit());
                self.clear_addr_flag();
                self.i2c.cr1.modify(|_, w| w.ack().clear_bit());

                while self.check_and_clear_error_flags()?.btf().bit_is_clear() {}
                self.send_stop();
                buffer[0] = self.i2c.dr.read().dr().bits();
                buffer[1] = self.i2c.dr.read().dr().bits();

                self.wait_for_stop();
                self.i2c
                    .cr1
                    .modify(|_, w| w.pos().clear_bit().ack().clear_bit());
                self.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            buffer_len => {
                self.i2c.cr1.modify(|_, w| w.ack().set_bit());
                self.clear_addr_flag();

                let (first_bytes, last_3_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    while self.check_and_clear_error_flags()?.rx_ne().bit_is_clear() {}
                    *byte = self.i2c.dr.read().dr().bits();
                }

                while self.check_and_clear_error_flags()?.btf().bit_is_clear() {}
                self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                last_3_bytes[0] = self.i2c.dr.read().dr().bits();
                self.send_stop();
                last_3_bytes[1] = self.i2c.dr.read().dr().bits();
                while self.check_and_clear_error_flags()?.rx_ne().bit_is_clear() {}
                last_3_bytes[2] = self.i2c.dr.read().dr().bits();

                self.wait_for_stop();
                self.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
        }

        Ok(())
    }
}

impl<I2C, PINS> WriteRead for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        if !bytes.is_empty() {
            self.write_without_stop(addr, bytes)?;
        }

        if !buffer.is_empty() {
            self.read(addr, buffer)?;
        } else if !bytes.is_empty() {
            self.send_stop();
            self.wait_for_stop();
        }

        Ok(())
    }
}
