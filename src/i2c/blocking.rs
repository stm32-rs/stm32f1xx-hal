use super::*;

use crate::pac::DWT;

/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2c<I2C: Instance> {
    nb: I2c<I2C>,
    start_retries: u8,
    timeouts: DwtTimeouts,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DwtTimeouts {
    start: u32,
    addr: u32,
    data: u32,
}

impl<I2C: Instance> BlockingI2c<I2C> {
    /// Creates a blocking I2C1 object on pins PB6 and PB7 or PB8 and PB9 using the embedded-hal `BlockingI2c` trait.
    #[allow(clippy::too_many_arguments)]
    pub fn new<const R: u8>(
        i2c: impl Into<Rmp<I2C, R>>,
        pins: (impl RInto<I2C::Scl, R>, impl RInto<I2C::Sda, R>),
        mode: impl Into<Mode>,
        rcc: &mut Rcc,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        I2c::new(i2c, pins, mode, rcc).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            &rcc.clocks,
        )
    }
}

impl<I2C: Instance> I2c<I2C> {
    /// Generates a blocking I2C instance from a universal I2C object
    pub fn blocking(
        self,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
        clocks: &Clocks,
    ) -> BlockingI2c<I2C> {
        let sysclk_mhz = clocks.sysclk().to_MHz();
        BlockingI2c {
            nb: self,
            start_retries,
            timeouts: DwtTimeouts {
                start: start_timeout_us * sysclk_mhz,
                addr: addr_timeout_us * sysclk_mhz,
                data: data_timeout_us * sysclk_mhz,
            },
        }
    }
    pub fn blocking_default(self, clocks: Clocks) -> BlockingI2c<I2C> {
        let sysclk_mhz = clocks.sysclk().to_MHz();
        BlockingI2c {
            nb: self,
            start_retries: 10,
            timeouts: DwtTimeouts {
                start: 1000 * sysclk_mhz,
                addr: 1000 * sysclk_mhz,
                data: 1000 * sysclk_mhz,
            },
        }
    }
}

macro_rules! wait_for_flag {
    ($i2c:expr, $flag:ident, $nack:ident) => {{
        let sr1 = $i2c.sr1().read();

        if sr1.berr().bit_is_set() {
            $i2c.sr1().write(|w| w.berr().clear_bit());
            Err(Error::Bus.into())
        } else if sr1.arlo().bit_is_set() {
            $i2c.sr1().write(|w| w.arlo().clear_bit());
            Err(Error::ArbitrationLoss.into())
        } else if sr1.af().bit_is_set() {
            $i2c.sr1().write(|w| w.af().clear_bit());
            Err(Error::NoAcknowledge(NoAcknowledgeSource::$nack).into())
        } else if sr1.ovr().bit_is_set() {
            $i2c.sr1().write(|w| w.ovr().clear_bit());
            Err(Error::Overrun.into())
        } else if sr1.$flag().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            match $nb_expr {
                Err(nb::Error::Other(e)) => {
                    #[allow(unreachable_code)]
                    break Err(e);
                }
                Err(nb::Error::WouldBlock) => {
                    if $exit_cond {
                        break Err(Error::Timeout);
                    }
                }
                Ok(x) => break Ok(x),
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::cycle_count();
        let cycles = $cycles;
        busy_wait!($nb_expr, DWT::cycle_count().wrapping_sub(started) >= cycles)
    }};
}

impl<I2C: Instance> BlockingI2c<I2C> {
    /// Check if START condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_after_sent_start(&mut self) -> nb::Result<(), Error> {
        wait_for_flag!(self.nb.i2c, sb, Unknown)
    }

    /// Check if STOP condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_for_stop(&mut self) -> nb::Result<(), Error> {
        if self.nb.i2c.cr1().read().stop().is_no_stop() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send_start_and_wait(&mut self) -> Result<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries;
        let mut last_ret = Ok(());
        while retries_left > 0 {
            self.nb.send_start();
            last_ret = busy_wait_cycles!(self.wait_after_sent_start(), self.timeouts.start);
            if last_ret.is_err() {
                self.nb.reset();
            } else {
                break;
            }
            retries_left -= 1;
        }
        last_ret
    }

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> Result<(), Error> {
        self.nb.i2c.sr1().read();
        self.nb.send_addr(addr, read);
        let ret = busy_wait_cycles!(
            wait_for_flag!(self.nb.i2c, addr, Address),
            self.timeouts.addr
        );
        if let Err(Error::NoAcknowledge(_)) = ret {
            self.nb.send_stop();
        }
        ret
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.nb.i2c.sr1().read();
        self.nb.i2c.sr2().read();

        self.nb.i2c.dr().write(|w| w.dr().set(bytes[0]));

        for byte in &bytes[1..] {
            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tx_e, Data), self.timeouts.data)?;
            self.nb.i2c.dr().write(|w| w.dr().set(*byte));
        }
        busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf, Data), self.timeouts.data)?;

        Ok(())
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;

        let ret = self.write_bytes_and_wait(bytes);
        if let Err(Error::NoAcknowledge(_)) = ret {
            self.nb.send_stop();
        }
        ret
    }

    pub fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.write_without_stop(addr, bytes)?;
        self.nb.send_stop();
        busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;

        Ok(())
    }

    pub fn reset(&mut self){
        self.nb.reset();
    }

    pub fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;

        match buffer.len() {
            1 => {
                self.nb.i2c.cr1().modify(|_, w| w.ack().clear_bit());
                self.nb.i2c.sr1().read();
                self.nb.i2c.sr2().read();
                self.nb.send_stop();

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne, Data), self.timeouts.data)?;
                buffer[0] = self.nb.i2c.dr().read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb.i2c.cr1().modify(|_, w| w.ack().set_bit());
            }
            2 => {
                self.nb
                    .i2c
                    .cr1()
                    .modify(|_, w| w.pos().set_bit().ack().set_bit());
                self.nb.i2c.sr1().read();
                self.nb.i2c.sr2().read();
                self.nb.i2c.cr1().modify(|_, w| w.ack().clear_bit());

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf, Data), self.timeouts.data)?;
                self.nb.send_stop();
                buffer[0] = self.nb.i2c.dr().read().dr().bits();
                buffer[1] = self.nb.i2c.dr().read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb
                    .i2c
                    .cr1()
                    .modify(|_, w| w.pos().clear_bit().ack().clear_bit());
                self.nb.i2c.cr1().modify(|_, w| w.ack().set_bit());
            }
            buffer_len => {
                self.nb.i2c.cr1().modify(|_, w| w.ack().set_bit());
                self.nb.i2c.sr1().read();
                self.nb.i2c.sr2().read();

                let (first_bytes, last_two_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    busy_wait_cycles!(
                        wait_for_flag!(self.nb.i2c, rx_ne, Data),
                        self.timeouts.data
                    )?;
                    *byte = self.nb.i2c.dr().read().dr().bits();
                }

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf, Data), self.timeouts.data)?;
                self.nb.i2c.cr1().modify(|_, w| w.ack().clear_bit());
                last_two_bytes[0] = self.nb.i2c.dr().read().dr().bits();
                self.nb.send_stop();
                last_two_bytes[1] = self.nb.i2c.dr().read().dr().bits();
                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne, Data), self.timeouts.data)?;
                last_two_bytes[2] = self.nb.i2c.dr().read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb.i2c.cr1().modify(|_, w| w.ack().set_bit());
            }
        }

        Ok(())
    }

    pub fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        if !bytes.is_empty() {
            self.write_without_stop(addr, bytes)?;
        }

        if !buffer.is_empty() {
            self.read(addr, buffer)?;
        } else if !bytes.is_empty() {
            self.nb.send_stop();
            busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
        }

        Ok(())
    }

    pub fn transaction_slice(
        &mut self,
        _addr: u8,
        _ops_slice: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Error> {
        todo!();
    }

    pub(crate) fn transaction_slice_hal_02(
        &mut self,
        _addr: u8,
        _ops_slice: &mut [embedded_hal_02::blocking::i2c::Operation<'_>],
    ) -> Result<(), Error> {
        todo!();
    }
}

pub trait BlockingI2cExt: I2cExt {
    #[allow(clippy::too_many_arguments)]
    fn blocking_i2c(
        self,
        pins: (impl RInto<Self::Scl, 0>, impl RInto<Self::Sda, 0>),
        mode: impl Into<Mode>,
        rcc: &mut Rcc,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> BlockingI2c<Self> {
        Self::i2c(self, pins, mode, rcc).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            &rcc.clocks,
        )
    }
}

impl<I2C: Instance, const R: u8> Rmp<I2C, R> {
    #[allow(clippy::too_many_arguments)]
    pub fn blocking_i2c(
        self,
        pins: (impl RInto<I2C::Scl, R>, impl RInto<I2C::Sda, R>),
        mode: impl Into<Mode>,
        rcc: &mut Rcc,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> BlockingI2c<I2C> {
        self.i2c(pins, mode, rcc).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            &rcc.clocks,
        )
    }
}
