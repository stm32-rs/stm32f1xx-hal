use super::*;

/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2c<I2C, PINS> {
    nb: I2c<I2C, PINS>,
    start_retries: u8,
    timeouts: DwtTimeouts,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DwtTimeouts {
    start: u32,
    addr: u32,
    data: u32,
}

impl<PINS> BlockingI2c<I2C1, PINS> {
    /// Creates a blocking I2C1 object on pins PB6 and PB7 or PB8 and PB9 using the embedded-hal `BlockingI2c` trait.
    #[allow(clippy::too_many_arguments)]
    pub fn i2c1<M: Into<Mode>>(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: M,
        clocks: Clocks,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        BlockingI2c::<I2C1, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<PINS> BlockingI2c<I2C2, PINS> {
    /// Creates a blocking I2C2 object on pins PB10 and PB1
    #[allow(clippy::too_many_arguments)]
    pub fn i2c2<M: Into<Mode>>(
        i2c: I2C2,
        pins: PINS,
        mode: M,
        clocks: Clocks,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C2>,
    {
        BlockingI2c::<I2C2, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<I2C, PINS> I2c<I2C, PINS> {
    /// Generates a blocking I2C instance from a universal I2C object
    pub fn blocking(
        self,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
        clocks: Clocks,
    ) -> BlockingI2c<I2C, PINS> {
        let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
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
    pub fn blocking_default(self, clocks: Clocks) -> BlockingI2c<I2C, PINS> {
        let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
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
    ($i2c:expr, $flag:ident) => {{
        let sr1 = $i2c.sr1.read();

        if sr1.berr().bit_is_set() {
            $i2c.sr1.write(|w| w.berr().clear_bit());
            Err(Other(Error::Bus))
        } else if sr1.arlo().bit_is_set() {
            $i2c.sr1.write(|w| w.arlo().clear_bit());
            Err(Other(Error::Arbitration))
        } else if sr1.af().bit_is_set() {
            $i2c.sr1.write(|w| w.af().clear_bit());
            Err(Other(Error::Acknowledge))
        } else if sr1.ovr().bit_is_set() {
            $i2c.sr1.write(|w| w.ovr().clear_bit());
            Err(Other(Error::Overrun))
        } else if sr1.$flag().bit_is_set() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(WouldBlock) {
                break res;
            }
            if $exit_cond {
                break res;
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        busy_wait!(
            $nb_expr,
            DWT::get_cycle_count().wrapping_sub(started) >= cycles
        )
    }};
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Instance,
{
    #[allow(clippy::too_many_arguments)]
    fn _i2c<M: Into<Mode>>(
        i2c: I2C,
        pins: PINS,
        mode: M,
        clocks: Clocks,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        I2c::<I2C, _>::_i2c(i2c, pins, mode, clocks).blocking(
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
            clocks,
        )
    }
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Instance,
{
    /// Check if START condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_after_sent_start(&mut self) -> NbResult<(), Error> {
        wait_for_flag!(self.nb.i2c, sb)
    }

    /// Check if STOP condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_for_stop(&mut self) -> NbResult<(), Error> {
        if self.nb.i2c.cr1.read().stop().is_no_stop() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }

    fn send_start_and_wait(&mut self) -> NbResult<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries;
        let mut last_ret: NbResult<(), Error> = Err(WouldBlock);
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

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> NbResult<(), Error> {
        self.nb.i2c.sr1.read();
        self.nb.send_addr(addr, read);
        let ret = busy_wait_cycles!(wait_for_flag!(self.nb.i2c, addr), self.timeouts.addr);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> NbResult<(), Error> {
        self.nb.i2c.sr1.read();
        self.nb.i2c.sr2.read();

        self.nb.i2c.dr.write(|w| w.dr().bits(bytes[0]));

        for byte in &bytes[1..] {
            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tx_e), self.timeouts.data)?;
            self.nb.i2c.dr.write(|w| w.dr().bits(*byte));
        }
        busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf), self.timeouts.data)?;

        Ok(())
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> NbResult<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;

        let ret = self.write_bytes_and_wait(bytes);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }
}

impl<I2C, PINS> Write for BlockingI2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = NbError<Error>;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.nb.send_stop();
        busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;

        Ok(())
    }
}

impl<I2C, PINS> Read for BlockingI2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = NbError<Error>;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;

        match buffer.len() {
            1 => {
                self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                self.nb.i2c.sr1.read();
                self.nb.i2c.sr2.read();
                self.nb.send_stop();

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.timeouts.data)?;
                buffer[0] = self.nb.i2c.dr.read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            2 => {
                self.nb
                    .i2c
                    .cr1
                    .modify(|_, w| w.pos().set_bit().ack().set_bit());
                self.nb.i2c.sr1.read();
                self.nb.i2c.sr2.read();
                self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf), self.timeouts.data)?;
                self.nb.send_stop();
                buffer[0] = self.nb.i2c.dr.read().dr().bits();
                buffer[1] = self.nb.i2c.dr.read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb
                    .i2c
                    .cr1
                    .modify(|_, w| w.pos().clear_bit().ack().clear_bit());
                self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            buffer_len => {
                self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
                self.nb.i2c.sr1.read();
                self.nb.i2c.sr2.read();

                let (first_bytes, last_two_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.timeouts.data)?;
                    *byte = self.nb.i2c.dr.read().dr().bits();
                }

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf), self.timeouts.data)?;
                self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                last_two_bytes[0] = self.nb.i2c.dr.read().dr().bits();
                self.nb.send_stop();
                last_two_bytes[1] = self.nb.i2c.dr.read().dr().bits();
                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.timeouts.data)?;
                last_two_bytes[2] = self.nb.i2c.dr.read().dr().bits();

                busy_wait_cycles!(self.wait_for_stop(), self.timeouts.data)?;
                self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
        }

        Ok(())
    }
}

impl<I2C, PINS> WriteRead for BlockingI2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = NbError<Error>;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
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
}
