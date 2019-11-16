//! Inter-Integrated Circuit (I2C) bus

use crate::afio::MAPR;
use crate::time::Hertz;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::{Alternate, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use nb::Error::{Other, WouldBlock};
use nb::{Error as NbError, Result as NbResult};
use crate::rcc::{Clocks, APB1, Enable, Reset};
use crate::pac::{DWT, I2C1, I2C2};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
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
    #[doc(hidden)]
    _Extensible,
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
        Mode::Standard{frequency: frequency.into()}
    }

    pub fn fast<F: Into<Hertz>>(frequency: F, duty_cycle: DutyCycle) -> Self {
        Mode::Fast{frequency: frequency.into(), duty_cycle}
    }

    pub fn get_frequency(&self) -> Hertz {
        match self {
            &Mode::Standard { frequency } => frequency,
            &Mode::Fast { frequency, .. } => frequency,
        }
    }
}

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
    pclk1: u32,
}

pub struct BlockingI2c<I2C, PINS> {
    nb: I2c<I2C, PINS>,
    start_timeout: u32,
    start_retries: u8,
    addr_timeout: u32,
    data_timeout: u32,
}

impl<PINS> I2c<I2C1, PINS> {
    pub fn i2c1(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        I2c::_i2c1(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C1, PINS> {
    pub fn i2c1(
        i2c: I2C1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        mapr.modify_mapr(|_, w| w.i2c1_remap().bit(PINS::REMAP));
        BlockingI2c::_i2c1(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<PINS> I2c<I2C2, PINS> {
    pub fn i2c2(i2c: I2C2, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut APB1) -> Self
    where
        PINS: Pins<I2C2>,
    {
        I2c::_i2c2(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C2, PINS> {
    pub fn i2c2(
        i2c: I2C2,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C2>,
    {
        BlockingI2c::_i2c2(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

pub fn blocking_i2c<I2C, PINS>(
    i2c: I2c<I2C, PINS>,
    clocks: Clocks,
    start_timeout_us: u32,
    start_retries: u8,
    addr_timeout_us: u32,
    data_timeout_us: u32,
) -> BlockingI2c<I2C, PINS> {
    let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
    return BlockingI2c {
        nb: i2c,
        start_timeout: start_timeout_us * sysclk_mhz,
        start_retries,
        addr_timeout: addr_timeout_us * sysclk_mhz,
        data_timeout: data_timeout_us * sysclk_mhz,
    };
}

macro_rules! wait_for_flag {
    ($i2c:expr, $flag:ident) => {{
        let sr1 = $i2c.sr1.read();

        if sr1.berr().bit_is_set() {
            Err(Other(Error::Bus))
        } else if sr1.arlo().bit_is_set() {
            Err(Other(Error::Arbitration))
        } else if sr1.af().bit_is_set() {
            Err(Other(Error::Acknowledge))
        } else if sr1.ovr().bit_is_set() {
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

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<PINS> I2c<$I2CX, PINS> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX(
                    i2c: $I2CX,
                    pins: PINS,
                    mode: Mode,
                    clocks: Clocks,
                    apb: &mut APB1,
                ) -> Self {
                    $I2CX::enable(apb);
                    $I2CX::reset(apb);

                    let pclk1 = clocks.pclk1().0;

                    assert!(mode.get_frequency().0 <= 400_000);

                    let mut i2c = I2c { i2c, pins, mode, pclk1 };
                    i2c.init();
                    i2c
                }

                fn init(&mut self) {
                    let freq = self.mode.get_frequency();
                    let pclk1_mhz = (self.pclk1 / 1000000) as u16;

                    self.i2c.cr2.write(|w| unsafe {
                        w.freq().bits(pclk1_mhz as u8)
                    });
                    self.i2c.cr1.write(|w| w.pe().clear_bit());

                    match self.mode {
                        Mode::Standard { .. } => {
                            self.i2c.trise.write(|w| {
                                w.trise().bits((pclk1_mhz + 1) as u8)
                            });
                            self.i2c.ccr.write(|w| unsafe {
                                w.ccr().bits(((self.pclk1 / (freq.0 * 2)) as u16).max(4))
                            });
                        },
                        Mode::Fast { ref duty_cycle, .. } => {
                            self.i2c.trise.write(|w| {
                                w.trise().bits((pclk1_mhz * 300 / 1000 + 1) as u8)
                            });

                            self.i2c.ccr.write(|w| {
                                let (freq, duty) = match duty_cycle {
                                    &DutyCycle::Ratio2to1 => (((self.pclk1 / (freq.0* 3)) as u16).max(1), false),
                                    &DutyCycle::Ratio16to9 => (((self.pclk1 / (freq.0 * 25)) as u16).max(1), true)
                                };

                                unsafe {
                                    w.ccr().bits(freq).duty().bit(duty).f_s().set_bit()
                                }
                            });
                        }
                    };

                    self.i2c.cr1.modify(|_, w| w.pe().set_bit());
                }

                fn reset(&mut self) {
                    self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
                    self.i2c.cr1.reset();
                    self.init();
                }

                fn send_start(&mut self) {
                    self.i2c.cr1.modify(|_, w| w.start().set_bit());
                }

                fn wait_after_sent_start(&mut self) -> NbResult<(), Error> {
                    wait_for_flag!(self.i2c, sb)
                }

                fn send_addr(&self, addr: u8, read: bool) {
                    self.i2c.dr.write(|w| { w.dr().bits(addr << 1 | (if read {1} else {0})) });
                }

                fn wait_after_sent_addr(&self) -> NbResult<(), Error> {
                    wait_for_flag!(self.i2c, addr)?;
                    self.i2c.sr2.read();
                    Ok(())
                }

                fn send_stop(&self) {
                    self.i2c.cr1.modify(|_, w| w.stop().set_bit());
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, PINS) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> BlockingI2c<$I2CX, PINS> {
                pub fn $i2cX(
                    i2c: $I2CX,
                    pins: PINS,
                    mode: Mode,
                    clocks: Clocks,
                    apb: &mut APB1,
                    start_timeout_us: u32,
                    start_retries: u8,
                    addr_timeout_us: u32,
                    data_timeout_us: u32
                ) -> Self {
                    blocking_i2c(I2c::$i2cX(i2c, pins, mode, clocks, apb),
                        clocks, start_timeout_us, start_retries,
                        addr_timeout_us, data_timeout_us)
                }

                fn send_start_and_wait(&mut self) -> NbResult<(), Error> {
                    // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
                    // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced Stop
                    let mut retries_left = self.start_retries;
                    let mut last_ret: NbResult<(), Error> = Err(WouldBlock);
                    while retries_left > 0 {
                        self.nb.send_start();
                        last_ret = busy_wait_cycles!(self.nb.wait_after_sent_start(), self.start_timeout);
                        if let Err(_) = last_ret {
                            self.nb.reset();
                        } else {
                            break;
                        }
                        retries_left -= 1;
                    }
                    last_ret
                }

                fn send_addr_and_wait(&self, addr: u8, read: bool) -> NbResult<(), Error> {
                    self.nb.send_addr(addr, read);
                    busy_wait_cycles!(self.nb.wait_after_sent_addr(), self.addr_timeout)
                }

                fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> NbResult<(), Error> {
                    self.send_start_and_wait()?;
                    self.send_addr_and_wait(addr, false)?;

                    for byte in bytes {
                        busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tx_e), self.data_timeout)?;
                        self.nb.i2c.dr.write(|w| { w.dr().bits(*byte) });
                    }
                    busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tx_e), self.data_timeout)?;

                    Ok(())
                }
            }

            impl<PINS> Write for BlockingI2c<$I2CX, PINS> {
                type Error = NbError<Error>;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                    self.write_without_stop(addr, bytes)?;
                    self.nb.send_stop();

                    Ok(())
                }
            }

            impl<PINS> Read for BlockingI2c<$I2CX, PINS> {
                type Error = NbError<Error>;

                fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    self.send_start_and_wait()?;

                    match buffer.len() {
                        1 => {
                            self.nb.send_addr(addr, true);
                            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, addr), self.addr_timeout)?;
                            self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                            let _ = self.nb.i2c.sr2.read();
                            self.nb.send_stop();

                            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.data_timeout)?;
                            buffer[0] = self.nb.i2c.dr.read().dr().bits();
                        }
                        2 => {
                            self.nb.i2c.cr1.modify(|_, w| w.pos().set_bit().ack().set_bit());
                            self.send_addr_and_wait(addr, true)?;
                            self.nb.i2c.cr1.modify(|_, w| w.pos().clear_bit().ack().clear_bit());

                            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf), self.data_timeout)?;
                            self.nb.send_stop();
                            buffer[0] = self.nb.i2c.dr.read().dr().bits();
                            buffer[1] = self.nb.i2c.dr.read().dr().bits();
                        }
                        buffer_len => {
                            self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
                            self.send_addr_and_wait(addr, true)?;

                            let (first_bytes, last_two_bytes) = buffer.split_at_mut(buffer_len - 3);
                            for byte in first_bytes {
                                self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
                                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.data_timeout)?;
                                *byte = self.nb.i2c.dr.read().dr().bits();
                            }

                            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, btf), self.data_timeout)?;
                            self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                            last_two_bytes[0] = self.nb.i2c.dr.read().dr().bits();
                            self.nb.send_stop();
                            last_two_bytes[1] = self.nb.i2c.dr.read().dr().bits();
                            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rx_ne), self.data_timeout)?;
                            last_two_bytes[2] = self.nb.i2c.dr.read().dr().bits();
                        }
                    }

                    Ok(())
                }
            }

            impl<PINS> WriteRead for BlockingI2c<$I2CX, PINS> {
                type Error = NbError<Error>;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Self::Error> {
                    if !bytes.is_empty() {
                        self.write_without_stop(addr, bytes)?;
                    }

                    if !buffer.is_empty() {
                        self.read(addr, buffer)?;
                    } else if !bytes.is_empty() {
                        self.nb.send_stop();
                    }

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (_i2c1),
    I2C2: (_i2c2),
}
