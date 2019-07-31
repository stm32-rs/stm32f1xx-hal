//! # Serial Peripheral Interface

use core::ptr;

use nb;

pub use crate::hal::spi::{Mode, Phase, Polarity};
use crate::pac::{SPI1, SPI2};

use crate::afio::MAPR;
use crate::gpio::gpioa::{PA5, PA6, PA7};
use crate::gpio::gpiob::{PB13, PB14, PB15, PB3, PB4, PB5};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{RccBus, Clocks, Enable, Reset};
use crate::time::Hertz;
use crate::dma::dma1::{C3, C5};
use crate::dma::{Transmit, TxDma, Transfer, R, Static, TransferPayload};

use core::sync::atomic::{self, Ordering};

use as_slice::AsSlice;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<SPI> {
    const REMAP: bool;
}

impl Pins<SPI1>
    for (
        PA5<Alternate<PushPull>>,
        PA6<Input<Floating>>,
        PA7<Alternate<PushPull>>,
    )
{
    const REMAP: bool = false;
}

impl Pins<SPI1>
    for (
        PB3<Alternate<PushPull>>,
        PB4<Input<Floating>>,
        PB5<Alternate<PushPull>>,
    )
{
    const REMAP: bool = true;
}

impl Pins<SPI2>
    for (
        PB13<Alternate<PushPull>>,
        PB14<Input<Floating>>,
        PB15<Alternate<PushPull>>,
    )
{
    const REMAP: bool = false;
}

pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

impl<PINS> Spi<SPI1, PINS> {
    pub fn spi1<F>(
        spi: SPI1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut <SPI1 as RccBus>::Bus,
    ) -> Self
    where
        F: Into<Hertz>,
        PINS: Pins<SPI1>,
    {
        mapr.modify_mapr(|_, w| w.spi1_remap().bit(PINS::REMAP));
        Spi::_spi1(spi, pins, mode, freq.into(), clocks, apb)
    }
}

impl<PINS> Spi<SPI2, PINS> {
    pub fn spi2<F>(
        spi: SPI2,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut <SPI2 as RccBus>::Bus,
    ) -> Self
    where
        F: Into<Hertz>,
        PINS: Pins<SPI2>,
    {
        Spi::_spi2(spi, pins, mode, freq.into(), clocks, apb)
    }
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident),)+) => {
        $(
            impl<PINS> Spi<$SPIX, PINS> {
                fn $spiX(
                    spi: $SPIX,
                    pins: PINS,
                    mode: Mode,
                    freq: Hertz,
                    clocks: Clocks,
                    apb: &mut <$SPIX as RccBus>::Bus,
                ) -> Self {
                    // enable or reset $SPIX
                    $SPIX::enable(apb);
                    $SPIX::reset(apb);

                    // disable SS output
                    spi.cr2.write(|w| w.ssoe().clear_bit());

                    let br = match clocks.pclk2().0 / freq.0 {
                        0 => unreachable!(),
                        1..=2 => 0b000,
                        3..=5 => 0b001,
                        6..=11 => 0b010,
                        12..=23 => 0b011,
                        24..=47 => 0b100,
                        48..=95 => 0b101,
                        96..=191 => 0b110,
                        _ => 0b111,
                    };

                    // mstr: master configuration
                    // lsbfirst: MSB first
                    // ssm: enable software slave management (NSS pin free for other uses)
                    // ssi: set nss high = master mode
                    // dff: 8 bit frames
                    // bidimode: 2-line unidirectional
                    // spe: enable the SPI bus
                    spi.cr1.write(|w|
                        w.cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .mstr()
                            .set_bit()
                            .br()
                            .bits(br)
                            .lsbfirst()
                            .clear_bit()
                            .ssm()
                            .set_bit()
                            .ssi()
                            .set_bit()
                            .rxonly()
                            .clear_bit()
                            .dff()
                            .clear_bit()
                            .bidimode()
                            .clear_bit()
                            .spe()
                            .set_bit()
                    );

                    Spi { spi, pins }
                }

                pub fn free(self) -> ($SPIX, PINS) {
                    (self.spi, self.pins)
                }
            }

            impl<PINS> crate::hal::spi::FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }

            }

            impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

hal! {
    SPI1: (_spi1),
    SPI2: (_spi2),
}

// DMA

pub struct SpiPayload<SPI, PINS> {
    spi: Spi<SPI, PINS>
}

pub type SpiTxDma<SPI, PINS, CHANNEL> = TxDma<SpiPayload<SPI, PINS>, CHANNEL>;

macro_rules! spi_dma {
    ($SPIi:ident, $TCi:ident) => {
        impl<PINS> Transmit for SpiTxDma<$SPIi, PINS, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<PINS> Spi<$SPIi, PINS> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiTxDma<$SPIi, PINS, $TCi> {
                let payload = SpiPayload{
                    spi: self
                };
                SpiTxDma {payload, channel}
            }
        }

        impl<PINS> TransferPayload for SpiTxDma<$SPIi, PINS, $TCi> {
            fn start(&mut self) {
                self.payload.spi.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                self.channel.start();
            }
            fn stop(&mut self) {
                self.payload.spi.spi.cr2.modify(|_, w| w.txdmaen().clear_bit());
                self.channel.stop();
            }
        }

        impl<A, B, PIN> crate::dma::WriteDma<A, B, u8> for SpiTxDma<$SPIi, PIN, $TCi>
        where
            A: AsSlice<Element=u8>,
            B: Static<A>
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                {
                    let buffer = buffer.borrow().as_slice();
                    self.channel.set_peripheral_address(unsafe{ &(*$SPIi::ptr()).dr as *const _ as u32 }, false);
                    self.channel.set_memory_address(buffer.as_ptr() as u32, true);
                    self.channel.set_transfer_length(buffer.len());
                }
                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr.modify(|_, w| { w
                    .mem2mem() .clear_bit()
                    .pl()      .medium()
                    .msize()   .bits8()
                    .psize()   .bits8()
                    .circ()    .clear_bit()
                    .dir()     .set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }
    }
}

spi_dma!(SPI1, C3);
spi_dma!(SPI2, C5);
