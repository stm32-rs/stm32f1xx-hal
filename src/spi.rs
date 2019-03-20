//! # Serial Peripheral Interface

use core::ptr;

pub use crate::hal::spi::{Mode, Phase, Polarity};
use nb;
#[cfg(feature = "spi1")]
use crate::pac::SPI1;
#[cfg(feature = "spi2")]
use crate::pac::SPI2;

use crate::afio::MAPR;
#[cfg(feature = "gpioa")]
use crate::gpio::gpioa;
#[cfg(feature = "gpiob")]
use crate::gpio::gpiob;
#[cfg(feature = "gpioc")]
use crate::gpio::gpioc;
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{Clocks, APB1, APB2};
use crate::time::Hertz;

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

//#[cfg(feature = "case36")]
#[cfg(feature = "spi1")]
impl Pins<SPI1>
    for ( // NSS: PA4
        gpioa::PA5<Alternate<PushPull>>, // SCK
        gpioa::PA6<Input<Floating>>,     // MISO
        gpioa::PA7<Alternate<PushPull>>, // MOSI
    )
{
    const REMAP: bool = false;
}

//#[cfg(feature = "case36")]
#[cfg(feature = "spi1")]
impl Pins<SPI1>
    for ( // NSS: PA15
        gpiob::PB3<Alternate<PushPull>>, // SCK
        gpiob::PB4<Input<Floating>>,     // MISO
        gpiob::PB5<Alternate<PushPull>>, // MOSI
    )
{
    const REMAP: bool = true;
}

#[cfg(feature = "case48")]
#[cfg(feature = "spi2")]
impl Pins<SPI2>
    for ( // NSS: PB12
        gpiob::PB13<Alternate<PushPull>>, // SCK
        gpiob::PB14<Input<Floating>>,     // MISO
        gpiob::PB15<Alternate<PushPull>>, // MOSI
    )
{
    const REMAP: bool = false;
}

#[cfg(feature = "case64")]
#[cfg(feature = "spi3")]
impl Pins<SPI3>
    for ( // NSS: PA15
        gpiob::PB3<Alternate<PushPull>>, // SCK
        gpiob::PB4<Input<Floating>>,     // MISO
        gpiob::PB5<Alternate<PushPull>>, // MOSI
    )
{
    const REMAP: bool = false;
}

//#[cfg(feature = "case64")]
//#[cfg(feature = "spi3")]
#[cfg(any(feature = "stm32f105", feature = "stm32f107"))]
impl Pins<SPI3>
    for ( // NSS: PA4
        gpioc::PC10<Alternate<PushPull>>, // SCK
        gpioc::PC11<Input<Floating>>,     // MISO
        gpioc::PC12<Alternate<PushPull>>, // MOSI
    )
{
    const REMAP: bool = true;
}

pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

#[cfg(feature = "spi1")]
impl<PINS> Spi<SPI1, PINS> {
    pub fn spi1<F>(
        spi: SPI1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB2,
    ) -> Self
    where
        F: Into<Hertz>,
        PINS: Pins<SPI1>,
    {
        mapr.mapr().modify(|_, w| w.spi1_remap().bit(PINS::REMAP));
        Spi::_spi1(spi, pins, mode, freq.into(), clocks, apb)
    }
}

#[cfg(feature = "spi2")]
impl<PINS> Spi<SPI2, PINS> {
    pub fn spi2<F>(
        spi: SPI2,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
    where
        F: Into<Hertz>,
        PINS: Pins<SPI2>,
    {
        Spi::_spi2(spi, pins, mode, freq.into(), clocks, apb)
    }
}

#[cfg(feature = "spi3")]
impl<PINS> Spi<SPI3, PINS> {
    pub fn spi3<F>(
        spi: SPI3,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
    where
        F: Into<Hertz>,
        PINS: Pins<SPI3>,
    {
        mapr.mapr().modify(|_, w| w.spi3_remap().bit(PINS::REMAP));
        Spi::_spi3(spi, pins, mode, freq.into(), clocks, apb)
    }
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $spiXen:ident, $spiXrst:ident, $APB:ident),)+) => {
        $(
            impl<PINS> Spi<$SPIX, PINS> {
                fn $spiX(
                    spi: $SPIX,
                    pins: PINS,
                    mode: Mode,
                    freq: Hertz,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self {
                    // enable or reset $SPIX
                    apb.enr().modify(|_, w| w.$spiXen().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // disable SS output
                    spi.cr2.write(|w| w.ssoe().clear_bit());

                    let br = match clocks.pclk2().0 / freq.0 {
                        0 => unreachable!(),
                        1...2 => 0b000,
                        3...5 => 0b001,
                        6...11 => 0b010,
                        12...23 => 0b011,
                        24...47 => 0b100,
                        48...95 => 0b101,
                        96...191 => 0b110,
                        _ => 0b111,
                    };

                    // mstr: master configuration
                    // lsbfirst: MSB first
                    // ssm: enable software slave management (NSS pin free for other uses)
                    // ssi: set nss high = master mode
                    // dff: 8 bit frames
                    // bidimode: 2-line unidirectional
                    // spe: enable the SPI bus
                    spi.cr1.write(|w| unsafe {
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
                    });

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

#[cfg(feature = "spi1")]
hal! { SPI1: (_spi1, spi1en, spi1rst, APB2), }
#[cfg(feature = "spi2")]
hal! { SPI2: (_spi2, spi2en, spi2rst, APB1), }
#[cfg(feature = "spi3")]
hal! { SPI3: (_spi3, spi3en, spi3rst, APB1), }
