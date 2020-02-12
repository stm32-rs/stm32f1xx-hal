/*!
  # Serial Peripheral Interface

  ## Alternate function remapping

  ### SPI1

  | Function | Spi1NoRemap | Spi1Remap |
  |:----:|:-----------:|:---------:|
  | SCK  |     PA5     |    PB3   |
  | MISO |     PA6     |    PB4   |
  | MOSI |     PA7     |    PB5   |

  ### SPI2

  | Function | Spi2NoRemap |
  |:----:|:-----------:|
  | SCK  |     PB13     |
  | MISO |     PB14     |
  | MOSI |     PB15     |

  ### SPI3

  Available only on high density devices.

  | Function | Spi3NoRemap | Spi3Remap |
  |:----:|:-----------:|:---------:|
  | SCK  |     PB3     |    PC10   |
  | MISO |     PB4     |    PC11   |
  | MOSI |     PB5     |    PC12   |
*/

use core::ops::Deref;
use core::ptr;

use nb;

pub use crate::hal::spi::{Mode, Phase, Polarity, FullDuplex};
#[cfg(feature = "high")]
use crate::pac::SPI3;
use crate::pac::{SPI1, SPI2};

use crate::afio::MAPR;
use crate::dma::dma1::{C3, C5};
use crate::dma::{Static, Transfer, TransferPayload, Transmit, TxDma, R};
use crate::gpio::gpioa::{PA5, PA6, PA7};
use crate::gpio::gpiob::{PB13, PB14, PB15, PB3, PB4, PB5};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{sealed::RccBus, Clocks, Enable, GetBusFreq, Reset};
use crate::time::Hertz;

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

use core::marker::PhantomData;

mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: bool;
    }
    pub trait Sck<REMAP> {}
    pub trait Miso<REMAP> {}
    pub trait Mosi<REMAP> {}
    pub struct _Sck;
    pub struct _Miso;
    pub struct _Mosi;
}
use sealed::{Miso, Mosi, Remap, Sck};

pub trait Pins<REMAP, P> {
    type _Pos;
}
macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($TRAIT:ident),+ ), ( $($POS:ident),* ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<REMAP, $($PINX,)+> Pins<REMAP, ($(sealed::$POS),+)> for ($($PINX),+)
            where
                $($PINX: $TRAIT<REMAP>,)+
            {
                type _Pos = ($(sealed::$POS),+);
            }
        )+
    };
}

pins_impl!(
    (SCK, MISO, MOSI), (Sck, Miso, Mosi), (_Sck, _Miso, _Mosi);
    (SCK, MOSI, MISO), (Sck, Mosi, Miso), (_Sck, _Mosi, _Miso);
    (MOSI, SCK, MISO), (Mosi, Sck, Miso), (_Mosi, _Sck, _Miso);
    (MOSI, MISO, SCK), (Mosi, Miso, Sck), (_Mosi, _Miso, _Sck);
    (MISO, MOSI, SCK), (Miso, Mosi, Sck), (_Miso, _Mosi, _Sck);
    (MISO, SCK, MOSI), (Miso, Sck, Mosi), (_Miso, _Sck, _Mosi);
);

pub struct Spi<SPI, REMAP, PINS> {
    spi: SPI,
    pins: PINS,
    _remap: PhantomData<REMAP>,
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;
/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;
/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

impl<REMAP> Sck<REMAP> for NoSck {}
impl<REMAP> Miso<REMAP> for NoMiso {}
impl<REMAP> Mosi<REMAP> for NoMosi {}

macro_rules! remap {
    ($name:ident, $SPIX:ident, $state:literal, $SCK:ident, $MISO:ident, $MOSI:ident) => {
        pub struct $name;
        impl Remap for $name {
            type Periph = $SPIX;
            const REMAP: bool = $state;
        }
        impl Sck<$name> for $SCK<Alternate<PushPull>> {}
        impl Miso<$name> for $MISO<Input<Floating>> {}
        impl Mosi<$name> for $MOSI<Alternate<PushPull>> {}
    };
}

remap!(Spi1NoRemap, SPI1, false, PA5, PA6, PA7);
remap!(Spi1Remap, SPI1, true, PB3, PB4, PB5);
remap!(Spi2NoRemap, SPI2, false, PB13, PB14, PB15);
#[cfg(feature = "high")]
remap!(Spi3NoRemap, SPI3, false, PB3, PB4, PB5);
#[cfg(feature = "stm32f105")]
remap!(Spi3Remap, SPI3, true, PC10, PC11, PC12);

impl<REMAP, PINS> Spi<SPI1, REMAP, PINS> {
    pub fn spi1<F, POS>(
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
        REMAP: Remap<Periph = SPI1>,
        PINS: Pins<REMAP, POS>,
    {
        mapr.modify_mapr(|_, w| w.spi1_remap().bit(REMAP::REMAP));
        Spi::<SPI1, _, _>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

impl<REMAP, PINS> Spi<SPI2, REMAP, PINS> {
    pub fn spi2<F, POS>(
        spi: SPI2,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut <SPI2 as RccBus>::Bus,
    ) -> Self
    where
        F: Into<Hertz>,
        REMAP: Remap<Periph = SPI2>,
        PINS: Pins<REMAP, POS>,
    {
        Spi::<SPI2, _, _>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

#[cfg(feature = "high")]
impl<REMAP, PINS> Spi<SPI3, REMAP, PINS> {
    pub fn spi3<F, POS>(
        spi: SPI3,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut <SPI3 as RccBus>::Bus,
    ) -> Self
    where
        F: Into<Hertz>,
        REMAP: Remap<Periph = SPI3>,
        PINS: Pins<REMAP, POS>,
    {
        Spi::<SPI3, _, _>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

type SpiRegisterBlock = crate::pac::spi1::RegisterBlock;

impl<SPI, REMAP, PINS> Spi<SPI, REMAP, PINS>
where
    SPI: Deref<Target = SpiRegisterBlock> + Enable + Reset,
    SPI::Bus: GetBusFreq,
{
    fn _spi(
        spi: SPI,
        pins: PINS,
        mode: Mode,
        freq: Hertz,
        clocks: Clocks,
        apb: &mut SPI::Bus,
    ) -> Self {
        // enable or reset SPI
        SPI::enable(apb);
        SPI::reset(apb);

        // disable SS output
        spi.cr2.write(|w| w.ssoe().clear_bit());

        let br = match SPI::Bus::get_frequency(&clocks).0 / freq.0 {
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

        spi.cr1.write(|w| {
            w
                // clock phase from config
                .cpha()
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                // clock polarity from config
                .cpol()
                .bit(mode.polarity == Polarity::IdleHigh)
                // mstr: master configuration
                .mstr()
                .set_bit()
                // baudrate value
                .br()
                .bits(br)
                // lsbfirst: MSB first
                .lsbfirst()
                .clear_bit()
                // ssm: enable software slave management (NSS pin free for other uses)
                .ssm()
                .set_bit()
                // ssi: set nss high = master mode
                .ssi()
                .set_bit()
                // dff: 8 bit frames
                .dff()
                .clear_bit()
                // bidimode: 2-line unidirectional
                .bidimode()
                .clear_bit()
                // both TX and RX are used
                .rxonly()
                .clear_bit()
                // spe: enable the SPI bus
                .spe()
                .set_bit()
        });

        Spi {
            spi,
            pins,
            _remap: PhantomData,
        }
    }

    pub fn free(self) -> (SPI, PINS) {
        (self.spi, self.pins)
    }
}

impl<SPI, REMAP, PINS> crate::hal::spi::FullDuplex<u8> for Spi<SPI, REMAP, PINS>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
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
            return Ok(unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u8) });
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

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<SPI, REMAP, PINS> where
    SPI: Deref<Target = SpiRegisterBlock>
{
}

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::Write<u8> for Spi<SPI, REMAP, PINS> where
    SPI: Deref<Target = SpiRegisterBlock>
{
     type Error = Error;

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        // Write each word when the tx buffer is empty
        for word in words {
            loop {
                let sr = self.spi.sr.read();
                if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *word) }
                    if sr.modf().bit_is_set() {
                        return Err(Error::ModeFault);
                    }
                    break;
                }
            }
        }
        // Wait for final TXE
        loop {
            let sr = self.spi.sr.read();
            if sr.txe().bit_is_set() {
              break;
            }
        }
        // Wait for final !BSY
        loop {
            let sr = self.spi.sr.read();
            if !sr.bsy().bit_is_set() {
              break;
            }
        }
        // Clear OVR set due to dropped received values
        // NOTE(read_volatile) see note aboev
        unsafe {
           let _ = ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
        }
        let _ = self.spi.sr.read();
        Ok(())
    }
}

// DMA

pub struct SpiPayload<SPI, REMAP, PINS> {
    spi: Spi<SPI, REMAP, PINS>,
}

pub type SpiTxDma<SPI, REMAP, PINS, CHANNEL> = TxDma<SpiPayload<SPI, REMAP, PINS>, CHANNEL>;

macro_rules! spi_dma {
    ($SPIi:ident, $TCi:ident) => {
        impl<REMAP, PINS> Transmit for SpiTxDma<$SPIi, REMAP, PINS, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<REMAP, PINS> Spi<$SPIi, REMAP, PINS> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiTxDma<$SPIi, REMAP, PINS, $TCi> {
                let payload = SpiPayload { spi: self };
                SpiTxDma { payload, channel }
            }
        }

        impl<REMAP, PINS> TransferPayload for SpiTxDma<$SPIi, REMAP, PINS, $TCi> {
            fn start(&mut self) {
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().set_bit());
                self.channel.start();
            }
            fn stop(&mut self) {
                self.payload
                    .spi
                    .spi
                    .cr2
                    .modify(|_, w| w.txdmaen().clear_bit());
                self.channel.stop();
            }
        }

        impl<A, B, REMAP, PIN> crate::dma::WriteDma<A, B, u8> for SpiTxDma<$SPIi, REMAP, PIN, $TCi>
        where
            A: AsSlice<Element = u8>,
            B: Static<A>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                {
                    let buffer = buffer.borrow().as_slice();
                    self.channel.set_peripheral_address(
                        unsafe { &(*$SPIi::ptr()).dr as *const _ as u32 },
                        false,
                    );
                    self.channel
                        .set_memory_address(buffer.as_ptr() as u32, true);
                    self.channel.set_transfer_length(buffer.len());
                }
                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr.modify(|_, w| {
                    w
                        // memory to memory mode disabled
                        .mem2mem()
                        .clear_bit()
                        // medium channel priority level
                        .pl()
                        .medium()
                        // 8-bit memory size
                        .msize()
                        .bits8()
                        // 8-bit peripheral size
                        .psize()
                        .bits8()
                        // circular mode disabled
                        .circ()
                        .clear_bit()
                        // read from memory
                        .dir()
                        .set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }
    };
}

spi_dma!(SPI1, C3);
spi_dma!(SPI2, C5);
