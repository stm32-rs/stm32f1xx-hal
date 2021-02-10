/*!
  # Serial Peripheral Interface
  To construct the SPI instances, use the `Spi::spiX` functions.

  The pin parameter is a tuple containing `(sck, miso, mosi)` which should be configured as `(Alternate<PushPull>, Input<Floating>, Alternate<PushPull>)`.
  As some STM32F1xx chips have 5V tolerant SPI pins, it is also possible to configure Sck and Mosi outputs as `Alternate<PushPull>`. Then
  a simple Pull-Up to 5V can be used to use SPI on a 5V bus without a level shifter.

  You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins

  - `SPI1` can use `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)`.
  - `SPI2` can use `(PB13, PB14, PB15)`
  - `SPI3` can use `(PB3, PB4, PB5)` or `(PC10, PC11, PC12)`


  ## Initialisation example

  ```rust
    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let pins = (
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 100.khz(), clocks, &mut rcc.apb1);
  ```
*/

use core::ops::Deref;
use core::ptr;

pub use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};
#[cfg(any(feature = "high", feature = "connectivity"))]
use crate::pac::SPI3;
use crate::pac::{SPI1, SPI2};

use crate::afio::MAPR;
use crate::dma::dma1::{C3, C5};
#[cfg(feature = "connectivity")]
use crate::dma::dma2::C2;
use crate::dma::{Transfer, TransferPayload, Transmit, TxDma, R};
use crate::gpio::gpioa::{PA5, PA6, PA7};
use crate::gpio::gpiob::{PB13, PB14, PB15, PB3, PB4, PB5};
#[cfg(feature = "connectivity")]
use crate::gpio::gpioc::{PC10, PC11, PC12};
use crate::gpio::{Alternate, Floating, Input, OpenDrain, PushPull};
use crate::rcc::{Clocks, Enable, GetBusFreq, Reset, APB1, APB2};
use crate::time::Hertz;

use core::sync::atomic::{self, Ordering};
use embedded_dma::StaticReadBuffer;

/// SPI error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
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

pub struct Spi<SPI, REMAP, PINS, FRAMESIZE> {
    spi: SPI,
    pins: PINS,
    _remap: PhantomData<REMAP>,
    _framesize: PhantomData<FRAMESIZE>,
}

/// The bit format to send the data in
#[derive(Debug, Clone, Copy)]
pub enum SpiBitFormat {
    /// Least significant bit first
    LsbFirst,
    /// Most significant bit first
    MsbFirst,
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
        impl Sck<$name> for $SCK<Alternate<OpenDrain>> {}
        impl Miso<$name> for $MISO<Input<Floating>> {}
        impl Mosi<$name> for $MOSI<Alternate<PushPull>> {}
        impl Mosi<$name> for $MOSI<Alternate<OpenDrain>> {}
    };
}

remap!(Spi1NoRemap, SPI1, false, PA5, PA6, PA7);
remap!(Spi1Remap, SPI1, true, PB3, PB4, PB5);
remap!(Spi2NoRemap, SPI2, false, PB13, PB14, PB15);
#[cfg(feature = "high")]
remap!(Spi3NoRemap, SPI3, false, PB3, PB4, PB5);
#[cfg(feature = "connectivity")]
remap!(Spi3Remap, SPI3, true, PC10, PC11, PC12);

impl<REMAP, PINS> Spi<SPI1, REMAP, PINS, u8> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Alternate<PushPull>, Input<Floating>, Alternate<PushPull>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi1<F, POS>(
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
        REMAP: Remap<Periph = SPI1>,
        PINS: Pins<REMAP, POS>,
    {
        mapr.modify_mapr(|_, w| w.spi1_remap().bit(REMAP::REMAP));
        Spi::<SPI1, _, _, u8>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

impl<REMAP, PINS> Spi<SPI2, REMAP, PINS, u8> {
    /**
      Constructs an SPI instance using SPI2 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB13, PB14, PB15)` configured as `(Alternate<PushPull>, Input<Floating>, Alternate<PushPull>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi2<F, POS>(
        spi: SPI2,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
    where
        F: Into<Hertz>,
        REMAP: Remap<Periph = SPI2>,
        PINS: Pins<REMAP, POS>,
    {
        Spi::<SPI2, _, _, u8>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

#[cfg(any(feature = "high", feature = "connectivity"))]
impl<REMAP, PINS> Spi<SPI3, REMAP, PINS, u8> {
    /**
      Constructs an SPI instance using SPI3 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB3, PB4, PB5)` or `(PC10, PC11, PC12)` configured as `(Alternate<PushPull>, Input<Floating>, Alternate<PushPull>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi3<F, POS>(
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
        REMAP: Remap<Periph = SPI3>,
        PINS: Pins<REMAP, POS>,
    {
        mapr.modify_mapr(|_, w| w.spi3_remap().bit(REMAP::REMAP));
        Spi::<SPI3, _, _, u8>::_spi(spi, pins, mode, freq.into(), clocks, apb)
    }
}

pub type SpiRegisterBlock = crate::pac::spi1::RegisterBlock;

pub trait SpiReadWrite<T> {
    fn read_data_reg(&mut self) -> T;
    fn write_data_reg(&mut self, data: T);
    fn spi_write(&mut self, words: &[T]) -> Result<(), Error>;
    fn spi_write_iter(&mut self, words: impl Iterator<Item = T>) -> Result<(), Error>;
}

impl<SPI, REMAP, PINS, FrameSize> SpiReadWrite<FrameSize> for Spi<SPI, REMAP, PINS, FrameSize>
where
    SPI: Deref<Target = SpiRegisterBlock>,
    FrameSize: Copy,
{
    fn read_data_reg(&mut self) -> FrameSize {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const FrameSize) }
    }

    fn write_data_reg(&mut self, data: FrameSize) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut FrameSize, data) }
    }

    fn spi_write(&mut self, words: &[FrameSize]) -> Result<(), Error> {
        self.spi_write_iter(words.iter().cloned())
    }

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn spi_write_iter(&mut self, words: impl Iterator<Item = FrameSize>) -> Result<(), Error> {
        // Write each word when the tx buffer is empty
        for word in words {
            loop {
                let sr = self.spi.sr.read();
                if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    // unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *word) }
                    self.write_data_reg(word);
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
        // NOTE(read_volatile) see note above
        // unsafe {
        //     let _ = ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
        // }
        let _ = self.read_data_reg();
        let _ = self.spi.sr.read();
        Ok(())
    }
}

impl<SPI, REMAP, PINS, FrameSize> Spi<SPI, REMAP, PINS, FrameSize>
where
    SPI: Deref<Target = SpiRegisterBlock>,
    FrameSize: Copy,
{
    #[deprecated(since = "0.6.0", note = "Please use release instead")]
    pub fn free(self) -> (SPI, PINS) {
        self.release()
    }
    pub fn release(self) -> (SPI, PINS) {
        (self.spi, self.pins)
    }

    /// Select which frame format is used for data transfers
    pub fn bit_format(&mut self, format: SpiBitFormat) {
        match format {
            SpiBitFormat::LsbFirst => self.spi.cr1.modify(|_, w| w.lsbfirst().set_bit()),
            SpiBitFormat::MsbFirst => self.spi.cr1.modify(|_, w| w.lsbfirst().clear_bit()),
        }
    }
}

impl<SPI, REMAP, PINS> Spi<SPI, REMAP, PINS, u8>
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
            _framesize: PhantomData,
        }
    }
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> Spi<SPI, REMAP, PINS, u16> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr1.modify(|_, w| w.dff().set_bit());
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        Spi {
            spi: self.spi,
            pins: self.pins,
            _remap: PhantomData,
            _framesize: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS> Spi<SPI, REMAP, PINS, u16>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> Spi<SPI, REMAP, PINS, u8> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr1.modify(|_, w| w.dff().clear_bit());
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        Spi {
            spi: self.spi,
            pins: self.pins,
            _remap: PhantomData,
            _framesize: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS, FrameSize> crate::hal::spi::FullDuplex<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize>
where
    SPI: Deref<Target = SpiRegisterBlock>,
    FrameSize: Copy,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<FrameSize, Error> {
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
            return Ok(self.read_data_reg());
        } else {
            nb::Error::WouldBlock
        })
    }

    fn send(&mut self, data: FrameSize) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            nb::Error::Other(Error::Crc)
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            self.write_data_reg(data);
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<SPI, REMAP, PINS, FrameSize> crate::hal::blocking::spi::transfer::Default<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize>
where
    SPI: Deref<Target = SpiRegisterBlock>,
    FrameSize: Copy,
{
}

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::Write<u8> for Spi<SPI, REMAP, PINS, u8>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.spi_write(words)
    }
}

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::Write<u16> for Spi<SPI, REMAP, PINS, u16>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Error> {
        self.spi_write(words)
    }
}

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::WriteIter<u8> for Spi<SPI, REMAP, PINS, u8>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u8>,
    {
        self.spi_write_iter(words.into_iter())
    }
}

impl<SPI, REMAP, PINS> crate::hal::blocking::spi::WriteIter<u16> for Spi<SPI, REMAP, PINS, u16>
where
    SPI: Deref<Target = SpiRegisterBlock>,
{
    type Error = Error;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u16>,
    {
        self.spi_write_iter(words.into_iter())
    }
}

// DMA

pub struct SpiPayload<SPI, REMAP, PINS> {
    spi: Spi<SPI, REMAP, PINS, u8>,
}

pub type SpiTxDma<SPI, REMAP, PINS, CHANNEL> = TxDma<SpiPayload<SPI, REMAP, PINS>, CHANNEL>;

macro_rules! spi_dma {
    ($SPIi:ident, $TCi:ident) => {
        impl<REMAP, PINS> Transmit for SpiTxDma<$SPIi, REMAP, PINS, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<REMAP, PINS> Spi<$SPIi, REMAP, PINS, u8> {
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

        impl<B, REMAP, PIN> crate::dma::WriteDma<B, u8> for SpiTxDma<$SPIi, REMAP, PIN, $TCi>
        where
            B: StaticReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_read_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*$SPIi::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

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
#[cfg(feature = "connectivity")]
spi_dma!(SPI3, C2);
