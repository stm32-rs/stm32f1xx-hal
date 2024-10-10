/*!
  # Serial Peripheral Interface
  To construct the SPI instances, use the `Spi::spiX` functions.

  The pin parameter is a tuple containing `(sck, miso, mosi)` which should be configured as `(Alternate<...>, Input<...>, Alternate<...>)`.
  As some STM32F1xx chips have 5V tolerant SPI pins, it is also possible to configure Sck and Mosi outputs as `Alternate<PushPull>`. Then
  a simple Pull-Up to 5V can be used to use SPI on a 5V bus without a level shifter.

  You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins

  - `SPI1` can use `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)`.
  - `SPI2` can use `(PB13, PB14, PB15)`
  - `SPI3` can use `(PB3, PB4, PB5)` or only in connectivity line devices `(PC10, PC11, PC12)`


  ## Initialisation example

  ```rust
    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split();

    let pins = (
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 100.khz(), clocks);
  ```
*/

mod hal_02;
mod hal_1;

use core::ops::{Deref, DerefMut};
use core::ptr;

use crate::pac::{self, RCC};

use crate::afio::{Remap, MAPR};
use crate::dma::dma1;
#[cfg(feature = "connectivity")]
use crate::dma::dma2;
use crate::dma::{Receive, RxDma, RxTxDma, Transfer, TransferPayload, Transmit, TxDma, R, W};
use crate::gpio::{self, Alternate, Cr, Input, NoPin, PinMode, PushPull};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::Hertz;

use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};

/// Clock polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    /// Clock signal low when idle
    IdleLow,
    /// Clock signal high when idle
    IdleHigh,
}

/// Clock phase
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Data in "captured" on the first clock transition
    CaptureOnFirstTransition,
    /// Data in "captured" on the second clock transition
    CaptureOnSecondTransition,
}

/// SPI mode
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    /// Clock polarity
    pub polarity: Polarity,
    /// Clock phase
    pub phase: Phase,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// an error condition occurs(Crcerr, Overrun, ModeFault)
    Error,
}

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

pub struct MasterPins<SCK, MISO, MOSI> {
    pub sck: SCK,
    pub mi: MISO,
    pub mo: MOSI,
}
pub struct SlavePins<SCK, MISO, MOSI> {
    pub sck: SCK,
    pub so: MISO,
    pub si: MOSI,
}

pub mod spi1 {
    use super::*;

    remap! {
        pac::SPI1: [
            PA5, PA6, PA7 => MAPR: 0;
            PB3, PB4, PB5  => MAPR: 1;
        ]
    }
}

pub mod spi2 {
    use super::*;

    remap! {
        pac::SPI2: [
            PB13, PB14, PB15;
        ]
    }
}
#[cfg(any(feature = "high", feature = "connectivity"))]
pub mod spi3 {
    use super::*;

    #[cfg(not(feature = "connectivity"))]
    remap! {
        pac::SPI3: [
            PB3, PB4, PB5;
        ]
    }
    #[cfg(feature = "connectivity")]
    remap! {
        pac::SPI3: [
            PB3, PB4, PB5 => MAPR: 0;
            PC10, PC11, PC12  => MAPR: 1;
        ]
    }
}

macro_rules! remap {
    ($PER:ty: [
        $($SCK:ident, $MISO:ident, $MOSI:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        pub enum Sck {
            $(
                $SCK(gpio::$SCK<Alternate>),
            )+
            None(NoPin<PushPull>),
        }

        pub enum Mi {
            $(
                $MISO(gpio::$MISO<Input>),
            )+
            None(NoPin<PushPull>),
        }

        pub enum So<Otype> {
            $(
                $MISO(gpio::$MISO<Alternate<Otype>>),
            )+
            None(NoPin<PushPull>),
        }

        pub enum Mo {
            $(
                $MOSI(gpio::$MOSI<Alternate>),
            )+
            None(NoPin<PushPull>),
        }

        pub enum Si {
            $(
                $MOSI(gpio::$MOSI<Input>),
            )+
            None(NoPin<PushPull>),
        }

        $(
            // For master mode
            impl From<(gpio::$SCK<Alternate>, gpio::$MISO, gpio::$MOSI<Alternate> $(, &mut $MAPR)?)> for MasterPins<Sck, Mi, Mo> {
                fn from(p: (gpio::$SCK<Alternate>, gpio::$MISO, gpio::$MOSI<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(<$PER>::remap(p.3, $remap);)?
                    Self { sck: Sck::$SCK(p.0), mi: Mi::$MISO(p.1), mo: Mo::$MOSI(p.2) }
                }
            }

            impl From<(gpio::$SCK, gpio::$MISO, gpio::$MOSI $(, &mut $MAPR)?)> for MasterPins<Sck, Mi, Mo> {
                fn from(p: (gpio::$SCK, gpio::$MISO, gpio::$MOSI $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let miso = p.1;
                    let mosi = p.2.into_mode(&mut cr);
                    $(<$PER>::remap(p.3, $remap);)?
                    Self { sck: Sck::$SCK(sck), mi: Mi::$MISO(miso), mo: Mo::$MOSI(mosi) }
                }
            }

            impl From<(gpio::$SCK<Alternate>, gpio::$MISO $(, &mut $MAPR)?)> for MasterPins<Sck, Mi, Mo> {
                fn from(p: (gpio::$SCK<Alternate>, gpio::$MISO $(, &mut $MAPR)?)) -> Self {
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { sck: Sck::$SCK(p.0), mi: Mi::$MISO(p.1), mo: Mo::None(NoPin::new()) }
                }
            }

            impl From<(gpio::$SCK, gpio::$MISO $(, &mut $MAPR)?)> for super::MasterPins<Sck, Mi, Mo> {
                fn from(p: (gpio::$SCK, gpio::$MISO $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let miso = p.1;
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { sck: Sck::$SCK(sck), mi: Mi::$MISO(miso), mo: Mo::None(NoPin::new()) }
                }
            }

            impl From<(gpio::$SCK, gpio::$MOSI $(, &mut $MAPR)?)> for super::MasterPins<Sck, Mi, Mo> {
                fn from(p: (gpio::$SCK, gpio::$MOSI $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let mosi = p.1.into_mode(&mut cr);
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { sck: Sck::$SCK(sck), mi: Mi::None(NoPin::new()), mo: Mo::$MOSI(mosi) }
                }
            }

            // For slave mode

            impl<Otype> From<(gpio::$SCK<Alternate>, gpio::$MISO<Alternate<Otype>>, gpio::$MOSI $(, &mut $MAPR)?)> for SlavePins<Sck, So<Otype>, Si> {
                fn from(p: (gpio::$SCK<Alternate>, gpio::$MISO<Alternate<Otype>>, gpio::$MOSI $(, &mut $MAPR)?)) -> Self {
                    $(<$PER>::remap(p.3, $remap);)?
                    Self { sck: Sck::$SCK(p.0), so: So::$MISO(p.1), si: Si::$MOSI(p.2) }
                }
            }

            impl<Otype> From<(gpio::$SCK, gpio::$MISO, gpio::$MOSI $(, &mut $MAPR)?)> for SlavePins<Sck, So<Otype>, Si>
            where
                Alternate<Otype>: PinMode,
            {
                fn from(p: (gpio::$SCK, gpio::$MISO, gpio::$MOSI $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let miso = p.1.into_mode(&mut cr);
                    let mosi = p.2;
                    $(<$PER>::remap(p.3, $remap);)?
                    Self { sck: Sck::$SCK(sck), so: So::$MISO(miso), si: Si::$MOSI(mosi) }
                }
            }

            impl<Otype> From<(gpio::$SCK, gpio::$MISO $(, &mut $MAPR)?)> for SlavePins<Sck, So<Otype>, Si>
            where
                Alternate<Otype>: PinMode,
            {
                fn from(p: (gpio::$SCK, gpio::$MISO $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let miso = p.1.into_mode(&mut cr);
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { sck: Sck::$SCK(sck), so: So::$MISO(miso), si: Si::None(NoPin::new()) }
                }
            }

            impl From<(gpio::$SCK, gpio::$MOSI $(, &mut $MAPR)?)> for SlavePins<Sck, So<PushPull>, Si> {
                fn from(p: (gpio::$SCK, gpio::$MOSI $(, &mut $MAPR)?)) -> Self {
                    let mut cr = Cr;
                    let sck = p.0.into_mode(&mut cr);
                    let mosi = p.1;
                    $(<$PER>::remap(p.2, $remap);)?
                    Self { sck: Sck::$SCK(sck), so: So::None(NoPin::new()), si: Si::$MOSI(mosi) }
                }
            }
        )+
    }
}
use remap;

pub trait SpiExt: Sized + Instance {
    fn spi(
        self,
        pins: impl Into<MasterPins<Self::Sck, Self::Mi, Self::Mo>>,
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u8>;
    fn spi_u16(
        self,
        pins: impl Into<MasterPins<Self::Sck, Self::Mi, Self::Mo>>,
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u16> {
        Self::spi(self, pins, mode, freq, clocks).frame_size_16bit()
    }
    fn spi_slave(
        self,
        pins: impl Into<SlavePins<Self::Sck, Self::So<PushPull>, Self::Si>>,
        mode: Mode,
    ) -> SpiSlave<Self, u8, PushPull>;
    fn spi_slave_u16(
        self,
        pins: impl Into<SlavePins<Self::Sck, Self::So<PushPull>, Self::Si>>,
        mode: Mode,
    ) -> SpiSlave<Self, u16, PushPull> {
        Self::spi_slave(self, pins, mode).frame_size_16bit()
    }
}

impl<SPI: Instance> SpiExt for SPI {
    fn spi(
        self,
        pins: impl Into<MasterPins<Self::Sck, Self::Mi, Self::Mo>>,
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u8> {
        Spi::new(self, pins, mode, freq, clocks)
    }
    fn spi_slave(
        self,
        pins: impl Into<SlavePins<Self::Sck, Self::So<PushPull>, Self::Si>>,
        mode: Mode,
    ) -> SpiSlave<Self, u8, PushPull> {
        SpiSlave::new(self, pins, mode)
    }
}

pub struct SpiInner<SPI, W> {
    spi: SPI,
    _framesize: PhantomData<W>,
}

impl<SPI, W> SpiInner<SPI, W> {
    fn new(spi: SPI) -> Self {
        Self {
            spi,
            _framesize: PhantomData,
        }
    }
}

/// Spi in Master mode
pub struct Spi<SPI: Instance, W> {
    inner: SpiInner<SPI, W>,
    pins: (SPI::Sck, SPI::Mi, SPI::Mo),
}

/// Spi in Slave mode
pub struct SpiSlave<SPI: Instance, W, Otype = PushPull> {
    inner: SpiInner<SPI, W>,
    pins: (SPI::Sck, SPI::So<Otype>, SPI::Si),
}

impl<SPI: Instance, W> Deref for Spi<SPI, W> {
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, W> DerefMut for Spi<SPI, W> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<SPI: Instance, W, Otype> Deref for SpiSlave<SPI, W, Otype> {
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, W, Otype> DerefMut for SpiSlave<SPI, W, Otype> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

/// The bit format to send the data in
#[derive(Debug, Clone, Copy)]
pub enum SpiBitFormat {
    /// Least significant bit first
    LsbFirst,
    /// Most significant bit first
    MsbFirst,
}

pub trait Instance:
    crate::Sealed + Deref<Target = crate::pac::spi1::RegisterBlock> + Enable + Reset + BusClock
{
    type Sck;
    type Mi;
    type So<Otype>;
    type Mo;
    type Si;
}

impl Instance for pac::SPI1 {
    type Sck = spi1::Sck;
    type Mi = spi1::Mi;
    type So<Otype> = spi1::So<Otype>;
    type Mo = spi1::Mo;
    type Si = spi1::Si;
}
impl Instance for pac::SPI2 {
    type Sck = spi2::Sck;
    type Mi = spi2::Mi;
    type So<Otype> = spi2::So<Otype>;
    type Mo = spi2::Mo;
    type Si = spi2::Si;
}
#[cfg(any(feature = "high", feature = "connectivity"))]
impl Instance for pac::SPI3 {
    type Sck = spi3::Sck;
    type Mi = spi3::Mi;
    type So<Otype> = spi3::So<Otype>;
    type Mo = spi3::Mo;
    type Si = spi3::Si;
}

impl<SPI: Instance> Spi<SPI, u8> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Alternate<PushPull>, Input<...>, Alternate<PushPull>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn new(
        spi: SPI,
        pins: impl Into<MasterPins<SPI::Sck, SPI::Mi, SPI::Mo>>,
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Self {
        // enable or reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2().write(|w| w.ssoe().clear_bit());

        let br = match SPI::clock(clocks) / freq {
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

        let pins = pins.into();

        spi.cr1().write(|w| {
            // clock phase from config
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
            // clock polarity from config
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            // mstr: master configuration
            w.mstr().set_bit();
            // baudrate value
            w.br().set(br);
            // lsbfirst: MSB first
            w.lsbfirst().clear_bit();
            // ssm: enable software slave management (NSS pin free for other uses)
            w.ssm().set_bit();
            // ssi: set nss high = master mode
            w.ssi().set_bit();
            // dff: 8 bit frames
            w.dff().clear_bit();
            // bidimode: 2-line unidirectional
            w.bidimode().clear_bit();
            // both TX and RX are used
            w.rxonly().clear_bit();
            // spe: enable the SPI bus
            w.spe().set_bit()
        });

        Spi {
            inner: SpiInner::new(spi),
            pins: (pins.sck, pins.mi, pins.mo),
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (SPI, (SPI::Sck, SPI::Mi, SPI::Mo)) {
        (self.inner.spi, self.pins)
    }
}

impl<SPI: Instance, Otype> SpiSlave<SPI, u8, Otype> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Input<Floating>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn new(
        spi: SPI,
        pins: impl Into<SlavePins<SPI::Sck, SPI::So<Otype>, SPI::Si>>,
        mode: Mode,
    ) -> Self {
        // enable or reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2().write(|w| w.ssoe().clear_bit());

        let pins = pins.into();

        spi.cr1().write(|w| {
            // clock phase from config
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
            // clock polarity from config
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            // mstr: slave configuration
            w.mstr().clear_bit();
            // lsbfirst: MSB first
            w.lsbfirst().clear_bit();
            // ssm: enable software slave management (NSS pin free for other uses)
            w.ssm().set_bit();
            // ssi: set nss low = slave mode
            w.ssi().clear_bit();
            // dff: 8 bit frames
            w.dff().clear_bit();
            // bidimode: 2-line unidirectional
            w.bidimode().clear_bit();
            // both TX and RX are used
            w.rxonly().clear_bit();
            // spe: enable the SPI bus
            w.spe().set_bit()
        });

        SpiSlave {
            inner: SpiInner::new(spi),
            pins: (pins.sck, pins.so, pins.si),
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (SPI, (SPI::Sck, SPI::So<Otype>, SPI::Si)) {
        (self.inner.spi, self.pins)
    }
}

pub trait SpiReadWrite<T> {
    fn read_data_reg(&mut self) -> T;
    fn write_data_reg(&mut self, data: T);
    fn spi_write(&mut self, words: &[T]) -> Result<(), Error>;
}

impl<SPI: Instance, W: Copy> SpiReadWrite<W> for SpiInner<SPI, W> {
    fn read_data_reg(&mut self) -> W {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(self.spi.dr().as_ptr() as *const W) }
    }

    fn write_data_reg(&mut self, data: W) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(self.spi.dr().as_ptr() as *mut W, data) }
    }

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn spi_write(&mut self, words: &[W]) -> Result<(), Error> {
        // Write each word when the tx buffer is empty
        for word in words {
            loop {
                let sr = self.spi.sr().read();
                if sr.txe().bit_is_set() {
                    self.write_data_reg(*word);
                    if sr.modf().bit_is_set() {
                        return Err(Error::ModeFault);
                    }
                    break;
                }
            }
        }
        // Wait for final TXE
        while !self.is_tx_empty() {}
        // Wait for final !BSY
        while self.is_busy() {}
        // Clear OVR set due to dropped received values
        let _ = self.read_data_reg();
        let _ = self.spi.sr().read();
        Ok(())
    }
}

impl<SPI: Instance, W: Copy> SpiInner<SPI, W> {
    /// Select which frame format is used for data transfers
    pub fn bit_format(&mut self, format: SpiBitFormat) {
        self.spi
            .cr1()
            .modify(|_, w| w.lsbfirst().bit(matches!(format, SpiBitFormat::LsbFirst)));
    }

    /// Starts listening to the SPI by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        self.spi.cr2().modify(|_, w| match event {
            Event::Rxne => w.rxneie().set_bit(),
            Event::Txe => w.txeie().set_bit(),
            Event::Error => w.errie().set_bit(),
        });
    }

    /// Stops listening to the SPI by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        self.spi.cr2().modify(|_, w| match event {
            Event::Rxne => w.rxneie().clear_bit(),
            Event::Txe => w.txeie().clear_bit(),
            Event::Error => w.errie().clear_bit(),
        });
    }

    /// Returns true if the tx register is empty (and can accept data)
    #[inline]
    pub fn is_tx_empty(&self) -> bool {
        self.spi.sr().read().txe().bit_is_set()
    }

    /// Returns true if the rx register is not empty (and can be read)
    #[inline]
    pub fn is_rx_not_empty(&self) -> bool {
        self.spi.sr().read().rxne().bit_is_set()
    }

    /// Returns true if the transfer is in progress
    #[inline]
    pub fn is_busy(&self) -> bool {
        self.spi.sr().read().bsy().bit_is_set()
    }

    /// Returns true if data are received and the previous data have not yet been read from SPI_DR.
    #[inline]
    pub fn is_overrun(&self) -> bool {
        self.spi.sr().read().ovr().bit_is_set()
    }
}

impl<SPI: Instance> Spi<SPI, u8> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> Spi<SPI, u16> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().set_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, Otype> SpiSlave<SPI, u8, Otype> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> SpiSlave<SPI, u16, Otype> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().set_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        SpiSlave {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance> Spi<SPI, u16> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> Spi<SPI, u16> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().clear_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, Otype> SpiSlave<SPI, u16, Otype> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> SpiSlave<SPI, u8, Otype> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().clear_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        SpiSlave {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI, W> SpiInner<SPI, W>
where
    SPI: Instance,
    W: Copy,
{
    pub fn read_nonblocking(&mut self) -> nb::Result<W, Error> {
        let sr = self.spi.sr().read();

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
    pub fn write_nonblocking(&mut self, data: W) -> nb::Result<(), Error> {
        let sr = self.spi.sr().read();

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
    pub fn write(&mut self, words: &[W]) -> Result<(), Error> {
        self.spi_write(words)
    }
}

// DMA

pub type SpiTxDma<SPI, CHANNEL> = TxDma<Spi<SPI, u8>, CHANNEL>;
pub type SpiRxDma<SPI, CHANNEL> = RxDma<Spi<SPI, u8>, CHANNEL>;
pub type SpiRxTxDma<SPI, RXCHANNEL, TXCHANNEL> = RxTxDma<Spi<SPI, u8>, RXCHANNEL, TXCHANNEL>;

pub type SpiSlaveTxDma<SPI, CHANNEL, Otype> = TxDma<SpiSlave<SPI, u8, Otype>, CHANNEL>;
pub type SpiSlaveRxDma<SPI, CHANNEL, Otype> = RxDma<SpiSlave<SPI, u8, Otype>, CHANNEL>;
pub type SpiSlaveRxTxDma<SPI, RXCHANNEL, TXCHANNEL, Otype> =
    RxTxDma<SpiSlave<SPI, u8, Otype>, RXCHANNEL, TXCHANNEL>;

macro_rules! spi_dma {
    (
        $SPIi:ty,
        rx: $RCi:ty,
        tx: $TCi:ty,
        $rxdma:ident,
        $txdma:ident,
        $rxtxdma:ident,
        $slaverxdma:ident,
        $slavetxdma:ident,
        $slaverxtxdma:ident
    ) => {
        pub type $rxdma = SpiRxDma<$SPIi, $RCi>;
        pub type $txdma = SpiTxDma<$SPIi, $TCi>;
        pub type $rxtxdma = SpiRxTxDma<$SPIi, $RCi, $TCi>;

        impl Transmit for SpiTxDma<$SPIi, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl Receive for SpiRxDma<$SPIi, $RCi> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl Transmit for SpiRxTxDma<$SPIi, $RCi, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl Receive for SpiRxTxDma<$SPIi, $RCi, $TCi> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl Spi<$SPIi, u8> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiTxDma<$SPIi, $TCi> {
                self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
                SpiTxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_dma(self, channel: $RCi) -> SpiRxDma<$SPIi, $RCi> {
                self.spi.cr2().modify(|_, w| w.rxdmaen().set_bit());
                SpiRxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_tx_dma(
                self,
                rxchannel: $RCi,
                txchannel: $TCi,
            ) -> SpiRxTxDma<$SPIi, $RCi, $TCi> {
                self.spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().set_bit().txdmaen().set_bit());
                SpiRxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }

        impl SpiTxDma<$SPIi, $TCi> {
            pub fn release(self) -> (Spi<$SPIi, u8>, $TCi) {
                let SpiTxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.txdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl SpiRxDma<$SPIi, $RCi> {
            pub fn release(self) -> (Spi<$SPIi, u8>, $RCi) {
                let SpiRxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.rxdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl SpiRxTxDma<$SPIi, $RCi, $TCi> {
            pub fn release(self) -> (Spi<$SPIi, u8>, $RCi, $TCi) {
                let SpiRxTxDma {
                    payload,
                    rxchannel,
                    txchannel,
                } = self;
                payload
                    .spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit());
                (payload, rxchannel, txchannel)
            }
        }

        impl TransferPayload for SpiTxDma<$SPIi, $TCi> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl TransferPayload for SpiRxDma<$SPIi, $RCi> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl TransferPayload for SpiRxTxDma<$SPIi, $RCi, $TCi> {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<B> crate::dma::ReadDma<B, u8> for SpiRxDma<$SPIi, $RCi>
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }

        impl<B> crate::dma::WriteDma<B, u8> for SpiTxDma<$SPIi, $TCi>
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.read_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }

        impl<RXB, TXB> crate::dma::ReadWriteDma<RXB, TXB, u8> for SpiRxTxDma<$SPIi, $RCi, $TCi>
        where
            RXB: WriteBuffer<Word = u8>,
            TXB: ReadBuffer<Word = u8>,
        {
            fn read_write(
                mut self,
                mut rxbuffer: RXB,
                txbuffer: TXB,
            ) -> Transfer<W, (RXB, TXB), Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (rxptr, rxlen) = unsafe { rxbuffer.write_buffer() };
                let (txptr, txlen) = unsafe { txbuffer.read_buffer() };

                if rxlen != txlen {
                    panic!("receive and send buffer lengths do not match!");
                }

                self.rxchannel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.rxchannel.set_memory_address(rxptr as u32, true);
                self.rxchannel.set_transfer_length(rxlen);

                self.txchannel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.txchannel.set_memory_address(txptr as u32, true);
                self.txchannel.set_transfer_length(txlen);

                atomic::compiler_fence(Ordering::Release);
                self.rxchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.txchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::w((rxbuffer, txbuffer), self)
            }
        }

        pub type $slaverxdma<Otype = PushPull> = SpiSlaveRxDma<$SPIi, $RCi, Otype>;
        pub type $slavetxdma<Otype = PushPull> = SpiSlaveTxDma<$SPIi, $TCi, Otype>;
        pub type $slaverxtxdma<Otype = PushPull> = SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype>;

        impl<Otype> Transmit for SpiSlaveTxDma<$SPIi, $TCi, Otype> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<Otype> Receive for SpiSlaveRxDma<$SPIi, $RCi, Otype> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<Otype> Transmit for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<Otype> Receive for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<Otype> SpiSlave<$SPIi, u8, Otype> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiSlaveTxDma<$SPIi, $TCi, Otype> {
                self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
                SpiSlaveTxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_dma(self, channel: $RCi) -> SpiSlaveRxDma<$SPIi, $RCi, Otype> {
                self.spi.cr2().modify(|_, w| w.rxdmaen().set_bit());
                SpiSlaveRxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_tx_dma(
                self,
                rxchannel: $RCi,
                txchannel: $TCi,
            ) -> SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype> {
                self.spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().set_bit().txdmaen().set_bit());
                SpiSlaveRxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }

        impl<Otype> SpiSlaveTxDma<$SPIi, $TCi, Otype> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype>, $TCi) {
                let SpiSlaveTxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.txdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<Otype> SpiSlaveRxDma<$SPIi, $RCi, Otype> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype>, $RCi) {
                let SpiSlaveRxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.rxdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<Otype> SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype>, $RCi, $TCi) {
                let SpiSlaveRxTxDma {
                    payload,
                    rxchannel,
                    txchannel,
                } = self;
                payload
                    .spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit());
                (payload, rxchannel, txchannel)
            }
        }

        impl<Otype> TransferPayload for SpiSlaveTxDma<$SPIi, $TCi, Otype> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<Otype> TransferPayload for SpiSlaveRxDma<$SPIi, $RCi, Otype> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<Otype> TransferPayload for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype> {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<B, Otype> crate::dma::ReadDma<B, u8> for SpiSlaveRxDma<$SPIi, $RCi, Otype>
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }

        impl<B, Otype> crate::dma::WriteDma<B, u8> for SpiSlaveTxDma<$SPIi, $TCi, Otype>
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.read_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }

        impl<RXB, TXB, Otype> crate::dma::ReadWriteDma<RXB, TXB, u8>
            for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype>
        where
            RXB: WriteBuffer<Word = u8>,
            TXB: ReadBuffer<Word = u8>,
        {
            fn read_write(
                mut self,
                mut rxbuffer: RXB,
                txbuffer: TXB,
            ) -> Transfer<W, (RXB, TXB), Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (rxptr, rxlen) = unsafe { rxbuffer.write_buffer() };
                let (txptr, txlen) = unsafe { txbuffer.read_buffer() };

                if rxlen != txlen {
                    panic!("receive and send buffer lengths do not match!");
                }

                self.rxchannel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.rxchannel.set_memory_address(rxptr as u32, true);
                self.rxchannel.set_transfer_length(rxlen);

                self.txchannel.set_peripheral_address(
                    unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.txchannel.set_memory_address(txptr as u32, true);
                self.txchannel.set_transfer_length(txlen);

                atomic::compiler_fence(Ordering::Release);
                self.rxchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.txchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::w((rxbuffer, txbuffer), self)
            }
        }
    };
}

spi_dma!(
    pac::SPI1,
    rx: dma1::C2,
    tx: dma1::C3,
    Spi1RxDma,
    Spi1TxDma,
    Spi1RxTxDma,
    SpiSlave1RxDma,
    SpiSlave1TxDma,
    SpiSlave1RxTxDma
);
spi_dma!(
    pac::SPI2,
    rx: dma1::C4,
    tx: dma1::C5,
    Spi2RxDma,
    Spi2TxDma,
    Spi2RxTxDma,
    SpiSlave2RxDma,
    SpiSlave2TxDma,
    SpiSlave2RxTxDma
);
#[cfg(feature = "connectivity")]
spi_dma!(
    pac::SPI3,
    rx: dma2::C1,
    tx: dma2::C2,
    Spi3RxDma,
    Spi3TxDma,
    Spi3RxTxDma,
    SpiSlave3RxDma,
    SpiSlave3TxDma,
    SpiSlave3RxTxDma
);
