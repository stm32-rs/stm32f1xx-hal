/*!
  # Serial Peripheral Interface
  To construct the SPI instances, use the `Spi::spiX` functions.

  The pin parameter is a tuple containing `(Some(sck), Some(miso), Some(mosi))` which should be configured as `(Alternate<...>, Input<...>, Alternate<...>)`.
  As some STM32F1xx chips have 5V tolerant SPI pins, it is also possible to configure Sck and Mosi outputs as `Alternate<PushPull>`. Then
  a simple Pull-Up to 5V can be used to use SPI on a 5V bus without a level shifter.

  You can also use `None::<PA6>` if you don't want to use the pins

  ## Alternate function remapping

  ## SPI1

  | Function \ Remap        | 0 (default) | 1    |
  |-------------------------|-------------|------|
  | SCK (A-PP : I-F)        | PA5         | PB3  |
  | MISO (I-F/PU : A-PP/OD) | PA6         | PB4  |
  | MOSI (A-PP : I-F/PU)    | PA7         | PB5  |

  ## SPI2

  | Function                |      |
  |-------------------------|------|
  | SCK (A-PP : I-F)        | PB13 |
  | MISO (I-F/PU : A-PP/OD) | PB14 |
  | MOSI (A-PP : I-F/PU)    | PB15 |

  ## SPI3

  | Function \ Remap        | 0 (default) | 1 (conn. devices) |
  |-------------------------|-------------|-------------------|
  | SCK (A-PP : I-F)        | PB3         | PC10              |
  | MISO (I-F/PU : A-PP/OD) | PB4         | PC11              |
  | MOSI (A-PP : I-F/PU)    | PB5         | PC12              |

  ## Initialisation example

  ```rust
    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split();

    let pins = (
        Some(gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh)),
        Some(gpiob.pb14.into_floating_input(&mut gpiob.crh)),
        Some(gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh)),
    );

    // or just

    let pins = (
        Some(gpiob.pb13),
        Some(gpiob.pb14),
        Some(gpiob.pb15),
    );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = dp.SPI2.spi(pins, spi_mode, 100.khz(), &clocks);
  ```
*/

mod hal_02;
mod hal_1;

use core::ops::{Deref, DerefMut};
use core::ptr;

use crate::pac::{self, RCC};

use crate::afio::{self, RInto, Rmp};
use crate::dma::dma1;
#[cfg(feature = "connectivity")]
use crate::dma::dma2;
use crate::dma::{self, Receive, RxDma, RxTxDma, Transfer, TransferPayload, Transmit, TxDma};
use crate::gpio::{Floating, PushPull, UpMode};
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

pub trait SpiExt: Sized + Instance {
    fn spi<PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::MSck, 0>>,
            Option<impl RInto<Self::Mi<PULL>, 0>>,
            Option<impl RInto<Self::Mo, 0>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u8, PULL>;
    fn spi_u16<PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::MSck, 0>>,
            Option<impl RInto<Self::Mi<PULL>, 0>>,
            Option<impl RInto<Self::Mo, 0>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u16, PULL> {
        Self::spi(self, pins, mode, freq, clocks).frame_size_16bit()
    }
    fn spi_slave<Otype, PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::SSck, 0>>,
            Option<impl RInto<Self::So<Otype>, 0>>,
            Option<impl RInto<Self::Si<PULL>, 0>>,
        ),
        mode: Mode,
    ) -> SpiSlave<Self, u8, Otype, PULL>;
    fn spi_slave_u16<Otype, PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::SSck, 0>>,
            Option<impl RInto<Self::So<Otype>, 0>>,
            Option<impl RInto<Self::Si<PULL>, 0>>,
        ),
        mode: Mode,
    ) -> SpiSlave<Self, u16, Otype, PULL> {
        Self::spi_slave(self, pins, mode).frame_size_16bit()
    }
}

impl<SPI: Instance> SpiExt for SPI {
    fn spi<PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::MSck, 0>>,
            Option<impl RInto<Self::Mi<PULL>, 0>>,
            Option<impl RInto<Self::Mo, 0>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, u8, PULL> {
        Spi::new(self, pins, mode, freq, clocks)
    }
    fn spi_slave<Otype, PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<Self::SSck, 0>>,
            Option<impl RInto<Self::So<Otype>, 0>>,
            Option<impl RInto<Self::Si<PULL>, 0>>,
        ),
        mode: Mode,
    ) -> SpiSlave<Self, u8, Otype, PULL> {
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
pub struct Spi<SPI: Instance, W, PULL = Floating> {
    inner: SpiInner<SPI, W>,
    #[allow(clippy::type_complexity)]
    pins: (Option<SPI::MSck>, Option<SPI::Mi<PULL>>, Option<SPI::Mo>),
}

/// Spi in Slave mode
pub struct SpiSlave<SPI: Instance, W, Otype = PushPull, PULL = Floating> {
    inner: SpiInner<SPI, W>,
    #[allow(clippy::type_complexity)]
    pins: (
        Option<SPI::SSck>,
        Option<SPI::So<Otype>>,
        Option<SPI::Si<PULL>>,
    ),
}

impl<SPI: Instance, W, PULL> Deref for Spi<SPI, W, PULL> {
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, W, PULL> DerefMut for Spi<SPI, W, PULL> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<SPI: Instance, W, Otype, PULL> Deref for SpiSlave<SPI, W, Otype, PULL> {
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, W, Otype, PULL> DerefMut for SpiSlave<SPI, W, Otype, PULL> {
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
    crate::Sealed
    + Deref<Target = crate::pac::spi1::RegisterBlock>
    + Enable
    + Reset
    + BusClock
    + afio::SpiCommon
{
}

impl Instance for pac::SPI1 {}
impl Instance for pac::SPI2 {}
#[cfg(any(feature = "high", feature = "connectivity"))]
impl Instance for pac::SPI3 {}

impl<SPI: Instance, const R: u8> Rmp<SPI, R> {
    pub fn spi<PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<SPI::MSck, R>>,
            Option<impl RInto<SPI::Mi<PULL>, R>>,
            Option<impl RInto<SPI::Mo, R>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<SPI, u8, PULL> {
        Spi::_new(self.0, pins, mode, freq, clocks)
    }
    pub fn spi_u16<PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<SPI::MSck, R>>,
            Option<impl RInto<SPI::Mi<PULL>, R>>,
            Option<impl RInto<SPI::Mo, R>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<SPI, u16, PULL> {
        self.spi(pins, mode, freq, clocks).frame_size_16bit()
    }
    pub fn spi_slave<Otype, PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<SPI::SSck, R>>,
            Option<impl RInto<SPI::So<Otype>, R>>,
            Option<impl RInto<SPI::Si<PULL>, R>>,
        ),
        mode: Mode,
    ) -> SpiSlave<SPI, u8, Otype, PULL> {
        SpiSlave::_new(self.0, pins, mode)
    }
    pub fn spi_slave_u16<Otype, PULL: UpMode>(
        self,
        pins: (
            Option<impl RInto<SPI::SSck, R>>,
            Option<impl RInto<SPI::So<Otype>, R>>,
            Option<impl RInto<SPI::Si<PULL>, R>>,
        ),
        mode: Mode,
    ) -> SpiSlave<SPI, u16, Otype, PULL> {
        self.spi_slave(pins, mode).frame_size_16bit()
    }
}

impl<SPI: Instance, PULL: UpMode> Spi<SPI, u8, PULL> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Alternate<PushPull>, Input<...>, Alternate<PushPull>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn new(
        spi: SPI,
        pins: (
            Option<impl RInto<SPI::MSck, 0>>,
            Option<impl RInto<SPI::Mi<PULL>, 0>>,
            Option<impl RInto<SPI::Mo, 0>>,
        ),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Self {
        Self::_new(spi, pins, mode, freq, clocks)
    }

    fn _new<const R: u8>(
        spi: SPI,
        pins: (
            Option<impl RInto<SPI::MSck, R>>,
            Option<impl RInto<SPI::Mi<PULL>, R>>,
            Option<impl RInto<SPI::Mo, R>>,
        ),
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

        let pins = (
            pins.0.map(RInto::rinto),
            pins.1.map(RInto::rinto),
            pins.2.map(RInto::rinto),
        );

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
            pins,
        }
    }
}

impl<SPI: Instance, PULL> Spi<SPI, u8, PULL> {
    #[allow(clippy::type_complexity)]
    pub fn release(
        self,
    ) -> (
        SPI,
        (Option<SPI::MSck>, Option<SPI::Mi<PULL>>, Option<SPI::Mo>),
    ) {
        (self.inner.spi, self.pins)
    }
}

impl<SPI: Instance, Otype, PULL: UpMode> SpiSlave<SPI, u8, Otype, PULL> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Input<Floating>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn new(
        spi: SPI,
        pins: (
            Option<impl RInto<SPI::SSck, 0>>,
            Option<impl RInto<SPI::So<Otype>, 0>>,
            Option<impl RInto<SPI::Si<PULL>, 0>>,
        ),
        mode: Mode,
    ) -> Self {
        Self::_new(spi, pins, mode)
    }
    fn _new<const R: u8>(
        spi: SPI,
        pins: (
            Option<impl RInto<SPI::SSck, R>>,
            Option<impl RInto<SPI::So<Otype>, R>>,
            Option<impl RInto<SPI::Si<PULL>, R>>,
        ),
        mode: Mode,
    ) -> Self {
        // enable or reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2().write(|w| w.ssoe().clear_bit());

        let pins = (
            pins.0.map(RInto::rinto),
            pins.1.map(RInto::rinto),
            pins.2.map(RInto::rinto),
        );

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
            pins,
        }
    }
}

impl<SPI: Instance, Otype, PULL> SpiSlave<SPI, u8, Otype, PULL> {
    #[allow(clippy::type_complexity)]
    pub fn release(
        self,
    ) -> (
        SPI,
        (
            Option<SPI::SSck>,
            Option<SPI::So<Otype>>,
            Option<SPI::Si<PULL>>,
        ),
    ) {
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

impl<SPI: Instance, PULL> Spi<SPI, u8, PULL> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> Spi<SPI, u16, PULL> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().set_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, Otype, PULL> SpiSlave<SPI, u8, Otype, PULL> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> SpiSlave<SPI, u16, Otype, PULL> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().set_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        SpiSlave {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, PULL> Spi<SPI, u16, PULL> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> Spi<SPI, u16, PULL> {
        self.spi.cr1().modify(|_, w| w.spe().clear_bit());
        self.spi.cr1().modify(|_, w| w.dff().clear_bit());
        self.spi.cr1().modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, Otype, PULL> SpiSlave<SPI, u16, Otype, PULL> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> SpiSlave<SPI, u8, Otype, PULL> {
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

pub type SpiTxDma<SPI, CHANNEL, PULL = Floating> = TxDma<Spi<SPI, u8, PULL>, CHANNEL>;
pub type SpiRxDma<SPI, CHANNEL, PULL = Floating> = RxDma<Spi<SPI, u8, PULL>, CHANNEL>;
pub type SpiRxTxDma<SPI, RXCHANNEL, TXCHANNEL, PULL = Floating> =
    RxTxDma<Spi<SPI, u8, PULL>, RXCHANNEL, TXCHANNEL>;

pub type SpiSlaveTxDma<SPI, CHANNEL, Otype, PULL = Floating> =
    TxDma<SpiSlave<SPI, u8, Otype, PULL>, CHANNEL>;
pub type SpiSlaveRxDma<SPI, CHANNEL, Otype, PULL = Floating> =
    RxDma<SpiSlave<SPI, u8, Otype, PULL>, CHANNEL>;
pub type SpiSlaveRxTxDma<SPI, RXCHANNEL, TXCHANNEL, Otype, PULL = Floating> =
    RxTxDma<SpiSlave<SPI, u8, Otype, PULL>, RXCHANNEL, TXCHANNEL>;

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
        pub type $rxdma<PULL = Floating> = SpiRxDma<$SPIi, $RCi, PULL>;
        pub type $txdma<PULL = Floating> = SpiTxDma<$SPIi, $TCi, PULL>;
        pub type $rxtxdma<PULL = Floating> = SpiRxTxDma<$SPIi, $RCi, $TCi, PULL>;

        impl<PULL> Transmit for SpiTxDma<$SPIi, $TCi, PULL> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<PULL> Receive for SpiRxDma<$SPIi, $RCi, PULL> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<PULL> Transmit for SpiRxTxDma<$SPIi, $RCi, $TCi, PULL> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<PULL> Receive for SpiRxTxDma<$SPIi, $RCi, $TCi, PULL> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<PULL> Spi<$SPIi, u8, PULL> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiTxDma<$SPIi, $TCi, PULL> {
                self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
                SpiTxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_dma(self, channel: $RCi) -> SpiRxDma<$SPIi, $RCi, PULL> {
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
            ) -> SpiRxTxDma<$SPIi, $RCi, $TCi, PULL> {
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

        impl<PULL> SpiTxDma<$SPIi, $TCi, PULL> {
            pub fn release(self) -> (Spi<$SPIi, u8, PULL>, $TCi) {
                let SpiTxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.txdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<PULL> SpiRxDma<$SPIi, $RCi, PULL> {
            pub fn release(self) -> (Spi<$SPIi, u8, PULL>, $RCi) {
                let SpiRxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.rxdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<PULL> SpiRxTxDma<$SPIi, $RCi, $TCi, PULL> {
            pub fn release(self) -> (Spi<$SPIi, u8, PULL>, $RCi, $TCi) {
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

        impl<PULL> TransferPayload for SpiTxDma<$SPIi, $TCi, PULL> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<PULL> TransferPayload for SpiRxDma<$SPIi, $RCi, PULL> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<PULL> TransferPayload for SpiRxTxDma<$SPIi, $RCi, $TCi, PULL> {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<B, PULL> crate::dma::ReadDma<B, u8> for SpiRxDma<$SPIi, $RCi, PULL>
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
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

        impl<B, PULL> crate::dma::WriteDma<B, u8> for SpiTxDma<$SPIi, $TCi, PULL>
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
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

        impl<RXB, TXB, PULL> crate::dma::ReadWriteDma<RXB, TXB, u8>
            for SpiRxTxDma<$SPIi, $RCi, $TCi, PULL>
        where
            RXB: WriteBuffer<Word = u8>,
            TXB: ReadBuffer<Word = u8>,
        {
            fn read_write(
                mut self,
                mut rxbuffer: RXB,
                txbuffer: TXB,
            ) -> Transfer<dma::W, (RXB, TXB), Self> {
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

        pub type $slaverxdma<Otype = PushPull, PULL = Floating> =
            SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL>;
        pub type $slavetxdma<Otype = PushPull, PULL = Floating> =
            SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL>;
        pub type $slaverxtxdma<Otype = PushPull, PULL = Floating> =
            SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL>;

        impl<Otype, PULL> Transmit for SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<Otype, PULL> Receive for SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<Otype, PULL> Transmit for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<Otype, PULL> Receive for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<Otype, PULL> SpiSlave<$SPIi, u8, Otype, PULL> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL> {
                self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
                SpiSlaveTxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_dma(self, channel: $RCi) -> SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL> {
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
            ) -> SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL> {
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

        impl<Otype, PULL> SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype, PULL>, $TCi) {
                let SpiSlaveTxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.txdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<Otype, PULL> SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype, PULL>, $RCi) {
                let SpiSlaveRxDma { payload, channel } = self;
                payload.spi.cr2().modify(|_, w| w.rxdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<Otype, PULL> SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL> {
            pub fn release(self) -> (SpiSlave<$SPIi, u8, Otype, PULL>, $RCi, $TCi) {
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

        impl<Otype, PULL> TransferPayload for SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<Otype, PULL> TransferPayload for SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<Otype, PULL> TransferPayload for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL> {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<B, Otype, PULL> crate::dma::ReadDma<B, u8> for SpiSlaveRxDma<$SPIi, $RCi, Otype, PULL>
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
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

        impl<B, Otype, PULL> crate::dma::WriteDma<B, u8> for SpiSlaveTxDma<$SPIi, $TCi, Otype, PULL>
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
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

        impl<RXB, TXB, Otype, PULL> crate::dma::ReadWriteDma<RXB, TXB, u8>
            for SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype, PULL>
        where
            RXB: WriteBuffer<Word = u8>,
            TXB: ReadBuffer<Word = u8>,
        {
            fn read_write(
                mut self,
                mut rxbuffer: RXB,
                txbuffer: TXB,
            ) -> Transfer<dma::W, (RXB, TXB), Self> {
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
