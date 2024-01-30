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

use core::ops::Deref;
use core::ptr;

pub use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};
use crate::pac::{self, RCC};

use crate::afio::MAPR;
use crate::dma::dma1;
#[cfg(feature = "connectivity")]
use crate::dma::dma2;
use crate::dma::{Receive, RxDma, RxTxDma, Transfer, TransferPayload, Transmit, TxDma, R, W};
use crate::gpio::{self, Alternate, Input};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::Hertz;

use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};

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

/// Spi in Master mode (type state)
pub struct Master;
/// Spi in Slave mode (type state)
pub struct Slave;

mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: bool;
    }
    pub trait Sck<REMAP> {}
    pub trait Miso<REMAP> {}
    pub trait Mosi<REMAP> {}
    pub trait Ssck<REMAP> {}
    pub trait So<REMAP> {}
    pub trait Si<REMAP> {}
}
pub use sealed::Remap;
use sealed::{Miso, Mosi, Sck, Si, So, Ssck};

pub trait Pins<REMAP, OPERATION = Master> {}

impl<REMAP, SCK, MISO, MOSI> Pins<REMAP, Master> for (SCK, MISO, MOSI)
where
    SCK: Sck<REMAP>,
    MISO: Miso<REMAP>,
    MOSI: Mosi<REMAP>,
{
}

impl<REMAP, SCK, MISO, MOSI> Pins<REMAP, Slave> for (SCK, MISO, MOSI)
where
    SCK: Ssck<REMAP>,
    MISO: So<REMAP>,
    MOSI: Si<REMAP>,
{
}

pub struct Spi<SPI, REMAP, PINS, FRAMESIZE, OPERATION = Master> {
    spi: SPI,
    pins: PINS,
    _remap: PhantomData<REMAP>,
    _framesize: PhantomData<FRAMESIZE>,
    _operation: PhantomData<OPERATION>,
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
impl<REMAP> So<REMAP> for NoMiso {}
impl<REMAP> Si<REMAP> for NoMosi {}

macro_rules! remap {
    ($name:ident, $SPIX:ty, $state:literal, $SCK:ident, $MISO:ident, $MOSI:ident) => {
        pub struct $name;
        impl Remap for $name {
            type Periph = $SPIX;
            const REMAP: bool = $state;
        }
        // Master mode pins
        impl<MODE> Sck<$name> for gpio::$SCK<Alternate<MODE>> {}
        impl<MODE> Miso<$name> for gpio::$MISO<Input<MODE>> {}
        impl<MODE> Mosi<$name> for gpio::$MOSI<Alternate<MODE>> {}
        // Slave mode pins
        impl<MODE> Ssck<$name> for gpio::$SCK<Input<MODE>> {}
        impl<MODE> So<$name> for gpio::$MISO<Alternate<MODE>> {}
        impl<MODE> Si<$name> for gpio::$MOSI<Input<MODE>> {}
    };
}

remap!(Spi1NoRemap, pac::SPI1, false, PA5, PA6, PA7);
remap!(Spi1Remap, pac::SPI1, true, PB3, PB4, PB5);
remap!(Spi2NoRemap, pac::SPI2, false, PB13, PB14, PB15);
#[cfg(any(feature = "high", feature = "connectivity"))]
remap!(Spi3NoRemap, pac::SPI3, false, PB3, PB4, PB5);
#[cfg(feature = "connectivity")]
remap!(Spi3Remap, pac::SPI3, true, PC10, PC11, PC12);

pub trait Instance:
    crate::Sealed + Deref<Target = crate::pac::spi1::RegisterBlock> + Enable + Reset + BusClock
{
}

impl Instance for pac::SPI1 {}
impl Instance for pac::SPI2 {}
#[cfg(any(feature = "high", feature = "connectivity"))]
impl Instance for pac::SPI3 {}

impl<REMAP, PINS> Spi<pac::SPI1, REMAP, PINS, u8, Master> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Alternate<...>, Input<...>, Alternate<...>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi1(
        spi: pac::SPI1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: Hertz,
        clocks: Clocks,
    ) -> Self
    where
        REMAP: Remap<Periph = pac::SPI1>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| w.spi1_remap().bit(REMAP::REMAP));
        Spi::<pac::SPI1, _, _, u8>::configure(spi, pins, mode, freq, clocks)
    }
}

impl<REMAP, PINS> Spi<pac::SPI1, REMAP, PINS, u8, Slave> {
    /**
      Constructs an SPI instance using SPI1 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PA5, PA6, PA7)` or `(PB3, PB4, PB5)` configured as `(Input<...>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi1_slave(spi: pac::SPI1, pins: PINS, mapr: &mut MAPR, mode: Mode) -> Self
    where
        REMAP: Remap<Periph = pac::SPI1>,
        PINS: Pins<REMAP, Slave>,
    {
        mapr.modify_mapr(|_, w| w.spi1_remap().bit(REMAP::REMAP));
        Spi::<pac::SPI1, _, _, u8, Slave>::configure(spi, pins, mode)
    }
}

impl<REMAP, PINS> Spi<pac::SPI2, REMAP, PINS, u8, Master> {
    /**
      Constructs an SPI instance using SPI2 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB13, PB14, PB15)` configured as `(Alternate<...>, Input<...>, Alternate<...>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi2(spi: pac::SPI2, pins: PINS, mode: Mode, freq: Hertz, clocks: Clocks) -> Self
    where
        REMAP: Remap<Periph = pac::SPI2>,
        PINS: Pins<REMAP>,
    {
        Spi::<pac::SPI2, _, _, u8>::configure(spi, pins, mode, freq, clocks)
    }
}

impl<REMAP, PINS> Spi<pac::SPI2, REMAP, PINS, u8, Slave> {
    /**
      Constructs an SPI instance using SPI2 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB13, PB14, PB15)` configured as `(Input<...>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    pub fn spi2_slave(spi: pac::SPI2, pins: PINS, mode: Mode) -> Self
    where
        REMAP: Remap<Periph = pac::SPI2>,
        PINS: Pins<REMAP, Slave>,
    {
        Spi::<pac::SPI2, _, _, u8, Slave>::configure(spi, pins, mode)
    }
}

#[cfg(any(feature = "high", feature = "connectivity"))]
impl<REMAP, PINS> Spi<pac::SPI3, REMAP, PINS, u8, Master> {
    /**
      Constructs an SPI instance using SPI3 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB3, PB4, PB5)` configured as `(Alternate<...>, Input<...>, Alternate<...>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    #[cfg(not(feature = "connectivity"))]
    pub fn spi3(spi: pac::SPI3, pins: PINS, mode: Mode, freq: Hertz, clocks: Clocks) -> Self
    where
        REMAP: Remap<Periph = pac::SPI3>,
        PINS: Pins<REMAP>,
    {
        Spi::<pac::SPI3, _, _, u8>::configure(spi, pins, mode, freq, clocks)
    }

    /**
      Constructs an SPI instance using SPI3 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB3, PB4, PB5)` or `(PC10, PC11, PC12)` configured as `(Alternate<...>, Input<...>, Alternate<...>)`.

      You can also use `NoSck`, `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    #[cfg(feature = "connectivity")]
    pub fn spi3(
        spi: pac::SPI3,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: Hertz,
        clocks: Clocks,
    ) -> Self
    where
        REMAP: Remap<Periph = pac::SPI3>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| w.spi3_remap().bit(REMAP::REMAP));
        Spi::<pac::SPI3, _, _, u8>::configure(spi, pins, mode, freq, clocks)
    }
}

#[cfg(any(feature = "high", feature = "connectivity"))]
impl<REMAP, PINS> Spi<pac::SPI3, REMAP, PINS, u8, Slave> {
    /**
      Constructs an SPI instance using SPI3 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB3, PB4, PB5)` configured as `(Input<...>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    #[cfg(not(feature = "connectivity"))]
    pub fn spi3_slave(spi: pac::SPI3, pins: PINS, mode: Mode) -> Self
    where
        REMAP: Remap<Periph = pac::SPI3>,
        PINS: Pins<REMAP>,
    {
        Spi::<pac::SPI3, _, _, u8, Slave>::configure(spi, pins, mode)
    }

    /**
      Constructs an SPI instance using SPI3 in 8bit dataframe mode.

      The pin parameter tuple (sck, miso, mosi) should be `(PB3, PB4, PB5)` or `(PC10, PC11, PC12)` configured as `(Input<...>, Alternate<...>, Input<...>)`.

      You can also use `NoMiso` or `NoMosi` if you don't want to use the pins
    */
    #[cfg(feature = "connectivity")]
    pub fn spi3_slave(spi: pac::SPI3, pins: PINS, mapr: &mut MAPR, mode: Mode) -> Self
    where
        REMAP: Remap<Periph = pac::SPI3>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| w.spi3_remap().bit(REMAP::REMAP));
        Spi::<pac::SPI3, _, _, u8, Slave>::configure(spi, pins, mode)
    }
}

pub trait SpiReadWrite<T> {
    fn read_data_reg(&mut self) -> T;
    fn write_data_reg(&mut self, data: T);
    fn spi_write(&mut self, words: &[T]) -> Result<(), Error>;
}

impl<SPI, REMAP, PINS, FrameSize, OP> SpiReadWrite<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
    FrameSize: Copy,
{
    fn read_data_reg(&mut self) -> FrameSize {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const FrameSize) }
    }

    fn write_data_reg(&mut self, data: FrameSize) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(ptr::addr_of!(self.spi.dr) as *mut FrameSize, data) }
    }

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn spi_write(&mut self, words: &[FrameSize]) -> Result<(), Error> {
        // Write each word when the tx buffer is empty
        for word in words {
            loop {
                let sr = self.spi.sr.read();
                if sr.txe().bit_is_set() {
                    // NOTE(write_volatile) see note above
                    // unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *word) }
                    self.write_data_reg(*word);
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

impl<SPI, REMAP, PINS, FrameSize, OP> Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
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

    /// Starts listening to the SPI by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.spi.cr2.modify(|_, w| w.rxneie().set_bit()),
            Event::Txe => self.spi.cr2.modify(|_, w| w.txeie().set_bit()),
            Event::Error => self.spi.cr2.modify(|_, w| w.errie().set_bit()),
        }
    }

    /// Stops listening to the SPI by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.spi.cr2.modify(|_, w| w.rxneie().clear_bit()),
            Event::Txe => self.spi.cr2.modify(|_, w| w.txeie().clear_bit()),
            Event::Error => self.spi.cr2.modify(|_, w| w.errie().clear_bit()),
        }
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        self.spi.sr.read().txe().bit_is_set()
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        self.spi.sr.read().rxne().bit_is_set()
    }

    /// Returns true if data are received and the previous data have not yet been read from SPI_DR.
    pub fn is_overrun(&self) -> bool {
        self.spi.sr.read().ovr().bit_is_set()
    }

    pub fn is_busy(&self) -> bool {
        self.spi.sr.read().bsy().bit_is_set()
    }
}

impl<SPI, REMAP, PINS> Spi<SPI, REMAP, PINS, u8, Master>
where
    SPI: Instance,
{
    fn configure(spi: SPI, pins: PINS, mode: Mode, freq: Hertz, clocks: Clocks) -> Self {
        // enable or reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2.write(|w| w.ssoe().clear_bit());

        let br = match SPI::clock(&clocks) / freq {
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
            _operation: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS> Spi<SPI, REMAP, PINS, u8, Slave>
where
    SPI: Instance,
{
    fn configure(spi: SPI, pins: PINS, mode: Mode) -> Self {
        // enable or reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2.write(|w| w.ssoe().clear_bit());

        spi.cr1.write(|w| {
            w
                // clock phase from config
                .cpha()
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
                // clock polarity from config
                .cpol()
                .bit(mode.polarity == Polarity::IdleHigh)
                // mstr: slave configuration
                .mstr()
                .clear_bit()
                // lsbfirst: MSB first
                .lsbfirst()
                .clear_bit()
                // ssm: enable software slave management (NSS pin free for other uses)
                .ssm()
                .set_bit()
                // ssi: set nss low = slave mode
                .ssi()
                .clear_bit()
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
            _operation: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS, OP> Spi<SPI, REMAP, PINS, u8, OP>
where
    SPI: Instance,
{
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
            _operation: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS, OP> Spi<SPI, REMAP, PINS, u16, OP>
where
    SPI: Instance,
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
            _operation: PhantomData,
        }
    }
}

impl<SPI, REMAP, PINS, FrameSize, OP> crate::hal::spi::FullDuplex<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
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

        Err(if sr.modf().bit_is_set() {
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

impl<SPI, REMAP, PINS, FrameSize, OP> crate::hal::blocking::spi::transfer::Default<FrameSize>
    for Spi<SPI, REMAP, PINS, FrameSize, OP>
where
    SPI: Instance,
    FrameSize: Copy,
{
}

impl<SPI, REMAP, PINS, OP> crate::hal::blocking::spi::Write<u8> for Spi<SPI, REMAP, PINS, u8, OP>
where
    SPI: Instance,
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

impl<SPI, REMAP, PINS, OP> crate::hal::blocking::spi::Write<u16> for Spi<SPI, REMAP, PINS, u16, OP>
where
    SPI: Instance,
{
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Error> {
        self.spi_write(words)
    }
}

// DMA

pub type SpiTxDma<SPI, REMAP, PINS, OP, CHANNEL> = TxDma<Spi<SPI, REMAP, PINS, u8, OP>, CHANNEL>;
pub type SpiRxDma<SPI, REMAP, PINS, OP, CHANNEL> = RxDma<Spi<SPI, REMAP, PINS, u8, OP>, CHANNEL>;
pub type SpiRxTxDma<SPI, REMAP, PINS, OP, RXCHANNEL, TXCHANNEL> =
    RxTxDma<Spi<SPI, REMAP, PINS, u8, OP>, RXCHANNEL, TXCHANNEL>;

macro_rules! spi_dma {
    ($SPIi:ty, $RCi:ty, $TCi:ty, $rxdma:ident, $txdma:ident, $rxtxdma:ident) => {
        pub type $rxdma<REMAP, PINS, OP> = SpiRxDma<$SPIi, REMAP, PINS, OP, $RCi>;
        pub type $txdma<REMAP, PINS, OP> = SpiTxDma<$SPIi, REMAP, PINS, OP, $TCi>;
        pub type $rxtxdma<REMAP, PINS, OP> = SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi>;

        impl<REMAP, PINS, OP> Transmit for SpiTxDma<$SPIi, REMAP, PINS, OP, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<REMAP, PINS, OP> Receive for SpiRxDma<$SPIi, REMAP, PINS, OP, $RCi> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<REMAP, PINS, OP> Transmit for SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<REMAP, PINS, OP> Receive for SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        impl<REMAP, PINS, OP> Spi<$SPIi, REMAP, PINS, u8, OP> {
            pub fn with_tx_dma(self, channel: $TCi) -> SpiTxDma<$SPIi, REMAP, PINS, OP, $TCi> {
                self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                SpiTxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_dma(self, channel: $RCi) -> SpiRxDma<$SPIi, REMAP, PINS, OP, $RCi> {
                self.spi.cr2.modify(|_, w| w.rxdmaen().set_bit());
                SpiRxDma {
                    payload: self,
                    channel,
                }
            }
            pub fn with_rx_tx_dma(
                self,
                rxchannel: $RCi,
                txchannel: $TCi,
            ) -> SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi> {
                self.spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().set_bit().txdmaen().set_bit());
                SpiRxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }

        impl<REMAP, PINS, OP> SpiTxDma<$SPIi, REMAP, PINS, OP, $TCi> {
            pub fn release(self) -> (Spi<$SPIi, REMAP, PINS, u8, OP>, $TCi) {
                let SpiTxDma { payload, channel } = self;
                payload.spi.cr2.modify(|_, w| w.txdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<REMAP, PINS, OP> SpiRxDma<$SPIi, REMAP, PINS, OP, $RCi> {
            pub fn release(self) -> (Spi<$SPIi, REMAP, PINS, u8, OP>, $RCi) {
                let SpiRxDma { payload, channel } = self;
                payload.spi.cr2.modify(|_, w| w.rxdmaen().clear_bit());
                (payload, channel)
            }
        }

        impl<REMAP, PINS, OP> SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi> {
            pub fn release(self) -> (Spi<$SPIi, REMAP, PINS, u8, OP>, $RCi, $TCi) {
                let SpiRxTxDma {
                    payload,
                    rxchannel,
                    txchannel,
                } = self;
                payload
                    .spi
                    .cr2
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit());
                (payload, rxchannel, txchannel)
            }
        }

        impl<REMAP, PINS, OP> TransferPayload for SpiTxDma<$SPIi, REMAP, PINS, OP, $TCi> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<REMAP, PINS, OP> TransferPayload for SpiRxDma<$SPIi, REMAP, PINS, OP, $RCi> {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<REMAP, PINS, OP> TransferPayload for SpiRxTxDma<$SPIi, REMAP, PINS, OP, $RCi, $TCi> {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<B, REMAP, PIN, OP> crate::dma::ReadDma<B, u8> for SpiRxDma<$SPIi, REMAP, PIN, OP, $RCi>
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*<$SPIi>::ptr()).dr as *const _ as u32 },
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
                        // write to memory
                        .dir()
                        .clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }

        impl<B, REMAP, PIN, OP> crate::dma::WriteDma<B, u8>
            for SpiTxDma<$SPIi, REMAP, PIN, OP, $TCi>
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.read_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*<$SPIi>::ptr()).dr as *const _ as u32 },
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

        impl<RXB, TXB, REMAP, PIN, OP> crate::dma::ReadWriteDma<RXB, TXB, u8>
            for SpiRxTxDma<$SPIi, REMAP, PIN, OP, $RCi, $TCi>
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
                    unsafe { &(*<$SPIi>::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.rxchannel.set_memory_address(rxptr as u32, true);
                self.rxchannel.set_transfer_length(rxlen);

                self.txchannel.set_peripheral_address(
                    unsafe { &(*<$SPIi>::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.txchannel.set_memory_address(txptr as u32, true);
                self.txchannel.set_transfer_length(txlen);

                atomic::compiler_fence(Ordering::Release);
                self.rxchannel.ch().cr.modify(|_, w| {
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
                        // write to memory
                        .dir()
                        .clear_bit()
                });
                self.txchannel.ch().cr.modify(|_, w| {
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

                Transfer::w((rxbuffer, txbuffer), self)
            }
        }
    };
}

spi_dma!(
    pac::SPI1,
    dma1::C2,
    dma1::C3,
    Spi1RxDma,
    Spi1TxDma,
    Spi1RxTxDma
);
spi_dma!(
    pac::SPI2,
    dma1::C4,
    dma1::C5,
    Spi2RxDma,
    Spi2TxDma,
    Spi2RxTxDma
);
#[cfg(feature = "connectivity")]
spi_dma!(
    pac::SPI3,
    dma2::C1,
    dma2::C2,
    Spi3RxDma,
    Spi3TxDma,
    Spi3RxTxDma
);
