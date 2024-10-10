//! # Serial Communication (USART)
//!
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter)
//!
//! ## Note
//!
//! When transmitting with the parity enabled, the value written in the MSB
//! (bit 7 or bit 8 depending on the word length) has no effect because it
//! is replaced by the parity.
//! When receiving with the parity enabled, the value read in the MSB
//! is the received parity bit.
//!
//! | Frame format               | Word Length | Parity |
//! | -------------------------- |:----------- |:------ |
//! | 7 data bits + 1 parity bit | 8 bits      | V      |
//! | 8 data bits                | 8 bits      |        |
//! | 8 data bits + 1 parity bit | 9 bits      | V      |
//! | 9 data bits                | 9 bits      |        |
//!
//!
//! ## Example usage:
//!
//!  ```rust
//! let dp = stm32f1xx_hal::Peripherals::take().unwrap();
//! let mut flash = dp.FLASH.constrain();
//! let mut rcc = dp.RCC.constrain();
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//! let mut afio = dp.AFIO.constrain();
//! let mut gpioa = dp.GPIOA.split();
//!
//! // USART1 on Pins A9 and A10
//! let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
//! let pin_rx = gpioa.pa10;
//! // Create an interface struct for USART1 with 9600 Baud
//! let serial = Serial::new(
//!     dp.USART1,
//!     (pin_tx, pin_rx),
//!     &mut afio.mapr,
//!     Config::default()
//!         .baudrate(9600.bps())
//!         .wordlength_9bits()
//!         .parity_none(),
//!     &clocks,
//! );
//!
//! // Separate into tx and rx channels
//! let (mut tx, mut rx) = serial.split();
//!
//! // Write data (9 bits) to the USART.
//! // Depending on the configuration, only the lower 7, 8, or 9 bits are used.
//! block!(tx.write_u16(0x1FF)).unwrap();
//!
//! // Write 'R' (8 bits) to the USART
//! block!(tx.write_u8(b'R')).unwrap();
//!
//! // Receive a data (9 bits) from the USART and store it in "received"
//! let received = block!(rx.read_u16()).unwrap();
//!
//! // Receive a data (8 bits) from the USART and store it in "received"
//! let received = block!(rx.read()).unwrap();
//!  ```

use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::afio::Remap;
use crate::dma::{dma1, CircBuffer, RxDma, Transfer, TxDma, R, W};
use crate::gpio::{self, Alternate, Cr, Floating, Input, NoPin, PinMode, PullUp, PushPull};
use crate::pac::{self, RCC};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{Bps, U32Ext};

mod hal_02;
mod hal_1;

pub trait InMode {}
impl InMode for Floating {}
impl InMode for PullUp {}

pub struct Pins<TX, RX> {
    pub tx: TX,
    pub rx: RX,
}

impl<TX, RX> From<(TX, RX)> for Pins<TX, RX> {
    fn from(value: (TX, RX)) -> Self {
        Self {
            tx: value.0,
            rx: value.1,
        }
    }
}

// USART REMAPPING, see: https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
// Section 9.3.8
pub mod usart1 {
    use super::*;

    remap! {
        pac::USART1: [
            Tx: PA9, Rx: PA10 => 0;
            Tx: PB6, Rx: PB7  => 1;
        ]
    }
}

pub mod usart2 {
    use super::*;

    remap! {
        pac::USART2: [
            Tx: PA2, Rx: PA3 => 0;
            Tx: PD5, Rx: PD6 => 1;
        ]
    }
}

pub mod usart3 {
    use super::*;

    remap! {
        pac::USART3: [
            Tx: PB10, Rx: PB11 => 0;
            Tx: PC10, Rx: PC11 => 1;
            Tx: PD8,  Rx: PD9  => 3;
        ]
    }
}

macro_rules! remap {
    ($PER:ty: [$(Tx: $TX:ident, Rx: $RX:ident => $remap:literal;)+]) => {
        pub enum Tx<Otype> {
            $(
                $TX(gpio::$TX<Alternate<Otype>>),
            )+
            None(NoPin<Otype>),
        }
        pub enum Rx<PULL> {
            $(
                $RX(gpio::$RX<Input<PULL>>),
            )+
            None(NoPin<PULL>),
        }

        $(
            impl<Otype, PULL: InMode> From<(gpio::$TX<Alternate<Otype>>, gpio::$RX<Input<PULL>>, &mut <$PER as Remap>::Mapr)> for Pins<Tx<Otype>, Rx<PULL>> {
                fn from(p: (gpio::$TX<Alternate<Otype>>, gpio::$RX<Input<PULL>>, &mut <$PER as Remap>::Mapr)) -> Self {
                    <$PER>::remap(p.2, $remap);
                    Self { tx: Tx::$TX(p.0), rx: Rx::$RX(p.1) }
                }
            }

            impl<Otype, PULL> From<(gpio::$TX, gpio::$RX, &mut <$PER as Remap>::Mapr)> for Pins<Tx<Otype>, Rx<PULL>>
            where
                Alternate<Otype>: PinMode,
                Input<PULL>: PinMode,
                PULL: InMode,
            {
                fn from(p: (gpio::$TX, gpio::$RX, &mut <$PER as Remap>::Mapr)) -> Self {
                    let mut cr = Cr;
                    let tx = p.0.into_mode(&mut cr);
                    let rx = p.1.into_mode(&mut cr);
                    <$PER>::remap(p.2, $remap);
                    Self { tx: Tx::$TX(tx), rx: Rx::$RX(rx) }
                }
            }

            impl<Otype> From<(gpio::$TX<Alternate<Otype>>, &mut <$PER as Remap>::Mapr)> for Pins<Tx<Otype>, Rx<Floating>> {
                fn from(p: (gpio::$TX<Alternate<Otype>>, &mut <$PER as Remap>::Mapr)) -> Self {
                    <$PER>::remap(p.1, $remap);
                    Self { tx: Tx::$TX(p.0), rx: Rx::None(NoPin::new()) }
                }
            }

            impl<Otype> From<(gpio::$TX, &mut <$PER as Remap>::Mapr)> for Pins<Tx<Otype>, Rx<Floating>>
            where
                Alternate<Otype>: PinMode,
            {
                fn from(p: (gpio::$TX, &mut <$PER as Remap>::Mapr)) -> Self {
                    let tx = p.0.into_mode(&mut Cr);
                    <$PER>::remap(p.1, $remap);
                    Self { tx: Tx::$TX(tx), rx: Rx::None(NoPin::new()) }
                }
            }

            impl<PULL> From<(gpio::$RX<Input<PULL>>, &mut <$PER as Remap>::Mapr)> for Pins<Tx<PushPull>, Rx<PULL>>
            where
                PULL: InMode,
            {
                fn from(p: (gpio::$RX<Input<PULL>>, &mut <$PER as Remap>::Mapr)) -> Self {
                    <$PER>::remap(p.1, $remap);
                    Self { tx: Tx::None(NoPin::new()), rx: Rx::$RX(p.0) }
                }
            }
        )+
    }
}
use remap;

pub trait SerialExt: Sized + Instance {
    fn serial(
        self,
        pins: impl Into<Pins<Self::Tx<PushPull>, Self::Rx<Floating>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self, PushPull, Floating>;
    fn tx(
        self,
        pins: impl Into<Pins<Self::Tx<PushPull>, Self::Rx<Floating>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Tx<Self> {
        self.serial(pins.into(), config, clocks).split().0
    }
    fn rx(
        self,
        pins: impl Into<Pins<Self::Tx<PushPull>, Self::Rx<Floating>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Rx<Self> {
        self.serial(pins.into(), config, clocks).split().1
    }
}

impl<USART: Instance> SerialExt for USART {
    fn serial(
        self,
        pins: impl Into<Pins<Self::Tx<PushPull>, Self::Rx<Floating>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self> {
        Serial::new(self, pins, config, clocks)
    }
}

use crate::pac::usart1 as uart_base;

pub trait Instance:
    crate::Sealed + Deref<Target = uart_base::RegisterBlock> + Enable + Reset + BusClock
{
    type Tx<Otype>;
    type Rx<PULL>;

    #[doc(hidden)]
    fn ptr() -> *const uart_base::RegisterBlock;
}

macro_rules! inst {
    ($($USARTX:ty, $usart:ident;)+) => {
        $(
            impl Instance for $USARTX {
                type Tx<Otype> = $usart::Tx<Otype>;
                type Rx<PULL> = $usart::Rx<PULL>;

                fn ptr() -> *const uart_base::RegisterBlock {
                    <$USARTX>::ptr()
                }
            }
        )+
    };
}

inst! {
    pac::USART1, usart1;
    pac::USART2, usart2;
    pac::USART3, usart3;
}

/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The peripheral receive buffer was overrun.
    Overrun,
    /// Received data does not conform to the peripheral configuration.
    /// Can be caused by a misconfigured device on either end of the serial line.
    FrameFormat,
    /// Parity check failed.
    Parity,
    /// Serial line is too noisy to read valid data.
    Noise,
    /// A different error occurred. The original error may contain more information.
    Other,
}

pub enum WordLength {
    /// When parity is enabled, a word has 7 data bits + 1 parity bit,
    /// otherwise 8 data bits.
    Bits8,
    /// When parity is enabled, a word has 8 data bits + 1 parity bit,
    /// otherwise 9 data bits.
    Bits9,
}

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub enum StopBits {
    /// 1 stop bit
    STOP1,
    /// 0.5 stop bits
    STOP0P5,
    /// 2 stop bits
    STOP2,
    /// 1.5 stop bits
    STOP1P5,
}

pub struct Config {
    pub baudrate: Bps,
    pub wordlength: WordLength,
    pub parity: Parity,
    pub stopbits: StopBits,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn wordlength(mut self, wordlength: WordLength) -> Self {
        self.wordlength = wordlength;
        self
    }

    pub fn wordlength_8bits(mut self) -> Self {
        self.wordlength = WordLength::Bits8;
        self
    }

    pub fn wordlength_9bits(mut self) -> Self {
        self.wordlength = WordLength::Bits9;
        self
    }

    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            baudrate: 115_200_u32.bps(),
            wordlength: WordLength::Bits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

impl From<Bps> for Config {
    fn from(baud: Bps) -> Self {
        Config::default().baudrate(baud)
    }
}

/// Serial abstraction
pub struct Serial<USART: Instance, Otype = PushPull, PULL = Floating> {
    pub tx: Tx<USART>,
    pub rx: Rx<USART>,
    #[allow(clippy::type_complexity)]
    pub token: ReleaseToken<USART, (USART::Tx<Otype>, USART::Rx<PULL>)>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Stores data for release
pub struct ReleaseToken<USART, PINS> {
    usart: USART,
    pins: PINS,
}

impl<USART: Instance, Otype> Serial<USART, Otype, Floating> {
    pub fn tx(
        usart: USART,
        pins: impl Into<Pins<USART::Tx<Otype>, USART::Rx<Floating>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Tx<USART> {
        Self::new(usart, pins.into(), config, clocks).split().0
    }
}

impl<USART: Instance, PULL> Serial<USART, PushPull, PULL> {
    pub fn rx(
        usart: USART,
        pins: impl Into<Pins<USART::Tx<PushPull>, USART::Rx<PULL>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Rx<USART> {
        Self::new(usart, pins.into(), config, clocks).split().1
    }
}

impl<USART: Instance, Otype, PULL> Serial<USART, Otype, PULL> {
    /// Configures the serial interface and creates the interface
    /// struct.
    ///
    /// `Bps` is the baud rate of the interface.
    ///
    /// `Clocks` passes information about the current frequencies of
    /// the clocks.  The existence of the struct ensures that the
    /// clock settings are fixed.
    ///
    /// The `serial` struct takes ownership over the `USARTX` device
    /// registers and the specified `PINS`
    ///
    /// `MAPR` and `APBX` are register handles which are passed for
    /// configuration. (`MAPR` is used to map the USART to the
    /// corresponding pins. `APBX` is used to reset the USART.)
    pub fn new(
        usart: USART,
        pins: impl Into<Pins<USART::Tx<Otype>, USART::Rx<PULL>>>,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Self {
        // Enable and reset USART
        let rcc = unsafe { &(*RCC::ptr()) };
        USART::enable(rcc);
        USART::reset(rcc);

        apply_config::<USART>(config.into(), clocks);

        let pins = pins.into();

        // UE: enable USART
        // TE: enable transceiver
        // RE: enable receiver
        usart.cr1().modify(|_r, w| {
            w.ue().set_bit();
            w.te().set_bit();
            w.re().set_bit();
            w
        });

        Serial {
            tx: Tx {
                _usart: PhantomData,
            },
            rx: Rx {
                _usart: PhantomData,
            },
            token: ReleaseToken {
                usart,
                pins: (pins.tx, pins.rx),
            },
        }
    }

    /// Reconfigure the USART instance.
    ///
    /// If a transmission is currently in progress, this returns
    /// [`nb::Error::WouldBlock`].
    pub fn reconfigure(
        &mut self,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> nb::Result<(), Error> {
        reconfigure(&mut self.tx, &mut self.rx, config, clocks)
    }

    /// Returns ownership of the borrowed register handles
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// ```
    /// let mut serial = Serial::new(usart, (tx_pin, rx_pin), &mut afio.mapr, 9600.bps(), &clocks);
    ///
    /// // You can split the `Serial`
    /// let Serial { tx, rx, token } = serial;
    ///
    /// // You can reunite the `Serial` back
    /// let serial = Serial { tx, rx, token };
    ///
    /// // Release `Serial`
    /// let (usart, (tx_pin, rx_pin)) = serial.release();
    /// ```
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (USART, (USART::Tx<Otype>, USART::Rx<PULL>)) {
        (self.token.usart, self.token.pins)
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and
    /// receiving (Rx)
    ///
    /// If in the future it will be necessary to free up resources,
    /// then you need to use a different method of separation:
    ///
    /// ```
    /// let Serial { tx, rx, token } = serial;
    /// ```
    pub fn split(self) -> (Tx<USART>, Rx<USART>) {
        (self.tx, self.rx)
    }
}

fn apply_config<USART: Instance>(config: Config, clocks: &Clocks) {
    let usart = unsafe { &*USART::ptr() };

    // Configure baud rate
    let brr = USART::clock(clocks).raw() / config.baudrate.0;
    assert!(brr >= 16, "impossible baud rate");
    usart.brr().write(|w| unsafe { w.bits(brr) });

    // Configure word
    usart.cr1().modify(|_r, w| {
        w.m().bit(match config.wordlength {
            WordLength::Bits8 => false,
            WordLength::Bits9 => true,
        });
        use crate::pac::usart1::cr1::PS;
        w.ps().variant(match config.parity {
            Parity::ParityOdd => PS::Odd,
            _ => PS::Even,
        });
        w.pce().bit(!matches!(config.parity, Parity::ParityNone));
        w
    });

    // Configure stop bits
    let stop_bits = match config.stopbits {
        StopBits::STOP1 => 0b00,
        StopBits::STOP0P5 => 0b01,
        StopBits::STOP2 => 0b10,
        StopBits::STOP1P5 => 0b11,
    };
    usart.cr2().modify(|_r, w| w.stop().set(stop_bits));
}

/// Reconfigure the USART instance.
///
/// If a transmission is currently in progress, this returns
/// [`nb::Error::WouldBlock`].
pub fn reconfigure<USART: Instance>(
    tx: &mut Tx<USART>,
    #[allow(unused_variables)] rx: &mut Rx<USART>,
    config: impl Into<Config>,
    clocks: &Clocks,
) -> nb::Result<(), Error> {
    // if we're currently busy transmitting, we have to wait until that is
    // over -- regarding reception, we assume that the caller -- with
    // exclusive access to the Serial instance due to &mut self -- knows
    // what they're doing.
    tx.flush()?;
    apply_config::<USART>(config.into(), clocks);
    Ok(())
}

impl<USART: Instance> Tx<USART> {
    /// Writes 9-bit words to the UART/USART
    ///
    /// If the UART/USART was configured with `WordLength::Bits9`, the 9 least significant bits will
    /// be transmitted and the other 7 bits will be ignored. Otherwise, the 8 least significant bits
    /// will be transmitted and the other 8 bits will be ignored.
    pub fn write_u16(&mut self, word: u16) -> nb::Result<(), Error> {
        let usart = unsafe { &*USART::ptr() };

        if usart.sr().read().txe().bit_is_set() {
            usart.dr().write(|w| w.dr().set(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn write_u8(&mut self, word: u8) -> nb::Result<(), Error> {
        self.write_u16(word as u16)
    }

    pub fn bwrite_all_u16(&mut self, buffer: &[u16]) -> Result<(), Error> {
        for &w in buffer {
            nb::block!(self.write_u16(w))?;
        }
        Ok(())
    }

    pub fn bwrite_all_u8(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for &w in buffer {
            nb::block!(self.write_u8(w))?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> nb::Result<(), Error> {
        let usart = unsafe { &*USART::ptr() };

        if usart.sr().read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn bflush(&mut self) -> Result<(), Error> {
        nb::block!(self.flush())
    }

    /// Start listening for transmit interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.txeie().set_bit()) };
    }

    /// Stop listening for transmit interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.txeie().clear_bit()) };
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr().read().txe().bit_is_set() }
    }

    pub fn is_tx_complete(&self) -> bool {
        unsafe { (*USART::ptr()).sr().read().tc().bit_is_set() }
    }
}

impl<USART: Instance> core::fmt::Write for Tx<USART> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write_u8(c)))
            .map_err(|_| core::fmt::Error)
    }
}

impl<USART: Instance> Rx<USART> {
    /// Reads 9-bit words from the UART/USART
    ///
    /// If the UART/USART was configured with `WordLength::Bits9`, the returned value will contain
    /// 9 received data bits and all other bits set to zero. Otherwise, the returned value will contain
    /// 8 received data bits and all other bits set to zero.
    pub fn read_u16(&mut self) -> nb::Result<u16, Error> {
        let usart = unsafe { &*USART::ptr() };
        let sr = usart.sr().read();

        // Check for any errors
        let err = if sr.pe().bit_is_set() {
            Some(Error::Parity)
        } else if sr.fe().bit_is_set() {
            Some(Error::FrameFormat)
        } else if sr.ne().bit_is_set() {
            Some(Error::Noise)
        } else if sr.ore().bit_is_set() {
            Some(Error::Overrun)
        } else {
            None
        };

        if let Some(err) = err {
            // Some error occurred. In order to clear that error flag, you have to
            // do a read from the sr register followed by a read from the dr register.
            let _ = usart.sr().read();
            let _ = usart.dr().read();
            Err(nb::Error::Other(err))
        } else {
            // Check if a byte is available
            if sr.rxne().bit_is_set() {
                // Read the received byte
                Ok(usart.dr().read().dr().bits())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    pub fn read(&mut self) -> nb::Result<u8, Error> {
        self.read_u16().map(|word16| word16 as u8)
    }

    /// Start listening for receive interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.rxneie().set_bit()) };
    }

    /// Stop listening for receive interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.rxneie().clear_bit()) };
    }

    /// Start listening for idle interrupt event
    pub fn listen_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.idleie().set_bit()) };
    }

    /// Stop listening for idle interrupt event
    pub fn unlisten_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1().modify(|_, w| w.idleie().clear_bit()) };
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { (*USART::ptr()).sr().read().idle().bit_is_set() }
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr().read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*USART::ptr()).sr().read();
            let _ = (*USART::ptr()).dr().read();
        }
    }
}

/// Interrupt event
pub enum Event {
    /// New data can be sent
    Txe,
    /// New data has been received
    Rxne,
    /// Idle line state detected
    Idle,
}

impl<USART: Instance, Otype, PULL> Serial<USART, Otype, PULL> {
    /// Starts listening to the USART by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.rx.listen(),
            Event::Txe => self.tx.listen(),
            Event::Idle => self.rx.listen_idle(),
        }
    }

    /// Stops listening to the USART by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.rx.unlisten(),
            Event::Txe => self.tx.unlisten(),
            Event::Idle => self.rx.unlisten_idle(),
        }
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        self.rx.is_idle()
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        self.tx.is_tx_empty()
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        self.rx.is_rx_not_empty()
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        self.rx.clear_idle_interrupt();
    }
}

impl<USART: Instance, PINS> core::fmt::Write for Serial<USART, PINS> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

pub type Rx1 = Rx<pac::USART1>;
pub type Tx1 = Tx<pac::USART1>;
pub type Rx2 = Rx<pac::USART2>;
pub type Tx2 = Tx<pac::USART2>;
pub type Rx3 = Rx<pac::USART3>;
pub type Tx3 = Tx<pac::USART3>;

use crate::dma::{Receive, TransferPayload, Transmit};

macro_rules! serialdma {
    (
        $USARTX:ty,
        $rxdma:ident,
        $txdma:ident,
        rx: $dmarxch:ty,
        tx: $dmatxch:ty
    ) => {
        pub type $rxdma = RxDma<Rx<$USARTX>, $dmarxch>;
        pub type $txdma = TxDma<Tx<$USARTX>, $dmatxch>;

        impl Receive for $rxdma {
            type RxChannel = $dmarxch;
            type TransmittedWord = u8;
        }

        impl Transmit for $txdma {
            type TxChannel = $dmatxch;
            type ReceivedWord = u8;
        }

        impl TransferPayload for $rxdma {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl TransferPayload for $txdma {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl Rx<$USARTX> {
            pub fn with_dma(self, channel: $dmarxch) -> $rxdma {
                unsafe {
                    (*<$USARTX>::ptr()).cr3().modify(|_, w| w.dmar().set_bit());
                }
                RxDma {
                    payload: self,
                    channel,
                }
            }
        }

        impl Tx<$USARTX> {
            pub fn with_dma(self, channel: $dmatxch) -> $txdma {
                unsafe {
                    (*<$USARTX>::ptr()).cr3().modify(|_, w| w.dmat().set_bit());
                }
                TxDma {
                    payload: self,
                    channel,
                }
            }
        }

        impl $rxdma {
            pub fn release(mut self) -> (Rx<$USARTX>, $dmarxch) {
                self.stop();
                unsafe {
                    (*<$USARTX>::ptr())
                        .cr3()
                        .modify(|_, w| w.dmar().clear_bit());
                }
                let RxDma { payload, channel } = self;
                (payload, channel)
            }
        }

        impl $txdma {
            pub fn release(mut self) -> (Tx<$USARTX>, $dmatxch) {
                self.stop();
                unsafe {
                    (*<$USARTX>::ptr())
                        .cr3()
                        .modify(|_, w| w.dmat().clear_bit());
                }
                let TxDma { payload, channel } = self;
                (payload, channel)
            }
        }

        impl<B> crate::dma::CircReadDma<B, u8> for $rxdma
        where
            &'static mut [B; 2]: WriteBuffer<Word = u8>,
            B: 'static,
        {
            fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);

                self.channel.ch().cr().modify(|_, w| {
                    w.mem2mem().clear_bit();
                    w.pl().medium();
                    w.msize().bits8();
                    w.psize().bits8();
                    w.circ().set_bit();
                    w.dir().clear_bit()
                });

                self.start();

                CircBuffer::new(buffer, self)
            }
        }

        impl<B> crate::dma::ReadDma<B, u8> for $rxdma
        where
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    w.mem2mem().clear_bit();
                    w.pl().medium();
                    w.msize().bits8();
                    w.psize().bits8();
                    w.circ().clear_bit();
                    w.dir().clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }

        impl<B> crate::dma::WriteDma<B, u8> for $txdma
        where
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.read_buffer() };

                self.channel.set_peripheral_address(
                    unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                    false,
                );

                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);

                self.channel.ch().cr().modify(|_, w| {
                    w.mem2mem().clear_bit();
                    w.pl().medium();
                    w.msize().bits8();
                    w.psize().bits8();
                    w.circ().clear_bit();
                    w.dir().set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }
    };
}

serialdma! {
    pac::USART1,
    RxDma1,
    TxDma1,
    rx: dma1::C5,
    tx: dma1::C4
}
serialdma! {
    pac::USART2,
    RxDma2,
    TxDma2,
    rx: dma1::C6,
    tx: dma1::C7
}
serialdma! {
    pac::USART3,
    RxDma3,
    TxDma3,
    rx: dma1::C3,
    tx: dma1::C2
}
