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
//! use stm32f1xx_hal::{pac, serial::Serial};
//!
//! let p = pac::Peripherals::take().unwrap();
//! let mut flash = p.FLASH.constrain();
//! let mut rcc = p.RCC.constrain();
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//! let mut afio = p.AFIO.constrain();
//!
//! // USART1 on Pins PA9 and PA10.
//! let mut gpioa = p.GPIOA.split();
//! let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
//! let pin_rx = gpioa.pa10;
//!
//! // Set up the usart device. Take ownership over the USART register and tx/rx pins.
//! let serial = Serial::new(
//!     p.USART1,
//!     (pin_tx, pin_rx),
//!     &mut afio.mapr,
//!     Config::default()
//!         .baudrate(9_600.bps())
//!         .wordlength_9bits()
//!         .parity_none(),
//!     &clocks,
//! );
//!
//! // Separate into tx and rx channels.
//! let (mut tx, mut rx) = serial.split();
//!
//! // Write data (as u16) to the USART.
//! // Depending on the configuration, only the lower 7, 8, or 9 bits are used.
//! block!(tx.write(0x1FFu16)).unwrap();
//!
//! // Write 'R' (as u8) to the USART
//! block!(tx.write(b'R')).unwrap();
//!
//! // Receive a data (as u16) from the USART and store it in "received".
//! let received: u16 = block!(rx.read()).unwrap();
//!
//! // Receive a data (as u8) from the USART and store it in "received".
//! let received: u8 = block!(rx.read()).unwrap();
//!  ```

#![deny(unsafe_op_in_unsafe_fn)]

use core::convert::Infallible;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{self, Ordering};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use embedded_hal::serial::Write;

use crate::afio::MAPR;
use crate::dma::{dma1, CircBuffer, RxDma, Transfer, TxDma, R, W};
use crate::gpio::{self, Alternate, Input};
use crate::pac::{RCC, USART1, USART2, USART3};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{Bps, U32Ext};

/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

// USART REMAPPING, see: https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
// Section 9.3.8
pub trait Pins<USART> {
    fn remap(mapr: &mut MAPR);
}

macro_rules! remap {
    ($($USART:ty, $TX:ident, $RX:ident => { $remapex:expr };)+) => {
        $(
            impl<INMODE, OUTMODE> Pins<$USART> for (gpio::$TX<Alternate<OUTMODE>>, gpio::$RX<Input<INMODE>>) {
                fn remap(mapr: &mut MAPR) {
                    mapr.modify_mapr($remapex);
                }
            }
        )+
    }
}

remap!(
    USART1, PA9, PA10 => { |_, w| w.usart1_remap().bit(false) };
    USART1, PB6, PB7  => { |_, w| w.usart1_remap().bit(true) };

    USART2, PA2, PA3 => { |_, w| w.usart2_remap().bit(false) };
    USART2, PD5, PD6 => { |_, w| w.usart2_remap().bit(true) };

    USART3, PB10, PB11 => { |_, w| unsafe { w.usart3_remap().bits(0b00)} };
    USART3, PC10, PC11 => { |_, w| unsafe { w.usart3_remap().bits(0b01)} };
    USART3, PD8, PD9 => { |_, w| unsafe { w.usart3_remap().bits(0b11)} };
);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordLength {
    /// When parity is enabled, a word has 7 data bits + 1 parity bit,
    /// otherwise 8 data bits.
    Bits8,
    /// When parity is enabled, a word has 8 data bits + 1 parity bit,
    /// otherwise 9 data bits.
    Bits9,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

#[derive(Debug, Clone, Copy, PartialEq)]
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

use crate::pac::usart1 as uart_base;

/// Serial abstraction
pub struct Serial<USART: Instance, TxPin, RxPin> {
    pub tx: Tx<USART>,
    pub rx: Rx<USART>,
    pub pin_info: PinInfo<USART, TxPin, RxPin>,
}

/// Stores information about used the tx/rx pins
pub struct PinInfo<USART, TxPin, RxPin> {
    _usart: PhantomData<USART>,
    tx: TxPin,
    rx: RxPin,
}

pub trait Instance:
    crate::Sealed + Deref<Target = uart_base::RegisterBlock> + Enable + Reset + BusClock
{
    #[doc(hidden)]
    unsafe fn steal() -> Self;
}

/// Serial receiver
pub struct Rx<USART>
where
    USART: Instance,
    Rx<USART>: embedded_hal::serial::Read<u16, Error = Error>,
{
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART>
where
    USART: Instance,
    Tx<USART>: embedded_hal::serial::Write<u8, Error = Infallible>
        + embedded_hal::serial::Write<u16, Error = Infallible>,
{
    _usart: PhantomData<USART>,
}

impl<USART: Instance> Rx<USART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

impl<USART: Instance> Tx<USART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

pub trait SerialExt: Sized + Instance {
    fn serial<TxPin, RxPin>(
        self,
        pins: (TxPin, RxPin),
        mapr: &mut MAPR,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self, TxPin, RxPin>
    where
        (TxPin, RxPin): Pins<Self>;
}

impl<USART: Instance> SerialExt for USART {
    fn serial<TxPin, RxPin>(
        self,
        pins: (TxPin, RxPin),
        mapr: &mut MAPR,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self, TxPin, RxPin>
    where
        (TxPin, RxPin): Pins<Self>,
    {
        Serial::new(self, pins, mapr, config, clocks)
    }
}

impl<USART, TxPin, RxPin> Serial<USART, TxPin, RxPin>
where
    USART: Instance,
    (TxPin, RxPin): Pins<USART>,
{
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
    /// `MAPR` register is used to map the USART to the corresponding pins.
    pub fn new(
        usart: USART,
        pins: (TxPin, RxPin),
        mapr: &mut MAPR,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        USART::enable(rcc);
        USART::reset(rcc);

        <(TxPin, RxPin) as Pins<USART>>::remap(mapr);

        apply_config::<USART>(&usart, config.into(), clocks);

        // UE: enable USART
        // RE: enable receiver
        // TE: enable transceiver
        usart.cr1.modify(|_r, w| {
            w.ue().set_bit();
            w.re().set_bit();
            w.te().set_bit();
            w
        });

        Self {
            tx: Tx::new(),
            rx: Rx::new(),
            pin_info: PinInfo {
                _usart: PhantomData,
                tx: pins.0,
                rx: pins.1,
            },
        }
    }
}
/*impl<USART, PINS, WORD> Serial<USART, PINS, WORD>
where
    USART: Instance,
{*/
fn apply_config<USART: Instance>(usart: &USART, config: Config, clocks: &Clocks) {
    // Configure baud rate
    let brr = USART::clock(clocks).raw() / config.baudrate.0;
    assert!(brr >= 16, "impossible baud rate");
    usart.brr.write(|w| unsafe { w.bits(brr) });

    // Configure word
    let (parity_is_used, parity_is_odd) = match config.parity {
        Parity::ParityNone => (false, false),
        Parity::ParityEven => (true, false),
        Parity::ParityOdd => (true, true),
    };
    usart.cr1.modify(|_r, w| {
        w.m().bit(match config.wordlength {
            WordLength::Bits8 => false,
            WordLength::Bits9 => true,
        });
        w.ps().bit(parity_is_odd);
        w.pce().bit(parity_is_used)
    });

    // Configure stop bits
    let stop_bits = match config.stopbits {
        StopBits::STOP1 => 0b00,
        StopBits::STOP0P5 => 0b01,
        StopBits::STOP2 => 0b10,
        StopBits::STOP1P5 => 0b11,
    };
    usart.cr2.modify(|_r, w| w.stop().bits(stop_bits));
}

/// Reconfigure the USART instance.
///
/// If a transmission is currently in progress, this returns
/// [`nb::Error::WouldBlock`].
pub fn reconfigure<USART: Instance>(
    _tx: &mut Tx<USART>,
    _rx: &mut Rx<USART>,
    config: impl Into<Config>,
    clocks: &Clocks,
) -> nb::Result<(), Infallible> {
    let usart = unsafe { USART::steal() };

    // if we're currently busy transmitting, we have to wait until that is
    // over -- regarding reception, we assume that the caller -- with
    // exclusive access to the Serial instance due to &mut self -- knows
    // what they're doing.
    if usart.sr.read().tc().bit_is_clear() {
        return nb::Result::Err(nb::Error::WouldBlock);
    }
    apply_config(&usart, config.into(), clocks);
    nb::Result::Ok(())
}

impl<USART, TxPin, RxPin> Serial<USART, TxPin, RxPin>
where
    USART: Instance,
    (TxPin, RxPin): Pins<USART>,
{
    /// Returns ownership of the borrowed register handles
    pub fn release(self) -> (USART, (TxPin, RxPin)) {
        let usart = unsafe { USART::steal() };
        (usart, (self.pin_info.tx, self.pin_info.rx))
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and
    /// receiving (Rx)
    pub fn split(self) -> (Tx<USART>, Rx<USART>) {
        (self.tx, self.rx)
    }

    /// Reconfigure the USART instance.
    ///
    /// If a transmission is currently in progress, this returns
    /// [`nb::Error::WouldBlock`].
    pub fn reconfigure(
        &mut self,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> nb::Result<(), Infallible> {
        reconfigure(&mut self.tx, &mut self.rx, config, clocks)
    }
}

macro_rules! hal {
    ($USARTX:ident) => {
        impl Instance for $USARTX {
            unsafe fn steal() -> Self {
                unsafe { crate::pac::Peripherals::steal().$USARTX }
            }
        }
    };
}

hal!(USART1);
hal!(USART2);
hal!(USART3);

impl<USART: Instance> Tx<USART> {
    /// Start listening for transmit interrupt event
    pub fn listen(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.txeie().set_bit()) };
    }

    /// Stop listening for transmit interrupt event
    pub fn unlisten(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.txeie().clear_bit()) };
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { USART::steal().sr.read().txe().bit_is_set() }
    }

    pub fn is_tx_complete(&self) -> bool {
        unsafe { USART::steal().sr.read().tc().bit_is_set() }
    }
}

impl<USART: Instance> Rx<USART> {
    /// Start listening for receive interrupt event
    pub fn listen(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.rxneie().set_bit()) };
    }

    /// Stop listening for receive interrupt event
    pub fn unlisten(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.rxneie().clear_bit()) };
    }

    /// Start listening for idle interrupt event
    pub fn listen_idle(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.idleie().set_bit()) };
    }

    /// Stop listening for idle interrupt event
    pub fn unlisten_idle(&mut self) {
        unsafe { USART::steal().cr1.modify(|_, w| w.idleie().clear_bit()) };
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { USART::steal().sr.read().idle().bit_is_set() }
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { USART::steal().sr.read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        let usart = unsafe { USART::steal() };
        let _ = usart.sr.read();
        let _ = usart.dr.read();
    }
}

/*
impl<USART, PINS> Serial<USART, PINS, u8>
where
    USART: Instance,
{
    /// Converts this Serial into a version that can read and write `u16` values instead of `u8`s
    ///
    /// This can be used with a word length of 9 bits.
    pub fn with_u16_data(self) -> Serial<USART, PINS, u16> {
        Serial {
            usart: self.usart,
            pins: self.pins,
            tx: Tx::new(),
            rx: Rx::new(),
        }
    }
}

impl<USART, PINS> Serial<USART, PINS, u16>
where
    USART: Instance,
{
    /// Converts this Serial into a version that can read and write `u8` values instead of `u16`s
    ///
    /// This can be used with a word length of 8 bits.
    pub fn with_u8_data(self) -> Serial<USART, PINS, u8> {
        Serial {
            usart: self.usart,
            pins: self.pins,
            tx: Tx::new(),
            rx: Rx::new(),
        }
    }
}

impl<USARTX> Rx<USARTX, u8> {
    pub fn with_u16_data(self) -> Rx<USARTX, u16> {
        Rx::new()
    }
}

impl<USARTX> Rx<USARTX, u16> {
    pub fn with_u8_data(self) -> Rx<USARTX, u8> {
        Rx::new()
    }
}

impl<USARTX> Tx<USARTX, u8> {
    pub fn with_u16_data(self) -> Tx<USARTX, u16> {
        Tx::new()
    }
}

impl<USARTX> Tx<USARTX, u16> {
    pub fn with_u8_data(self) -> Tx<USARTX, u8> {
        Tx::new()
    }
}

impl<USART, PINS, WORD> crate::hal::serial::Read<WORD> for Serial<USART, PINS, WORD>
where
    USART: Instance,
    Rx<USART, WORD>: crate::hal::serial::Read<WORD, Error = Error>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<WORD, Error> {
        self.rx.read()
    }
}
*/

impl<USART: Instance> crate::hal::serial::Read<u8> for Rx<USART> {
    type Error = Error;

    #[inline(always)]
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        Self::read_16().map(|word16| word16 as u8)
    }
}

/// Reads 9-bit words from the UART/USART
///
/// If the UART/USART was configured with `WordLength::Bits9`, the returned value will contain
/// 9 received data bits and all other bits set to zero. Otherwise, the returned value will contain
/// 8 received data bits and all other bits set to zero.
impl<USART: Instance> crate::hal::serial::Read<u16> for Rx<USART> {
    type Error = Error;

    #[inline(always)]
    fn read(&mut self) -> nb::Result<u16, Error> {
        Self::read_16()
    }
}

impl<USART: Instance> Rx<USART> {
    pub fn read_16() -> nb::Result<u16, Error> {
        let usart = unsafe { USART::steal() };
        let sr = usart.sr.read();

        // Check for any errors
        let err = if sr.pe().bit_is_set() {
            Some(Error::Parity)
        } else if sr.fe().bit_is_set() {
            Some(Error::Framing)
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
            let _ = usart.sr.read();
            let _ = usart.dr.read();
            Err(nb::Error::Other(err))
        } else {
            // Check if a byte is available
            if sr.rxne().bit_is_set() {
                // Read the received byte
                Ok(usart.dr.read().dr().bits())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
}

/*
impl<USART, PINS, WORD> crate::hal::serial::Write<WORD> for Serial<USART, PINS, WORD>
where
    USART: Instance,
    Tx<USART, WORD>: crate::hal::serial::Write<WORD, Error = Infallible>,
{
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }

    fn write(&mut self, byte: WORD) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }
}
*/

impl<USART: Instance> crate::hal::serial::Write<u8> for Tx<USART> {
    type Error = Infallible;

    #[inline(always)]
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_16(word as u16)
    }

    #[inline(always)]
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush()
    }
}

/// Writes 9-bit words to the UART/USART
///
/// If the UART/USART was configured with `WordLength::Bits9`, the 9 least significant bits will
/// be transmitted and the other 7 bits will be ignored. Otherwise, the 8 least significant bits
/// will be transmitted and the other 8 bits will be ignored.
impl<USART: Instance> crate::hal::serial::Write<u16> for Tx<USART> {
    type Error = Infallible;

    #[inline(always)]
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush()
    }

    #[inline(always)]
    fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        self.write_16(word)
    }
}

impl<USART: Instance> Tx<USART> {
    pub fn flush(&mut self) -> nb::Result<(), Infallible> {
        let usart = unsafe { USART::steal() };

        if usart.sr.read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn write_16(&mut self, word: u16) -> nb::Result<(), Infallible> {
        let usart = unsafe { USART::steal() };

        if usart.sr.read().txe().bit_is_set() {
            usart.dr.write(|w| w.dr().bits(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USART: Instance> crate::hal::blocking::serial::Write<u16> for Tx<USART> {
    type Error = Infallible;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        for &w in buffer {
            nb::block!(<Self as crate::hal::serial::Write<u16>>::write(self, w))?;
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        nb::block!(<Self as crate::hal::serial::Write<u16>>::flush(self))
    }
}

impl<USART: Instance> crate::hal::blocking::serial::Write<u8> for Tx<USART> {
    type Error = Infallible;

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        for &w in buffer {
            nb::block!(<Self as crate::hal::serial::Write<u8>>::write(self, w))?;
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        nb::block!(<Self as crate::hal::serial::Write<u8>>::flush(self))
    }
}

/*
impl<USART, PINS> crate::hal::blocking::serial::Write<u16> for Serial<USART, PINS, u16>
where
    USART: Instance,
{
    type Error = Infallible;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}

impl<USART, PINS> crate::hal::blocking::serial::Write<u8> for Serial<USART, PINS, u8>
where
    USART: Instance,
{
    type Error = Infallible;

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(buffer)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}

impl<USART, PINS> core::fmt::Write for Serial<USART, PINS>
where
    Tx<USART>: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}
*/

impl<USART: Instance> core::fmt::Write for Tx<USART> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| core::fmt::Error)
    }
}

use crate::dma::{Receive, TransferPayload, Transmit};

macro_rules! serialdma {
    ($(
        $USARTX:ident: (
            $rxdma:ident,
            $txdma:ident,
            $dmarxch:ty,
            $dmatxch:ty,
        ),
    )+) => {
        $(
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
                    unsafe { (*$USARTX::ptr()).cr3.modify(|_, w| w.dmar().set_bit()); }
                    RxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl Tx<$USARTX> {
                pub fn with_dma(self, channel: $dmatxch) -> $txdma {
                    unsafe { (*$USARTX::ptr()).cr3.modify(|_, w| w.dmat().set_bit()); }
                    TxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl $rxdma {
                #[deprecated(since = "0.7.1", note = "Please use release instead")]
                pub fn split(self) -> (Rx<$USARTX>, $dmarxch) {
                    self.release()
                }
                pub fn release(mut self) -> (Rx<$USARTX>, $dmarxch) {
                    self.stop();
                    unsafe { (*$USARTX::ptr()).cr3.modify(|_, w| w.dmar().clear_bit()); }
                    let RxDma {payload, channel} = self;
                    (
                        payload,
                        channel
                    )
                }
            }

            impl $txdma {
                #[deprecated(since = "0.7.1", note = "Please use release instead")]
                pub fn split(self) -> (Tx<$USARTX>, $dmatxch) {
                    self.release()
                }
                pub fn release(mut self) -> (Tx<$USARTX>, $dmatxch) {
                    self.stop();
                    unsafe { (*$USARTX::ptr()).cr3.modify(|_, w| w.dmat().clear_bit()); }
                    let TxDma {payload, channel} = self;
                    (
                        payload,
                        channel,
                    )
                }
            }

            impl<B> crate::dma::CircReadDma<B, u8> for $rxdma
            where
                &'static mut [B; 2]: StaticWriteBuffer<Word = u8>,
                B: 'static,
            {
                fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.static_write_buffer() };
                    self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);

                    self.channel.ch().cr.modify(|_, w| { w
                        .mem2mem() .clear_bit()
                        .pl()      .medium()
                        .msize()   .bits8()
                        .psize()   .bits8()
                        .circ()    .set_bit()
                        .dir()     .clear_bit()
                    });

                    self.start();

                    CircBuffer::new(buffer, self)
                }
            }

            impl<B> crate::dma::ReadDma<B, u8> for $rxdma
            where
                B: StaticWriteBuffer<Word = u8>,
            {
                fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.static_write_buffer() };
                    self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| { w
                        .mem2mem() .clear_bit()
                        .pl()      .medium()
                        .msize()   .bits8()
                        .psize()   .bits8()
                        .circ()    .clear_bit()
                        .dir()     .clear_bit()
                    });
                    self.start();

                    Transfer::w(buffer, self)
                }
            }

            impl<B> crate::dma::WriteDma<B, u8> for $txdma
            where
                B: StaticReadBuffer<Word = u8>,
            {
                fn write(mut self, buffer: B) -> Transfer<R, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.static_read_buffer() };

                    self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);

                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

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
        )+
    }
}

serialdma! {
    USART1: (
        RxDma1,
        TxDma1,
        dma1::C5,
        dma1::C4,
    ),
    USART2: (
        RxDma2,
        TxDma2,
        dma1::C6,
        dma1::C7,
    ),
    USART3: (
        RxDma3,
        TxDma3,
        dma1::C3,
        dma1::C2,
    ),
}
