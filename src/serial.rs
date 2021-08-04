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
//! // prelude: create handles to the peripherals and registers
//! let p = crate::pac::Peripherals::take().unwrap();
//! let cp = cortex_m::Peripherals::take().unwrap();
//! let mut flash = p.FLASH.constrain();
//! let mut rcc = p.RCC.constrain();
//! let clocks = rcc.cfgr.freeze(&mut flash.acr);
//! let mut afio = p.AFIO.constrain();
//! let mut gpioa = p.GPIOA.split();
//!
//! // USART1 on Pins A9 and A10
//! let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
//! let pin_rx = gpioa.pa10;
//! // Create an interface struct for USART1 with 9600 Baud
//! let serial = Serial::usart1(
//!     p.USART1,
//!     (pin_tx, pin_rx),
//!     &mut afio.mapr,
//!     Config::default().baudrate(9_600.bps()),
//!     clocks,
//! );
//!
//! // Switching the 'Word' type parameter for the 'Read' and 'Write' traits between u8 and u16.
//! let serial = serial.with_u16_data();
//! let serial = serial.with_u8_data();
//!
//! // Separate into tx and rx channels
//! let (mut tx, mut rx) = serial.split();
//!
//! // Switch tx to u16.
//! let mut tx = tx.with_u16_data();
//!
//! // Write data to the USART.
//! // Depending on the configuration, only the lower 7, 8, or 9 bits are used.
//! block!(tx.write(0x1FF)).ok();
//!
//! // Switch tx back to u8
//! let mut tx = tx.with_u8_data();
//!
//! // Write 'R' to the USART
//! block!(tx.write(b'R')).ok();
//!
//! // Switch rx to u16.
//! let mut rx = rx.with_u16_data();
//!
//! // Receive a data from the USART and store it in "received"
//! let received: u16 = block!(rx.read()).unwrap();
//!
//! // Switch rx back to u8.
//! let mut rx = rx.with_u8_data();
//!
//! // Receive a data from the USART and store it in "received"
//! let received: u8 = block!(rx.read()).unwrap();
//!  ```

use core::marker::PhantomData;
use core::ops::Deref;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use crate::pac::{RCC, USART1, USART2, USART3};
use core::convert::Infallible;
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use embedded_hal::serial::Write;

use crate::afio::MAPR;
use crate::dma::{dma1, CircBuffer, RxDma, Transfer, TxDma, R, W};
use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};
use crate::gpio::gpioc::{PC10, PC11};
use crate::gpio::gpiod::{PD5, PD6, PD8, PD9};
use crate::gpio::{Alternate, Input};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{Bps, U32Ext};

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

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
    const REMAP: u8;
}

impl<INMODE, OUTMODE> Pins<USART1> for (PA9<Alternate<OUTMODE>>, PA10<Input<INMODE>>) {
    const REMAP: u8 = 0;
}

impl<INMODE, OUTMODE> Pins<USART1> for (PB6<Alternate<OUTMODE>>, PB7<Input<INMODE>>) {
    const REMAP: u8 = 1;
}

impl<INMODE, OUTMODE> Pins<USART2> for (PA2<Alternate<OUTMODE>>, PA3<Input<INMODE>>) {
    const REMAP: u8 = 0;
}

impl<INMODE, OUTMODE> Pins<USART2> for (PD5<Alternate<OUTMODE>>, PD6<Input<INMODE>>) {
    const REMAP: u8 = 1;
}

impl<INMODE, OUTMODE> Pins<USART3> for (PB10<Alternate<OUTMODE>>, PB11<Input<INMODE>>) {
    const REMAP: u8 = 0;
}

impl<INMODE, OUTMODE> Pins<USART3> for (PC10<Alternate<OUTMODE>>, PC11<Input<INMODE>>) {
    const REMAP: u8 = 1;
}

impl<INMODE, OUTMODE> Pins<USART3> for (PD8<Alternate<OUTMODE>>, PD9<Input<INMODE>>) {
    const REMAP: u8 = 0b11;
}

pub enum WordLength {
    /// When parity is enabled, a word has 7 data bits + 1 parity bit.
    DataBits8,
    /// When parity is enabled, a word has 8 data bits + 1 parity bit.
    DataBits9,
}

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "0.5 stop bits"]
    STOP0P5,
    #[doc = "2 stop bits"]
    STOP2,
    #[doc = "1.5 stop bits"]
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

    pub fn wordlength_8(mut self) -> Self {
        self.wordlength = WordLength::DataBits8;
        self
    }

    pub fn wordlength_9(mut self) -> Self {
        self.wordlength = WordLength::DataBits9;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            wordlength: WordLength::DataBits8,
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
pub struct Serial<USART, PINS, WORD = u8> {
    usart: USART,
    pins: PINS,
    tx: Tx<USART, WORD>,
    rx: Rx<USART, WORD>,
}

pub trait Instance:
    crate::Sealed + Deref<Target = uart_base::RegisterBlock> + Enable + Reset + BusClock
{
    #[doc(hidden)]
    fn ptr() -> *const uart_base::RegisterBlock;
}

/// Serial receiver
pub struct Rx<USART, WORD = u8> {
    _usart: PhantomData<USART>,
    _word: PhantomData<WORD>,
}

/// Serial transmitter
pub struct Tx<USART, WORD = u8> {
    _usart: PhantomData<USART>,
    _word: PhantomData<WORD>,
}

impl<USART, WORD> Rx<USART, WORD> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USART, WORD> Tx<USART, WORD> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USART, PINS, WORD> Serial<USART, PINS, WORD>
where
    USART: Instance,
{
    fn init(self, config: Config, clocks: Clocks, remap: impl FnOnce()) -> Self {
        // enable and reset $USARTX
        let rcc = unsafe { &(*RCC::ptr()) };
        USART::enable(rcc);
        USART::reset(rcc);

        remap();
        self.apply_config(config, clocks);

        // UE: enable USART
        // RE: enable receiver
        // TE: enable transceiver
        self.usart
            .cr1
            .modify(|_r, w| w.ue().set_bit().re().set_bit().te().set_bit());

        self
    }

    fn apply_config(&self, config: Config, clocks: Clocks) {
        // Configure baud rate
        let brr = USART::clock(&clocks).0 / config.baudrate.0;
        assert!(brr >= 16, "impossible baud rate");
        self.usart.brr.write(|w| unsafe { w.bits(brr) });

        let (parity_control_enable, parity) = match config.parity {
            Parity::ParityNone => (false, false),
            Parity::ParityEven => (true, false),
            Parity::ParityOdd => (true, true),
        };
        self.usart.cr1.modify(|_r, w| {
            w.m()
                .bit(match config.wordlength {
                    WordLength::DataBits8 => false,
                    WordLength::DataBits9 => true,
                })
                .ps()
                .bit(parity)
                .pce()
                .bit(parity_control_enable)
        });

        // Configure stop bits
        let stop_bits = match config.stopbits {
            StopBits::STOP1 => 0b00,
            StopBits::STOP0P5 => 0b01,
            StopBits::STOP2 => 0b10,
            StopBits::STOP1P5 => 0b11,
        };
        self.usart.cr2.modify(|_r, w| w.stop().bits(stop_bits));
    }

    /// Reconfigure the USART instance.
    ///
    /// If a transmission is currently in progress, this returns
    /// [`nb::Error::WouldBlock`].
    pub fn reconfigure(
        &mut self,
        config: impl Into<Config>,
        clocks: Clocks,
    ) -> nb::Result<(), Infallible> {
        let sr = self.usart.sr.read();
        // if we're currently busy transmitting, we have to wait until that is
        // over -- regarding reception, we assume that the caller -- with
        // exclusive access to the Serial instance due to &mut self -- knows
        // what they're doing.
        if sr.tc().bit_is_clear() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }
        self.apply_config(config.into(), clocks);
        nb::Result::Ok(())
    }

    /// Starts listening to the USART by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
        }
    }

    /// Stops listening to the USART by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
        }
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        self.usart.sr.read().idle().bit_is_set()
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        self.usart.sr.read().txe().bit_is_set()
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        self.usart.sr.read().rxne().bit_is_set()
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*USART::ptr()).sr.read();
            let _ = (*USART::ptr()).dr.read();
        }
    }

    /// Returns ownership of the borrowed register handles
    pub fn release(self) -> (USART, PINS) {
        (self.usart, self.pins)
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and
    /// receiving (Rx)
    pub fn split(self) -> (Tx<USART, WORD>, Rx<USART, WORD>) {
        (self.tx, self.rx)
    }
}

macro_rules! hal {
    (
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $usartX_remap:ident,
            $bit:ident,
            $closure:expr,
        ),
    ) => {
        impl Instance for $USARTX {
            fn ptr() -> *const uart_base::RegisterBlock {
                <$USARTX>::ptr() as *const _
            }
        }

        $(#[$meta])*
        /// The behaviour of the functions is equal for all three USARTs.
        /// Except that they are using the corresponding USART hardware and pins.
        impl<PINS> Serial<$USARTX, PINS> {
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
            pub fn $usartX(
                usart: $USARTX,
                pins: PINS,
                mapr: &mut MAPR,
                config: impl Into<Config>,
                clocks: Clocks,
            ) -> Self
            where
                PINS: Pins<$USARTX>,
            {
                #[allow(unused_unsafe)]
                Serial { usart, pins, tx: Tx::new(), rx: Rx::new() }.init(config.into(), clocks, || {
                    mapr.modify_mapr(|_, w| unsafe {
                        #[allow(clippy::redundant_closure_call)]
                        w.$usartX_remap().$bit(($closure)(PINS::REMAP))
                    })
                })
            }
        }

    };
}

hal! {
    /// # USART1 functions
    USART1: (
        usart1,
        usart1_remap,
        bit,
        |remap| remap == 1,
    ),
}
hal! {
    /// # USART2 functions
    USART2: (
        usart2,
        usart2_remap,
        bit,
        |remap| remap == 1,
    ),
}
hal! {
    /// # USART3 functions
    USART3: (
        usart3,
        usart3_remap,
        bits,
        |remap| remap,
    ),
}

impl<USART> Tx<USART>
where
    USART: Instance,
{
    /// Start listening for transmit interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) };
    }

    /// Stop listening for transmit interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) };
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().txe().bit_is_set() }
    }
}

impl<USART> Rx<USART>
where
    USART: Instance,
{
    /// Start listening for receive interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) };
    }

    /// Stop listening for receive interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) };
    }

    /// Start listening for idle interrupt event
    pub fn listen_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) };
    }

    /// Stop listening for idle interrupt event
    pub fn unlisten_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.idleie().clear_bit()) };
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().idle().bit_is_set() }
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*USART::ptr()).sr.read();
            let _ = (*USART::ptr()).dr.read();
        }
    }
}

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

impl<USART> crate::hal::serial::Read<u8> for Rx<USART, u8>
where
    USART: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // Delegate to the Read<u16> implementation, then truncate to 8 bits
        Rx::<USART, u16>::new().read().map(|word16| word16 as u8)
    }
}

/// Reads 9-bit words from the UART/USART
///
/// If the UART/USART was configured with `WordLength::DataBits9`, the returned value will contain
/// 9 received data bits and all other bits set to zero. Otherwise, the returned value will contain
/// 8 received data bits and all other bits set to zero.
impl<USART> crate::hal::serial::Read<u16> for Rx<USART, u16>
where
    USART: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        let usart = unsafe { &*USART::ptr() };
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
            // do a read from the sr register followed by a read from the dr
            // register
            // NOTE(read_volatile) see `write_volatile` below
            unsafe {
                ptr::read_volatile(&usart.sr as *const _ as *const _);
                ptr::read_volatile(&usart.dr as *const _ as *const _);
            }
            Err(nb::Error::Other(err))
        } else {
            // Check if a byte is available
            if sr.rxne().bit_is_set() {
                // Read the received byte
                // NOTE(read_volatile) see `write_volatile` below
                Ok(unsafe { ptr::read_volatile(&usart.dr as *const _ as *const _) })
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
}

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

impl<USART> crate::hal::serial::Write<u8> for Tx<USART, u8>
where
    USART: Instance,
{
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        // Delegate to u16 version
        Tx::<USART, u16>::new().write(u16::from(word))
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // Delegate to u16 version
        Tx::<USART, u16>::new().flush()
    }
}

/// Writes 9-bit words to the UART/USART
///
/// If the UART/USART was configured with `WordLength::DataBits9`, the 9 least significant bits will
/// be transmitted and the other 7 bits will be ignored. Otherwise, the 8 least significant bits
/// will be transmitted and the other 8 bits will be ignored.
impl<USART> crate::hal::serial::Write<u16> for Tx<USART, u16>
where
    USART: Instance,
{
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let sr = unsafe { &*USART::ptr() }.sr.read();

        if sr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        let usart = unsafe { &*USART::ptr() };
        let sr = usart.sr.read();

        if sr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            unsafe { &*USART::ptr() }.dr.write(|w| w.dr().bits(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USART> crate::hal::blocking::serial::Write<u16> for Tx<USART, u16>
where
    USART: Instance,
{
    type Error = Infallible;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        for &b in buffer {
            loop {
                match self.write(b) {
                    Err(nb::Error::WouldBlock) => continue,
                    Err(nb::Error::Other(err)) => return Err(err),
                    Ok(()) => break,
                }
            }
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        loop {
            match <Self as crate::hal::serial::Write<u16>>::flush(self) {
                Ok(()) => return Ok(()),
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
    }
}

impl<USART> crate::hal::blocking::serial::Write<u8> for Tx<USART, u8>
where
    USART: Instance,
{
    type Error = Infallible;

    fn bwrite_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        for &b in bytes {
            loop {
                match self.write(b) {
                    Err(nb::Error::WouldBlock) => continue,
                    Err(nb::Error::Other(err)) => return Err(err),
                    Ok(()) => break,
                }
            }
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        loop {
            match <Self as crate::hal::serial::Write<u8>>::flush(self) {
                Ok(()) => return Ok(()),
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
    }
}

impl<USART, PINS> crate::hal::blocking::serial::Write<u16> for Serial<USART, PINS, u16>
where
    USART: Instance,
{
    type Error = Infallible;

    fn bwrite_all(&mut self, bytes: &[u16]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(bytes)
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

    fn bwrite_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(bytes)
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

impl<USART> core::fmt::Write for Tx<USART>
where
    Tx<USART>: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| core::fmt::Error)
    }
}

pub type Rx1 = Rx<USART1>;
pub type Tx1 = Tx<USART1>;
pub type Rx2 = Rx<USART2>;
pub type Tx2 = Tx<USART2>;
pub type Rx3 = Rx<USART3>;
pub type Tx3 = Tx<USART3>;

pub type Rx1_16 = Rx<USART1, u16>;
pub type Tx1_16 = Tx<USART1, u16>;
pub type Rx2_16 = Rx<USART2, u16>;
pub type Tx2_16 = Tx<USART2, u16>;
pub type Rx3_16 = Rx<USART3, u16>;
pub type Tx3_16 = Tx<USART3, u16>;

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
