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
//! ## Alternate function remapping
//!
//! ### USART1
//!
//! | Function \ Remap | 0 (default) | 1    |
//! |------------------|-------------|------|
//! | TX (A-PP/OD)     | PA9         | PB6  |
//! | RX (I-F/PU)      | PA10        | PB7  |
//!
//! ### USART2
//!
//! | Function \ Remap | 0 (default) | 1    |
//! |------------------|-------------|------|
//! | TX (A-PP/OD)     | PA2         | PD5  |
//! | RX (I-F/PU)      | PA3         | PD6  |
//!
//! ### USART3
//!
//! | Function \ Remap | 0 (default) | 1    | 2   |
//! |------------------|-------------|------|-----|
//! | TX (A-PP/OD)     | PB10        | PC10 | PD8 |
//! | RX (I-F/PU)      | PB11        | PC11 | PD9 |
//!
//! ### UART4/UART5
//!
//! | Function     | UART4 | UART5 |
//! |--------------|------ | ------|
//! | TX (A-PP/OD) | PC10  | PC12  |
//! | RX (I-F/PU)  | PC11  | PD2   |
//!
//! ## Example usage:
//!
//!  ```rust
//! let dp = stm32f1xx_hal::Peripherals::take().unwrap();
//! let mut flash = dp.FLASH.constrain();
//! let mut rcc = dp.RCC.constrain();
//! let clocks = rcc.freeze(rcc::Config::default(), &mut flash.acr);
//! let mut afio = dp.AFIO.constrain(&mut rcc);
//! let mut gpioa = dp.GPIOA.split(&mut rcc);
//!
//! // USART1 on Pins A9 and A10
//! let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
//! let pin_rx = gpioa.pa10;
//! // Create an interface struct for USART1 with 9600 Baud
//! let serial = Serial::new(
//!     dp.USART1.remap(&mut afio.mapr),
//!     (pin_tx, pin_rx),
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

use core::ops::Deref;
use core::sync::atomic::{self, Ordering};
use embedded_dma::{ReadBuffer, WriteBuffer};
use enumflags2::BitFlags;

use crate::afio::{self, RInto, Rmp};
use crate::gpio::{Floating, PushPull, UpMode};
use crate::pac::{self};
use crate::rcc::{BusClock, Clocks, Enable, Rcc, Reset};

mod config;
mod dma;
mod hal_02;
mod hal_1;
mod rbext;
pub use config::*;
pub use dma::*;
use rbext::RBExt;

use crate::pacext::uart::{SrR, UartRB};

pub trait SerialExt: Sized + Instance {
    fn serial<Otype, PULL: UpMode>(
        self,
        pins: (
            impl RInto<Self::Tx<Otype>, 0>,
            impl RInto<Self::Rx<PULL>, 0>,
        ),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Serial<Self, Otype, PULL>;
    fn tx<Otype>(
        self,
        tx_pin: impl RInto<Self::Tx<Otype>, 0>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Tx<Self>;
    fn rx<PULL: UpMode>(
        self,
        rx_pin: impl RInto<Self::Rx<PULL>, 0>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Rx<Self>;
}

impl<USART: Instance> SerialExt for USART {
    fn serial<Otype, PULL: UpMode>(
        self,
        pins: (
            impl RInto<Self::Tx<Otype>, 0>,
            impl RInto<Self::Rx<PULL>, 0>,
        ),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Serial<Self, Otype, PULL> {
        Serial::new(self, pins, config, rcc)
    }
    fn tx<Otype>(
        self,
        tx_pin: impl RInto<Self::Tx<Otype>, 0>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Tx<Self> {
        Serial::tx(self, tx_pin, config, rcc)
    }
    fn rx<PULL: UpMode>(
        self,
        rx_pin: impl RInto<Self::Rx<PULL>, 0>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Rx<Self> {
        Serial::rx(self, rx_pin, config, rcc)
    }
}

pub trait Instance:
    crate::Sealed
    + crate::Ptr<RB: RBExt>
    + crate::Steal
    + Deref<Target = Self::RB>
    + Enable
    + Reset
    + BusClock
    + afio::SerialAsync
{
}

macro_rules! inst {
    ($($USARTX:ty;)+) => {
        $(
            impl Instance for $USARTX { }
        )+
    };
}

inst! {
    pac::USART1;
    pac::USART2;
    pac::USART3;
}
#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
inst! {
    pac::UART4;
    pac::UART5;
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

/// UART interrupt events
#[enumflags2::bitflags]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum Event {
    /// IDLE interrupt enable
    Idle = 1 << 4,
    /// RXNE interrupt enable
    RxNotEmpty = 1 << 5,
    /// Transmission complete interrupt enable
    TransmissionComplete = 1 << 6,
    /// TXE interrupt enable
    TxEmpty = 1 << 7,
    /// PE interrupt enable
    ParityError = 1 << 8,
}

/// UART interrupt events
#[enumflags2::bitflags]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum RxEvent {
    /// IDLE interrupt enable
    Idle = 1 << 4,
    /// RXNE interrupt enable
    RxNotEmpty = 1 << 5,
    /// PE interrupt enable
    ParityError = 1 << 8,
}

/// UART interrupt events
#[enumflags2::bitflags]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum TxEvent {
    /// Transmission complete interrupt enable
    TransmissionComplete = 1 << 6,
    /// TXE interrupt enable
    TxEmpty = 1 << 7,
}

/// UART/USART status flags
#[enumflags2::bitflags]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum Flag {
    /// Parity error
    ParityError = 1 << 0,
    /// Framing error
    FramingError = 1 << 1,
    /// Noise detected flag
    Noise = 1 << 2,
    /// Overrun error
    Overrun = 1 << 3,
    /// IDLE line detected
    Idle = 1 << 4,
    /// Read data register not empty
    RxNotEmpty = 1 << 5,
    /// Transmission complete
    TransmissionComplete = 1 << 6,
    /// Transmit data register empty
    TxEmpty = 1 << 7,
    /// LIN break detection flag
    LinBreak = 1 << 8,
    /// CTS flag
    Cts = 1 << 9,
}

/// UART clearable flags
#[enumflags2::bitflags]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum CFlag {
    /// Read data register not empty
    RxNotEmpty = 1 << 5,
    /// Transmission complete
    TransmissionComplete = 1 << 6,
    /// LIN break detection flag
    LinBreak = 1 << 8,
    /// CTS flag
    Cts = 1 << 9,
}

/// Trait for [`Rx`] interrupt handling.
pub trait RxISR {
    /// Return true if the line idle status is set
    fn is_idle(&self) -> bool;

    /// Return true if the rx register is not empty (and can be read)
    fn is_rx_not_empty(&self) -> bool;

    /// Clear idle line interrupt flag
    fn clear_idle_interrupt(&self);
}

/// Trait for [`Tx`] interrupt handling.
pub trait TxISR {
    /// Return true if the tx register is empty (and can accept data)
    fn is_tx_empty(&self) -> bool;
}

/// Serial abstraction
pub struct Serial<USART: Instance, Otype = PushPull, PULL = Floating> {
    pub tx: Tx<USART>,
    pub rx: Rx<USART>,
    #[allow(clippy::type_complexity)]
    pub token: ReleaseToken<(Option<USART::Tx<Otype>>, Option<USART::Rx<PULL>>)>,
}

/// Serial transmitter
pub struct Tx<USART> {
    usart: USART,
}

/// Serial receiver
pub struct Rx<USART> {
    usart: USART,
}

/// Stores data for release
pub struct ReleaseToken<PINS> {
    pins: PINS,
}

impl<USART: Instance, const R: u8> Rmp<USART, R> {
    pub fn serial<Otype, PULL: UpMode>(
        self,
        pins: (
            impl RInto<USART::Tx<Otype>, R>,
            impl RInto<USART::Rx<PULL>, R>,
        ),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Serial<USART, Otype, PULL> {
        Serial::_new(self.0, (Some(pins.0), Some(pins.1)), config, rcc)
    }
    pub fn tx<Otype>(
        self,
        tx_pin: impl RInto<USART::Tx<Otype>, R>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Tx<USART> {
        Serial::_new(
            self.0,
            (Some(tx_pin), None::<USART::Rx<Floating>>),
            config,
            rcc,
        )
        .split()
        .0
    }
    pub fn rx<PULL: UpMode>(
        self,
        rx_pin: impl RInto<USART::Rx<PULL>, R>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Rx<USART> {
        Serial::_new(
            self.0,
            (None::<USART::Tx<Floating>>, Some(rx_pin)),
            config,
            rcc,
        )
        .split()
        .1
    }
}

impl<USART: Instance, Otype> Serial<USART, Otype, Floating> {
    pub fn tx<const R: u8>(
        usart: impl Into<Rmp<USART, R>>,
        tx_pin: impl RInto<USART::Tx<Otype>, R>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Tx<USART> {
        usart.into().tx(tx_pin, config, rcc)
    }
}

impl<USART: Instance, PULL: UpMode> Serial<USART, PushPull, PULL> {
    pub fn rx<const R: u8>(
        usart: impl Into<Rmp<USART, R>>,
        rx_pin: impl RInto<USART::Rx<PULL>, R>,
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Rx<USART> {
        usart.into().rx(rx_pin, config, rcc)
    }
}

impl<USART: Instance, Otype, PULL: UpMode> Serial<USART, Otype, PULL> {
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
    pub fn new<const R: u8>(
        usart: impl Into<Rmp<USART, R>>,
        pins: (
            impl RInto<USART::Tx<Otype>, R>,
            impl RInto<USART::Rx<PULL>, R>,
        ),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Self {
        usart.into().serial(pins, config, rcc)
    }

    fn _new<const R: u8>(
        usart: USART,
        pins: (
            Option<impl RInto<USART::Tx<Otype>, R>>,
            Option<impl RInto<USART::Rx<PULL>, R>>,
        ),
        config: impl Into<Config>,
        rcc: &mut Rcc,
    ) -> Self {
        // Enable and reset USART
        USART::enable(rcc);
        USART::reset(rcc);

        apply_config::<USART>(config.into(), &rcc.clocks);

        let pins = (pins.0.map(RInto::rinto), pins.1.map(RInto::rinto));

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
            tx: Tx { usart },
            rx: Rx {
                usart: unsafe { USART::steal() },
            },
            token: ReleaseToken { pins },
        }
    }
}

impl<USART: Instance, Otype, PULL> Serial<USART, Otype, PULL> {
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
    /// let serial = Serial::new(
    ///     usart.remap(&mut afio.mapr),
    ///     (tx_pin, rx_pin),
    ///     9600.bps(),
    ///     &mut rcc);
    ///
    /// // Or
    ///
    /// let serial = usart
    ///     .remap(&mut afio.mapr)
    ///     .serial((tx_pin, rx_pin), 9600.bps(), &mut rcc);
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
    pub fn release(self) -> (USART, (Option<USART::Tx<Otype>>, Option<USART::Rx<PULL>>)) {
        (self.tx.usart, self.token.pins)
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
    usart.brr().write(|w| unsafe { w.bits(brr as u16) });

    // Configure word
    usart.cr1().modify(|_r, w| {
        w.m().bit(match config.wordlength {
            WordLength::DataBits8 => false,
            WordLength::DataBits9 => true,
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
    usart.set_stopbits(config.stopbits);
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
        if self.usart.sr().read().txe().bit_is_set() {
            self.usart.dr().write(|w| w.dr().set(word));
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
        if self.usart.sr().read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn bflush(&mut self) -> Result<(), Error> {
        nb::block!(self.flush())
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
        let sr = self.usart.sr().read();

        // Check for any errors
        let err = if sr.pe().bit_is_set() {
            Some(Error::Parity)
        } else if sr.fe().bit_is_set() {
            Some(Error::FrameFormat)
        } else if sr.nf().bit_is_set() {
            Some(Error::Noise)
        } else if sr.ore().bit_is_set() {
            Some(Error::Overrun)
        } else {
            None
        };

        if let Some(err) = err {
            // Some error occurred. In order to clear that error flag, you have to
            // do a read from the sr register followed by a read from the dr register.
            let _ = self.usart.sr().read();
            let _ = self.usart.dr().read();
            Err(nb::Error::Other(err))
        } else {
            // Check if a byte is available
            if sr.rxne().bit_is_set() {
                // Read the received byte
                Ok(self.usart.dr().read().dr().bits())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    pub fn read_u8(&mut self) -> nb::Result<u8, Error> {
        self.usart.read_u8()
    }

    // Synonym to `read_u8`
    pub fn read(&mut self) -> nb::Result<u8, Error> {
        self.usart.read_u8()
    }
}

impl<UART: Instance, Otype, PULL> RxISR for Serial<UART, Otype, PULL>
where
    Rx<UART>: RxISR,
{
    fn is_idle(&self) -> bool {
        self.rx.is_idle()
    }

    fn is_rx_not_empty(&self) -> bool {
        self.rx.is_rx_not_empty()
    }

    /// This clears `Idle`, `Overrun`, `Noise`, `FrameError` and `ParityError` flags
    fn clear_idle_interrupt(&self) {
        self.rx.clear_idle_interrupt();
    }
}

impl<UART: Instance> RxISR for Rx<UART> {
    fn is_idle(&self) -> bool {
        self.usart.is_idle()
    }

    fn is_rx_not_empty(&self) -> bool {
        self.usart.is_rx_not_empty()
    }

    /// This clears `Idle`, `Overrun`, `Noise`, `FrameError` and `ParityError` flags
    fn clear_idle_interrupt(&self) {
        self.usart.clear_idle_interrupt();
    }
}

impl<UART: Instance> TxISR for Serial<UART>
where
    Tx<UART>: TxISR,
{
    fn is_tx_empty(&self) -> bool {
        self.tx.is_tx_empty()
    }
}

impl<UART: Instance> TxISR for Tx<UART> {
    fn is_tx_empty(&self) -> bool {
        self.usart.is_tx_empty()
    }
}

impl<UART: Instance, Otype, PULL> crate::ClearFlags for Serial<UART, Otype, PULL> {
    type Flag = CFlag;

    #[inline(always)]
    fn clear_flags(&mut self, flags: impl Into<BitFlags<Self::Flag>>) {
        self.tx.usart.clear_flags(flags.into())
    }
}

impl<UART: Instance, Otype, PULL> crate::ReadFlags for Serial<UART, Otype, PULL> {
    type Flag = Flag;

    #[inline(always)]
    fn flags(&self) -> BitFlags<Self::Flag> {
        self.tx.usart.flags()
    }
}

impl<UART: Instance, Otype, PULL> crate::Listen for Serial<UART, Otype, PULL> {
    type Event = Event;

    #[inline(always)]
    fn listen_event(
        &mut self,
        disable: Option<BitFlags<Self::Event>>,
        enable: Option<BitFlags<Self::Event>>,
    ) {
        self.tx.usart.listen_event(disable, enable)
    }
}

impl<UART: Instance> crate::Listen for Rx<UART> {
    type Event = RxEvent;

    #[inline(always)]
    fn listen_event(
        &mut self,
        disable: Option<BitFlags<Self::Event>>,
        enable: Option<BitFlags<Self::Event>>,
    ) {
        self.usart.listen_rx(disable, enable)
    }
}

impl<UART: Instance> crate::Listen for Tx<UART> {
    type Event = TxEvent;

    #[inline(always)]
    fn listen_event(
        &mut self,
        disable: Option<BitFlags<Self::Event>>,
        enable: Option<BitFlags<Self::Event>>,
    ) {
        self.usart.listen_tx(disable, enable)
    }
}

impl<USART: Instance, Otype, PULL> core::fmt::Write for Serial<USART, Otype, PULL> {
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
