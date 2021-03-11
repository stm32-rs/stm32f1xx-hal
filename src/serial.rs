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
//! let mut afio = p.AFIO.constrain(&mut rcc.apb2);
//! let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
//!
//! // USART1 on Pins A9 and A10
//! let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
//! let pin_rx = gpioa.pa10;
//! // Create an interface struct for USART1 with 9600 Baud
//! let serial = Serial::usart1(
//!     p.USART1,
//!     (pin_tx, pin_rx),
//!     &mut afio.mapr,
//!     Config::default().baudrate(9_600.bps()).wordlength_9bits(),
//!     clocks,
//!     &mut rcc.apb2,
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
use core::sync::atomic::{self, Ordering};

use crate::pac::{USART1, USART2, USART3};
use core::convert::Infallible;
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use embedded_hal::serial::Write;

use crate::afio::MAPR;
use crate::dma::{dma1, CircBuffer, RxDma, Transfer, TxDma, R, W};
use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};
use crate::gpio::gpioc::{PC10, PC11};
use crate::gpio::gpiod::{PD5, PD6, PD8, PD9};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{sealed::RccBus, Clocks, Enable, GetBusFreq, Reset, APB1, APB2};
use crate::time::{Bps, U32Ext};

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
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

impl Pins<USART1> for (PA9<Alternate<PushPull>>, PA10<Input<Floating>>) {
    const REMAP: u8 = 0;
}

impl Pins<USART1> for (PB6<Alternate<PushPull>>, PB7<Input<Floating>>) {
    const REMAP: u8 = 1;
}

impl Pins<USART2> for (PA2<Alternate<PushPull>>, PA3<Input<Floating>>) {
    const REMAP: u8 = 0;
}

impl Pins<USART2> for (PD5<Alternate<PushPull>>, PD6<Input<Floating>>) {
    const REMAP: u8 = 0;
}

impl Pins<USART3> for (PB10<Alternate<PushPull>>, PB11<Input<Floating>>) {
    const REMAP: u8 = 0;
}

impl Pins<USART3> for (PC10<Alternate<PushPull>>, PC11<Input<Floating>>) {
    const REMAP: u8 = 1;
}

impl Pins<USART3> for (PD8<Alternate<PushPull>>, PD9<Input<Floating>>) {
    const REMAP: u8 = 0b11;
}

pub enum WordLength {
    /// When parity is enabled, a word has 7 data bits + 1 parity bit.
    Bits8,
    /// When parity is enabled, a word has 8 data bits + 1 parity bit.
    Bits9,
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

    pub fn wordlength(mut self, word_size: WordLength) -> Self {
        self.wordlength = word_size;
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
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            wordlength: WordLength::Bits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

/// Serial abstraction
pub struct Serial<USART, PINS, WORD = u8> {
    usart: USART,
    pins: PINS,
    _word: PhantomData<WORD>,
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

/// Internal trait for the serial read / write logic.
trait UsartReadWrite: Deref<Target = crate::pac::usart1::RegisterBlock> {
    fn read(&self) -> nb::Result<u16, Error> {
        let sr = self.sr.read();

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
            self.sr.read();
            self.dr.read();
            Err(nb::Error::Other(err))
        } else {
            // Check if a data is available
            if sr.rxne().bit_is_set() {
                // Read the received data
                Ok(self.dr.read().dr().bits())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    fn write(&self, word: u16) -> nb::Result<(), Infallible> {
        let sr = self.sr.read();

        if sr.txe().bit_is_set() {
            self.dr.write(|w| w.dr().bits(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&self) -> nb::Result<(), Infallible> {
        let sr = self.sr.read();

        if sr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
impl UsartReadWrite for &crate::pac::usart1::RegisterBlock {}

macro_rules! hal {
    ($(
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $usartX_remap:ident,
            $bit:ident,
            $closure:expr,
            $APBx:ident,
        ),
    )+) => {
        $(
            $(#[$meta])*
            /// The behaviour of the functions is equal for all three USARTs.
            /// Except that they are using the corresponding USART hardware and pins.
            impl<PINS> Serial<$USARTX, PINS, u8> {

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
                    config: Config,
                    clocks: Clocks,
                    apb: &mut $APBx,
                ) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // enable and reset $USARTX
                    $USARTX::enable(apb);
                    $USARTX::reset(apb);

                    #[allow(unused_unsafe)]
                    mapr.modify_mapr(|_, w| unsafe{
                            #[allow(clippy::redundant_closure_call)]
                            w.$usartX_remap().$bit(($closure)(PINS::REMAP))
                        });

                    // enable DMA transfers
                    usart.cr3.write(|w| w.dmat().set_bit().dmar().set_bit());

                    // Configure baud rate
                    let brr = <$USARTX as RccBus>::Bus::get_frequency(&clocks).0 / config.baudrate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    let (parity_control_enable, parity) = match config.parity {
                        Parity::ParityNone => (false, false),
                        Parity::ParityEven => (true, false),
                        Parity::ParityOdd => (true, true),
                    };

                    usart.cr1.modify(|_r, w| {
                        w
                            .m().bit(match config.wordlength {
                                WordLength::Bits8 => false,
                                WordLength::Bits9 => true,
                            })
                            .ps().bit(parity)
                            .pce().bit(parity_control_enable)
                    });

                    // Configure stop bits
                    let stop_bits = match config.stopbits {
                        StopBits::STOP1 => 0b00,
                        StopBits::STOP0P5 => 0b01,
                        StopBits::STOP2 => 0b10,
                        StopBits::STOP1P5 => 0b11,
                    };
                    usart.cr2.modify(|_r, w| {
                        w.stop().bits(stop_bits)
                    });

                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .cr1
                        .modify(|_r, w| w.ue().set_bit().re().set_bit().te().set_bit());

                    Serial {
                        usart,
                        pins,
                        _word: PhantomData,
                    }
                }

                /// Starts listening to the USART by enabling the _Received data
                /// ready to be read (RXNE)_ interrupt and _Transmit data
                /// register empty (TXE)_ interrupt
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                        Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                    }
                }

                /// Stops listening to the USART by disabling the _Received data
                /// ready to be read (RXNE)_ interrupt and _Transmit data
                /// register empty (TXE)_ interrupt
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                        Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                    }
                }
            }

            impl<WORD> Tx<$USARTX, WORD> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) };
                }
            }

            impl<WORD> Rx<$USARTX, WORD> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) };
                }
            }

            impl crate::hal::serial::Read<u8> for Rx<$USARTX, u8> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    unsafe { &*$USARTX::ptr() }.read().map(|w| w as u8)
                }
            }

            impl crate::hal::serial::Read<u16> for Rx<$USARTX, u16> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u16, Error> {
                    unsafe { &*$USARTX::ptr() }.read()
                }
            }

            impl crate::hal::serial::Write<u8> for Tx<$USARTX, u8> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    unsafe { &*$USARTX::ptr() }.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    unsafe { &*$USARTX::ptr() }.write(byte as u16)
                }
            }

            impl crate::hal::serial::Write<u16> for Tx<$USARTX, u16> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    unsafe { &*$USARTX::ptr() }.flush()
                }

                fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
                    unsafe { &*$USARTX::ptr() }.write(word)
                }
            }

            impl<PINS> crate::hal::serial::Read<u8> for Serial<$USARTX, PINS, u8> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    self.usart.deref().read().map(|w| w as u8)
                }
            }

            impl<PINS> crate::hal::serial::Read<u16> for Serial<$USARTX, PINS, u16> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u16, Error> {
                    self.usart.deref().read()
                }
            }

            impl<PINS> crate::hal::serial::Write<u8> for Serial<$USARTX, PINS, u8> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    self.usart.deref().flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    self.usart.deref().write(byte as u16)
                }
            }

            impl<PINS> crate::hal::serial::Write<u16> for Serial<$USARTX, PINS, u16> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    self.usart.deref().flush()
                }

                fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
                    self.usart.deref().write(word)
                }
            }
        )+
    }
}

impl<USARTX, PINS, WORD> Serial<USARTX, PINS, WORD> {
    /// Returns ownership of the borrowed register handles
    pub fn release(self) -> (USARTX, PINS) {
        (self.usart, self.pins)
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and
    /// receiving (Rx)
    pub fn split(self) -> (Tx<USARTX, WORD>, Rx<USARTX, WORD>) {
        (
            Tx {
                _usart: PhantomData,
                _word: PhantomData,
            },
            Rx {
                _usart: PhantomData,
                _word: PhantomData,
            },
        )
    }
}

impl<USARTX, PINS> Serial<USARTX, PINS, u8> {
    pub fn with_u16_data(self) -> Serial<USARTX, PINS, u16> {
        Serial {
            usart: self.usart,
            pins: self.pins,
            _word: PhantomData,
        }
    }
}

impl<USARTX, PINS> Serial<USARTX, PINS, u16> {
    pub fn with_u8_data(self) -> Serial<USARTX, PINS, u8> {
        Serial {
            usart: self.usart,
            pins: self.pins,
            _word: PhantomData,
        }
    }
}

impl<USARTX> Rx<USARTX, u8> {
    pub fn with_u16_data(self) -> Rx<USARTX, u16> {
        Rx {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USARTX> Rx<USARTX, u16> {
    pub fn with_u8_data(self) -> Rx<USARTX, u8> {
        Rx {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USARTX> Tx<USARTX, u8> {
    pub fn with_u16_data(self) -> Tx<USARTX, u16> {
        Tx {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USARTX> Tx<USARTX, u16> {
    pub fn with_u8_data(self) -> Tx<USARTX, u8> {
        Tx {
            _usart: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<USART> core::fmt::Write for Tx<USART, u8>
where
    Tx<USART, u8>: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

hal! {
    /// # USART1 functions
    USART1: (
        usart1,
        usart1_remap,
        bit,
        |remap| remap == 1,
        APB2,
    ),
    /// # USART2 functions
    USART2: (
        usart2,
        usart2_remap,
        bit,
        |remap| remap == 1,
        APB1,
    ),
    /// # USART3 functions
    USART3: (
        usart3,
        usart3_remap,
        bits,
        |remap| remap,
        APB1,
    ),
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
            pub type $rxdma = RxDma<Rx<$USARTX, u8>, $dmarxch>;
            pub type $txdma = TxDma<Tx<$USARTX, u8>, $dmatxch>;

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

            impl Rx<$USARTX, u8> {
                pub fn with_dma(self, channel: $dmarxch) -> $rxdma {
                    RxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl Tx<$USARTX, u8> {
                pub fn with_dma(self, channel: $dmatxch) -> $txdma {
                    TxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl $rxdma {
                pub fn split(mut self) -> (Rx<$USARTX, u8>, $dmarxch) {
                    self.stop();
                    let RxDma {payload, channel} = self;
                    (
                        payload,
                        channel
                    )
                }
            }

            impl $txdma {
                pub fn split(mut self) -> (Tx<$USARTX, u8>, $dmatxch) {
                    self.stop();
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
