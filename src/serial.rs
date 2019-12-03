//! # Serial Communication (USART)
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter)
//!
//!
//! ## Example usage:
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
//!     Config::default().baudrate(9_600.bps()),
//!     clocks,
//!     &mut rcc.apb2,
//! );
//!
//! // separate into tx and rx channels
//! let (mut tx, mut rx) = serial.split();
//!
//! // Write 'R' to the USART
//! block!(tx.write(b'R')).ok();
//! // Receive a byte from the USART and store it in "received"
//! let received = block!(rx.read()).unwrap();
//!  ```

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use nb;
use crate::pac::{USART1, USART2, USART3};
use core::convert::Infallible;
use embedded_hal::serial::Write;

use crate::afio::MAPR;
use crate::dma::{dma1, CircBuffer, Static, Transfer, R, W, RxDma, TxDma};
use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7};
use crate::gpio::gpioc::{PC10, PC11};
use crate::gpio::gpiod::{PD5, PD6, PD8, PD9};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{RccBus, Clocks, Enable, Reset};
use crate::time::{U32Ext, Bps};

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
}

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
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
    pub parity: Parity,
    pub stopbits: StopBits,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
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
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $usartX_remap:ident,
            $pclk:ident,
            $bit:ident,
            $closure:expr,
        ),
    )+) => {
        $(
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
                    config: Config,
                    clocks: Clocks,
                    apb: &mut <$USARTX as RccBus>::Bus,
                ) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // enable and reset $USARTX
                    $USARTX::enable(apb);
                    $USARTX::reset(apb);

                    #[allow(unused_unsafe)]
                    mapr.modify_mapr(|_, w| unsafe{
                            w.$usartX_remap().$bit(($closure)(PINS::REMAP))
                        });

                    // enable DMA transfers
                    usart.cr3.write(|w| w.dmat().set_bit().dmar().set_bit());

                    // Configure baud rate
                    let brr = clocks.$pclk().0 / config.baudrate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    // Configure parity and word length
                    // Unlike most uart devices, the "word length" of this usart device refers to
                    // the size of the data plus the parity bit. I.e. "word length"=8, parity=even
                    // results in 7 bits of data. Therefore, in order to get 8 bits and one parity
                    // bit, we need to set the "word" length to 9 when using parity bits.
                    let (word_length, parity_control_enable, parity) = match config.parity {
                        Parity::ParityNone => (false, false, false),
                        Parity::ParityEven => (true, true, false),
                        Parity::ParityOdd => (true, true, true),
                    };
                    usart.cr1.modify(|_r, w| {
                        w
                            .m().bit(word_length)
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

                    Serial { usart, pins }
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

                /// Returns ownership of the borrowed register handles
                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }

                /// Separates the serial struct into separate channel objects for sending (Tx) and
                /// receiving (Rx)
                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }
            }

            impl Tx<$USARTX> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) };
                }
            }

            impl Rx<$USARTX> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) };
                }
            }

            impl crate::hal::serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).sr.read() };

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
                        // Some error occured. In order to clear that error flag, you have to
                        // do a read from the sr register followed by a read from the dr
                        // register
                        // NOTE(read_volatile) see `write_volatile` below
                        unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).sr as *const _ as *const _);
                            ptr::read_volatile(&(*$USARTX::ptr()).dr as *const _ as *const _);
                        }
                        Err(nb::Error::Other(err))
                    } else {
                        // Check if a byte is available
                        if sr.rxne().bit_is_set() {
                            // Read the received byte
                            // NOTE(read_volatile) see `write_volatile` below
                            Ok(unsafe {
                                ptr::read_volatile(&(*$USARTX::ptr()).dr as *const _ as *const _)
                            })
                        } else {
                            Err(nb::Error::WouldBlock)
                        }
                    }
                }
            }

            impl crate::hal::serial::Write<u8> for Tx<$USARTX> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).sr.read() };

                    if sr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).sr.read() };

                    if sr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).dr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }
        )+
    }
}

impl<USART> core::fmt::Write for Tx<USART>
where
    Tx<USART>: embedded_hal::serial::Write<u8>,
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
        pclk2,
        bit,
        |remap| remap == 1,
    ),
    /// # USART2 functions
    USART2: (
        usart2,
        usart2_remap,
        pclk1,
        bit,
        |remap| remap == 1,
    ),
    /// # USART3 functions
    USART3: (
        usart3,
        usart3_remap,
        pclk1,
        bits,
        |remap| remap,
    ),
}

pub type Rx1 = Rx<USART1>;
pub type Tx1 = Tx<USART1>;
pub type Rx2 = Rx<USART2>;
pub type Tx2 = Tx<USART2>;
pub type Rx3 = Rx<USART3>;
pub type Tx3 = Tx<USART3>;

use crate::dma::{Transmit, Receive, TransferPayload};

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
                    RxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl Tx<$USARTX> {
                pub fn with_dma(self, channel: $dmatxch) -> $txdma {
                    TxDma {
                        payload: self,
                        channel,
                    }
                }
            }

            impl $rxdma {
                pub fn split(mut self) -> (Rx<$USARTX>, $dmarxch) {
                    self.stop();
                    let RxDma {payload, channel} = self;
                    (
                        payload,
                        channel
                    )
                }
            }

            impl $txdma {
                pub fn split(mut self) -> (Tx<$USARTX>, $dmatxch) {
                    self.stop();
                    let TxDma {payload, channel} = self;
                    (
                        payload,
                        channel,
                    )
                }
            }

            impl<B> crate::dma::CircReadDma<B, u8> for $rxdma where B: as_slice::AsMutSlice<Element=u8> {
                fn circ_read(mut self, buffer: &'static mut [B; 2],
                ) -> CircBuffer<B, Self>
                {
                    {
                        let buffer = buffer[0].as_mut_slice();
                        self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);
                        self.channel.set_memory_address(buffer.as_ptr() as u32, true);
                        self.channel.set_transfer_length(buffer.len() * 2);

                        atomic::compiler_fence(Ordering::Release);

                        self.channel.ch().cr.modify(|_, w| { w
                            .mem2mem() .clear_bit()
                            .pl()      .medium()
                            .msize()   .bits8()
                            .psize()   .bits8()
                            .circ()    .set_bit()
                            .dir()     .clear_bit()
                        });
                    }

                    self.start();

                    CircBuffer::new(buffer, self)
                }
            }

            impl<B> crate::dma::ReadDma<B, u8> for $rxdma where B: as_slice::AsMutSlice<Element=u8> {
                fn read(mut self, buffer: &'static mut B,
                ) -> Transfer<W, &'static mut B, Self>
                {
                    {
                        let buffer = buffer.as_mut_slice();
                        self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);
                        self.channel.set_memory_address(buffer.as_ptr() as u32, true);
                        self.channel.set_transfer_length(buffer.len());
                    }
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

            impl<A, B> crate::dma::WriteDma<A, B, u8> for $txdma where A: as_slice::AsSlice<Element=u8>, B: Static<A> {
                fn write(mut self, buffer: B
                ) -> Transfer<R, B, Self>
                {
                    {
                        let buffer = buffer.borrow().as_slice();

                        self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);

                        self.channel.set_memory_address(buffer.as_ptr() as u32, true);
                        self.channel.set_transfer_length(buffer.len());
                    }

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
