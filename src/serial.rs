use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use cast::u16;
use hal;
use nb;
use stm32::{USART1, USART2, USART3};
use void::Void;

use afio::MAPR;
//use dma::{dma1, CircBuffer, Static, Transfer, R, W};
use dma::{CircBuffer, Static, Transfer, R, W};
use gpio::gpioa::{PA10, PA2, PA3, PA9};
use gpio::gpiob::{PB10, PB11, PB6, PB7};
use gpio::{Alternate, Floating, Input, PushPull};
use rcc::{Clocks, APB1, APB2};
use time::Bps;

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

// impl Pins<USART2> for (PD5<Alternate<PushPull>>, PD6<Input<Floating>>) {
//     const REMAP: u8 = 0;
// }

impl Pins<USART3> for (PB10<Alternate<PushPull>>, PB11<Input<Floating>>) {
    const REMAP: u8 = 0;
}

// impl Pins<USART3> for (PC10<Alternate<PushPull>>, PC11<Input<Floating>>) {
//     const REMAP: u8 = 1;
// }

// impl Pins<USART3> for (PD8<Alternate<PushPull>>, PD9<Input<Floating>>) {
//     const REMAP: u8 = 0b11;
// }

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
        $USARTX:ident: (
            $usartX:ident,
            $usartXen:ident,
            $usartXrst:ident,
            $usartX_remap:ident,
            $bit:ident,
            $closure:expr,
            $APB:ident
        ),
    )+) => {
        $(
            impl<PINS> Serial<$USARTX, PINS> {
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    mapr: &mut MAPR,
                    baud_rate: Bps,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // enable and reset $USARTX
                    apb.enr().modify(|_, w| w.$usartXen().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().clear_bit());

                    #[allow(unused_unsafe)]
                    mapr.mapr()
                        .modify(|_, w| unsafe{
                            w.$usartX_remap().$bit(($closure)(PINS::REMAP))
                        });

                    // enable DMA transfers
                    usart.cr3.write(|w| w.dmat().set_bit().dmar().set_bit());

                    let brr = clocks.pclk2().0 / baud_rate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .cr1
                        .write(|w| w.ue().set_bit().re().set_bit().te().set_bit());

                    Serial { usart, pins }
                }

                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                        Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                    }
                }

                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                        Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                    }
                }

                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }

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

            impl hal::serial::Read<u8> for Rx<$USARTX> {
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

            /*
            impl<B> ReadDma<B> for Rx<$USARTX> where B: AsMut<[u8]> {
                fn circ_read(self, mut chan: Self::Dma, buffer: &'static mut [B; 2],
                ) -> CircBuffer<B, Self::Dma>
                {
                    {
                        let buffer = buffer[0].as_mut();
                        chan.cmar().write(|w| {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });
                        chan.cndtr().write(|w| {
                            w.ndt().bits(u16(buffer.len() * 2).unwrap())
                        });
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).dr as *const _ as usize as u32)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        chan.ccr().modify(|_, w| {
                            w.mem2mem()
                                .clear_bit()
                                .pl()
                                .medium()
                                .msize()
                                .bit8()
                                .psize()
                                .bit8()
                                .minc()
                                .set_bit()
                                .pinc()
                                .clear_bit()
                                .circ()
                                .set_bit()
                                .dir()
                                .clear_bit()
                                .en()
                                .set_bit()
                        });
                    }

                    CircBuffer::new(buffer, chan)
                }

                fn read_exact(self, mut chan: Self::Dma, buffer: &'static mut B,
                ) -> Transfer<W, &'static mut B, Self::Dma, Self>
                {
                    {
                        let buffer = buffer.as_mut();
                        chan.cmar().write(|w| {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });
                        chan.cndtr().write(|w| {
                            w.ndt().bits(u16(buffer.len()).unwrap())
                        });
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).dr as *const _ as usize as u32)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        chan.ccr().modify(|_, w| {
                            w.mem2mem()
                                .clear_bit()
                                .pl()
                                .medium()
                                .msize()
                                .bit8()
                                .psize()
                                .bit8()
                                .minc()
                                .set_bit()
                                .pinc()
                                .clear_bit()
                                .circ()
                                .clear_bit()
                                .dir()
                                .clear_bit()
                                .en()
                                .set_bit()
                        });
                    }

                    Transfer::w(buffer, chan, self)
                }
            }
            */

            /*
            impl<A, B> WriteDma<A, B> for Tx<$USARTX> where A: AsRef<[u8]>, B: Static<A> {
                fn write_all(self, mut chan: Self::Dma, buffer: B
                ) -> Transfer<R, B, Self::Dma, Self>
                {
                    {
                        let buffer = buffer.borrow().as_ref();
                        chan.cmar().write(|w| {
                            w.ma().bits(buffer.as_ptr() as usize as u32)
                        });
                        chan.cndtr().write(|w| {
                            w.ndt().bits(u16(buffer.len()).unwrap())
                        });
                        chan.cpar().write(|w| unsafe {
                            w.pa().bits(&(*$USARTX::ptr()).dr as *const _ as usize as u32)
                        });

                        // TODO can we weaken this compiler barrier?
                        // NOTE(compiler_fence) operations on `buffer` should not be reordered after
                        // the next statement, which starts the DMA transfer
                        atomic::compiler_fence(Ordering::SeqCst);

                        chan.ccr().modify(|_, w| {
                            w.mem2mem()
                                .clear_bit()
                                .pl()
                                .medium()
                                .msize()
                                .bit8()
                                .psize()
                                .bit8()
                                .minc()
                                .set_bit()
                                .pinc()
                                .clear_bit()
                                .circ()
                                .clear_bit()
                                .dir()
                                .set_bit()
                                .en()
                                .set_bit()
                        });
                    }

                    Transfer::r(buffer, chan, self)
                }
            }
            */

            impl hal::serial::Write<u8> for Tx<$USARTX> {
                type Error = Void;

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

hal! {
    USART1: (
        usart1,
        usart1en,
        usart1rst,
        usart1_remap,
        bit,
        |remap| remap == 1,
        APB2
    ),
    USART2: (
        usart2,
        usart2en,
        usart2rst,
        usart2_remap,
        bit,
        |remap| remap == 1,
        APB1
    ),
    USART3: (
        usart3,
        usart3en,
        usart3rst,
        usart3_remap,
        bits,
        |remap| remap,
        APB1
    ),
}

/*
use dma::DmaChannel;

impl DmaChannel for Rx<USART1> {
    type Dma = dma1::C5;
}

impl DmaChannel for Tx<USART1> {
    type Dma = dma1::C4;
}

impl DmaChannel for Rx<USART2> {
    type Dma = dma1::C6;
}

impl DmaChannel for Tx<USART2> {
    type Dma = dma1::C7;
}

impl DmaChannel for Rx<USART3> {
    type Dma = dma1::C3;
}

impl DmaChannel for Tx<USART3> {
    type Dma = dma1::C2;
}

pub trait ReadDma<B>: DmaChannel
where
    B: AsMut<[u8]>,
    Self: core::marker::Sized,
{
    fn circ_read(self, chan: Self::Dma, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self::Dma>;
    fn read_exact(
        self,
        chan: Self::Dma,
        buffer: &'static mut B,
    ) -> Transfer<W, &'static mut B, Self::Dma, Self>;
}

pub trait WriteDma<A, B>: DmaChannel
where
    A: AsRef<[u8]>,
    B: Static<A>,
    Self: core::marker::Sized,
{
    fn write_all(self, chan: Self::Dma, buffer: B) -> Transfer<R, B, Self::Dma, Self>;
}
*/
