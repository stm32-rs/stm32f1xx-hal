//! # Direct Memory Access
#![allow(dead_code)]

use core::marker::PhantomData;
use core::ops;

use stable_deref_trait::StableDeref;

use crate::rcc::AHB;

#[derive(Debug)]
pub enum Error {
    Overrun,
    #[doc(hidden)]
    _Extensible,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

type DataSize = crate::stm32::dma1::ch::cr::PSIZE_A;
pub type Priority = crate::stm32::dma1::ch::cr::PL_A;

pub struct CircBufferLen<EL, PAYLOAD>
where
    EL : 'static,
{
    buffer: &'static mut [EL],
    payload: PAYLOAD,
    position: usize,
}

impl<EL, PAYLOAD> CircBufferLen<EL, PAYLOAD> {
    pub(crate) fn new(buf: &'static mut [EL], payload: PAYLOAD) -> Self {
        CircBufferLen {
            buffer: buf,
            payload,
            position: 0,
        }
    }
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD> {
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait Static<B> {
    fn borrow(&self) -> &B;
}

impl<B> Static<B> for &'static B {
    fn borrow(&self) -> &B {
        *self
    }
}

impl<B> Static<B> for &'static mut B {
    fn borrow(&self) -> &B {
        *self
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub trait Transferable<BUFFER, DMA> {
    fn is_done(&self) -> bool;
    fn wait(self) -> (BUFFER, DMA);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD> {
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD> {
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD> {
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> ops::Deref for Transfer<R, BUFFER, PAYLOAD> {
    type Target = BUFFER;

    fn deref(&self) -> &BUFFER {
        &self.buffer
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, {
        $($CX:ident: (
            $chX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use core::marker::Copy;
                use core::mem::size_of;
                use core::sync::atomic::{self, Ordering};
                use core::ptr;
                use core::cmp::min;

                use crate::pac::{$DMAX, dma1};

                use crate::dma::{
                    CircBuffer,
                    CircBufferLen,
                    DataSize,
                    DmaExt,
                    Error,
                    Event,
                    Half,
                    Priority,
                    Transfer,
                    W,
                    RxDma,
                    TxDma,
                    TransferPayload,
                    Transferable};
                use crate::rcc::{AHB, Enable};

                pub struct Channels((), $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { _0: () }

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.ch().par.write(|w| w.pa().bits(address) );
                            self.ch().cr.modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.ch().mar.write(|w| w.ma().bits(address) );
                            self.ch().cr.modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        pub fn set_transfer_length(&mut self, len: usize) {
                            self.ch().ndtr.write(|w| w.ndt().bits(cast::u16(len).unwrap()));
                        }

                        /// Starts the DMA transfer
                        pub fn start(&mut self) {
                            self.ch().cr.modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        pub fn stop(&mut self) {
                            self.ifcr().write(|w| w.$cgifX().set_bit());
                            self.ch().cr.modify(|_, w| w.en().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        pub fn in_progress(&self) -> bool {
                            self.isr().$tcifX().bit_is_clear()
                        }
                    }

                    impl $CX {
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ch().cr.modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        pub fn ch(&mut self) -> &dma1::CH {
                            unsafe { &(*$DMAX::ptr()).$chX }
                        }

                        pub fn isr(&self) -> dma1::isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).isr.read() }
                        }

                        pub fn ifcr(&self) -> &dma1::IFCR {
                            unsafe { &(*$DMAX::ptr()).ifcr }
                        }

                        pub fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { &(*$DMAX::ptr())}.$chX.ndtr.read().bits()
                        }
                    }

                    impl<EL, PAYLOAD> CircBufferLen<EL, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                        EL: Copy,
                    {
                        pub unsafe fn setup(&mut self, peripheral_address : u32, priority: Priority) {
                            self.payload.channel.set_peripheral_address(peripheral_address, false);
                            self.payload.channel.set_memory_address(self.buffer.as_ptr() as u32, true);
                            self.payload.channel.set_transfer_length(self.buffer.len());

                            atomic::compiler_fence(Ordering::Release);

                            let bits = match size_of::<EL>() {
                                1 => DataSize::BITS8,
                                2 => DataSize::BITS16,
                                4 => DataSize::BITS32,
                                _ => panic!("Unsupported element size.")
                            };

                            self.payload.channel.ch().cr.modify(|_, w| {
                                w.mem2mem().clear_bit()
                                .pl()      .variant(priority)
                                .circ()    .set_bit()
                                .dir()     .clear_bit()
                                .msize()   .variant(bits)
                                .psize()   .variant(bits)
                            });

                            self.payload.start();
                        }


                        /// Return the number of elements available to read
                        pub fn len(&mut self) -> usize {
                            let blen = self.buffer.len();
                            let ndtr = self.payload.channel.get_ndtr() as usize;
                            let pos_at = self.position;

                            // the position the DMA would write to next
                            let pos_to = blen - ndtr;

                            if pos_at > pos_to {
                                // the buffer wraps around
                                blen + pos_to - pos_at
                            } else {
                                // the buffer does not wrap around
                                pos_to - pos_at
                            }
                        }

                        pub fn read(&mut self, dat: &mut [EL]) -> usize {
                            let blen = self.buffer.len();
                            let len = self.len();
                            let pos = self.position;
                            let read = min(dat.len(), len);

                            if pos + read <= blen {
                                // the read operation does not wrap around the
                                // circular buffer, perform a single read
                                dat[0..read].copy_from_slice(&self.buffer[pos..pos + read]);
                                self.position = pos + read;
                                if self.position >= blen {
                                    self.position = 0;
                                }
                            } else {
                                // the read operation wraps around the circular buffer,
                                let left = blen - pos;
                                // copy until the end of the buffer
                                dat[0..left].copy_from_slice(&self.buffer[pos..blen]);
                                // copy from the beginning of the buffer until the amount to read
                                dat[left..read].copy_from_slice(&self.buffer[0..read - left]);
                                self.position = read - left;
                            }

                            // return the number of bytes read
                            read
                        }
                    }

                    impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        /// Peeks into the readable half of the buffer
                        pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&B, Half) -> R,
                        {
                            let half_being_read = self.readable_half()?;

                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };

                            // XXX does this need a compiler barrier?
                            let ret = f(buf, half_being_read);


                            let isr = self.payload.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if (half_being_read == Half::First && second_half_is_done) ||
                                (half_being_read == Half::Second && first_half_is_done) {
                                Err(Error::Overrun)
                            } else {
                                Ok(ret)
                            }
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let isr = self.payload.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.payload.channel.ifcr().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.payload.channel.ifcr().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }

                        /// Stops the transfer and returns the underlying buffer and RxDma
                        pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, $CX>) {
                            self.payload.stop();

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transferable<BUFFER, RxDma<PAYLOAD, $CX>> for Transfer<MODE, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transferable<BUFFER, TxDma<PAYLOAD, $CX>> for Transfer<MODE, BUFFER, TxDma<PAYLOAD, $CX>>
                    where
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<W, &'static mut BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.channel.get_ndtr() as usize;

                            let slice = self.buffer.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        $DMAX::enable(ahb);

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$chX.cr.reset();
                        )+

                        Channels((), $($CX { _0: () }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, {
        C1: (
            ch1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            ch2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            ch3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            ch4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            ch5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
        C6: (
            ch6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6
        ),
        C7: (
            ch7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7
        ),
    }),

    DMA2: (dma2, {
        C1: (
            ch1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            ch2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            ch3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            ch4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            ch5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

pub trait CircReadDmaLen<B, RS>: Receive
where
    B: as_slice::AsMutSlice<Element = RS>,
    Self: core::marker::Sized,
{
    fn circ_read_len(self, buffer: &'static mut B) -> CircBufferLen<RS, Self>;
}

pub trait CircReadDma<B, RS>: Receive
where
    B: as_slice::AsMutSlice<Element = RS>,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

pub trait ReadDma<B, RS>: Receive
where
    B: as_slice::AsMutSlice<Element = RS>,
    Self: core::marker::Sized,
{
    fn read(self, buffer: &'static mut B) -> Transfer<W, &'static mut B, Self>;
}

pub trait WriteDma<B, TS>: Transmit
where
    B: core::ops::Deref + 'static,
    B::Target: as_slice::AsSlice<Element = TS> + Unpin,
    B: StableDeref,
    Self: core::marker::Sized,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}
