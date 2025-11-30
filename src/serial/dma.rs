use super::*;
#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
use crate::dma::dma2;
use crate::dma::{self, dma1, Ch, CircBuffer, DmaExt, RxDma, Transfer, TxDma};

use crate::dma::{Receive, TransferPayload, Transmit};

use crate::pacext::uart::Cr3W;

impl<USART: Instance, DMA: DmaExt, const C: u8> TransferPayload for RxDma<Rx<USART>, Ch<DMA, C>> {
    fn start(&mut self) {
        self.channel.start();
    }
    fn stop(&mut self) {
        self.channel.stop();
    }
}

impl<USART: Instance, DMA: DmaExt, const C: u8> TransferPayload for TxDma<Tx<USART>, Ch<DMA, C>> {
    fn start(&mut self) {
        self.channel.start();
    }
    fn stop(&mut self) {
        self.channel.stop();
    }
}

impl<USART: Instance, CH> Rx<USART>
where
    Self: Receive<RxChannel = CH>,
{
    pub fn with_dma(self, channel: CH) -> RxDma<Self, CH> {
        self.usart.cr3().modify(|_, w| w.dmar().set_bit());
        RxDma {
            payload: self,
            channel,
        }
    }
}

impl<USART: Instance, CH> Tx<USART>
where
    Self: Transmit<TxChannel = CH>,
{
    pub fn with_dma(self, channel: CH) -> TxDma<Self, CH> {
        self.usart.cr3().modify(|_, w| w.dmat().set_bit());
        TxDma {
            payload: self,
            channel,
        }
    }
}

impl<USART: Instance, CH> RxDma<Rx<USART>, CH>
where
    Self: TransferPayload,
{
    pub fn release(mut self) -> (Rx<USART>, CH) {
        self.stop();
        self.payload.usart.cr3().modify(|_, w| w.dmar().clear_bit());
        (self.payload, self.channel)
    }
}

impl<USART: Instance, CH> TxDma<Tx<USART>, CH>
where
    Self: TransferPayload,
{
    pub fn release(mut self) -> (Tx<USART>, CH) {
        self.stop();
        self.payload.usart.cr3().modify(|_, w| w.dmat().clear_bit());
        (self.payload, self.channel)
    }
}

impl<USART: Instance, DMA: DmaExt, const C: u8, B> crate::dma::CircReadDma<B, u8>
    for RxDma<Rx<USART>, Ch<DMA, C>>
where
    Rx<USART>: Receive<RxChannel = Ch<DMA, C>>,
    &'static mut [B; 2]: WriteBuffer<Word = u8>,
    B: 'static,
{
    fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        self.channel
            .set_peripheral_address(self.payload.usart.dr().as_ptr() as u32, false);
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

impl<USART: Instance, DMA: DmaExt, const C: u8, B> crate::dma::ReadDma<B, u8>
    for RxDma<Rx<USART>, Ch<DMA, C>>
where
    Rx<USART>: Receive<RxChannel = Ch<DMA, C>>,
    B: WriteBuffer<Word = u8>,
{
    fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        self.channel
            .set_peripheral_address(self.payload.usart.dr().as_ptr() as u32, false);
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

impl<USART: Instance, DMA: DmaExt, const C: u8, B> crate::dma::WriteDma<B, u8>
    for TxDma<Tx<USART>, Ch<DMA, C>>
where
    Tx<USART>: Transmit<TxChannel = Ch<DMA, C>>,
    B: ReadBuffer<Word = u8>,
{
    fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.read_buffer() };

        self.channel
            .set_peripheral_address(self.payload.usart.dr().as_ptr() as u32, false);

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

        impl Receive for Rx<$USARTX> {
            type RxChannel = $dmarxch;
            type TransmittedWord = u8;
        }

        impl Transmit for Tx<$USARTX> {
            type TxChannel = $dmatxch;
            type ReceivedWord = u8;
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
#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
serialdma! {
    pac::UART4,
    RxDma4,
    TxDma4,
    rx: dma2::C3,
    tx: dma2::C5
}
