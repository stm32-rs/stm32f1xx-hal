use super::*;

#[cfg(feature = "connectivity")]
use crate::dma::dma2;
use crate::dma::{self, Receive, RxDma, RxTxDma, Transfer, TransferPayload, Transmit, TxDma};
use crate::dma::{dma1, Ch, DmaExt};

pub type SpiTxDma<SPI, CHANNEL> = TxDma<Spi<SPI, u8>, CHANNEL>;
pub type SpiRxDma<SPI, CHANNEL> = RxDma<Spi<SPI, u8>, CHANNEL>;
pub type SpiRxTxDma<SPI, RXCHANNEL, TXCHANNEL> = RxTxDma<Spi<SPI, u8>, RXCHANNEL, TXCHANNEL>;

pub type SpiSlaveTxDma<SPI, CHANNEL, Otype> = TxDma<SpiSlave<SPI, u8, Otype>, CHANNEL>;
pub type SpiSlaveRxDma<SPI, CHANNEL, Otype> = RxDma<SpiSlave<SPI, u8, Otype>, CHANNEL>;
pub type SpiSlaveRxTxDma<SPI, RXCHANNEL, TXCHANNEL, Otype> =
    RxTxDma<SpiSlave<SPI, u8, Otype>, RXCHANNEL, TXCHANNEL>;

macro_rules! spi_impl {
    ($Spi:ident) => {
        impl<SPI: Instance, DMA: DmaExt, const C: u8> TransferPayload
            for RxDma<$Spi<SPI, u8>, Ch<DMA, C>>
        {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<SPI: Instance, DMA: DmaExt, const C: u8> TransferPayload
            for TxDma<$Spi<SPI, u8>, Ch<DMA, C>>
        {
            fn start(&mut self) {
                self.channel.start();
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl<SPI: Instance, DMA1: DmaExt, const C1: u8, DMA2: DmaExt, const C2: u8> TransferPayload
            for RxTxDma<$Spi<SPI, u8>, Ch<DMA1, C1>, Ch<DMA2, C2>>
        {
            fn start(&mut self) {
                self.rxchannel.start();
                self.txchannel.start();
            }
            fn stop(&mut self) {
                self.txchannel.stop();
                self.rxchannel.stop();
            }
        }

        impl<SPI: Instance, CH> $Spi<SPI, u8>
        where
            Self: Transmit<TxChannel = CH>,
        {
            pub fn with_tx_dma(self, channel: CH) -> TxDma<$Spi<SPI, u8>, CH> {
                self.spi.cr2().modify(|_, w| w.txdmaen().set_bit());
                TxDma {
                    payload: self,
                    channel,
                }
            }
        }
        impl<SPI: Instance, CH> $Spi<SPI, u8>
        where
            Self: Receive<RxChannel = CH>,
        {
            pub fn with_rx_dma(self, channel: CH) -> RxDma<$Spi<SPI, u8>, CH> {
                self.spi.cr2().modify(|_, w| w.rxdmaen().set_bit());
                RxDma {
                    payload: self,
                    channel,
                }
            }
        }
        impl<SPI: Instance, RXCH, TXCH> $Spi<SPI, u8>
        where
            Self: Receive<RxChannel = RXCH>,
            Self: Transmit<TxChannel = TXCH>,
        {
            pub fn with_rx_tx_dma(
                self,
                rxchannel: RXCH,
                txchannel: TXCH,
            ) -> RxTxDma<$Spi<SPI, u8>, RXCH, TXCH> {
                self.spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().set_bit().txdmaen().set_bit());
                RxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }

        impl<SPI: Instance, CH> TxDma<$Spi<SPI, u8>, CH> {
            pub fn release(self) -> ($Spi<SPI, u8>, CH) {
                // self.stop(); ?
                self.payload
                    .spi
                    .cr2()
                    .modify(|_, w| w.txdmaen().clear_bit());
                (self.payload, self.channel)
            }
        }

        impl<SPI: Instance, CH> RxDma<$Spi<SPI, u8>, CH> {
            pub fn release(self) -> ($Spi<SPI, u8>, CH) {
                // self.stop(); ?
                self.payload
                    .spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().clear_bit());
                (self.payload, self.channel)
            }
        }

        impl<SPI: Instance, RXCH, TXCH> RxTxDma<$Spi<SPI, u8>, RXCH, TXCH> {
            pub fn release(self) -> ($Spi<SPI, u8>, RXCH, TXCH) {
                // self.stop(); ?
                self.payload
                    .spi
                    .cr2()
                    .modify(|_, w| w.rxdmaen().clear_bit().txdmaen().clear_bit());
                (self.payload, self.rxchannel, self.txchannel)
            }
        }

        impl<SPI: Instance, DMA: DmaExt, const C: u8, B> crate::dma::ReadDma<B, u8>
            for RxDma<$Spi<SPI, u8>, Ch<DMA, C>>
        where
            $Spi<SPI, u8>: Receive<RxChannel = Ch<DMA, C>>,
            B: WriteBuffer<Word = u8>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.write_buffer() };
                self.channel
                    .set_peripheral_address(unsafe { (*SPI::ptr()).dr().as_ptr() as u32 }, false);
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }

        impl<SPI: Instance, DMA: DmaExt, const C: u8, B> crate::dma::WriteDma<B, u8>
            for TxDma<$Spi<SPI, u8>, Ch<DMA, C>>
        where
            $Spi<SPI, u8>: Transmit<TxChannel = Ch<DMA, C>>,
            B: ReadBuffer<Word = u8>,
        {
            fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.read_buffer() };
                self.channel
                    .set_peripheral_address(unsafe { (*SPI::ptr()).dr().as_ptr() as u32 }, false);
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::r(buffer, self)
            }
        }

        impl<
                SPI: Instance,
                RXDMA: DmaExt,
                const RXC: u8,
                RXB,
                TXDMA: DmaExt,
                const TXC: u8,
                TXB,
            > crate::dma::ReadWriteDma<RXB, TXB, u8>
            for RxTxDma<$Spi<SPI, u8>, Ch<RXDMA, RXC>, Ch<TXDMA, TXC>>
        where
            $Spi<SPI, u8>:
                Receive<RxChannel = Ch<RXDMA, RXC>> + Transmit<TxChannel = Ch<TXDMA, TXC>>,
            RXB: WriteBuffer<Word = u8>,
            TXB: ReadBuffer<Word = u8>,
        {
            fn read_write(
                mut self,
                mut rxbuffer: RXB,
                txbuffer: TXB,
            ) -> Transfer<dma::W, (RXB, TXB), Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (rxptr, rxlen) = unsafe { rxbuffer.write_buffer() };
                let (txptr, txlen) = unsafe { txbuffer.read_buffer() };

                if rxlen != txlen {
                    panic!("receive and send buffer lengths do not match!");
                }

                self.rxchannel
                    .set_peripheral_address(unsafe { (*SPI::ptr()).dr().as_ptr() as u32 }, false);
                self.rxchannel.set_memory_address(rxptr as u32, true);
                self.rxchannel.set_transfer_length(rxlen);

                self.txchannel
                    .set_peripheral_address(unsafe { (*SPI::ptr()).dr().as_ptr() as u32 }, false);
                self.txchannel.set_memory_address(txptr as u32, true);
                self.txchannel.set_transfer_length(txlen);

                atomic::compiler_fence(Ordering::Release);
                self.rxchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // write to memory
                    w.dir().clear_bit()
                });
                self.txchannel.ch().cr().modify(|_, w| {
                    // memory to memory mode disabled
                    w.mem2mem().clear_bit();
                    // medium channel priority level
                    w.pl().medium();
                    // 8-bit memory size
                    w.msize().bits8();
                    // 8-bit peripheral size
                    w.psize().bits8();
                    // circular mode disabled
                    w.circ().clear_bit();
                    // read from memory
                    w.dir().set_bit()
                });
                self.start();

                Transfer::w((rxbuffer, txbuffer), self)
            }
        }
    };
}

spi_impl!(Spi);
spi_impl!(SpiSlave);

macro_rules! spi_dma {
    (
        $SPIi:ty,
        rx: $RCi:ty,
        tx: $TCi:ty,
        $rxdma:ident,
        $txdma:ident,
        $rxtxdma:ident,
        $slaverxdma:ident,
        $slavetxdma:ident,
        $slaverxtxdma:ident
    ) => {
        pub type $rxdma = SpiRxDma<$SPIi, $RCi>;
        pub type $txdma = SpiTxDma<$SPIi, $TCi>;
        pub type $rxtxdma = SpiRxTxDma<$SPIi, $RCi, $TCi>;

        impl Transmit for Spi<$SPIi, u8> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl Receive for Spi<$SPIi, u8> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }

        pub type $slaverxdma<Otype = PushPull> = SpiSlaveRxDma<$SPIi, $RCi, Otype>;
        pub type $slavetxdma<Otype = PushPull> = SpiSlaveTxDma<$SPIi, $TCi, Otype>;
        pub type $slaverxtxdma<Otype = PushPull> = SpiSlaveRxTxDma<$SPIi, $RCi, $TCi, Otype>;

        impl<Otype> Transmit for SpiSlave<$SPIi, u8, Otype> {
            type TxChannel = $TCi;
            type ReceivedWord = u8;
        }

        impl<Otype> Receive for SpiSlave<$SPIi, u8, Otype> {
            type RxChannel = $RCi;
            type TransmittedWord = u8;
        }
    };
}

spi_dma!(
    pac::SPI1,
    rx: dma1::C2,
    tx: dma1::C3,
    Spi1RxDma,
    Spi1TxDma,
    Spi1RxTxDma,
    SpiSlave1RxDma,
    SpiSlave1TxDma,
    SpiSlave1RxTxDma
);
spi_dma!(
    pac::SPI2,
    rx: dma1::C4,
    tx: dma1::C5,
    Spi2RxDma,
    Spi2TxDma,
    Spi2RxTxDma,
    SpiSlave2RxDma,
    SpiSlave2TxDma,
    SpiSlave2RxTxDma
);
#[cfg(feature = "connectivity")]
spi_dma!(
    pac::SPI3,
    rx: dma2::C1,
    tx: dma2::C2,
    Spi3RxDma,
    Spi3TxDma,
    Spi3RxTxDma,
    SpiSlave3RxDma,
    SpiSlave3TxDma,
    SpiSlave3RxTxDma
);
