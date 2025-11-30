use super::*;

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl")))]
use crate::dma::dma2;
use crate::dma::{dma1, Ch, CircBuffer, DmaExt, Receive, RxDma, Transfer, TransferPayload, W};

pub struct AdcPayload<ADC, PINS, MODE> {
    adc: Adc<ADC>,
    pins: PINS,
    _mode: PhantomData<MODE>,
}

pub type AdcDma<ADC, PINS, MODE, CHANNEL> = RxDma<AdcPayload<ADC, PINS, MODE>, CHANNEL>;

impl<ADC: Instance, PINS, DMA: DmaExt, const C: u8> TransferPayload
    for AdcDma<ADC, PINS, Continuous, Ch<DMA, C>>
{
    fn start(&mut self) {
        self.channel.start();
        self.payload.adc.rb.cr2().modify(|_, w| w.cont().set_bit());
        self.payload.adc.rb.cr2().modify(|_, w| w.adon().set_bit());
    }
    fn stop(&mut self) {
        self.channel.stop();
        self.payload
            .adc
            .rb
            .cr2()
            .modify(|_, w| w.cont().clear_bit());
    }
}

impl<ADC: Instance, PINS, DMA: DmaExt, const C: u8> TransferPayload
    for AdcDma<ADC, PINS, Scan, Ch<DMA, C>>
{
    fn start(&mut self) {
        self.channel.start();
        self.payload.adc.rb.cr2().modify(|_, w| w.adon().set_bit());
    }
    fn stop(&mut self) {
        self.channel.stop();
    }
}

impl<ADC: Instance> Adc<ADC> {
    pub fn with_dma<PIN, CH>(mut self, pins: PIN, dma_ch: CH) -> AdcDma<ADC, PIN, Continuous, CH>
    where
        AdcPayload<ADC, PIN, Continuous>: Receive<RxChannel = CH>,
        PIN: Channel<ADC, ID = u8>,
    {
        self.rb.cr1().modify(|_, w| w.discen().clear_bit());
        self.rb
            .cr2()
            .modify(|_, w| w.align().variant(self.align.into()));
        self.set_channel_sample_time(PIN::channel(), self.sample_time);
        self.rb
            .sqr3()
            .modify(|_, w| unsafe { w.sq1().bits(PIN::channel()) });
        self.rb.cr2().modify(|_, w| w.dma().set_bit());

        let payload = AdcPayload {
            adc: self,
            pins,
            _mode: PhantomData,
        };
        RxDma {
            payload,
            channel: dma_ch,
        }
    }

    pub fn with_scan_dma<PINS, CH>(mut self, pins: PINS, dma_ch: CH) -> AdcDma<ADC, PINS, Scan, CH>
    where
        AdcPayload<ADC, PINS, Scan>: Receive<RxChannel = CH>,
        Self: SetChannels<PINS>,
    {
        self.rb.cr2().modify(|_, w| {
            w.adon().clear_bit();
            w.dma().clear_bit();
            w.cont().clear_bit();
            w.align().variant(self.align.into())
        });
        self.rb
            .cr1()
            .modify(|_, w| w.scan().set_bit().discen().clear_bit());
        self.set_samples();
        self.set_sequence();
        self.rb
            .cr2()
            .modify(|_, w| w.dma().set_bit().adon().set_bit());

        let payload = AdcPayload {
            adc: self,
            pins,
            _mode: PhantomData,
        };
        RxDma {
            payload,
            channel: dma_ch,
        }
    }
}

impl<ADC: Instance, PINS, CH> AdcDma<ADC, PINS, Continuous, CH>
where
    Self: TransferPayload,
{
    pub fn split(mut self) -> (Adc<ADC>, PINS, CH) {
        self.stop();

        let AdcDma { payload, channel } = self;
        payload.adc.rb.cr2().modify(|_, w| w.dma().clear_bit());
        payload.adc.rb.cr1().modify(|_, w| w.discen().set_bit());

        (payload.adc, payload.pins, channel)
    }
}

impl<ADC: Instance, PINS, CH> AdcDma<ADC, PINS, Scan, CH>
where
    Self: TransferPayload,
{
    pub fn split(mut self) -> (Adc<ADC>, PINS, CH) {
        self.stop();

        let AdcDma { payload, channel } = self;
        payload.adc.rb.cr2().modify(|_, w| w.dma().clear_bit());
        payload.adc.rb.cr1().modify(|_, w| w.discen().set_bit());
        payload.adc.rb.cr1().modify(|_, w| w.scan().clear_bit());

        (payload.adc, payload.pins, channel)
    }
}

impl<ADC: Instance, B, PINS, MODE, DMA: DmaExt, const C: u8> crate::dma::CircReadDma<B, u16>
    for AdcDma<ADC, PINS, MODE, Ch<DMA, C>>
where
    AdcPayload<ADC, PINS, MODE>: Receive<RxChannel = Ch<DMA, C>>,
    Self: TransferPayload,
    &'static mut [B; 2]: WriteBuffer<Word = u16>,
    B: 'static,
{
    fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        self.channel
            .set_peripheral_address(unsafe { (*ADC::ptr()).dr().as_ptr() as u32 }, false);
        self.channel.set_memory_address(ptr as u32, true);
        self.channel.set_transfer_length(len);

        atomic::compiler_fence(Ordering::Release);

        self.channel.ch().cr().modify(|_, w| {
            w.mem2mem().clear_bit();
            w.pl().medium();
            w.msize().bits16();
            w.psize().bits16();
            w.circ().set_bit();
            w.dir().clear_bit()
        });

        self.start();

        CircBuffer::new(buffer, self)
    }
}

impl<ADC: Instance, B, PINS, MODE, DMA: DmaExt, const C: u8> crate::dma::ReadDma<B, u16>
    for AdcDma<ADC, PINS, MODE, Ch<DMA, C>>
where
    AdcPayload<ADC, PINS, MODE>: Receive<RxChannel = Ch<DMA, C>>,
    Self: TransferPayload,
    B: WriteBuffer<Word = u16>,
{
    fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
        // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
        // until the end of the transfer.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        self.channel
            .set_peripheral_address(unsafe { (*ADC::ptr()).dr().as_ptr() as u32 }, false);
        self.channel.set_memory_address(ptr as u32, true);
        self.channel.set_transfer_length(len);

        atomic::compiler_fence(Ordering::Release);
        self.channel.ch().cr().modify(|_, w| {
            w.mem2mem().clear_bit();
            w.pl().medium();
            w.msize().bits16();
            w.psize().bits16();
            w.circ().clear_bit();
            w.dir().clear_bit()
        });
        self.start();

        Transfer::w(buffer, self)
    }
}

macro_rules! adcdma {
    ($ADCX:ty: (
        $rxdma:ident,
        $dmarxch:ty,
    )) => {
        pub type $rxdma<PINS, MODE> = AdcDma<$ADCX, PINS, MODE, $dmarxch>;

        impl<PINS, MODE> Receive for AdcPayload<$ADCX, PINS, MODE> {
            type RxChannel = $dmarxch;
            type TransmittedWord = u16;
        }
    };
}

adcdma! {
    pac::ADC1: (
        AdcDma1,
        dma1::C1,
    )
}

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl")))]
adcdma! {
    pac::ADC3: (
        AdcDma3,
        dma2::C5,
    )
}
