//! ADC interface DMA RX transfer test

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::singleton;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use stm32f1xx_hal::gpio::gpioa::{PA0, PA2};
use stm32f1xx_hal::{adc, gpio::Analog, pac, prelude::*};

pub struct AdcPins(PA0<Analog>, PA2<Analog>);
impl adc::SetChannels<AdcPins> for adc::Adc<pac::ADC1> {
    fn set_samples(&mut self) {
        self.set_channel_sample_time(0, adc::SampleTime::T_28);
        self.set_channel_sample_time(2, adc::SampleTime::T_28);
    }
    fn set_sequence(&mut self) {
        self.set_regular_sequence(&[0, 2, 0, 2]);
    }
}

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    let dma_ch1 = p.DMA1.split(&mut rcc.ahb).1;

    // Setup ADC
    let adc1 = adc::Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);

    // Setup GPIOA
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let adc_ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);

    // single-channel continuous conversion
    let (adc1, adc_ch0, dma_ch1) = {
        let adc_dma = adc1.with_dma(adc_ch0, dma_ch1);
        let buf = singleton!(: [u16; 8] = [0; 8]).unwrap();

        // The read method consumes the buf and self, starts the adc and dma transfer and returns a
        // RxDma struct. The wait method consumes the RxDma struct, waits for the whole transfer to
        // be completed and then returns the updated buf and underlying adc_dma struct. For non
        // blocking, one can call the is_done method of RxDma and only call wait after that method
        // returns true.
        let (_buf, adc_dma) = adc_dma.read(buf).wait();
        hprintln!("single-channel continuous conversion={:?}", _buf).ok();

        // Consumes the AdcDma struct, restores adc configuration to previous state and returns the
        // Adc struct in normal mode.
        adc_dma.split()
    };

    // multi-channel single-shot conversion
    let (adc1, AdcPins(adc_ch0, adc_ch2), dma_ch1) = {
        let pins = AdcPins(adc_ch0, adc_ch2);
        let adc_dma = adc1.with_scan_dma::<_, adc::Single>(pins, dma_ch1);

        // TODO: should match with # of conversions specified by AdcPins
        let buf = singleton!(: [u16; 4] = [0; 4]).unwrap();

        let (_buf, adc_dma) = adc_dma.read(buf).wait();
        hprintln!("multi-channel single-shot conversion={:?}", _buf).ok();

        adc_dma.split()
    };

    // multi-channel continuous conversion
    let (_adc1, AdcPins(_adc_ch0, _adc_ch2), _dma_ch1) = {
        let pins = AdcPins(adc_ch0, adc_ch2);
        let adc_dma = adc1.with_scan_dma::<_, adc::Continuous>(pins, dma_ch1);
        let buf = singleton!(: [u16; 8] = [0; 8]).unwrap();

        let (_buf, adc_dma) = adc_dma.read(buf).wait();
        hprintln!("multi-channel continuous conversion={:?}", _buf).ok();

        adc_dma.split()
    };

    loop {}
}
