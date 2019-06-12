//! ADC interface DMA RX transfer test

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};

use stm32f1xx_hal::{
    prelude::*,
    pac,
    adc,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // Aquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let _clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    let dma_ch1 = p.DMA1.split(&mut rcc.ahb).1;

    // Setup ADC
    let adc1 = adc::Adc::adc1(p.ADC1, &mut rcc.apb2);

    // Setup GPIOA
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);

    let adc_dma = adc1.with_dma(adc_ch0, dma_ch1);
    let buf = singleton!(: [u16; 8] = [0; 8]).unwrap();

    let (_buf, adc_dma) = adc_dma.read(buf).wait();
    asm::bkpt();
    
    let (_adc1, _adc_ch0, _dma_ch1) = adc_dma.split();
    asm::bkpt();

    loop {}
}