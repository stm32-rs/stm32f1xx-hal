//! ADC interface circular DMA RX transfer test

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};

use cortex_m_rt::entry;
use stm32f1xx_hal::{adc, dma::Half, pac, prelude::*};

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    let dma_ch1 = p.DMA1.split().1;

    // Setup ADC
    let adc1 = adc::Adc::adc1(p.ADC1, clocks);

    // Setup GPIOA
    let mut gpioa = p.GPIOA.split();

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);

    let adc_dma = adc1.with_dma(adc_ch0, dma_ch1);
    let buf = singleton!(: [[u16; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = adc_dma.circ_read(buf);

    while circ_buffer.readable_half().unwrap() != Half::First {}

    let _first_half = circ_buffer.peek(|half, _| *half).unwrap();

    while circ_buffer.readable_half().unwrap() != Half::Second {}

    let _second_half = circ_buffer.peek(|half, _| *half).unwrap();

    let (_buf, adc_dma) = circ_buffer.stop();
    let (_adc1, _adc_ch0, _dma_ch1) = adc_dma.split();
    asm::bkpt();

    loop {}
}
