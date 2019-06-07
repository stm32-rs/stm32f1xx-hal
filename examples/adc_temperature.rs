#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    adc,
};
use cortex_m_rt::{entry,exception,ExceptionFrame};

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Aquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Several examples of HSE, PCLK1, ADC clocks
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(8.mhz()).pclk1(2.mhz()).adcclk(1.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(16.mhz()).pclk1(4.mhz()).adcclk(2.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(24.mhz()).pclk1(9.mhz()).adcclk(3.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(24.mhz()).pclk1(12.mhz()).adcclk(4.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(24.mhz()).pclk1(12.mhz()).adcclk(6.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(36.mhz()).pclk1(12.mhz()).adcclk(6.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(32.mhz()).pclk1(16.mhz()).adcclk(8.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(40.mhz()).pclk1(20.mhz()).adcclk(10.mhz()).freeze(&mut flash.acr);
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(48.mhz()).pclk1(24.mhz()).adcclk(12.mhz()).freeze(&mut flash.acr);
    let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(56.mhz()).pclk1(28.mhz()).adcclk(14.mhz()).freeze(&mut flash.acr);

    hprintln!("sysclk freq: {}", clocks.sysclk().0).unwrap();
    hprintln!("adc freq: {}", clocks.adcclk().0).unwrap();

    // Setup ADC
    let mut adc = adc::Adc::adc1(p.ADC1, &mut rcc.apb2, clocks.adcclk());

    // Read temperature sensor
    loop {
        let temp = adc.read_temp();

        hprintln!("temp: {}", temp).unwrap();
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
