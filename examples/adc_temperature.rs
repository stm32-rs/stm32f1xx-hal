#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{adc, pac, prelude::*};

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(56.MHz())
        .pclk1(28.MHz())
        .adcclk(14.MHz())
        .freeze(&mut flash.acr);
    /*
    // Alternative configuration using dividers and multipliers directly
    let clocks = rcc.cfgr.freeze_with_config(rcc::Config {
        hse: Some(8_000_000),
        pllmul: Some(7),
        hpre: rcc::HPre::DIV1,
        ppre1: rcc::PPre::DIV2,
        ppre2: rcc::PPre::DIV1,
        usbpre: rcc::UsbPre::DIV1_5,
        adcpre: rcc::AdcPre::DIV2,
    }, &mut flash.acr);*/
    hprintln!("sysclk freq: {}", clocks.sysclk());
    hprintln!("adc freq: {}", clocks.adcclk());

    // Setup ADC
    let mut adc = adc::Adc::adc1(p.ADC1, clocks);

    // Read temperature sensor
    loop {
        let temp = adc.read_temp();

        hprintln!("temp: {}", temp);
    }
}
