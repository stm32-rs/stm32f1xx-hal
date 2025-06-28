#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, rcc};

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Acquire peripherals
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.freeze(
        rcc::Config::hse(8.MHz())
            .sysclk(56.MHz())
            .pclk1(28.MHz())
            .adcclk(14.MHz()),
        &mut flash.acr,
    );

    /*
    // Alternative configuration using dividers and multipliers directly
    let rcc = p.RCC.freeze(
        rcc::RawConfig {
            hse: Some(8_000_000),
            pllmul: Some(7),
            hpre: rcc::HPre::Div1,
            ppre1: rcc::PPre::Div2,
            ppre2: rcc::PPre::Div1,
            usbpre: rcc::UsbPre::Div1_5,
            adcpre: rcc::AdcPre::Div2,
            ..Default::default()
        },
        &mut flash.acr,
    );*/
    hprintln!("sysclk freq: {}", rcc.clocks.sysclk());
    hprintln!("adc freq: {}", rcc.clocks.adcclk());

    // Setup ADC
    let mut adc = p.ADC1.adc(&mut rcc);

    // Read temperature sensor
    loop {
        let temp = adc.read_temp();

        hprintln!("temp: {}", temp);
    }
}
