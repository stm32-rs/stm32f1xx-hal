#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate panic_semihosting;
extern crate stm32f1xx_hal;

use core::fmt::Write;
use cortex_m_semihosting::hio;
use stm32f1xx_hal::prelude::*;

use rt::ExceptionFrame;
use stm32f1xx_hal::adc;

#[entry]
fn main() -> ! {
    // Aquire peripherals
    let p = stm32f1xx_hal::stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Configure ADC clocks
    // Default value is the slowest possible ADC clock: PCLK2 / 8. Meanwhile ADC
    // clock is configurable. So its frequency may be tweaked to meet certain
    // practical needs. User specified value is be approximated using supported
    // prescaler values 2/4/6/8.
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);
    hio::hstdout()
        .map(|mut hio| writeln!(hio, "adc freq: {}", clocks.adcclk().0).unwrap())
        .unwrap();

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(p.ADC1, &mut rcc.apb2);

    #[cfg(feature = "stm32f103")]
    let mut adc2 = adc::Adc::adc2(p.ADC2, &mut rcc.apb2);

    // Setup GPIOB
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // Configure pb0, pb1 as an analog input
    let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

    #[cfg(feature = "stm32f103")]
    let mut ch1 = gpiob.pb1.into_analog(&mut gpiob.crl);

    loop {
        let data: u16 = adc1.read(&mut ch0).unwrap();
        hio::hstdout()
            .map(|mut hio| writeln!(hio, "adc1: {}", data).unwrap())
            .unwrap();

        #[cfg(feature = "stm32f103")]
        {
            let data1: u16 = adc2.read(&mut ch1).unwrap();
            hio::hstdout()
                .map(|mut hio| writeln!(hio, "adc2: {}", data1).unwrap())
                .unwrap();
        }
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
