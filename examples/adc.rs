//! Makes an analog reading on channel 0 and prints it to itm

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_rt as rt;
extern crate cortex_m;
extern crate cortex_m_semihosting;
extern crate panic_semihosting;
extern crate stm32f1xx_hal;
extern crate embedded_hal;
extern crate nb;

use nb::block;

use core::fmt::Write;

use cortex_m_semihosting::hio;

use stm32f1xx_hal::prelude::*;

use rt::ExceptionFrame;
use stm32f1xx_hal::adc::{self};
use embedded_hal::adc::OneShot;

#[entry]
fn main() -> ! {
    // Aquire the peripherals
    let p = stm32f1xx_hal::stm32::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    // Set up the ADC
    let mut adc = adc::Adc::adc2(p.ADC2, &mut rcc.apb2);

    // Configure gpioa 0 as an analog input
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut pb1 = gpiob.pb1.into_analog(&mut gpiob.crl);


    loop {
        // Make a reading
        let reading = block!(adc.read(&mut pb1)).unwrap();

        // Aquire stdout and print the result of an analog reading
        // NOTE: This will probably freeze when running without a debugger connected.
        hio::hstdout().map(|mut hio| {
            writeln!(hio, "reading: {}", reading).unwrap()
        }).unwrap();
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
