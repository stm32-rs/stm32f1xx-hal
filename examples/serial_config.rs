//! Serial Config test

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{self, Serial},
};

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    //let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split();

    // USART1
    // let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);

    // USART1
    // let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);

    // USART2
    // let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);

    // USART3
    // Configure pb10 as a push_pull output, this will be the tx pin
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);

    // Set up the usart device. Take ownership over the USART register and tx pin. The rest of
    // the registers are used to enable and configure the device.
    let mut tx = Serial::tx(
        p.USART3,
        tx,
        serial::Config::default()
            .baudrate(9600.bps())
            .stopbits(serial::StopBits::STOP2)
            .wordlength_9bits()
            .parity_odd(),
        &clocks,
    );

    let sent = b'U';
    block!(tx.write_u8(sent)).unwrap();
    block!(tx.write_u8(sent)).unwrap();

    loop {}
}
