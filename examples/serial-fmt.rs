//! Serial interface write formatted strings test
//!
//! You need to connect the Tx pin to the Rx pin of a serial-usb converter
//! so you can see the message in a serial console (e.g. Arduino console).

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};

use core::fmt::Write;

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    // Prepare the alternate function I/O registers
    //let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split(&mut rcc);

    // USART1
    // let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    // let rx = gpioa.pa10;

    // USART1
    // let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let rx = gpiob.pb7;

    // USART2
    // let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    // let rx = gpioa.pa3;

    // USART3
    // Configure pb10 as a push_pull output, this will be the tx pin
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // Take ownership over pb11
    let rx = gpiob.pb11;

    // Set up the usart device. Take ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let serial = Serial::new(
        p.USART3,
        (tx, rx),
        Config::default().baudrate(9600.bps()),
        &mut rcc,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, _rx) = serial.split();

    let number = 103;
    writeln!(tx, "Hello formatted string {}", number).unwrap();

    // for windows
    // write!(tx, "Hello formatted string {}\r\n", number).unwrap();

    loop {}
}
