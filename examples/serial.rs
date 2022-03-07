//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;

use nb::block;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};
use unwrap_infallible::UnwrapInfallible;

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
    let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split();

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
    let mut serial = Serial::new(
        p.USART3,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        &clocks,
    );

    // Loopback test. Write `X` and wait until the write is successful.
    let sent = b'X';
    block!(serial.tx.write(sent)).unwrap_infallible();

    // Read the byte that was just sent. Blocks until the read is complete
    let received = block!(serial.rx.read()).unwrap();

    // Since we have connected tx and rx, the byte we sent should be the one we received
    assert_eq!(received, sent);

    // Trigger a breakpoint to allow us to inspect the values
    asm::bkpt();

    // You can also split the serial struct into a receiving and a transmitting part
    let (mut tx, mut rx) = serial.split();
    let sent = b'Y';
    block!(tx.write(sent)).unwrap_infallible();
    let received = block!(rx.read()).unwrap();
    assert_eq!(received, sent);
    asm::bkpt();

    loop {}
}
