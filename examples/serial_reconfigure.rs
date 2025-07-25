//! Serial interface reconfiguration test
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
    serial::{self, Config, Serial},
};

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
    let mut serial = Serial::new(
        p.USART3,
        (tx, rx),
        Config::default().baudrate(9600.bps()),
        &mut rcc,
    );

    // Loopback test. Write `X` and wait until the write is successful.
    let sent = b'X';
    block!(serial.tx.write_u8(sent)).unwrap();

    // Read the byte that was just sent. Blocks until the read is complete
    let received = block!(serial.rx.read()).unwrap();

    // Since we have connected tx and rx, the byte we sent should be the one we received
    assert_eq!(received, sent);

    // Trigger a breakpoint to allow us to inspect the values
    asm::bkpt();

    // You can reconfigure the serial port to use a different baud rate at runtime.
    // This may block for a while if the transmission is still in progress.
    block!(serial.reconfigure(Config::default().baudrate(115_200.bps()), &rcc.clocks)).unwrap();

    // Let's see if it works.'
    let sent = b'Y';
    block!(serial.tx.write_u8(sent)).unwrap();
    let received = block!(serial.rx.read()).unwrap();
    assert_eq!(received, sent);
    asm::bkpt();

    // You can reconfigure the serial port after split.
    let (mut tx, mut rx) = serial.split();
    block!(serial::reconfigure(
        &mut tx,
        &mut rx,
        Config::default().baudrate(9600.bps()),
        &rcc.clocks
    ))
    .unwrap();

    loop {}
}
