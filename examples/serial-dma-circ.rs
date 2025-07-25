//! Serial interface circular DMA RX transfer test

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::{asm, singleton};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    dma::Half,
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    //let mut afio = p.AFIO.constrain();
    let channels = p.DMA1.split(&mut rcc);

    let mut gpioa = p.GPIOA.split(&mut rcc);
    // let mut gpiob = p.GPIOB.split(&mut rcc);

    // USART1
    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    // USART1
    // let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let rx = gpiob.pb7;

    // USART2
    // let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    // let rx = gpioa.pa3;

    // USART3
    // let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // let rx = gpiob.pb11;

    let serial = Serial::new(
        p.USART1,
        (tx, rx),
        Config::default().baudrate(9_600.bps()),
        &mut rcc,
    );

    let rx = serial.rx.with_dma(channels.5);
    let buf = singleton!(: [[u8; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = rx.circ_read(buf);

    while circ_buffer.readable_half().unwrap() != Half::First {}

    let _first_half = circ_buffer.peek(|half, _| *half).unwrap();

    while circ_buffer.readable_half().unwrap() != Half::Second {}

    let _second_half = circ_buffer.peek(|half, _| *half).unwrap();

    asm::bkpt();

    loop {}
}
