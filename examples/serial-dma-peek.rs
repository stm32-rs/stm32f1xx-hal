//! Serial interface DMA RX transfer test

#![allow(clippy::empty_loop)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::{asm, singleton};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
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

    let serial = Serial::new(p.USART1, (tx, rx), Config::default(), &mut rcc);

    let rx = serial.rx.with_dma(channels.5);
    let buf = singleton!(: [u8; 8] = [0; 8]).unwrap();

    let t = rx.read(buf);

    while !t.is_done() {
        let _slice = t.peek();

        asm::bkpt();
    }

    asm::bkpt();

    loop {}
}
