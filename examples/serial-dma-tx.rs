//! Serial interface DMA TX transfer test

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain();
    let channels = p.DMA1.split();

    let mut gpioa = p.GPIOA.split();
    // let mut gpiob = p.GPIOB.split();

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

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
    );

    let tx = serial.split().0.with_dma(channels.4);

    let (_, tx) = tx.write(b"The quick brown fox").wait();

    asm::bkpt();

    let (_, tx) = tx.write(b" jumps").wait();

    asm::bkpt();

    tx.write(b" over the lazy dog.").wait();

    asm::bkpt();

    loop {}
}
