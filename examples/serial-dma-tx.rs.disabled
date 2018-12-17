//! Serial interface DMA TX transfer test

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use cortex_m::asm;
use hal::prelude::*;
use hal::serial::Serial;
use hal::stm32f103xx;
use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let p = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let channels = p.DMA1.split(&mut rcc.ahb);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

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
        9_600.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let tx = serial.split().0;

    let (_, c, tx) = tx.write_all(channels.4, b"The quick brown fox").wait();

    asm::bkpt();

    let (_, c, tx) = tx.write_all(c, b" jumps").wait();

    asm::bkpt();

    tx.write_all(c, b" over the lazy dog.").wait();

    asm::bkpt();

    loop {}
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
