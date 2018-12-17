//! Serial interface circular DMA RX transfer test

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m_rt as rt;
#[macro_use(singleton)]
extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use cortex_m::asm;
use hal::dma::Half;
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

    let rx = serial.split().1;
    let buf = singleton!(: [[u8; 8]; 2] = [[0; 8]; 2]).unwrap();

    let mut circ_buffer = rx.circ_read(channels.5, buf);

    while circ_buffer.readable_half().unwrap() != Half::First {}

    let _first_half = circ_buffer.peek(|half, _| *half).unwrap();

    while circ_buffer.readable_half().unwrap() != Half::Second {}

    let _second_half = circ_buffer.peek(|half, _| *half).unwrap();

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
