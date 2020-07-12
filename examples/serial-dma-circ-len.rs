//! Serial interface circular DMA RX transfer test

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m::singleton;
use cortex_m_semihosting::{hprint, hprintln};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    dma::CircReadDmaLen,
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

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
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    hprintln!("waiting for 5 bytes").unwrap();

    let rx = serial.split().1.with_dma(channels.5);
    // increase to reasonable size (e.g. 64, 128 etc.) for actual applications
    let buf = singleton!(: [u8; 8] = [0; 8]).unwrap();

    let mut circ_buffer = rx.circ_read_len(buf);

    // wait until we have 5 bytes
    while circ_buffer.len() < 5 {}

    let mut dat = [0 as u8; 4];
    assert!(circ_buffer.read(&mut dat[..]) == 4);

    hprintln!("[{}, {}, {}, {}]", dat[0], dat[1], dat[2], dat[3]).unwrap();

    // try to read again, now only one byte is returned
    hprintln!("read {}", circ_buffer.read(&mut dat)).unwrap();

    hprintln!("[{}]", dat[0]).unwrap();

    // wait for the buffer to have 4 bytes again
    while circ_buffer.len() < 4 {}

    // all four bytes should be read in one go
    assert!(circ_buffer.read(&mut dat) == 4);

    hprintln!("[{}, {}, {}, {}]", dat[0], dat[1], dat[2], dat[3]).unwrap();

    loop {
        let read = circ_buffer.read(&mut dat);

        if read > 0 {
            for c in &dat[..read] {
                hprint!("{}", *c as char).unwrap();
            }
            hprintln!("").unwrap();
        }
    }
}
