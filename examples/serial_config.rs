//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use nb::block;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    serial::{self, Serial},
    timer::Timer,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

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

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let serial = Serial::usart3(
        p.USART3,
        (tx, rx),
        &mut afio.mapr,
        serial::Config::default()
            .baudrate(9600.bps())
            .stopbits(serial::StopBits::STOP2)
            .parity_odd(),
        clocks,
        &mut rcc.apb1,
    );

    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, _rx) = serial.split();

    let sent = b'U';
    block!(tx.write(sent)).ok();
    block!(tx.write(sent)).ok();


    loop {}
}
