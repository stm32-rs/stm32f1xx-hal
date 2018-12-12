//! Interfacing the MPU9250

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate mpu9250;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;

use cortex_m::asm;
use hal::delay::Delay;
use hal::prelude::*;
use hal::spi::Spi;
use hal::stm32f103xx;
use mpu9250::Mpu9250;
use rt::{entry, exception, ExceptionFrame};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    // SPI2
    // let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    // let miso = gpiob.pb14;
    // let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut delay = Delay::new(cp.SYST, clocks);

    let mut mpu9250 = Mpu9250::marg(spi, nss, &mut delay).unwrap();

    // sanity checks
    assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let _a = mpu9250.all().unwrap();

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
