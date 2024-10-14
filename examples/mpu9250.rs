//! Interfacing the MPU9250

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use mpu9250::Mpu9250;
use stm32f1xx_hal as hal;

use hal::{pac, prelude::*, spi::Spi};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split();
    // let mut gpiob = dp.GPIOB.split();

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    // SPI1
    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;

    // SPI2
    // let sck = gpiob.pb13;
    // let miso = gpiob.pb14;
    // let mosi = gpiob.pb15;

    let spi = Spi::new(
        dp.SPI1,
        (Some(sck), Some(miso), Some(mosi)),
        mpu9250::MODE.into(),
        1.MHz(),
        &clocks,
    );

    let mut delay = cp.SYST.delay(&clocks);

    let mut mpu9250 = Mpu9250::marg_default(spi, nss, &mut delay).unwrap();

    // sanity checks
    assert_eq!(mpu9250.who_am_i().unwrap(), 0x71);
    assert_eq!(mpu9250.ak8963_who_am_i().unwrap(), 0x48);

    let _a = mpu9250.all::<[f32; 3]>().unwrap();

    asm::bkpt();

    loop {}
}
