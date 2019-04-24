//! Testing PWM input

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    pwm_input::*,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut dbg = p.DBGMCU;

    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    let pb4 = gpiob.pb4.into_alternate_open_drain(&mut gpiob.crl);
    let pb5 = gpiob.pb5.into_alternate_open_drain(&mut gpiob.crl);

    let pwm_input = p.TIM3.pwm_input(
        (pb4, pb5),
        &mut rcc.apb1,
        &mut afio.mapr,
        &mut dbg,
        &clocks,
        Configuration::Frequency(10.khz()),
    );

    loop {
        let _freq = pwm_input
            .read_frequency(ReadMode::Instant, &clocks)
            .unwrap();
        let _duty_cycle = pwm_input.read_duty(ReadMode::Instant).unwrap();
    }
}
