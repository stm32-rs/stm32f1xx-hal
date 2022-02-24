//! Testing PWM input

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{pwm_input::*, Timer},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain();
    let mut dbg = p.DBGMCU;

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    let pb5 = gpiob.pb5;

    let pwm_input = Timer::new(p.TIM3, &clocks).pwm_input(
        (pb4, pb5),
        &mut afio.mapr,
        &mut dbg,
        Configuration::Frequency(10.kHz()),
    );

    loop {
        let _freq = pwm_input
            .read_frequency(ReadMode::Instant, &clocks)
            .unwrap();
        let _duty_cycle = pwm_input.read_duty(ReadMode::Instant).unwrap();
    }
}
