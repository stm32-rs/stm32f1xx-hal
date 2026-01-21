//! Testing PWM output for custom pin combinations

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{pac, prelude::*};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let mut afio = p.AFIO.constrain(&mut rcc);
    let gpioa = p.GPIOA.split(&mut rcc);
    let mut gpiob = p.GPIOB.split(&mut rcc);
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // TIM3
    let p0 = pb4.into_alternate_push_pull(&mut gpiob.crl);
    let p1 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let (pwm, pwm_channels) = p.TIM3.remap(&mut afio.mapr).pwm_hz(1.kHz(), &mut rcc);

    let max = pwm.get_max_duty();

    let mut c1 = pwm_channels.0.with(p0);
    let mut c2 = pwm_channels.1.with(p1);

    // Enable the individual channels
    c1.enable();
    c2.enable();

    // full
    c1.set_duty(max);
    c2.set_duty(max);

    asm::bkpt();

    // dim
    c2.set_duty(max / 4);

    asm::bkpt();

    // zero
    c1.set_duty(0);
    c2.set_duty(0);

    asm::bkpt();

    loop {}
}
