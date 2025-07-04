//! Testing PWM output for custom pin combinations

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

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

    let pwm = Timer::new(p.TIM3, &mut rcc).pwm_hz((p0, p1), &mut afio.mapr, 1.kHz());

    let max = pwm.get_max_duty();

    let mut pwm_channels = pwm.split();

    // Enable the individual channels
    pwm_channels.0.enable();
    pwm_channels.1.enable();

    // full
    pwm_channels.0.set_duty(max);
    pwm_channels.1.set_duty(max);

    asm::bkpt();

    // dim
    pwm_channels.1.set_duty(max / 4);

    asm::bkpt();

    // zero
    pwm_channels.0.set_duty(0);
    pwm_channels.1.set_duty(0);

    asm::bkpt();

    loop {}
}
