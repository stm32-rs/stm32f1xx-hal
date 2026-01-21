//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, time::ms};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let gpioa = p.GPIOA.split(&mut rcc);
    // let mut gpiob = p.GPIOB.split(&mut rcc);

    // TIM2
    let c1 = gpioa.pa0;
    let c2 = gpioa.pa1;
    let c3 = gpioa.pa2;
    // If you don't want to use all channels, just leave some out
    // let c4 = gpioa.pa3;

    // TIM3
    // let c1 = gpioa.pa6;
    // let c2 = gpioa.pa7;
    // let c3 = gpiob.pb0;
    // let c4 = gpiob.pb1;

    // TIM4 (Only available with the "medium" density feature)
    // let c1 = gpiob.pb6;
    // let c2 = gpiob.pb7;
    // let c3 = gpiob.pb8;
    // let c4 = gpiob.pb9;

    //let mut pwm =
    //    Timer::new(p.TIM2, &mut rcc).pwm_hz(pins, 1.kHz());
    // or
    let (mut pwm_mgr, (pwm_c1, pwm_c2, pwm_c3, ..)) = p.TIM2.pwm_hz(1.kHz(), &mut rcc);

    // Enable clock on each of the channels
    let mut c1 = pwm_c1.with(c1);
    c1.enable();
    let mut c2 = pwm_c2.with(c2);
    c2.enable();
    let mut c3 = pwm_c3.with(c3);
    c3.enable();

    //// Operations affecting all defined channels on the Timer

    // Adjust period to 0.5 seconds
    pwm_mgr.set_period(ms(500).into_rate());

    asm::bkpt();

    // Return to the original frequency
    pwm_mgr.set_period(1.kHz());

    asm::bkpt();

    let max = pwm_mgr.get_max_duty();

    //// Operations affecting single channels can be accessed through
    //// the Pwm object or via dereferencing to the pin.

    // Use the Pwm object to set C3 to full strength
    c3.set_duty(max);

    asm::bkpt();

    // Use the Pwm object to set C3 to be dim
    c3.set_duty(max / 4);

    asm::bkpt();

    // Use the Pwm object to set C3 to be zero
    c3.set_duty(0);

    asm::bkpt();

    loop {}
}
