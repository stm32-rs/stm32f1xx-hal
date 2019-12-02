//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::{Tim2NoRemap, Timer},
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    // If you don't want to use all channels, just leave some out
    // let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);
    let pins = (c1, c2, c3);

    // TIM3
    // let c1 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    // let c2 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    // let c3 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    // let c4 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);

    // TIM4 (Only available with the "medium" density feature)
    // let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
    // let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
    // let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

    let mut pwm = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
        .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 1.khz())
        .2;

    let max = pwm.get_max_duty();

    pwm.enable();

    // full
    pwm.set_duty(max);

    asm::bkpt();

    // dim
    pwm.set_duty(max / 4);

    asm::bkpt();

    // zero
    pwm.set_duty(0);

    asm::bkpt();

    loop {}
}
