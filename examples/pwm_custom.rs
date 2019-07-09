//! Testing PWM output for custom pin combinations

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use stm32f1xx_hal::{
    gpio::gpiob::{PB4, PB5},
    gpio::{Alternate, PushPull},
    pac,
    prelude::*,
    pwm::{Pins, Pwm, C1, C2},
    stm32::TIM3,
};

use cortex_m_rt::entry;

// Using PB5 channel for TIM3 PWM output 
// struct MyChannels(PB5<Alternate<PushPull>>);
// impl Pins<TIM3>  for MyChannels {
//     const REMAP: u8 = 0b10;
//     const C1: bool = false;
//     const C2: bool = true;
//     const C3: bool = false;
//     const C4: bool = false;
//     type Channels = Pwm<TIM3, C2>;
// }

// Using PB4 and PB5 channels for TIM3 PWM output
struct MyChannels(PB4<Alternate<PushPull>>, PB5<Alternate<PushPull>>);
impl Pins<TIM3> for MyChannels {
    const REMAP: u8 = 0b10;
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = (Pwm<TIM3, C1>, Pwm<TIM3, C2>);
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // TIM3
    let p0 = pb4.into_alternate_push_pull(&mut gpiob.crl);
    let p1 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let mut pwm = p.TIM3.pwm(
        MyChannels(p0, p1),
        &mut afio.mapr,
        1.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let max = pwm.0.get_max_duty();

    pwm.0.enable();
    pwm.1.enable();

    // full
    pwm.0.set_duty(max);
    pwm.1.set_duty(max);

    asm::bkpt();

    // dim
    pwm.1.set_duty(max / 4);

    asm::bkpt();

    // zero
    pwm.0.set_duty(0);
    pwm.1.set_duty(0);

    asm::bkpt();

    loop {}
}
