//! Testing PWM input

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32f1xx_hal as hal;

use hal::prelude::*;
use hal::pwm_input::*;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let mut dbg = p.DBG;

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
