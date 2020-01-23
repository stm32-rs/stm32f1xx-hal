//! Testing the Quadrature Encoder Interface

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use stm32f1xx_hal::{delay::Delay, pac, prelude::*, qei::QeiOptions, timer::Timer};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // let gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let gpiob = dp.GPIOB.split(&mut rcc.apb2);

    // TIM2
    // let c1 = gpioa.pa0;
    // let c2 = gpioa.pa1;

    // TIM3
    // let c1 = gpioa.pa6;
    // let c2 = gpioa.pa7;

    // TIM4
    let c1 = gpiob.pb6;
    let c2 = gpiob.pb7;

    let qei = Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1).qei(
        (c1, c2),
        &mut afio.mapr,
        QeiOptions::default(),
    );
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        let before = qei.count();
        delay.delay_ms(1_000_u16);
        let after = qei.count();

        let elapsed = after.wrapping_sub(before) as i16;

        hprintln!("{}", elapsed).unwrap();
    }
}
