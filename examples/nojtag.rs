//! Disables the JTAG ports to give access to pb3, pb4 and PA15

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut pa15 = pa15.into_push_pull_output(&mut gpioa.crh);
    let mut pb3 = pb3.into_push_pull_output(&mut gpiob.crl);
    let mut pb4 = pb4.into_push_pull_output(&mut gpiob.crl);

    loop {
        pa15.toggle();
        pb3.toggle();
        pb4.toggle();
        cortex_m::asm::delay(8_000_000);
    }
}
