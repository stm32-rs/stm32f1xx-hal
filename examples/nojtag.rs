//! Disables the JTAG ports to give access to pb3, pb4 and PA15

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut gpioa = p.GPIOA.split();
    let mut gpiob = p.GPIOB.split();
    let mut afio = p.AFIO.constrain();

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
