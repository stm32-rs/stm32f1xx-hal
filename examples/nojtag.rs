//! Disables the JTAG ports to give access to pb3, pb4 and PA15

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use stm32f1xx_hal::{
    prelude::*,
    pac,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    afio.mapr.disable_jtag();

    gpiob.pb4.into_push_pull_output(&mut gpiob.crl).set_low();

    loop {}
}
