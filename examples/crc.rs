//! CRC calculation

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut crc = p.CRC.new(&mut rcc.ahb);

    crc.reset();
    crc.write(0x12345678);

    let val = crc.read();
    hprintln!("found={:08x}, expected={:08x}", val, 0xdf8a8a2bu32).ok();

    loop {}
}
