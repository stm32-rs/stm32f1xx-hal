#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_halt as _;
use stm32f1xx_hal::{adc, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut rng = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks).as_rng();

    loop {
        let mut buf = [0; 8];
        if let Ok(_) = rng.read(&mut buf) {
            hprintln!("buf: {:02X?}", buf).unwrap();
        }
    }
}
