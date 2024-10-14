#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_itm as _;

use cortex_m::iprintln;

use cortex_m_rt::entry;
use mfrc522::{comm::eh02::spi::SpiInterface, Mfrc522};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let _stim = &mut cp.ITM.stim[0];
    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;
    let spi = Spi::new(
        dp.SPI1,
        (Some(sck), Some(miso), Some(mosi)),
        MODE,
        1.MHz(),
        &clocks,
    );

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let itf = SpiInterface::new(spi).with_nss(nss);
    let mut mfrc522 = Mfrc522::new(itf).init().unwrap();

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high();

    loop {
        if let Ok(atqa) = mfrc522.reqa() {
            if let Ok(uid) = mfrc522.select(&atqa) {
                iprintln!(_stim, "* {:?}", uid.as_bytes());
            }
        }
    }
}
