#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_itm as _;

use cortex_m::iprintln;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    spi::Spi,
};
use mfrc522::Mfrc522;
use cortex_m_rt::entry;
use embedded_hal::digital::{v1_compat::OldOutputPin, v2::OutputPin};

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let _stim = &mut cp.ITM.stim[0];
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        mfrc522::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let mut mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high().unwrap();

    loop {
        if let Ok(atqa) = mfrc522.reqa() {
            if let Ok(uid) = mfrc522.select(&atqa) {
                iprintln!(_stim, "* {:?}", uid);
            }
        }
    }
}
