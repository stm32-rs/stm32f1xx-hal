//! Outputs the current time in seconds to hstdout using the real time clock

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use fugit::RateExtU32;
use stm32f1xx_hal::rtc::RestoredOrNewRtc::{New, Restored};
use stm32f1xx_hal::{pac, prelude::*, rtc::Rtc};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut pwr = p.PWR;
    let rcc = p.RCC.constrain();
    let mut backup_domain = rcc.bkp.constrain(p.BKP, &mut pwr);

    // Initializes rtc every startup, use only if you don't have a battery.
    // let rtc = Rtc::new(p.RTC, &mut backup_domain);

    // Restores Rtc: that happens in case it was already running, a battery is connected,
    // and it was already initialized before.
    // If you are going to use ::new with battery, the time will lack behind
    // due to unnecessary reinitialization of the crystal,
    // as well as reset of the selected frequency.
    // Else, the rtc is initialized.
    let rtc = match Rtc::restore_or_new(p.RTC, &mut backup_domain) {
        Restored(rtc) => rtc, // The rtc is restored from previous configuration. You may verify the frequency you want if needed.
        New(mut rtc) => {
            // The rtc was just initialized, the clock source selected, frequency is 1.Hz()
            // Initialize rtc with desired parameters
            rtc.select_frequency(2u32.Hz()); // Set the frequency to 2 Hz. This will stay same after reset
            rtc
        }
    };

    loop {
        hprintln!("time: {}", rtc.current_time());
    }
}
