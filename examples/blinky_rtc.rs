//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive a LED, see
//! section 5.1.2 of the reference manaual for an explanation.
//! This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    rtc::Rtc,
};

use nb::block;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut pwr = dp.PWR;
    let mut rcc = dp.RCC.constrain();

    // Set up the GPIO pin
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Set up the RTC
    // Enable writes to the backup domain
    let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);
    // Start the RTC
    let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    let mut led_on = false;
    loop {
        // Set the current time to 0
        rtc.set_seconds(0);
        // Trigger the alarm in 5 seconds
        rtc.set_alarm(5);
        block!(rtc.wait_alarm()).unwrap();
        if led_on {
            led.set_low().unwrap();
            led_on = false;
        }
        else {
            led.set_high().unwrap();
            led_on = true;
        }
    }
}
