//! Blinks an LED using the real time clock to time the blinks

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f1xx_hal as hal;
#[macro_use(block)]
extern crate nb;

use hal::prelude::*;
use hal::stm32;
use rt::{entry, exception, ExceptionFrame};
use hal::rtc::Rtc;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut pwr = dp.PWR;
    let mut rcc = dp.RCC.constrain();

    // Set up the GPIO pin
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Set up the RTC
    // Enable writes to the backup domain
    let backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);
    // Start the LSE which is used as the clock for the RTC
    let lse = rcc.lse.freeze(&backup_domain);
    // Aquire the RTC
    let mut rtc = Rtc::rtc(dp.RTC, lse, &backup_domain);

    let mut led_on = false;
    loop {
        // Set the current time to 0
        rtc.set_cnt(0);
        // Trigger the alarm in 5 seconds
        rtc.set_alarm(5);
        block!(rtc.wait_alarm()).unwrap();
        if led_on {
            led.set_low();
            led_on = false;
        }
        else {
            led.set_high();
            led_on = true;
        }
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
