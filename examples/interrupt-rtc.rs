//! Demonstrates real time clock based one shot interrupts

#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_semihosting as _; // panic handler
use stm32f1xx_hal::{
    pac::{interrupt, Interrupt, Peripherals, NVIC},
    prelude::*,
    rtc::Rtc,
};

#[interrupt]
fn RTC() {
    hprintln!("RTC fired.").ok();

    // Disable the RTC interrupt so this won't fire again
    NVIC::mask(Interrupt::RTC);
}

#[entry]
fn main() -> ! {
    hprintln!("Entered main.").ok();

    // Extract needed peripherals
    let dp = Peripherals::take().expect("Peripherals have been taken before");
    let rcc = dp.RCC.constrain();
    let mut apb1 = rcc.apb1;
    let mut pwr = dp.PWR;
    let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut apb1, &mut pwr);

    // Create and configure Rtc
    let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);
    rtc.select_frequency(1.hz());
    rtc.listen_alarm();

    hprintln!("Entering main loop.").ok();
    loop {
        rtc.set_alarm(rtc.current_time() + 1);

        // Clear the RTC interrupt's pending state
        NVIC::unpend(Interrupt::RTC);
        unsafe {
            // Enable the RTC interrupt (again)
            // Safe as this is not a mask-based critical section
            NVIC::unmask(Interrupt::RTC);
        }

        // Wait for an event like the RTC interrupt to occur
        asm::wfe();
        hprintln!("An event occured.").ok();
    }
}
