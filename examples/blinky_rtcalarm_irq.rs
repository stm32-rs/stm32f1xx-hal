//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive a LED, see
//! section 5.1.2 of the reference manual for an explanation.
//! This is not an issue on the blue pill.

#![no_std]
#![no_main]

use panic_halt as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    gpio::*,
    pac::{interrupt, Interrupt, Peripherals, EXTI},
    prelude::*,
    rtc::*,
};

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

// A type definition for the GPIO pin to be used for our LED
type LEDPIN = gpioc::PC13<Output<PushPull>>;

// Make LED pin globally available
static G_LED: Mutex<RefCell<Option<LEDPIN>>> = Mutex::new(RefCell::new(None));
// Make RTC globally available
static G_RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));
// Make EXTI registers globally available
static G_EXTI: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));

// Toggle LED every 3 seconds
const TOGGLE_INTERVAL_SECONDS: u32 = 3;

#[interrupt]
fn RTCALARM() {
    static mut LED: Option<LEDPIN> = None;
    static mut RTC: Option<Rtc> = None;
    static mut EXTI: Option<EXTI> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED.borrow(cs).replace(None).unwrap()
        })
    });
    let rtc = RTC.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_RTC.borrow(cs).replace(None).unwrap())
    });
    let exti = EXTI.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_EXTI.borrow(cs).replace(None).unwrap())
    });

    exti.pr.write(|w| w.pr17().set_bit());
    rtc.set_alarm(rtc.current_time() + TOGGLE_INTERVAL_SECONDS);

    let _ = led.toggle();
}

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (Peripherals::take(), c_m_Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let mut pwr = dp.PWR;
            let mut rcc = dp.RCC.constrain();

            // Set up the GPIO pin
            let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
            let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
            let _ = led.set_high(); // Turn off

            *G_LED.borrow(cs).borrow_mut() = Some(led);

            // Set up the EXTI (see notes in section 18.4.2 of reference manual)
            let exti = dp.EXTI;
            exti.ftsr.write(|w| w.tr17().set_bit());
            exti.imr.write(|w| w.mr17().set_bit());

            *G_EXTI.borrow(cs).borrow_mut() = Some(exti);

            // Set up the RTC
            // Enable writes to the backup domain
            let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);
            // Start the RTC
            let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);
            rtc.set_time(0);
            rtc.set_alarm(0);
            rtc.listen_alarm();

            *G_RTC.borrow(cs).borrow_mut() = Some(rtc);

            // Enable RTCALARM IRQ, set prio 1 and clear any pending IRQs
            let mut nvic = cp.NVIC;
            // Calling 'set_priority()' and 'unmask()' requires 'unsafe {}'
            // - https://docs.rs/stm32f1xx-hal/0.5.3/stm32f1xx_hal/stm32/struct.NVIC.html#method.set_priority
            unsafe {
                nvic.set_priority(Interrupt::RTCALARM, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::RTCALARM);
            }
            // Mark as pending
            cortex_m::peripheral::NVIC::pend(Interrupt::RTCALARM);
        });
    }

    loop {
        wfi();
    }
}
