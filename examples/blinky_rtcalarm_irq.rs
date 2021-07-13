//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Please note according to RM0008:
//! "Due to the fact that the switch only sinks a limited amount of current (3 mA), the use of
//! GPIOs PC13 to PC15 in output mode is restricted: the speed has to be limited to 2MHz with
//! a maximum load of 30pF and these IOs must not be used as a current source (e.g. to drive a LED)"

#![no_std]
#![no_main]

use panic_halt as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    gpio::{gpioc, Output, PushPull},
    pac::{interrupt, Interrupt, Peripherals, EXTI},
    prelude::*,
    rtc::Rtc,
};

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;

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

// The f100 does not have an RTC, so this example is disabled
#[cfg(feature = "stm32f101")]
#[entry]
fn main() -> ! {
    loop {
        continue;
    }
}

#[cfg(not(feature = "stm32f101"))]
#[interrupt]
fn RTCALARM() {
    static mut LED: Option<LEDPIN> = None;
    static mut RTC: Option<Rtc> = None;
    static mut EXTI: Option<EXTI> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_LED.borrow(cs).replace(None).unwrap())
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

#[cfg(not(feature = "stm32f101"))]
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut pwr = dp.PWR;
    let rcc = dp.RCC.constrain();

    // Set up the GPIO pin
    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let _ = led.set_high(); // Turn off

    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));

    // Set up the EXTI (see notes in section 18.4.2 of reference manual)
    let exti = dp.EXTI;
    exti.ftsr.write(|w| w.tr17().set_bit());
    exti.imr.write(|w| w.mr17().set_bit());

    cortex_m::interrupt::free(|cs| *G_EXTI.borrow(cs).borrow_mut() = Some(exti));

    // Set up the RTC
    // Enable writes to the backup domain
    let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut pwr);
    // Start the RTC
    let mut rtc = Rtc::rtc(dp.RTC, &mut backup_domain);
    rtc.set_time(0);
    rtc.set_alarm(TOGGLE_INTERVAL_SECONDS);
    rtc.listen_alarm();

    cortex_m::interrupt::free(|cs| *G_RTC.borrow(cs).borrow_mut() = Some(rtc));

    // Enable RTCALARM IRQ
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::RTCALARM) };

    loop {
        wfi();
    }
}
