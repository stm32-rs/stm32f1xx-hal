//! blinky timer using interrupts on TIM2
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Please note according to RM0008:
//! "Due to the fact that the switch only sinks a limited amount of current (3 mA), the use of
//! GPIOs PC13 to PC15 in output mode is restricted: the speed has to be limited to 2MHz with
//! a maximum load of 30pF and these IOs must not be used as a current source (e.g. to drive a LED)"

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    gpio::{gpioc, Output, PinState, PushPull},
    pac::{interrupt, Interrupt, Peripherals, TIM2},
    prelude::*,
    rcc,
    timer::{CounterMs, Event},
};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

// NOTE You can uncomment 'hprintln' here and in the code below for a bit more
// verbosity at runtime, at the cost of throwing off the timing of the blink
// (using 'semihosting' for printing debug info anywhere slows program
// execution down)
//use cortex_m_semihosting::hprintln;

// A type definition for the GPIO pin to be used for our LED
type LedPin = gpioc::PC13<Output<PushPull>>;

// Make LED pin globally available
static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CounterMs<TIM2>>>> = Mutex::new(RefCell::new(None));

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the timer TIM2 times out
#[interrupt]
fn TIM2() {
    static mut LED: Option<LedPin> = None;
    static mut TIM: Option<CounterMs<TIM2>> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_LED.borrow(cs).replace(None).unwrap()
        })
    });

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });

    led.toggle();
    tim.wait().ok();
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hsi().sysclk(8.MHz()).pclk1(8.MHz()),
        &mut flash.acr,
    );

    // Configure PC13 pin to blink LED
    let mut gpioc = dp.GPIOC.split(&mut rcc);
    let led = Output::new(gpioc.pc13, &mut gpioc.crh, PinState::High);
    //or
    //let led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);

    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));

    // Set up a timer expiring after 1s
    let mut timer = dp.TIM2.counter_ms(&mut rcc);
    timer.start(1.secs()).unwrap();

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    loop {}
}
