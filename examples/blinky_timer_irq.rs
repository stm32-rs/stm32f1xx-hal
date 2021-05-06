//! blinky timer using interrupts on TIM2
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Please note according to RM0008:
//! "Due to the fact that the switch only sinks a limited amount of current (3 mA), the use of
//! GPIOs PC13 to PC15 in output mode is restricted: the speed has to be limited to 2MHz with
//! a maximum load of 30pF and these IOs must not be used as a current source (e.g. to drive a LED)"

#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    gpio::{gpioc, Output, PushPull},
    pac::{interrupt, Interrupt, Peripherals, TIM2},
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

use core::cell::RefCell;
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

// NOTE You can uncomment 'hprintln' here and in the code below for a bit more
// verbosity at runtime, at the cost of throwing off the timing of the blink
// (using 'semihosting' for printing debug info anywhere slows program
// execution down)
//use cortex_m_semihosting::hprintln;

// A type definition for the GPIO pin to be used for our LED
type LEDPIN = gpioc::PC13<Output<PushPull>>;

// Make LED pin globally available
static G_LED: Mutex<RefCell<Option<LEDPIN>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));

// Define an interupt handler, i.e. function to call when interrupt occurs.
// This specific interrupt will "trip" when the timer TIM2 times out
#[interrupt]
fn TIM2() {
    static mut LED: Option<LEDPIN> = None;
    static mut TIM: Option<CountDownTimer<TIM2>> = None;

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

    let _ = tim.wait();
    let _ = led.toggle();
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);

    // Configure PC13 pin to blink LED
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let _ = led.set_high(); // Turn off

    // Move the pin into our global storage
    cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));

    // Set up a timer expiring after 1s
    let mut timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());

    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);

    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));

    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    loop {
        wfi();
    }
}
