// blinky timer using interrupts on TIM2
//
// This demo based off of the following demo:
// - https://github.com/stm32-rs/stm32f0xx-hal/blob/master/examples/blinky_timer_irq.rs
// with some information about STM32F1 interrupts/peripherals from:
// - https://github.com/geomatsi/rust-blue-pill-tests/blob/master/src/bin/blink-timer-irq-safe.rs

#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    gpio::*,
    prelude::*,
    pac::{interrupt, Interrupt, Peripherals, TIM2},
    timer::*,
};

use core::cell::RefCell;
use cortex_m::{
    asm::wfi,
    interrupt::Mutex,
    peripheral::Peripherals as c_m_Peripherals};
use cortex_m_rt::entry;

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

    //hprintln!("TIM2 IRQ fired").unwrap();
    led.toggle().ok();
    tim.wait().ok();
}

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (Peripherals::take(), c_m_Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            // Configure PC13 pin to blink LED
            let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
            let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

            // Move the pin into our global storage
            *G_LED.borrow(cs).borrow_mut() = Some(led);

            // Set up a timer expiring after 1s
            let mut timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());

            // Generate an interrupt when the timer expires
            timer.listen(Event::Update);

            // Move the timer into our global storage
            *G_TIM.borrow(cs).borrow_mut() = Some(timer);

            // Enable TIM2 IRQ, set prio 1 and clear any pending IRQs
            let mut nvic = cp.NVIC;
            // Calling 'set_priority()' and 'unmask()' requires 'unsafe {}'
            // - https://docs.rs/stm32f1xx-hal/0.5.3/stm32f1xx_hal/stm32/struct.NVIC.html#method.set_priority
            unsafe {
                nvic.set_priority(Interrupt::TIM2, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
            }
            // Clear the interrupt state
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
        });
    }

    //hprintln!("Entering main loop...").unwrap();
    loop {
        // From 'cortex_m::asm::wfi'
        wfi();
    }
}
