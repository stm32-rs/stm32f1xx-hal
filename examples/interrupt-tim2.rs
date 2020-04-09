//! Demonstrates timer based one shot interrupts

#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_semihosting as _; // panic handler
use stm32f1xx_hal::{
    pac::{interrupt, Interrupt, Peripherals, NVIC},
    prelude::*,
    timer::{Event, Timer},
};

#[interrupt]
fn TIM2() {
    hprintln!("TIM2 fired.").ok();

    // Disable the TIM2 interrupt so this won't fire again
    NVIC::mask(Interrupt::TIM2);
}

#[entry]
fn main() -> ! {
    hprintln!("Start").ok();

    // Extract needed peripherals
    let dp = Peripherals::take().expect("Peripherals have been taken before");
    let rcc = dp.RCC.constrain();
    let mut acr = dp.FLASH.constrain().acr;
    let clocks = rcc.cfgr.freeze(&mut acr);
    let mut apb1 = rcc.apb1;

    // Create and configure CountDownTimer<TIM2>
    let mut timer = Timer::tim2(dp.TIM2, &clocks, &mut apb1).start_count_down(1.hz());
    timer.listen(Event::Update);

    hprintln!("Entering main loop.").ok();
    loop {
        timer.clear_update_interrupt_flag();

        // Clear the TIM2 interrupt's pending state
        NVIC::unpend(Interrupt::TIM2);
        unsafe {
            // Enable the TIM2 interrupt (again)
            // Safe as this is not a mask-based critical section
            NVIC::unmask(Interrupt::TIM2);
        }

        // Wait for an event like the TIM2 interrupt to occur
        asm::wfe();
        hprintln!("An event occured.").ok();
    }
}
