//! Uses the timer interrupt to blink a led with different frequencies.
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manaual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
use panic_halt as _;

use cortex_m::asm::wfi;
use rtfm::app;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::{ Timer, Event },
    gpio::{ gpioc::PC13, State, Output, PushPull },
};

#[app(device = stm32f1xx_hal::pac)]
const APP: () = {

    static mut LED: PC13<Output<PushPull>> = ();
    static mut TIMER_HANDLER: Timer<pac::TIM1> = ();
    static mut LED_STATE: bool = false;
    
    #[init]
    fn init() -> init::LateResources {

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Acquire the GPIOC peripheral
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, State::High);
        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer = Timer::tim1(device.TIM1, 1.hz(), clocks, &mut rcc.apb2);
        timer.listen(Event::Update);

        // Init the static resources to use them later through RTFM
        init::LateResources {
            LED: led,
            TIMER_HANDLER: timer,
        }
    }

    #[idle]
    fn idle() -> ! {

        loop {
            // Waits for interrupt
            wfi();
        }
    }

    #[interrupt(priority = 1, resources = [LED, TIMER_HANDLER, LED_STATE])]
    fn TIM1_UP() {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        // Count used to change the timer update frequency
        static mut COUNT: u8 = 0;

        if *resources.LED_STATE {
            // Uses resourcers managed by rtfm to turn led off (on bluepill)
            resources.LED.set_high();
            *resources.LED_STATE = false;
        } else {
            resources.LED.set_low();
            *resources.LED_STATE = true;
        }
        *COUNT += 1;

        if *COUNT == 4 {
            // Changes timer update frequency
            resources.TIMER_HANDLER.start(2.hz());
        } else if *COUNT == 12 {
            resources.TIMER_HANDLER.start(1.hz());
            *COUNT = 0;
        }

        // Clears the update flag
        resources.TIMER_HANDLER.clear_update_interrupt_flag();
    }
};
