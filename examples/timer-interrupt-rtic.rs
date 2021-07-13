//! Uses the timer interrupt to blink a led with different frequencies.
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]

// you can put a breakpoint on `rust_begin_unwind` to catch panics
use panic_halt as _;

use rtic::app;

use stm32f1xx_hal::{
    gpio::{gpioc::PC13, Output, PinState, PushPull},
    pac,
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        led: PC13<Output<PushPull>>,
        timer_handler: CountDownTimer<pac::TIM1>,

        #[init(false)]
        led_state: bool,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Acquire the GPIOC peripheral
        let mut gpioc = cx.device.GPIOC.split();

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);
        // Configure the syst timer to trigger an update every second and enables interrupt
        let mut timer = Timer::tim1(cx.device.TIM1, &clocks).start_count_down(1.hz());
        timer.listen(Event::Update);

        // Init the static resources to use them later through RTIC
        init::LateResources {
            led,
            timer_handler: timer,
        }
    }

    // Optional.
    //
    // https://rtic.rs/0.5/book/en/by-example/app.html#idle
    // > When no idle function is declared, the runtime sets the SLEEPONEXIT bit and then
    // > sends the microcontroller to sleep after running init.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM1_UP, priority = 1, resources = [led, timer_handler, led_state])]
    fn tick(cx: tick::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.

        // Count used to change the timer update frequency
        static mut COUNT: u8 = 0;

        if *cx.resources.led_state {
            // Uses resources managed by rtic to turn led off (on bluepill)
            cx.resources.led.set_high();
            *cx.resources.led_state = false;
        } else {
            cx.resources.led.set_low();
            *cx.resources.led_state = true;
        }
        *COUNT += 1;

        if *COUNT == 4 {
            // Changes timer update frequency
            cx.resources.timer_handler.start(2.hz());
        } else if *COUNT == 12 {
            cx.resources.timer_handler.start(1.hz());
            *COUNT = 0;
        }

        // Clears the update flag
        cx.resources.timer_handler.clear_update_interrupt_flag();
    }
};
