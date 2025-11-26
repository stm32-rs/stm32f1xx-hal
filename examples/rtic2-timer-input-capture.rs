#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use stm32f1xx_hal::{
    pac,
    pac::{TIM2, TIM3},
    prelude::*,
    rcc,
    timer::{CaptureChannel, CaptureHzManager, CapturePolarity, Event, PwmChannel, Timer},
};

use rtic::app;

#[app(device = pac, dispatchers = [USART1], peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tim3: CaptureHzManager<TIM3>,
        ch1: CaptureChannel<TIM3, 0>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp
            .RCC
            .freeze(rcc::Config::hsi().sysclk(48.MHz()), &mut flash.acr);
        let gpioa = dp.GPIOA.split(&mut rcc);

        // Configuration of TIM2 in PWM mode
        let timer = Timer::new(dp.TIM2, &mut rcc);
        let (_, (ch1, ..)) = timer.pwm_hz(893.Hz());
        let mut tim_2: PwmChannel<TIM2, 0> = ch1.with(gpioa.pa0);
        tim_2.set_duty(50);
        tim_2.enable();

        // It is necessary to connect pins PA0 and PA5 through a resistor of 1 kΩ - 10 kΩ

        // Configuration of TIM3 in input capture mode
        let (mut tim3, (ch1, ..)) = Timer::new(dp.TIM3, &mut rcc).capture_hz(48.MHz());
        let mut ch1 = ch1.with(gpioa.pa6);
        tim3.listen(Event::C1);

        ch1.set_polarity(CapturePolarity::ActiveHigh);
        ch1.enable();

        defmt::info!("Start");

        (Shared {}, Local { tim3, ch1 })
    }

    #[task(binds = TIM3, local = [tim3, ch1, prev_capture: u32 = 0], priority = 3)]
    fn tim3_interrupt(cx: tim3_interrupt::Context) {
        if cx.local.tim3.get_interrupt().contains(Event::C1) {
            let timer_clock = cx.local.tim3.get_timer_clock();
            let max_auto_reload = cx.local.tim3.get_max_auto_reload();
            let current_capture = cx.local.ch1.get_capture();

            let delta = if current_capture >= *cx.local.prev_capture {
                current_capture - *cx.local.prev_capture
            } else {
                (max_auto_reload - *cx.local.prev_capture) + current_capture
            };

            let freq = timer_clock as f32 / delta as f32;

            defmt::info!("Freq: {} Hz", freq); // Output = Freq: 893.00665 Hz

            *cx.local.prev_capture = current_capture;
            cx.local.tim3.clear_interrupt(Event::C1);
        }
    }
}
