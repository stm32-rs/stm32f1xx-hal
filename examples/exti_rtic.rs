#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use stm32f1xx_hal::{
        gpio::{gpioa::PA0, gpioc::PC13, Edge, ExtiPin, Input, Output, PullDown, PushPull},
        prelude::*,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        button: PA0<Input<PullDown>>,
        led: PC13<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut afio = ctx.device.AFIO.constrain();

        let mut gpioc = ctx.device.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let mut gpioa = ctx.device.GPIOA.split();
        let mut button = gpioa.pa0.into_pull_down_input(&mut gpioa.crl);
        button.make_interrupt_source(&mut afio);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);

        (Shared {}, Local { button, led }, init::Monotonics())
    }

    #[task(binds = EXTI0, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }
}
