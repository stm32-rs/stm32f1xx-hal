#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

extern crate rtic;

use panic_halt as _;
use rtic::app;
use stm32f1xx_hal::{
    gpio::{gpioa::PA0, gpioc::PC13, Edge, ExtiPin, Input, Output, PullDown, PushPull},
    prelude::*,
};

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        button: PA0<Input<PullDown>>,
        led: PC13<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        let mut afio = ctx.device.AFIO.constrain();

        let mut gpioc = ctx.device.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let mut gpioa = ctx.device.GPIOA.split();
        let mut button = gpioa.pa0.into_pull_down_input(&mut gpioa.crl);
        button.make_interrupt_source(&mut afio);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);

        init::LateResources { button, led }
    }

    #[task(binds = EXTI0, resources = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.resources.button.clear_interrupt_pending_bit();
        ctx.resources.led.toggle();
    }
};
