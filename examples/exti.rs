//! Turns the user LED on
//!
//! Listens for interrupts on the pa7 pin. On any rising or falling edge, toggles
//! the pc13 pin (which is connected to the LED on the blue pill board, hence the `led` name).

#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac
};
use cortex_m_rt::entry;
use pac::interrupt;
use core::mem::MaybeUninit;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::*;

// These two are owned by the ISR. main() may only access them during the initialization phase,
// where the interrupt is not yet enabled (i.e. no concurrent accesses can occur).
// After enabling the interrupt, main() may not have any references to these objects any more.
// For the sake of minimalism, we do not use RTFM here, which would be the better way.
static mut LED  : MaybeUninit<stm32f1xx_hal::gpio::gpioc::PC13<Output<PushPull>>> = MaybeUninit::uninit();
static mut EXTI : MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA7<Input<Floating>>> = MaybeUninit::uninit();

#[interrupt]
fn EXTI9_5() {
    let led = unsafe { &mut *LED.as_mut_ptr()};
    let exti = unsafe { &mut *EXTI.as_mut_ptr()};

    if exti.check_interrupt() {
        led.toggle();

        // if we don't clear this bit, the ISR would trigger indefinitely
        exti.clear_interrupt_pending_bit();
    }
}

#[entry]
fn main() -> ! {
    // initialization phase
    let p = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();
    {
        // the scope ensures that the exti reference is dropped before the first ISR can be executed.

        let mut rcc = p.RCC.constrain();
        let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        let led = unsafe { &mut *LED.as_mut_ptr()};
        *led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let exti = unsafe { &mut *EXTI.as_mut_ptr()};
        *exti = gpioa.pa7.into_floating_input(&mut gpioa.crl);
        exti.make_interrupt_source(&mut afio);
        exti.trigger_on_edge(&p.EXTI, Edge::RISING_FALLING);
        exti.enable_interrupt(&p.EXTI);
    } // initialization ends here

    let mut nvic = cp.NVIC;
    nvic.enable(pac::Interrupt::EXTI9_5);

    loop {}
}
