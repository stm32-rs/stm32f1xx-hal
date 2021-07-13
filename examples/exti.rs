//! Turns the user LED on
//!
//! Listens for interrupts on the pa7 pin. On any rising or falling edge, toggles
//! the pc13 pin (which is connected to the LED on the blue pill board, hence the `led` name).

#![no_main]
#![no_std]

use panic_halt as _;

use core::mem::MaybeUninit;
use cortex_m_rt::entry;
use pac::interrupt;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::{pac, prelude::*};

// These two are owned by the ISR. main() may only access them during the initialization phase,
// where the interrupt is not yet enabled (i.e. no concurrent accesses can occur).
// After enabling the interrupt, main() may not have any references to these objects any more.
// For the sake of minimalism, we do not use RTIC here, which would be the better way.
static mut LED: MaybeUninit<stm32f1xx_hal::gpio::gpioc::PC13<Output<PushPull>>> =
    MaybeUninit::uninit();
static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA7<Input<Floating>>> =
    MaybeUninit::uninit();

#[interrupt]
fn EXTI9_5() {
    let led = unsafe { &mut *LED.as_mut_ptr() };
    let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };

    if int_pin.check_interrupt() {
        led.toggle();

        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin.clear_interrupt_pending_bit();
    }
}

#[entry]
fn main() -> ! {
    // initialization phase
    let p = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();
    {
        // the scope ensures that the int_pin reference is dropped before the first ISR can be executed.

        let mut gpioa = p.GPIOA.split();
        let mut gpioc = p.GPIOC.split();
        let mut afio = p.AFIO.constrain();

        let led = unsafe { &mut *LED.as_mut_ptr() };
        *led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *int_pin = gpioa.pa7.into_floating_input(&mut gpioa.crl);
        int_pin.make_interrupt_source(&mut afio);
        int_pin.trigger_on_edge(&p.EXTI, Edge::RisingFalling);
        int_pin.enable_interrupt(&p.EXTI);
    } // initialization ends here

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI9_5);
    }

    loop {}
}
