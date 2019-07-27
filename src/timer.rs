//! # Timer

use crate::hal::timer::{CountDown, Periodic};
use crate::pac::{TIM1, TIM2, TIM3, TIM4};
use cast::{u16, u32};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use nb;
use void::Void;

use crate::rcc::{Clocks, APB1};
#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
))]
use crate::rcc::APB2;

use crate::time::Hertz;

/// Associated clocks with timers
pub trait PclkSrc {
    fn get_clk(clocks: &Clocks) -> Hertz;
}

macro_rules! impl_pclk {
    ($TIMX:ident, $pclkX:ident) => {
        impl PclkSrc for $TIMX {
            fn get_clk(clocks: &Clocks) -> Hertz {
                clocks.$pclkX()
            }
        }
    };
}

impl_pclk! {TIM1, pclk2_tim}
impl_pclk! {TIM2, pclk1_tim}
impl_pclk! {TIM3, pclk1_tim}
impl_pclk! {TIM4, pclk1_tim}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

pub struct Timer<TIM> {
    tim: TIM,
    clocks: Clocks,
}

impl Timer<SYST> {
    pub fn syst<T>(mut syst: SYST, timeout: T, clocks: Clocks) -> Self
    where
        T: Into<Hertz>,
    {
        syst.set_clock_source(SystClkSource::Core);
        let mut timer = Timer { tim: syst, clocks };
        timer.start(timeout);
        timer
    }

    /// Starts listening for an `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Update => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Update => self.tim.disable_interrupt(),
        }
    }

    /// Stops the timer
    pub fn stop(&mut self) {
        self.tim.disable_counter();
    }

    /// Releases the SYST
    pub fn release(mut self) -> SYST {
        self.stop();
        self.tim
    }
}

impl CountDown for Timer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clocks.sysclk().0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.tim.set_reload(rvr);
        self.tim.clear_current();
        self.tim.enable_counter();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.tim.has_wrapped() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Periodic for Timer<SYST> {}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident, $apbX:ident),)+) => {
        $(
            impl Timer<$TIMX> {
                pub fn $timX<T>(tim: $TIMX, timeout: T, clocks: Clocks, apb1: &mut $apbX) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb1.enr().modify(|_, w| w.$timXen().set_bit());
                    apb1.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer { clocks, tim };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().set_bit()),
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().clear_bit()),
                    }
                }

                /// Stops the timer
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Releases the TIM Peripheral
                pub fn release(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }
            }

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.stop();

                    let frequency = timeout.into().0;
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| w.psc().bits(psc) );

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| w.arr().bits(arr) );

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Periodic for Timer<$TIMX> {}
        )+
    }
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
))]
hal! {
    TIM1: (tim1, tim1en, tim1rst, APB2),
}

hal! {
    TIM2: (tim2, tim2en, tim2rst, APB1),
    TIM3: (tim3, tim3en, tim3rst, APB1),
    TIM4: (tim4, tim4en, tim4rst, APB1),
}
