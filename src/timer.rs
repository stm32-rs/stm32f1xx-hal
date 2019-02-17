//! # Timer

use cast::{u16, u32};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use hal::timer::{CountDown, Periodic};
use nb;
use stm32::{TIM1, TIM2, TIM3, TIM4};
use void::Void;

use core::any::TypeId;

use rcc::{Clocks, APB1, APB2};
use time::Hertz;

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

                /// Return the bus clock frequency in hertz.
                fn get_bus_clock(&self) -> Hertz {
                    if TypeId::of::<$apbX>() == TypeId::of::<APB1>() {
                            Hertz(self.clocks.pclk1().0 * self.get_bus_frequency_multiplier())
                    } else if TypeId::of::<$apbX>() == TypeId::of::<APB2>() {
                        Hertz(self.clocks.pclk2().0 * self.get_bus_frequency_multiplier())
                    } else {
                        unreachable!()
                    }
                }

                /// Return the bus frequency multiplier.
                fn get_bus_frequency_multiplier(&self) -> u32 {
                    if TypeId::of::<$apbX>() == TypeId::of::<APB1>() {
                        if self.clocks.ppre1() == 1 {
                            1
                        } else {
                            2
                        }
                    } else if TypeId::of::<$apbX>() == TypeId::of::<APB2>() {
                         if self.clocks.ppre2() == 1 {
                            1
                         } else {
                            2
                         }
                    } else {
                        unreachable!()
                    }
                }
            }

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    let frequency = timeout.into().0;
                    let timer_clock = self.get_bus_clock();
                    let ticks = timer_clock.0 / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.tim.egr.write(|w| w.ug().set_bit());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Periodic for Timer<$TIMX> {}
        )+
    }
}

hal! {
    TIM1: (tim1, tim1en, tim1rst, APB2),
    TIM2: (tim2, tim2en, tim2rst, APB1),
    TIM3: (tim3, tim3en, tim3rst, APB1),
    TIM4: (tim4, tim4en, tim4rst, APB1),
}
