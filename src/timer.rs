//! # Timer

use crate::hal::timer::{CountDown, Periodic};
use crate::pac::{DBGMCU as DBG, TIM1, TIM2, TIM3, TIM4};
use cast::{u16, u32};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use nb;
use void::Void;
use crate::rcc::{RccBus, Clocks, Enable, Reset};

use crate::time::Hertz;

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Hertz,
}

pub struct CountDownTimer<TIM> {
    tim: TIM,
    clk: Hertz,
}


impl Timer<SYST> {
    pub fn syst(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self { tim: syst, clk: clocks.sysclk() }
    }

    pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<SYST>
    where
        T: Into<Hertz>,
    {
        let Self { tim, clk } = self;
        let mut timer = CountDownTimer { tim, clk };
        timer.start(timeout);
        timer
    }
}

impl CountDownTimer<SYST> {
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

impl CountDown for CountDownTimer<SYST> {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clk.0 / timeout.into().0 - 1;

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

impl Periodic for CountDownTimer<SYST> {}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $pclkX:ident, $dbg_timX_stop:ident),)+) => {
        $(
            impl Timer<$TIMX> {
                /// Initialize timer
                pub fn $timX(tim: $TIMX, clocks: &Clocks, apb: &mut <$TIMX as RccBus>::Bus) -> Self {
                    // enable and reset peripheral to a clean slate state
                    $TIMX::enable(apb);
                    $TIMX::reset(apb);

                    Self { tim, clk: clocks.$pclkX() }
                }

                /// Starts timer in count down mode at a given frequency
                pub fn start_count_down<T>(self, timeout: T) -> CountDownTimer<$TIMX>
                where
                    T: Into<Hertz>,
                {
                    let Self { tim, clk } = self;
                    let mut timer = CountDownTimer { tim, clk };
                    timer.start(timeout);
                    timer
                }

                /// Resets timer peripheral
                #[inline(always)]
                pub fn clocking_reset(&mut self, apb: &mut <$TIMX as RccBus>::Bus) {
                    $TIMX::reset(apb);
                }

                /// Stopping timer in debug mode can cause troubles when sampling the signal
                #[inline(always)]
                pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
                    dbg.cr.write(|w| w.$dbg_timX_stop().bit(state));
                }

                /// Releases the TIM Peripheral
                pub fn release(self) -> $TIMX {
                    self.tim
                }
            }

            impl CountDownTimer<$TIMX> {
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
                pub fn stop(mut self) -> Timer<$TIMX> {
                    self.unlisten(Event::Update);
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    let Self { tim, clk } = self;
                    Timer { tim, clk }
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Releases the TIM Peripheral
                pub fn release(self) -> $TIMX {
                    self.stop().release()
                }

                /// Returns the current counter value
                pub fn count(&self) -> u16 {
                    self.tim.cnt.read().cnt().bits()
                }

                /// Resets the counter and generates an update event
                pub fn reset(&mut self) {
                    // Sets the URS bit to prevent an interrupt from being triggered by
                    // the UG bit.
                    self.tim.cr1.modify(|_, w| w.urs().set_bit());

                    self.tim.egr.write(|w| w.ug().set_bit());
                    self.tim.cr1.modify(|_, w| w.urs().clear_bit());
                }
            }

            impl CountDown for CountDownTimer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    let frequency = timeout.into().0;
                    let timer_clock = self.clk;
                    let ticks = timer_clock.0 / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    self.tim.psc.write(|w| w.psc().bits(psc) );

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    self.tim.arr.write(|w| w.arr().bits(arr) );

                    // Trigger an update event to load the prescaler value to the clock
                    self.reset();

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

            impl Periodic for CountDownTimer<$TIMX> {}
        )+
    }
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
))]
hal! {
    TIM1: (tim1, pclk2_tim, dbg_tim1_stop),
}

hal! {
    TIM2: (tim2, pclk1_tim, dbg_tim2_stop),
    TIM3: (tim3, pclk1_tim, dbg_tim3_stop),
    TIM4: (tim4, pclk1_tim, dbg_tim4_stop),
}
