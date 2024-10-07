//! This module allows Timer peripherals to be configured as pwm input.
//! In this mode, the timer sample a squared signal to find it's frequency and duty cycle.

use core::marker::PhantomData;
use core::mem;

use crate::pac::{self, DBGMCU as DBG};

use crate::rcc::{BusTimerClock, Clocks};
use crate::time::Hertz;
use crate::timer::{InPins, InputPins, Timer};

/// PWM Input
pub struct PwmInput<TIM> {
    _timer: PhantomData<TIM>,
}

/// How the data is read from the timer
pub enum ReadMode {
    /// Return the latest captured data
    Instant,
    /// Wait for one period of the signal before computing the frequency and the duty cycle
    /// The microcontroller will be halted for at most two period of the input signal.
    WaitForNextCapture,
}

/// The error returned when reading a frequency from a timer
#[derive(Debug)]
pub enum Error {
    /// The signal frequency is too low to be sampled by the current timer configuration
    FrequencyTooLow,
}

/// Which frequency the timer will try to sample
pub enum Configuration {
    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample a wide range of frequency, at the expense
    /// of resolution.
    ///
    /// The minimum frequency that can be sampled is 20% the provided frequency.
    ///
    /// Use this mode if you do not know what to choose.
    Frequency(Hertz),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to sample the duty cycle with a high resolution.
    /// This will limit the frequency range where the timer can operate.
    ///
    /// The minimum frequency that can be sampled is 90% the provided frequency
    DutyCycle(Hertz),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample signal with a frequency higher than the
    /// provided value : there is no margin for lower frequencies.
    RawFrequency(Hertz),

    /// In this mode, the provided arr and presc are directly programmed in the register.
    RawValues { arr: u16, presc: u16 },
}

pub trait PwmInputExt: Sized + InputPins {
    fn pwm_input(
        self,
        pins: impl Into<InPins<Self::InCh1, Self::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self>;
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl PwmInputExt for pac::TIM1 {
    fn pwm_input(
        self,
        pins: impl Into<InPins<Self::InCh1, Self::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self> {
        Timer::new(self, clocks).pwm_input(pins, dbg, mode)
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl Timer<pac::TIM1> {
    pub fn pwm_input(
        mut self,
        pins: impl Into<InPins<<pac::TIM1 as InputPins>::InCh1, <pac::TIM1 as InputPins>::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
    ) -> PwmInput<pac::TIM1> {
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim1(tim, pins, clk, mode)
    }
}

impl PwmInputExt for pac::TIM2 {
    fn pwm_input(
        self,
        pins: impl Into<InPins<Self::InCh1, Self::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self> {
        Timer::new(self, clocks).pwm_input(pins, dbg, mode)
    }
}

impl Timer<pac::TIM2> {
    pub fn pwm_input(
        mut self,
        pins: impl Into<InPins<<pac::TIM2 as InputPins>::InCh1, <pac::TIM2 as InputPins>::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
    ) -> PwmInput<pac::TIM2> {
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim2(tim, pins, clk, mode)
    }
}

impl PwmInputExt for pac::TIM3 {
    fn pwm_input(
        self,
        pins: impl Into<InPins<Self::InCh1, Self::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self> {
        Timer::new(self, clocks).pwm_input(pins, dbg, mode)
    }
}

impl Timer<pac::TIM3> {
    pub fn pwm_input(
        mut self,
        pins: impl Into<InPins<<pac::TIM3 as InputPins>::InCh1, <pac::TIM3 as InputPins>::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
    ) -> PwmInput<pac::TIM3> {
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim3(tim, pins, clk, mode)
    }
}

#[cfg(feature = "medium")]
impl PwmInputExt for pac::TIM4 {
    fn pwm_input(
        self,
        pins: impl Into<InPins<Self::InCh1, Self::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self> {
        Timer::new(self, &clocks).pwm_input(pins, dbg, mode)
    }
}

#[cfg(feature = "medium")]
impl Timer<pac::TIM4> {
    pub fn pwm_input(
        mut self,
        pins: impl Into<InPins<<pac::TIM4 as InputPins>::InCh1, <pac::TIM4 as InputPins>::InCh2>>,
        dbg: &mut DBG,
        mode: Configuration,
    ) -> PwmInput<pac::TIM4> {
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim4(tim, pins, clk, mode)
    }
}

/// Courtesy of @TeXitoi (https://github.com/stm32-rs/stm32f1xx-hal/pull/10#discussion_r259535503)
fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u16) {
    if freq == 0 {
        return (u16::MAX, u16::MAX);
    }
    let presc = clock / freq.saturating_mul(u16::MAX as u32 + 1);
    let arr = clock / freq.saturating_mul(presc + 1);
    (core::cmp::max(1, arr as u16), presc as u16)
}
macro_rules! hal {
    ($TIMX:ty: $timX:ident) => {
        fn $timX(
            tim: $TIMX,
            pins: impl Into<InPins<<$TIMX as InputPins>::InCh1, <$TIMX as InputPins>::InCh2>>,
            clk: Hertz,
            mode: Configuration,
        ) -> PwmInput<$TIMX> {
            let _pins = pins.into();

            use Configuration::*;
            // Disable capture on both channels during setting
            // (for Channel X bit is CCXE)
            tim.ccer().modify(|_, w| {
                w.cc1e().clear_bit();
                w.cc2e().clear_bit();
                w.cc1p().clear_bit();
                w.cc2p().set_bit()
            });

            // Define the direction of the channel (input/output)
            // and the used input
            tim.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti1());

            tim.dier().write(|w| w.cc1ie().set_bit());

            // Configure slave mode control register
            // Selects the trigger input to be used to synchronize the counter
            // 101: Filtered Timer Input 1 (TI1FP1)
            // ---------------------------------------
            // Slave Mode Selection :
            //  100: Reset Mode - Rising edge of the selected trigger input (TRGI)
            //  reinitializes the counter and generates an update of the registers.
            tim.smcr()
                .modify(|_, w| unsafe { w.ts().bits(0b101).sms().bits(0b100) });

            match mode {
                Frequency(f) => {
                    let freq = f.raw();
                    let max_freq = if freq > 5 { freq / 5 } else { 1 };
                    let (arr, presc) = compute_arr_presc(max_freq, clk.raw());
                    tim.arr().write(|w| w.arr().set(arr));
                    tim.psc().write(|w| w.psc().set(presc));
                }
                DutyCycle(f) => {
                    let freq = f.raw();
                    let max_freq = if freq > 2 {
                        freq / 2 + freq / 4 + freq / 8
                    } else {
                        1
                    };
                    let (arr, presc) = compute_arr_presc(max_freq, clk.raw());
                    tim.arr().write(|w| w.arr().set(arr));
                    tim.psc().write(|w| w.psc().set(presc));
                }
                RawFrequency(f) => {
                    let freq = f.raw();
                    let (arr, presc) = compute_arr_presc(freq, clk.raw());
                    tim.arr().write(|w| w.arr().set(arr));
                    tim.psc().write(|w| w.psc().set(presc));
                }
                RawValues { arr, presc } => {
                    tim.arr().write(|w| w.arr().set(arr));
                    tim.psc().write(|w| w.psc().set(presc));
                }
            }

            // Enable Capture on both channels
            tim.ccer()
                .modify(|_, w| w.cc1e().set_bit().cc2e().set_bit());

            tim.cr1().modify(|_, w| w.cen().set_bit());
            unsafe { mem::MaybeUninit::uninit().assume_init() }
        }

        impl PwmInput<$TIMX> {
            /// Return the frequency sampled by the timer
            pub fn read_frequency(&self, mode: ReadMode, clocks: &Clocks) -> Result<Hertz, Error> {
                if let ReadMode::WaitForNextCapture = mode {
                    self.wait_for_capture();
                }

                let presc = unsafe { (*<$TIMX>::ptr()).psc().read().bits() as u16 };
                let ccr1 = unsafe { (*<$TIMX>::ptr()).ccr1().read().bits() as u16 };

                // Formulas :
                //
                // F_timer = F_pclk / (PSC+1)*(ARR+1)
                // Frac_arr = (CCR1+1)/(ARR+1)
                // F_signal = F_timer/Frac_arr
                // <=> F_signal = [(F_plck)/((PSC+1)*(ARR+1))] * [(ARR+1)/(CCR1+1)]
                // <=> F_signal = F_pclk / ((PSC+1)*(CCR1+1))
                //
                // Where :
                // * PSC is the prescaler register
                // * ARR is the auto-reload register
                // * F_timer is the number of time per second where the timer overflow under normal
                // condition
                //
                if ccr1 == 0 {
                    Err(Error::FrequencyTooLow)
                } else {
                    let clk = <$TIMX>::timer_clock(&clocks);
                    Ok(clk / ((presc + 1) as u32 * (ccr1 + 1) as u32))
                }
            }

            /// Return the duty in the form of a fraction : (duty_cycle/period)
            pub fn read_duty(&self, mode: ReadMode) -> Result<(u16, u16), Error> {
                if let ReadMode::WaitForNextCapture = mode {
                    self.wait_for_capture();
                }

                // Formulas :
                // Duty_cycle = (CCR2+1)/(CCR1+1)
                let ccr1 = unsafe { (*<$TIMX>::ptr()).ccr1().read().bits() as u16 };
                let ccr2 = unsafe { (*<$TIMX>::ptr()).ccr2().read().bits() as u16 };
                if ccr1 == 0 {
                    Err(Error::FrequencyTooLow)
                } else {
                    Ok((ccr2, ccr1))
                }
            }

            /// Wait until the timer has captured a period
            fn wait_for_capture(&self) {
                unsafe { &(*<$TIMX>::ptr()) }.sr().write(|w| {
                    w.uif().clear_bit();
                    w.cc1if().clear_bit();
                    w.cc1of().clear_bit()
                });
                while unsafe { (*<$TIMX>::ptr()).sr().read().cc1if().bit_is_clear() } {}
            }
        }
    };
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
hal!(pac::TIM1: tim1);

hal!(pac::TIM2: tim2);
hal!(pac::TIM3: tim3);

#[cfg(feature = "medium")]
hal!(pac::TIM4: tim4);
