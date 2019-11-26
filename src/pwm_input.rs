//! This module allows Timer peripherals to be configured as pwm input.
//! In this mode, the timer sample a squared signal to find it's frequency and duty cycle.

use core::marker::PhantomData;
use core::mem;

use crate::pac::DBGMCU as DBG;
#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
use crate::pac::TIM1;
use crate::pac::{TIM2, TIM3};
#[cfg(feature = "medium")]
use crate::pac::TIM4;

use crate::afio::MAPR;
use crate::gpio::{self, Floating, Input};
use crate::rcc::Clocks;
use crate::time::Hertz;
use crate::timer::Timer;

pub trait Pins<REMAP> {}

use crate::timer::sealed::{Remap, Ch1, Ch2};

impl<TIM, REMAP, P1, P2> Pins<REMAP> for (P1, P2)
where
    REMAP: Remap<Periph = TIM>,
    P1: Ch1<REMAP> + gpio::Mode<Input<Floating>>,
    P2: Ch2<REMAP> + gpio::Mode<Input<Floating>> {}

/// PWM Input
pub struct PwmInput<TIM, REMAP, PINS> {
    _timer: PhantomData<TIM>,
    _remap: PhantomData<REMAP>,
    _pins: PhantomData<PINS>,
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
pub enum Configuration<T>
where
    T: Into<Hertz>,
{
    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample a wide range of frequency, at the expense
    /// of resolution.
    ///
    /// The minimum frequency that can be sampled is 20% the provided frequency.
    ///
    /// Use this mode if you do not know what to choose.
    Frequency(T),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to sample the duty cycle with a high resolution.
    /// This will limit the frequency range where the timer can operate.
    ///
    /// The minimum frequency that can be sampled is 90% the provided frequency
    DutyCycle(T),

    /// In this mode an algorithm calculates the optimal value for the autoreload register and the
    /// prescaler register in order to be able to sample signal with a frequency higher than the
    /// provided value : there is no margin for lower frequencies.
    RawFrequency(T),

    /// In this mode, the provided arr and presc are directly programmed in the register.
    RawValues { arr: u16, presc: u16 },
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
impl Timer<TIM1> {
    pub fn pwm_input<REMAP, PINS, T>(
        mut self,
        pins: PINS,
        mapr: &mut MAPR,
        dbg: &mut DBG,
        mode: Configuration<T>,
    ) -> PwmInput<TIM1, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim1(tim, pins, clk, mode)
    }
}

impl Timer<TIM2> {
    pub fn pwm_input<REMAP, PINS, T>(
        mut self,
        pins: PINS,
        mapr: &mut MAPR,
        dbg: &mut DBG,
        mode: Configuration<T>,
    ) -> PwmInput<TIM2, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim2(tim, pins, clk, mode)
    }
}

impl Timer<TIM3> {
    pub fn pwm_input<REMAP, PINS, T>(
        mut self,
        pins: PINS,
        mapr: &mut MAPR,
        dbg: &mut DBG,
        mode: Configuration<T>,
    ) -> PwmInput<TIM3, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim3(tim, pins, clk, mode)
    }
}

#[cfg(feature = "medium")]
impl Timer<TIM4> {
    pub fn pwm_input<REMAP, PINS, T>(
        mut self,
        pins: PINS,
        mapr: &mut MAPR,
        dbg: &mut DBG,
        mode: Configuration<T>,
    ) -> PwmInput<TIM4, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));
        self.stop_in_debug(dbg, false);
        let Self { tim, clk } = self;
        tim4(tim, pins, clk, mode)
    }
}

/// Courtesy of @TeXitoi (https://github.com/stm32-rs/stm32f1xx-hal/pull/10#discussion_r259535503)
fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u16) {
    if freq == 0 {
        return (core::u16::MAX, core::u16::MAX);
    }
    let presc = clock / freq.saturating_mul(core::u16::MAX as u32 + 1);
    let arr = clock / freq.saturating_mul(presc + 1);
    (core::cmp::max(1, arr as u16), presc as u16)
}
macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $pclkX:ident ),)+) => {
        $(
            fn $timX<REMAP, PINS,T>(
                tim: $TIMX,
                _pins: PINS,
                clk: Hertz,
                mode : Configuration<T>,
            ) -> PwmInput<$TIMX, REMAP, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP>,
                T : Into<Hertz>
            {
                use crate::pwm_input::Configuration::*;
                // Disable capture on both channels during setting
                // (for Channel X bit is CCXE)
                tim.ccer.modify(|_,w| w.cc1e().clear_bit().cc2e().clear_bit()
                                       .cc1p().clear_bit().cc2p().set_bit());

                // Define the direction of the channel (input/output)
                // and the used input
                tim.ccmr1_input().modify( |_,w| w.cc1s().ti1().cc2s().ti1());

                tim.dier.write(|w| w.cc1ie().set_bit());

                // Configure slave mode control register
                // Selects the trigger input to be used to synchronize the counter
                // 101: Filtered Timer Input 1 (TI1FP1)
                // ---------------------------------------
                // Slave Mode Selection :
                //  100: Reset Mode - Rising edge of the selected trigger input (TRGI)
                //  reinitializes the counter and generates an update of the registers.
                tim.smcr.modify( |_,w| unsafe {w.ts().bits(0b101).sms().bits(0b100)});

                match mode {
                    Frequency(f)  => {
                        let freq = f.into().0;
                        let max_freq = if freq > 5 {freq/5} else {1};
                        let (arr,presc) = compute_arr_presc(max_freq, clk.0);
                        tim.arr.write(|w| w.arr().bits(arr));
                        tim.psc.write(|w| w.psc().bits(presc) );
                    },
                    DutyCycle(f) => {
                        let freq = f.into().0;
                        let max_freq = if freq > 2 {freq/2 + freq/4 + freq/8} else {1};
                        let (arr,presc) = compute_arr_presc(max_freq, clk.0);
                        tim.arr.write(|w| w.arr().bits(arr));
                        tim.psc.write(|w| w.psc().bits(presc) );
                    },
                    RawFrequency(f) => {
                        let freq = f.into().0;
                        let (arr,presc) = compute_arr_presc(freq, clk.0);
                        tim.arr.write(|w| w.arr().bits(arr));
                        tim.psc.write(|w| w.psc().bits(presc) );
                    }
                    RawValues{arr, presc} => {
                        tim.arr.write(|w| w.arr().bits(arr));
                        tim.psc.write(|w| w.psc().bits(presc) );
                    }
                }

                // Enable Capture on both channels
                tim.ccer.modify(|_,w| w.cc1e().set_bit().cc2e().set_bit());

                tim.cr1.modify(|_,w| w.cen().set_bit());
                unsafe { mem::MaybeUninit::uninit().assume_init() }
            }

            impl<REMAP, PINS> PwmInput<$TIMX, REMAP, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP>,
            {
                /// Return the frequency sampled by the timer
                pub fn read_frequency(&self, mode : ReadMode, clocks : &Clocks) -> Result<Hertz,Error> {
                    match mode {
                        ReadMode::WaitForNextCapture => self.wait_for_capture(),
                        _ => (),
                    }
                    
                    let presc = unsafe { (*$TIMX::ptr()).psc.read().bits() as u16};
                    let ccr1 = unsafe { (*$TIMX::ptr()).ccr1.read().bits() as u16};

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
                        let clk : u32 = clocks.$pclkX().0;
                        Ok(Hertz(clk/((presc+1) as u32*(ccr1 + 1)as u32)))
                    }
                }

                /// Return the duty in the form of a fraction : (duty_cycle/period)
                pub fn read_duty(&self, mode : ReadMode) -> Result<(u16,u16),Error> {
                    match mode {
                        ReadMode::WaitForNextCapture => self.wait_for_capture(),
                        _ => (),
                    }

                    // Formulas :
                    // Duty_cycle = (CCR2+1)/(CCR1+1)
                    let ccr1 = unsafe { (*$TIMX::ptr()).ccr1.read().bits() as u16};
                    let ccr2 = unsafe { (*$TIMX::ptr()).ccr2.read().bits() as u16};
                    if ccr1 == 0 {
                        Err(Error::FrequencyTooLow)
                    } else {
                        Ok((ccr2,ccr1))
                    }
                }

                /// Wait until the timer has captured a period
                fn wait_for_capture(&self) {
                    unsafe { (*$TIMX::ptr()).sr.write(|w| w.uif().clear_bit().cc1if().clear_bit().cc1of().clear_bit())};
                    while unsafe { (*$TIMX::ptr()).sr.read().cc1if().bit_is_clear()} {}
                }
            }
        )+
    }
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
hal! {
    TIM1: (tim1, pclk2_tim),
}

hal! {
    TIM2: (tim2, pclk1_tim),
    TIM3: (tim3, pclk1_tim),
}

#[cfg(feature = "medium")]
hal! {
    TIM4: (tim4, pclk1_tim),
}
