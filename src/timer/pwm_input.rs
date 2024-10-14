//! This module allows Timer peripherals to be configured as pwm input.
//! In this mode, the timer sample a squared signal to find it's frequency and duty cycle.
//!
//! Also this module provides Quadrature Encoder Interface

use crate::pac::{self, DBGMCU as DBG};

use crate::afio::{RInto, Rmp, TimC};
use crate::rcc::{BusTimerClock, Clocks};
use crate::time::Hertz;
use crate::timer::{General, Timer};

use embedded_hal_02 as hal;
pub use hal::Direction;

pub trait Instance: General + TimC<0> + TimC<1> {}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl Instance for pac::TIM1 {}
impl Instance for pac::TIM2 {}
impl Instance for pac::TIM3 {}
#[cfg(feature = "medium")]
impl Instance for pac::TIM4 {}

/// PWM Input
#[allow(unused)]
pub struct PwmInput<TIM: Instance> {
    timer: TIM,
    pins: (<TIM as TimC<0>>::In, <TIM as TimC<1>>::In),
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

/// SMS (Slave Mode Selection) register
#[derive(Copy, Clone, Debug)]
pub enum SlaveMode {
    /// Counter counts up/down on TI2FP1 edge depending on TI1FP2 level.
    EncoderMode1 = 0b001,
    /// Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level.
    EncoderMode2 = 0b010,
    /// Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the
    /// level of the other input.
    EncoderMode3 = 0b011,
    /// Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and
    /// generates an update of the registers.
    ResetMode = 0b100,
    /// Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not
    /// reset). Only the start of the counter is controlled.
    TriggerMode = 0b110,
    /// External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    ExternalClockMode1 = 0b111,
}

/// Quadrature Encoder Interface (QEI) options
///
/// The `Default` implementation provides a configuration for a 4-count pulse which counts from
/// 0-65535. The counter wraps back to 0 on overflow.
#[derive(Copy, Clone, Debug)]
pub struct QeiOptions {
    /// Encoder slave mode
    pub slave_mode: SlaveMode,

    /// Autoreload value
    ///
    /// This value allows the maximum count to be configured, up to 65535. Setting a lower value
    /// will overflow the counter to 0 sooner.
    pub auto_reload_value: u16,
}

impl Default for QeiOptions {
    fn default() -> Self {
        Self {
            slave_mode: SlaveMode::EncoderMode3,
            auto_reload_value: u16::MAX,
        }
    }
}

/// Quadrature Encoder Interface (QEI)
pub struct Qei<TIM: Instance> {
    tim: TIM,
    pins: (<TIM as TimC<0>>::In, <TIM as TimC<1>>::In),
}

pub trait PwmInputExt: Sized + Instance {
    fn pwm_input(
        self,
        pins: (
            impl RInto<<Self as TimC<0>>::In, 0>,
            impl RInto<<Self as TimC<1>>::In, 0>,
        ),
        dbg: &mut DBG,
        mode: Configuration,
        clocks: &Clocks,
    ) -> PwmInput<Self>;
}

pub trait QeiExt: Sized + Instance {
    fn qei(
        self,
        pins: (
            impl RInto<<Self as TimC<0>>::In, 0>,
            impl RInto<<Self as TimC<1>>::In, 0>,
        ),
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self>;
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
    ($TIM:ty: $timX:ident, $qeix:ident) => {
        impl Timer<$TIM> {
            pub fn pwm_input(
                mut self,
                pins: (
                    impl RInto<<$TIM as TimC<0>>::In, 0>,
                    impl RInto<<$TIM as TimC<1>>::In, 0>,
                ),
                dbg: &mut DBG,
                mode: Configuration,
            ) -> PwmInput<$TIM> {
                self.stop_in_debug(dbg, false);
                let Self { tim, clk } = self;
                $timX(tim, pins, clk, mode)
            }
            pub fn qei(
                self,
                pins: (
                    impl RInto<<$TIM as TimC<0>>::In, 0>,
                    impl RInto<<$TIM as TimC<1>>::In, 0>,
                ),
                options: QeiOptions,
            ) -> Qei<$TIM> {
                let Self { tim, clk: _ } = self;
                $qeix(tim, pins, options)
            }
        }
        impl PwmInputExt for $TIM {
            fn pwm_input(
                self,
                pins: (
                    impl RInto<<Self as TimC<0>>::In, 0>,
                    impl RInto<<Self as TimC<1>>::In, 0>,
                ),
                dbg: &mut DBG,
                mode: Configuration,
                clocks: &Clocks,
            ) -> PwmInput<Self> {
                Timer::new(self, clocks).pwm_input(pins, dbg, mode)
            }
        }
        impl QeiExt for $TIM {
            fn qei(
                self,
                pins: (
                    impl RInto<<Self as TimC<0>>::In, 0>,
                    impl RInto<<Self as TimC<1>>::In, 0>,
                ),
                options: QeiOptions,
                clocks: &Clocks,
            ) -> Qei<Self> {
                Timer::new(self, clocks).qei(pins, options)
            }
        }

        impl<const R: u8> Rmp<$TIM, R> {
            pub fn pwm_input(
                self,
                pins: (
                    impl RInto<<$TIM as TimC<0>>::In, R>,
                    impl RInto<<$TIM as TimC<1>>::In, R>,
                ),
                dbg: &mut DBG,
                mode: Configuration,
                clocks: &Clocks,
            ) -> PwmInput<$TIM> {
                let mut tim = Timer::new(self.0, clocks);
                tim.stop_in_debug(dbg, false);
                let Timer { tim, clk } = tim;
                $timX(tim, pins, clk, mode)
            }
            pub fn qei(
                self,
                pins: (
                    impl RInto<<$TIM as TimC<0>>::In, R>,
                    impl RInto<<$TIM as TimC<1>>::In, R>,
                ),
                options: QeiOptions,
                clocks: &Clocks,
            ) -> Qei<$TIM> {
                let Timer { tim, clk: _ } = Timer::new(self.0, clocks);
                $qeix(tim, pins, options)
            }
        }

        fn $timX<const R: u8>(
            tim: $TIM,
            pins: (
                impl RInto<<$TIM as TimC<0>>::In, R>,
                impl RInto<<$TIM as TimC<1>>::In, R>,
            ),
            clk: Hertz,
            mode: Configuration,
        ) -> PwmInput<$TIM> {
            let pins = (pins.0.rinto(), pins.1.rinto());

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
            PwmInput { timer: tim, pins }
        }

        impl PwmInput<$TIM> {
            /// Return the frequency sampled by the timer
            pub fn read_frequency(&self, mode: ReadMode, clocks: &Clocks) -> Result<Hertz, Error> {
                if let ReadMode::WaitForNextCapture = mode {
                    self.wait_for_capture();
                }

                let presc = unsafe { (*<$TIM>::ptr()).psc().read().bits() as u16 };
                let ccr1 = unsafe { (*<$TIM>::ptr()).ccr1().read().bits() as u16 };

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
                    let clk = <$TIM>::timer_clock(&clocks);
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
                let ccr1 = unsafe { (*<$TIM>::ptr()).ccr1().read().bits() as u16 };
                let ccr2 = unsafe { (*<$TIM>::ptr()).ccr2().read().bits() as u16 };
                if ccr1 == 0 {
                    Err(Error::FrequencyTooLow)
                } else {
                    Ok((ccr2, ccr1))
                }
            }

            /// Wait until the timer has captured a period
            fn wait_for_capture(&self) {
                unsafe { &(*<$TIM>::ptr()) }.sr().write(|w| {
                    w.uif().clear_bit();
                    w.cc1if().clear_bit();
                    w.cc1of().clear_bit()
                });
                while unsafe { (*<$TIM>::ptr()).sr().read().cc1if().bit_is_clear() } {}
            }

            pub fn release(self) -> ($TIM, (<$TIM as TimC<0>>::In, <$TIM as TimC<1>>::In)) {
                (self.timer, self.pins)
            }
        }

        fn $qeix<const R: u8>(
            tim: $TIM,
            pins: (
                impl RInto<<$TIM as TimC<0>>::In, R>,
                impl RInto<<$TIM as TimC<1>>::In, R>,
            ),
            options: QeiOptions,
        ) -> Qei<$TIM> {
            let pins = (pins.0.rinto(), pins.1.rinto());
            // Configure TxC1 and TxC2 as captures
            tim.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti2());

            // enable and configure to capture on rising edge
            tim.ccer().write(|w| {
                w.cc1e().set_bit();
                w.cc1p().clear_bit();
                w.cc2e().set_bit();
                w.cc2p().clear_bit()
            });

            // configure as quadrature encoder
            tim.smcr().write(|w| w.sms().set(options.slave_mode as u8));

            tim.arr().write(|w| w.arr().set(options.auto_reload_value));
            tim.cr1().write(|w| w.cen().set_bit());

            Qei { tim, pins }
        }

        impl Qei<$TIM> {
            pub fn release(self) -> ($TIM, (<$TIM as TimC<0>>::In, <$TIM as TimC<1>>::In)) {
                (self.tim, self.pins)
            }
        }

        impl hal::Qei for Qei<$TIM> {
            type Count = u16;

            fn count(&self) -> u16 {
                self.tim.cnt().read().cnt().bits()
            }

            fn direction(&self) -> Direction {
                if self.tim.cr1().read().dir().bit_is_clear() {
                    Direction::Upcounting
                } else {
                    Direction::Downcounting
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
hal!(pac::TIM1: tim1, qei1);

hal!(pac::TIM2: tim2, qei2);
hal!(pac::TIM3: tim3, qei3);

#[cfg(feature = "medium")]
hal!(pac::TIM4: tim4, qei4);
