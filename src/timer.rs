/*!
  # Timer

  ## Alternate function remapping

  This is a list of the remap settings you can use to assign pins to PWM channels
  and the QEI peripherals

  ### TIM1

  Not available on STM32F101.

  | Channel | Tim1NoRemap | Tim1FullRemap |
  |:---:|:-----------:|:-------------:|
  | CH1 |     PA8     |       PE9     |
  | CH2 |     PA9     |       PE11    |
  | CH3 |     PA10    |       PE13    |
  | CH4 |     PA11    |       PE14    |

  ### TIM2

  | Channel | Tim2NoRemap | Tim2PartialRemap1 | Tim2PartialRemap2 | Tim2FullRemap |
  |:---:|:-----------:|:-----------------:|:-----------------:|:-------------:|
  | CH1 |     PA0     |        PA15       |        PA0        |      PA15     |
  | CH2 |     PA1     |        PB3        |        PA1        |      PB3      |
  | CH3 |     PA2     |        PA2        |        PB10       |      PB10     |
  | CH4 |     PA3     |        PA3        |        PB11       |      PB11     |

  ### TIM3

  | Channel | Tim3NoRemap | Tim3PartialRemap | Tim3FullRemap |
  |:---:|:-----------:|:----------------:|:-------------:|
  | CH1 |     PA6     |        PB4       |      PC6      |
  | CH2 |     PA7     |        PB5       |      PC7      |
  | CH3 |     PB0     |        PB0       |      PC8      |
  | CH4 |     PB1     |        PB1       |      PC9      |

  ### TIM4

  Not available on low density devices.

  | Channel | Tim4NoRemap | Tim4Remap |
  |:---:|:-----------:|:---------:|
  | CH1 |     PB6     |    PD12   |
  | CH2 |     PB7     |    PD13   |
  | CH3 |     PB8     |    PD14   |
  | CH4 |     PB9     |    PD15   |
*/
#![allow(non_upper_case_globals)]

use crate::afio::Rmp;
use crate::bb;
use crate::pac::{self, DBGMCU as DBG, RCC};

use crate::rcc::{self, Clocks};
use core::convert::TryFrom;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::time::Hertz;

#[cfg(feature = "rtic")]
pub mod monotonic;
#[cfg(feature = "rtic")]
pub use monotonic::*;
pub mod delay;
pub mod pwm_input;
pub use delay::*;
pub mod counter;
pub use counter::*;
pub mod pwm;
pub use pwm::*;

mod hal_02;
mod hal_1;

/// Timer wrapper
pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Hertz,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    C1 = 0,
    C2 = 1,
    C3 = 2,
    C4 = 3,
}

pub use crate::afio::{TimC, TimNC};

/// Channel wrapper
pub struct Ch<const C: u8, const COMP: bool>;
pub const C1: u8 = 0;
pub const C2: u8 = 1;
pub const C3: u8 = 2;
pub const C4: u8 = 3;

/// Enum for IO polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// Output Idle state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdleState {
    Reset,
    Set,
}

/// Interrupt events
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SysEvent {
    /// [Timer] timed out / count down ended
    Update,
}

bitflags::bitflags! {
    pub struct Event: u32 {
        const Update  = 1 << 0;
        const C1 = 1 << 1;
        const C2 = 1 << 2;
        const C3 = 1 << 3;
        const C4 = 1 << 4;
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    /// Timer is disabled
    Disabled,
    WrongAutoReload,
}

pub trait TimerExt: Sized {
    /// Non-blocking [Counter] with custom fixed precision
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ>;
    /// Non-blocking [Counter] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 65 sec for 16-bit timer and from 2 ms to 49 days for 32-bit timer.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn counter_ms(self, clocks: &Clocks) -> CounterMs<Self> {
        self.counter::<1_000>(clocks)
    }
    /// Non-blocking [Counter] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 65 ms for 16-bit timer and from 2 μs to 71 min for 32-bit timer.
    fn counter_us(self, clocks: &Clocks) -> CounterUs<Self> {
        self.counter::<1_000_000>(clocks)
    }
    /// Non-blocking [Counter] with dynamic precision which uses `Hertz` as Duration units
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self>;

    /// Blocking [Delay] with custom fixed precision
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ>;
    /// Blocking [Delay] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 49 days.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn delay_ms(self, clocks: &Clocks) -> DelayMs<Self> {
        self.delay::<1_000>(clocks)
    }
    /// Blocking [Delay] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 71 min.
    fn delay_us(self, clocks: &Clocks) -> DelayUs<Self> {
        self.delay::<1_000_000>(clocks)
    }
}

impl<TIM: Instance> TimerExt for TIM {
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ> {
        FTimer::new(self, clocks).counter()
    }
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self> {
        Timer::new(self, clocks).counter_hz()
    }
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ> {
        FTimer::new(self, clocks).delay()
    }
}

pub trait SysTimerExt: Sized {
    /// Creates timer which takes [Hertz] as Duration
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz;

    /// Creates timer with custom precision (core frequency recommended is known)
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ>;
    /// Creates timer with precision of 1 μs (1 MHz sampling)
    fn counter_us(self, clocks: &Clocks) -> SysCounterUs {
        self.counter::<1_000_000>(clocks)
    }
    /// Blocking [Delay] with custom precision
    fn delay(self, clocks: &Clocks) -> SysDelay;
}

impl SysTimerExt for SYST {
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz {
        Timer::syst(self, clocks).counter_hz()
    }
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ> {
        Timer::syst(self, clocks).counter()
    }
    fn delay(self, clocks: &Clocks) -> SysDelay {
        Timer::syst_external(self, clocks).delay()
    }
}

impl Timer<SYST> {
    /// Initialize SysTick timer
    pub fn syst(mut tim: SYST, clocks: &Clocks) -> Self {
        tim.set_clock_source(SystClkSource::Core);
        Self {
            tim,
            clk: clocks.hclk(),
        }
    }

    /// Initialize SysTick timer and set it frequency to `HCLK / 8`
    pub fn syst_external(mut tim: SYST, clocks: &Clocks) -> Self {
        tim.set_clock_source(SystClkSource::External);
        Self {
            tim,
            clk: clocks.hclk() / 8,
        }
    }

    pub fn configure(&mut self, clocks: &Clocks) {
        self.tim.set_clock_source(SystClkSource::Core);
        self.clk = clocks.hclk();
    }

    pub fn configure_external(&mut self, clocks: &Clocks) {
        self.tim.set_clock_source(SystClkSource::External);
        self.clk = clocks.hclk() / 8;
    }

    pub fn release(self) -> SYST {
        self.tim
    }

    /// Starts listening for an `event`
    pub fn listen(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.disable_interrupt(),
        }
    }

    /// Resets the counter
    pub fn reset(&mut self) {
        // According to the Cortex-M3 Generic User Guide, the interrupt request is only generated
        // when the counter goes from 1 to 0, so writing zero should not trigger an interrupt
        self.tim.clear_current();
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Ocm {
    Frozen = 0,
    ActiveOnMatch = 1,
    InactiveOnMatch = 2,
    Toggle = 3,
    ForceInactive = 4,
    ForceActive = 5,
    PwmMode1 = 6,
    PwmMode2 = 7,
}

// Center-aligned mode selection
pub use pac::tim1::cr1::CMS as CenterAlignedMode;

mod sealed {
    use super::{CenterAlignedMode, Event, IdleState, Ocm, Polarity, DBG};
    pub trait General {
        type Width: Into<u32> + From<u16>;
        fn max_auto_reload() -> u32;
        unsafe fn set_auto_reload_unchecked(&mut self, arr: u32);
        fn set_auto_reload(&mut self, arr: u32) -> Result<(), super::Error>;
        fn read_auto_reload() -> u32;
        fn enable_preload(&mut self, b: bool);
        fn enable_counter(&mut self, b: bool);
        fn is_counter_enabled(&self) -> bool;
        fn reset_counter(&mut self);
        fn set_prescaler(&mut self, psc: u16);
        fn read_prescaler(&self) -> u16;
        fn trigger_update(&mut self);
        fn clear_interrupt_flag(&mut self, event: Event);
        fn listen_interrupt(&mut self, event: Event, b: bool);
        fn get_interrupt_flag(&self) -> Event;
        fn read_count(&self) -> Self::Width;
        fn start_one_pulse(&mut self);
        fn cr1_reset(&mut self);
        fn cnt_reset(&mut self);
        fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool);
    }

    pub trait WithPwmCommon: General {
        const CH_NUMBER: u8;
        const COMP_CH_NUMBER: u8;
        fn read_cc_value(channel: u8) -> u32;
        fn set_cc_value(channel: u8, value: u32);
        fn enable_channel(channel: u8, b: bool);
        fn set_channel_polarity(channel: u8, p: Polarity);
        fn set_nchannel_polarity(channel: u8, p: Polarity);
    }

    pub trait Advanced: WithPwmCommon {
        fn enable_nchannel(channel: u8, b: bool);
        fn set_dtg_value(value: u8);
        fn read_dtg_value() -> u8;
        fn idle_state(channel: u8, comp: bool, s: IdleState);
        fn set_cms(mode: CenterAlignedMode);
    }

    pub trait WithPwm: WithPwmCommon {
        fn preload_output_channel_in_mode(&mut self, c: u8, mode: Ocm);
        fn freeze_output_channel(&mut self, c: u8);
        fn start_pwm(&mut self);
    }

    pub trait MasterTimer: General {
        type Mms;
        fn master_mode(&mut self, mode: Self::Mms);
    }

    pub trait Split {
        type Channels;
        fn split() -> Self::Channels;
    }
}
pub(crate) use sealed::{Advanced, General, MasterTimer, WithPwm, WithPwmCommon};

pub trait Instance:
    crate::Sealed + rcc::Enable + rcc::Reset + rcc::BusTimerClock + General
{
}

use sealed::Split;
macro_rules! split {
    ($TIM:ty: 1) => {
        split!($TIM, C1);
    };
    ($TIM:ty: 2) => {
        split!($TIM, C1, C2);
    };
    ($TIM:ty: 4) => {
        split!($TIM, C1, C2, C3, C4);
    };
    ($TIM:ty, $($C:ident),+) => {
        impl Split for $TIM {
            type Channels = ($(PwmChannelDisabled<$TIM, $C, 0>,)+);
            fn split() -> Self::Channels {
                ($(PwmChannelDisabled::<_, $C, 0>::new(),)+)
            }
        }
        impl<const R: u8> Split for Rmp<$TIM, R> {
            type Channels = ($(PwmChannelDisabled<$TIM, $C, R>,)+);
            fn split() -> Self::Channels {
                ($(PwmChannelDisabled::<_, $C, R>::new(),)+)
            }
        }
    };
}

macro_rules! hal {
    ($TIM:ty: [
        $Timer:ident,
        $bits:ty,
        $dbg_timX_stop:ident,
        $(c: ($cnum:tt $(, $aoe:ident)?),)?
        $(m: $timbase:ident,)?
    ]) => {
        impl Instance for $TIM { }
        impl crate::Steal for $TIM {
            unsafe fn steal() -> Self {
                Self::steal()
            }
        }
        pub type $Timer = Timer<$TIM>;

        impl General for $TIM {
            type Width = $bits;

            #[inline(always)]
            fn max_auto_reload() -> u32 {
                <$bits>::MAX as u32
            }
            #[inline(always)]
            unsafe fn set_auto_reload_unchecked(&mut self, arr: u32) {
                self.arr().write(|w| w.bits(arr))
            }
            #[inline(always)]
            fn set_auto_reload(&mut self, arr: u32) -> Result<(), Error> {
                // Note: Make it impossible to set the ARR value to 0, since this
                // would cause an infinite loop.
                if arr > 0 && arr <= Self::max_auto_reload() {
                    Ok(unsafe { self.set_auto_reload_unchecked(arr) })
                } else {
                    Err(Error::WrongAutoReload)
                }
            }
            #[inline(always)]
            fn read_auto_reload() -> u32 {
                let tim = unsafe { &*<$TIM>::ptr() };
                tim.arr().read().bits()
            }
            #[inline(always)]
            fn enable_preload(&mut self, b: bool) {
                self.cr1().modify(|_, w| w.arpe().bit(b));
            }
            #[inline(always)]
            fn enable_counter(&mut self, b: bool) {
                self.cr1().modify(|_, w| w.cen().bit(b));
            }
            #[inline(always)]
            fn is_counter_enabled(&self) -> bool {
                self.cr1().read().cen().is_enabled()
            }
            #[inline(always)]
            fn reset_counter(&mut self) {
                self.cnt().reset();
            }
            #[inline(always)]
            fn set_prescaler(&mut self, psc: u16) {
                self.psc().write(|w| w.psc().set(psc) );
            }
            #[inline(always)]
            fn read_prescaler(&self) -> u16 {
                self.psc().read().psc().bits()
            }
            #[inline(always)]
            fn trigger_update(&mut self) {
                // Sets the URS bit to prevent an interrupt from being triggered by
                // the UG bit
                self.cr1().modify(|_, w| w.urs().set_bit());
                self.egr().write(|w| w.ug().set_bit());
                self.cr1().modify(|_, w| w.urs().clear_bit());
            }
            #[inline(always)]
            fn clear_interrupt_flag(&mut self, event: Event) {
                self.sr().write(|w| unsafe { w.bits(0xffff & !event.bits()) });
            }
            #[inline(always)]
            fn listen_interrupt(&mut self, event: Event, b: bool) {
                self.dier().modify(|r, w| unsafe { w.bits(
                    if b {
                        r.bits() | event.bits()
                    } else {
                        r.bits() & !event.bits()
                    }
                ) });
            }
            #[inline(always)]
            fn get_interrupt_flag(&self) -> Event {
                Event::from_bits_truncate(self.sr().read().bits())
            }
            #[inline(always)]
            fn read_count(&self) -> Self::Width {
                self.cnt().read().bits() as Self::Width
            }
            #[inline(always)]
            fn start_one_pulse(&mut self) {
                self.cr1().modify(|_, w| w.opm().set_bit().cen().set_bit());
            }
            #[inline(always)]
            fn cr1_reset(&mut self) {
                self.cr1().reset();
            }
            #[inline(always)]
            fn cnt_reset(&mut self) {
                self.cnt().reset();
            }
            #[inline(always)]
            fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
                dbg.cr().modify(|_, w| w.$dbg_timX_stop().bit(state));
            }
        }
        $(
            impl WithPwmCommon for $TIM {
                const CH_NUMBER: u8 = $cnum;
                const COMP_CH_NUMBER: u8 = $cnum;

                #[inline(always)]
                fn read_cc_value(c: u8) -> u32 {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::CH_NUMBER {
                        tim.ccr(c as usize).read().bits()
                    } else {
                        0
                    }
                }

                #[inline(always)]
                fn set_cc_value(c: u8, value: u32) {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::CH_NUMBER {
                        tim.ccr(c as usize).write(|w| unsafe { w.bits(value) })
                    }
                }

                #[inline(always)]
                fn enable_channel(c: u8, b: bool) {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::CH_NUMBER {
                        unsafe { bb::write(tim.ccer(), c*4, b); }
                    }
                }

                #[inline(always)]
                fn set_channel_polarity(c: u8, p: Polarity) {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::CH_NUMBER {
                        unsafe { bb::write(tim.ccer(), c*4 + 1, p == Polarity::ActiveLow); }
                    }
                }

                #[inline(always)]
                fn set_nchannel_polarity(c: u8, p: Polarity) {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::COMP_CH_NUMBER {
                        unsafe { bb::write(tim.ccer(), c*4 + 3, p == Polarity::ActiveLow); }
                    }
                }
            }

            $(
                impl Advanced for $TIM {
                    fn enable_nchannel(c: u8, b: bool) {
                        let $aoe = ();
                        let tim = unsafe { &*<$TIM>::ptr() };
                        if c < Self::COMP_CH_NUMBER {
                            unsafe { bb::write(tim.ccer(), c*4 + 2, b); }
                        }
                    }
                    fn set_dtg_value(value: u8) {
                        let tim = unsafe { &*<$TIM>::ptr() };
                        tim.bdtr().modify(|_,w| w.dtg().set(value));
                    }
                    fn read_dtg_value() -> u8 {
                        let tim = unsafe { &*<$TIM>::ptr() };
                        tim.bdtr().read().dtg().bits()
                    }
                    fn idle_state(c: u8, comp: bool, s: IdleState) {
                        let tim = unsafe { &*<$TIM>::ptr() };
                        if !comp {
                            if c < Self::CH_NUMBER {
                                unsafe { bb::write(tim.cr2(), c*2 + 8, s == IdleState::Set); }
                            }
                        } else {
                            if c < Self::COMP_CH_NUMBER {
                                unsafe { bb::write(tim.cr2(), c*2 + 9, s == IdleState::Set); }
                            }
                        }
                    }
                    #[inline(always)]
                    fn set_cms(cms: CenterAlignedMode) {
                        let tim = unsafe { &*<$TIM>::ptr() };
                        tim.cr1().write(|w| w.cms().variant(cms));
                    }
                }
            )?

            with_pwm!($TIM: $cnum $(, $aoe)?);
            split!($TIM: $cnum);
        )?

        $(impl MasterTimer for $TIM {
            type Mms = pac::$timbase::cr2::MMS;
            fn master_mode(&mut self, mode: Self::Mms) {
                self.cr2().modify(|_,w| w.mms().variant(mode));
            }
        })?
    }
}

macro_rules! with_pwm {
    ($TIM:ty: [$($Cx:literal, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident;)+] $(, $aoe:ident)?) => {
        impl WithPwm for $TIM {
            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, c: u8, mode: Ocm) {
                match c {
                    $(
                        $Cx => {
                            self.$ccmrx_output()
                            .modify(|_, w| w.$ocxpe().set_bit().$ocxm().set(mode as _) );
                        }
                    )+
                    #[allow(unreachable_patterns)]
                    _ => {},
                }
            }
            fn freeze_output_channel(&mut self, c: u8) {
                match c {
                        $(
                            $Cx => {
                                self.$ccmrx_output()
                                .modify(|_, w| w.$ocxpe().clear_bit().$ocxm().set(Ocm::Frozen as _) );
                            }
                        )+
                        #[allow(unreachable_patterns)]
                        _ => {},
                    }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                $(let $aoe = self.bdtr().modify(|_, w| w.aoe().set_bit());)?
                self.cr1().modify(|_, w| w.cen().set_bit());
            }
        }
    };
    ($TIM:ty: 1) => {
        with_pwm!($TIM: [
            0, ccmr1_output, oc1pe, oc1m;
        ]);
    };
    ($TIM:ty: 2) => {
        with_pwm!($TIM: [
            0, ccmr1_output, oc1pe, oc1m;
            1, ccmr1_output, oc2pe, oc2m;
        ]);
    };
    ($TIM:ty: 4 $(, $aoe:ident)?) => {
        with_pwm!($TIM: [
            0, ccmr1_output, oc1pe, oc1m;
            1, ccmr1_output, oc2pe, oc2m;
            2, ccmr2_output, oc3pe, oc3m;
            3, ccmr2_output, oc4pe, oc4m;
        ] $(, $aoe)?);
    };
}

impl<TIM: Instance> Timer<TIM> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        Self {
            clk: TIM::timer_clock(clocks),
            tim,
        }
    }

    pub fn configure(&mut self, clocks: &Clocks) {
        self.clk = TIM::timer_clock(clocks);
    }

    pub fn counter_hz(self) -> CounterHz<TIM> {
        CounterHz(self)
    }

    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    pub fn get_interrupt(&mut self) -> Event {
        self.tim.get_interrupt_flag()
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    /// Stopping timer in debug mode can cause troubles when sampling the signal
    pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
        self.tim.stop_in_debug(dbg, state);
    }
}

impl<TIM: Instance + MasterTimer> Timer<TIM> {
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

/// Timer wrapper for fixed precision timers.
///
/// Uses `fugit::TimerDurationU32` for most of operations
pub struct FTimer<TIM, const FREQ: u32> {
    tim: TIM,
}

/// `FTimer` with precision of 1 μs (1 MHz sampling)
pub type FTimerUs<TIM> = FTimer<TIM, 1_000_000>;

/// `FTimer` with precision of 1 ms (1 kHz sampling)
///
/// NOTE: don't use this if your system frequency more than 65 MHz
pub type FTimerMs<TIM> = FTimer<TIM, 1_000>;

impl<TIM: Instance, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        let mut t = Self { tim };
        t.configure(clocks);
        t
    }

    /// Calculate prescaler depending on `Clocks` state
    pub fn configure(&mut self, clocks: &Clocks) {
        let clk = TIM::timer_clock(clocks);
        assert!(clk.raw() % FREQ == 0);
        let psc = clk.raw() / FREQ;
        self.tim.set_prescaler(u16::try_from(psc - 1).unwrap());
    }

    /// Creates `Counter` that imlements [embedded_hal_02::timer::CountDown]
    pub fn counter(self) -> Counter<TIM, FREQ> {
        Counter(self)
    }

    /// Creates `Delay` that imlements [embedded_hal_02::blocking::delay] traits
    pub fn delay(self) -> Delay<TIM, FREQ> {
        Delay(self)
    }

    /// Releases the TIM peripheral
    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    pub fn get_interrupt(&mut self) -> Event {
        self.tim.get_interrupt_flag()
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    /// Stopping timer in debug mode can cause troubles when sampling the signal
    pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
        self.tim.stop_in_debug(dbg, state);
    }
}

impl<TIM: Instance + MasterTimer, const FREQ: u32> FTimer<TIM, FREQ> {
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

#[inline(always)]
const fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u32) {
    let ticks = clock / freq;
    let psc = (ticks - 1) / (1 << 16);
    let arr = ticks / (psc + 1) - 1;
    (psc as u16, arr)
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
hal!(pac::TIM1: [Timer1, u16, dbg_tim1_stop, c: (4, _aoe), m: tim1,]);

hal!(pac::TIM2: [Timer2, u16, dbg_tim2_stop, c: (4), m: tim2,]);
hal!(pac::TIM3: [Timer3, u16, dbg_tim3_stop, c: (4), m: tim2,]);

#[cfg(feature = "medium")]
hal!(pac::TIM4: [Timer4, u16, dbg_tim4_stop, c: (4), m: tim2,]);

#[cfg(any(feature = "high", feature = "connectivity"))]
hal!(pac::TIM5: [Timer5, u16, dbg_tim5_stop, c: (4), m: tim2,]);

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
hal!(pac::TIM6: [Timer6, u16, dbg_tim6_stop, m: tim6,]);

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
hal!(pac::TIM7: [Timer7, u16, dbg_tim7_stop, m: tim6,]);

#[cfg(all(feature = "stm32f103", feature = "high"))]
hal!(pac::TIM8: [Timer8, u16, dbg_tim8_stop, c: (4, _aoe), m: tim1,]);

#[cfg(feature = "stm32f100")]
hal!(pac::TIM15: [Timer15, u16, dbg_tim15_stop, c: (2),]);
#[cfg(feature = "stm32f100")]
hal!(pac::TIM16: [Timer16, u16, dbg_tim16_stop, c: (1),]);
#[cfg(feature = "stm32f100")]
hal!(pac::TIM17: [Timer17, u16, dbg_tim17_stop, c: (1),]);

//TODO: restore these timers once stm32-rs has been updated
/*
 *   dbg_tim(12-13)_stop fields missing from 103 xl in stm32-rs
 *   dbg_tim(9-10)_stop fields missing from 101 xl in stm32-rs
#[cfg(any(
    feature = "xl",
    all(
        feature = "stm32f100",
        feature = "high",
)))]
hal! {
    TIM12: (tim12, dbg_tim12_stop),
    TIM13: (tim13, dbg_tim13_stop),
    TIM14: (tim14, dbg_tim14_stop),
}
#[cfg(feature = "xl")]
hal! {
    TIM9: (tim9, dbg_tim9_stop),
    TIM10: (tim10, dbg_tim10_stop),
    TIM11: (tim11, dbg_tim11_stop),
}
*/
