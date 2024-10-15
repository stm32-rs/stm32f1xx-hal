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
use crate::pac::{self, DBGMCU as DBG};

use crate::rcc::{self, BusTimerClock, Clocks, Rcc};
use core::convert::TryFrom;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::time::Hertz;

#[cfg(feature = "rtic1")]
pub mod monotonic;
#[cfg(feature = "rtic1")]
pub use monotonic::*;
#[cfg(feature = "rtic2")]
pub mod monotonics;
#[cfg(feature = "rtic2")]
pub use monotonics::*;
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

/// Compare/PWM polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// Capture polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapturePolarity {
    ActiveHigh,
    ActiveLow,
    ActiveBoth,
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
    fn counter<const FREQ: u32>(self, rcc: &mut Rcc) -> Counter<Self, FREQ>;
    /// Non-blocking [Counter] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 65 sec for 16-bit timer and from 2 ms to 49 days for 32-bit timer.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn counter_ms(self, rcc: &mut Rcc) -> CounterMs<Self> {
        self.counter::<1_000>(rcc)
    }
    /// Non-blocking [Counter] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 65 ms for 16-bit timer and from 2 μs to 71 min for 32-bit timer.
    fn counter_us(self, rcc: &mut Rcc) -> CounterUs<Self> {
        self.counter::<1_000_000>(rcc)
    }
    /// Non-blocking [Counter] with dynamic precision which uses `Hertz` as Duration units
    fn counter_hz(self, rcc: &mut Rcc) -> CounterHz<Self>;

    /// Blocking [Delay] with custom fixed precision
    fn delay<const FREQ: u32>(self, rcc: &mut Rcc) -> Delay<Self, FREQ>;
    /// Blocking [Delay] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 49 days.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn delay_ms(self, rcc: &mut Rcc) -> DelayMs<Self> {
        self.delay::<1_000>(rcc)
    }
    /// Blocking [Delay] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 71 min.
    fn delay_us(self, rcc: &mut Rcc) -> DelayUs<Self> {
        self.delay::<1_000_000>(rcc)
    }
}

impl<TIM: Instance> TimerExt for TIM {
    fn counter<const FREQ: u32>(self, rcc: &mut Rcc) -> Counter<Self, FREQ> {
        FTimer::new(self, rcc).counter()
    }
    fn counter_hz(self, rcc: &mut Rcc) -> CounterHz<Self> {
        Timer::new(self, rcc).counter_hz()
    }
    fn delay<const FREQ: u32>(self, rcc: &mut Rcc) -> Delay<Self, FREQ> {
        FTimer::new(self, rcc).delay()
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
/// Capture mode
/// Enum for configuring the mode of the Capture channels (CC1S, CC2S, CC3S, CC4S).
/// Defines how each channel is used in Input Capture mode, considering TI1, TI2, TI3, and TI4.
pub enum CaptureMode {
    /// Input Capture on the corresponding channel (e.g., CC1 -> TI1, CC2 -> TI2, etc.).
    InputCapture = 1,
    /// Input Capture on the inverted channel (e.g., CC1 -> TI2, CC2 -> TI1, CC3 -> TI4, CC4 -> TI3).
    InvChannelInputCapture = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
/// Enum for configuring the Input Capture prescaler.
/// Determines how many input events are required for one capture.
pub enum CapturePrescaler {
    /// No prescaler (00): Capture every input event.
    No = 0,
    /// Prescaler 2 (01): Capture every second input event.
    Two = 1,
    /// Prescaler 4 (10): Capture every fourth input event.
    Four = 2,
    /// Prescaler 8 (11): Capture every eighth input event.
    Eight = 3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
/// Enum representing the input capture filter settings.
pub enum CaptureFilter {
    /// No filter, sampling frequency = fDTS, N = 1
    NoFilter,
    /// Sampling frequency = fCK_INT, N = 2
    FckIntN2,
    /// Sampling frequency = fCK_INT, N = 4
    FckIntN4,
    /// Sampling frequency = fCK_INT, N = 8
    FckIntN8,
    /// Sampling frequency = fDTS/2, N = 6
    FdtsDiv2N6,
    /// Sampling frequency = fDTS/2, N = 8
    FdtsDiv2N8,
    /// Sampling frequency = fDTS/4, N = 6
    FdtsDiv4N6,
    /// Sampling frequency = fDTS/4, N = 8
    FdtsDiv4N8,
    /// Sampling frequency = fDTS/8, N = 6
    FdtsDiv8N6,
    /// Sampling frequency = fDTS/8, N = 8
    FdtsDiv8N8,
    /// Sampling frequency = fDTS/16, N = 5
    FdtsDiv16N5,
    /// Sampling frequency = fDTS/16, N = 6
    FdtsDiv16N6,
    /// Sampling frequency = fDTS/16, N = 8
    FdtsDiv16N8,
    /// Sampling frequency = fDTS/32, N = 5
    FdtsDiv32N5,
    /// Sampling frequency = fDTS/32, N = 6
    FdtsDiv32N6,
    /// Sampling frequency = fDTS/32, N = 8
    FdtsDiv32N8,
}

// Center-aligned mode selection
pub use pac::tim1::cr1::CMS as CenterAlignedMode;

mod sealed {
    use super::{
        CaptureFilter, CaptureMode, CapturePolarity, CapturePrescaler, CenterAlignedMode, Event,
        IdleState, Ocm, Polarity,
    };
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
    }

    pub trait WithChannel: General {
        const CH_NUMBER: u8;
        const COMP_CH_NUMBER: u8;
        fn read_cc_value(channel: u8) -> u32;
        fn set_cc_value(channel: u8, value: u32);
        fn enable_channel(channel: u8, b: bool);
        fn set_channel_polarity(channel: u8, p: Polarity);
        fn set_nchannel_polarity(channel: u8, p: Polarity);

        fn set_capture_channel_polarity(channel: u8, p: CapturePolarity);
    }

    #[allow(unused)]
    pub trait Advanced: WithChannel {
        fn enable_nchannel(channel: u8, b: bool);
        fn set_dtg_value(value: u8);
        fn read_dtg_value() -> u8;
        fn idle_state(channel: u8, comp: bool, s: IdleState);
        fn set_cms(mode: CenterAlignedMode);
    }

    pub trait WithPwm: WithChannel {
        fn preload_output_channel_in_mode(&mut self, c: u8, mode: Ocm);
        fn freeze_output_channel(&mut self, c: u8);
        fn start_pwm(&mut self);
    }

    #[allow(unused)]
    pub trait WithCapture: WithChannel {
        fn preload_capture(&mut self, c: u8, mode: CaptureMode);
        fn prescaler_capture(&mut self, c: u8, psc: CapturePrescaler);
        fn filter_capture(&mut self, c: u8, filter: CaptureFilter);
        fn start_capture(&mut self);
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
pub(crate) use sealed::{Advanced, General, MasterTimer, WithCapture, WithChannel, WithPwm};

pub trait Instance:
    rcc::Instance + rcc::RccBus<Bus: BusTimerClock> + rcc::StopInDebug + General
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
        $(c: ($cnum:tt, $ncnum:tt $(, $aoe:ident)?),)?
        $(m: $timbase:ident,)?
    ]) => {
        impl Instance for $TIM { }
        pub type $Timer = Timer<$TIM>;

        impl General for $TIM {
            type Width = $bits;

            #[inline(always)]
            fn max_auto_reload() -> u32 {
                <$bits>::MAX as u32
            }
            #[inline(always)]
            unsafe fn set_auto_reload_unchecked(&mut self, arr: u32) {
                self.arr().write(|w| w.bits(arr));
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
        }
        $(
            impl WithChannel for $TIM {
                const CH_NUMBER: u8 = $cnum;
                const COMP_CH_NUMBER: u8 = $ncnum;

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
                        tim.ccr(c as usize).write(|w| unsafe { w.bits(value) });
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

                #[inline(always)]
                fn set_capture_channel_polarity(c: u8, p: CapturePolarity) {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    if c < Self::CH_NUMBER {
                        match p {
                            CapturePolarity::ActiveLow => {
                                tim.ccer().modify(|r, w| {
                                    if c < Self::COMP_CH_NUMBER {
                                        unsafe { w.bits(r.bits() & !(1 << (c*4 + 3))); }
                                    }
                                    w.ccp(c).set_bit()
                                });
                            }
                            CapturePolarity::ActiveHigh => {
                                tim.ccer().modify(|r, w| {
                                    if c < Self::COMP_CH_NUMBER {
                                        unsafe { w.bits(r.bits() & !(1 << (c*4 + 3))); }
                                    }
                                    w.ccp(c).clear_bit()
                                });
                            }
                            CapturePolarity::ActiveBoth => {
                                tim.ccer().modify(|r, w| {
                                    if c < Self::COMP_CH_NUMBER {
                                        unsafe { w.bits(r.bits() | (1 << (c*4 + 3))); }
                                    }
                                    w.ccp(c).set_bit()
                                });
                            }
                        }

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

            with_output!($TIM: $cnum $(, $aoe)?);
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

macro_rules! with_output {
    ($TIM:ty: [$($Cx:literal, $ccmrx_input:ident, $ccmrx_output:ident, $ccxs:ident, $dc:literal;)+] $(, $aoe:ident)?) => {
        impl WithPwm for $TIM {
            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, c: u8, mode: Ocm) {
                match c {
                    $(
                        $Cx => {
                            let c = c-$dc;
                            self.$ccmrx_output()
                            .modify(|_, w| w.ocpe(c).set_bit().ocm(c).set(mode as _) );
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
                                let c = c-$dc;
                                self.$ccmrx_output()
                                .modify(|_, w| w.ocpe(c).clear_bit().ocm(c).set(Ocm::Frozen as _) );
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

        impl WithCapture for $TIM {
            #[inline(always)]
            fn preload_capture(&mut self, c: u8, mode: CaptureMode) {
                match c {
                    $(
                        $Cx => {
                            self.$ccmrx_input()
                            .modify(|_, w| unsafe { w.$ccxs().bits(mode as _) } );
                        }
                    )+
                    #[allow(unreachable_patterns)]
                    _ => {},
                }
            }

            #[inline(always)]
            fn prescaler_capture(&mut self, c: u8, psc: CapturePrescaler) {
                match c {
                    $(
                        $Cx => {
                            let c = c-$dc;
                            self.$ccmrx_input()
                            .modify(|_, w| unsafe { w.icpsc(c).bits(psc as _) } );
                        }
                    )+
                    #[allow(unreachable_patterns)]
                    _ => {},
                }
            }

            fn filter_capture(&mut self, c: u8, filter: CaptureFilter) {
                match c {
                    $(
                        $Cx => {
                            let c = c-$dc;
                            self.$ccmrx_input()
                            .modify(|_, w| unsafe { w.icf(c).bits(filter as _) } );
                        }
                    )+
                    #[allow(unreachable_patterns)]
                    _ => {},
                }
            }


            #[inline(always)]
            fn start_capture(&mut self) {
                self.cr1().modify(|_, w| w.cen().set_bit());
            }
        }
    };
    ($TIM:ty: 1) => {
        with_output!($TIM: [
            0, ccmr1_input, ccmr1_output, cc1s, 0;
        ]);
    };
    ($TIM:ty: 2) => {
        with_output!($TIM: [
            0, ccmr1_input, ccmr1_output, cc1s, 0;
            1, ccmr1_input, ccmr1_output, cc2s, 0;
        ]);
    };
    ($TIM:ty: 4 $(, $aoe:ident)?) => {
        with_output!($TIM: [
            0, ccmr1_input, ccmr1_output, cc1s, 0;
            1, ccmr1_input, ccmr1_output, cc2s, 0;
            2, ccmr2_input, ccmr2_output, cc3s, 2;
            3, ccmr2_input, ccmr2_output, cc4s, 2;
        ] $(, $aoe)?);
    };
}

impl<TIM: Instance> Timer<TIM> {
    /// Initialize timer
    pub fn new(tim: TIM, rcc: &mut Rcc) -> Self {
        // Enable and reset the timer peripheral
        TIM::enable(rcc);
        TIM::reset(rcc);

        Self {
            clk: TIM::Bus::timer_clock(&rcc.clocks),
            tim,
        }
    }

    pub fn configure(&mut self, clocks: &Clocks) {
        self.clk = TIM::Bus::timer_clock(clocks);
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
    pub fn new(tim: TIM, rcc: &mut Rcc) -> Self {
        // Enable and reset the timer peripheral
        TIM::enable(rcc);
        TIM::reset(rcc);

        let mut t = Self { tim };
        t.configure(&rcc.clocks);
        t
    }

    /// Calculate prescaler depending on `Clocks` state
    pub fn configure(&mut self, clocks: &Clocks) {
        let clk = TIM::Bus::timer_clock(clocks);
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
hal!(pac::TIM1: [Timer1, u16, c: (4, 4, _aoe), m: tim1,]);

hal!(pac::TIM2: [Timer2, u16, c: (4, 0), m: tim2,]);
hal!(pac::TIM3: [Timer3, u16, c: (4, 0), m: tim2,]);

#[cfg(feature = "medium")]
hal!(pac::TIM4: [Timer4, u16, c: (4, 0), m: tim2,]);

#[cfg(any(feature = "high", feature = "connectivity"))]
hal!(pac::TIM5: [Timer5, u16, c: (4, 0), m: tim2,]);

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
hal!(pac::TIM6: [Timer6, u16, m: tim6,]);

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
hal!(pac::TIM7: [Timer7, u16, m: tim6,]);

#[cfg(all(feature = "stm32f103", feature = "high"))]
hal!(pac::TIM8: [Timer8, u16, c: (4, 4, _aoe), m: tim1,]);

#[cfg(feature = "xl")]
hal!(pac::TIM9: [Timer9, u16, c: (2, 2),]);
#[cfg(feature = "xl")]
hal!(pac::TIM10: [Timer10, u16, c: (1, 1),]);
#[cfg(feature = "xl")]
hal!(pac::TIM11: [Timer11, u16, c: (1, 1),]);

#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
hal!(pac::TIM12: [Timer12, u16, c: (2, 2),]);
#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
hal!(pac::TIM13: [Timer13, u16, c: (1, 1),]);
#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
hal!(pac::TIM14: [Timer14, u16, c: (1, 1),]);

#[cfg(feature = "stm32f100")]
hal!(pac::TIM15: [Timer15, u16, c: (2, 2),]);
#[cfg(feature = "stm32f100")]
hal!(pac::TIM16: [Timer16, u16, c: (1, 1),]);
#[cfg(feature = "stm32f100")]
hal!(pac::TIM17: [Timer17, u16, c: (1, 1),]);
