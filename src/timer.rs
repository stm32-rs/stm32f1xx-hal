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

use crate::hal::timer::{Cancel, CountDown, Periodic};
use crate::pac::RCC;
#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
use crate::pac::TIM1;
#[cfg(feature = "medium")]
use crate::pac::TIM4;
#[cfg(any(feature = "high", feature = "connectivity"))]
use crate::pac::TIM5;
#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity",))]
use crate::pac::TIM6;
#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103",),),
    any(feature = "stm32f100", feature = "connectivity",)
))]
use crate::pac::TIM7;
#[cfg(all(feature = "stm32f103", feature = "high",))]
use crate::pac::TIM8;
use crate::pac::{DBGMCU as DBG, TIM2, TIM3};
#[cfg(feature = "stm32f100")]
use crate::pac::{TIM15, TIM16, TIM17};

use crate::rcc::{Clocks, Enable, GetBusFreq, RccBus, Reset};
use cast::{u16, u32, u64};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use void::Void;

use crate::time::Hertz;

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

#[derive(Debug, PartialEq)]
pub enum Error {
    /// Timer is canceled
    Canceled,
}

pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Hertz,
}

pub struct CountDownTimer<TIM> {
    tim: TIM,
    clk: Hertz,
}

pub(crate) mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: u8;
    }
    pub trait Ch1<REMAP> {}
    pub trait Ch2<REMAP> {}
    pub trait Ch3<REMAP> {}
    pub trait Ch4<REMAP> {}
}

macro_rules! remap {
    ($($name:ident: ($TIMX:ident, $state:literal, $P1:ident, $P2:ident, $P3:ident, $P4:ident),)+) => {
        $(
            pub struct $name;
            impl sealed::Remap for $name {
                type Periph = $TIMX;
                const REMAP: u8 = $state;
            }
            impl<MODE> sealed::Ch1<$name> for $P1<MODE> {}
            impl<MODE> sealed::Ch2<$name> for $P2<MODE> {}
            impl<MODE> sealed::Ch3<$name> for $P3<MODE> {}
            impl<MODE> sealed::Ch4<$name> for $P4<MODE> {}
        )+
    }
}

use crate::gpio::gpioa::{PA0, PA1, PA15, PA2, PA3, PA6, PA7};
use crate::gpio::gpiob::{PB0, PB1, PB10, PB11, PB3, PB4, PB5};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
use crate::gpio::{
    gpioa::{PA10, PA11, PA8, PA9},
    gpioe::{PE11, PE13, PE14, PE9},
};
#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
remap!(
    Tim1NoRemap: (TIM1, 0b00, PA8, PA9, PA10, PA11),
    //Tim1PartialRemap: (TIM1, 0b01, PA8, PA9, PA10, PA11),
    Tim1FullRemap: (TIM1, 0b11, PE9, PE11, PE13, PE14),
);

remap!(
    Tim2NoRemap: (TIM2, 0b00, PA0, PA1, PA2, PA3),
    Tim2PartialRemap1: (TIM2, 0b01, PA15, PB3, PA2, PA3),
    Tim2PartialRemap2: (TIM2, 0b10, PA0, PA1, PB10, PB11),
    Tim2FullRemap: (TIM2, 0b11, PA15, PB3, PB10, PB11),

    Tim3NoRemap: (TIM3, 0b00, PA6, PA7, PB0, PB1),
    Tim3PartialRemap: (TIM3, 0b10, PB4, PB5, PB0, PB1),
    Tim3FullRemap: (TIM3, 0b11, PC6, PC7, PC8, PC9),
);

#[cfg(feature = "medium")]
use crate::gpio::{
    gpiob::{PB6, PB7, PB8, PB9},
    gpiod::{PD12, PD13, PD14, PD15},
};
#[cfg(feature = "medium")]
remap!(
    Tim4NoRemap: (TIM4, 0b00, PB6, PB7, PB8, PB9),
    Tim4Remap: (TIM4, 0b01, PD12, PD13, PD14, PD15),
);

impl Timer<SYST> {
    pub fn syst(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Self {
            tim: syst,
            clk: clocks.hclk(),
        }
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

    pub fn release(self) -> SYST {
        self.tim
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

    /// Resets the counter
    pub fn reset(&mut self) {
        // According to the Cortex-M3 Generic User Guide, the interrupt request is only generated
        // when the counter goes from 1 to 0, so writing zero should not trigger an interrupt
        self.tim.clear_current();
    }

    /// Returns the number of microseconds since the last update event.
    /// *NOTE:* This method is not a very good candidate to keep track of time, because
    /// it is very easy to lose an update event.
    pub fn micros_since(&self) -> u32 {
        let reload_value = SYST::get_reload();
        let timer_clock = u64(self.clk.0);
        let ticks = u64(reload_value - SYST::get_current());

        // It is safe to make this cast since the maximum ticks is (2^24 - 1) and the minimum sysclk
        // is 4Mhz, which gives a maximum period of ~4.2 seconds which is < (2^32 - 1) microseconds
        u32(1_000_000 * ticks / timer_clock).unwrap()
    }

    /// Stops the timer
    pub fn stop(mut self) -> Timer<SYST> {
        self.tim.disable_counter();
        let Self { tim, clk } = self;
        Timer { tim, clk }
    }

    /// Releases the SYST
    pub fn release(self) -> SYST {
        self.stop().release()
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

impl Cancel for CountDownTimer<SYST> {
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if !self.tim.is_counter_enabled() {
            return Err(Self::Error::Canceled);
        }

        self.tim.disable_counter();
        Ok(())
    }
}

impl Periodic for CountDownTimer<SYST> {}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $APBx:ident, $dbg_timX_stop:ident$(,$master_timbase:ident)*),)+) => {
        $(
            impl Timer<$TIMX> {
                /// Initialize timer
                pub fn $timX(tim: $TIMX, clocks: &Clocks) -> Self {
                    // enable and reset peripheral to a clean slate state
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $TIMX::enable(rcc);
                    $TIMX::reset(rcc);

                    Self { tim, clk: <$TIMX as RccBus>::Bus::get_timer_frequency(&clocks) }
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

                $(
                    /// Starts timer in count down mode at a given frequency and additionally configures the timers master mode
                    pub fn start_master<T>(self, timeout: T, mode: crate::pac::$master_timbase::cr2::MMS_A) -> CountDownTimer<$TIMX>
                    where
                        T: Into<Hertz>,
                    {
                        let Self { tim, clk } = self;
                        let mut timer = CountDownTimer { tim, clk };
                        timer.tim.cr2.modify(|_,w| w.mms().variant(mode));
                        timer.start(timeout);
                        timer
                    }
                )?

                /// Starts the timer in count down mode with user-defined prescaler and auto-reload register
                pub fn start_raw(self, psc: u16, arr: u16) -> CountDownTimer<$TIMX>
                {
                    let Self { tim, clk } = self;
                    let mut timer = CountDownTimer { tim, clk };
                    timer.restart_raw(psc, arr);
                    timer
                }

                /// Resets timer peripheral
                #[inline(always)]
                pub fn clocking_reset(&mut self) {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $TIMX::reset(rcc);
                }

                /// Stopping timer in debug mode can cause troubles when sampling the signal
                #[inline(always)]
                pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
                    dbg.cr.modify(|_, w| w.$dbg_timX_stop().bit(state));
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

                /// Restarts the timer in count down mode with user-defined prescaler and auto-reload register
                pub fn restart_raw(&mut self, psc: u16, arr: u16)
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());

                    self.tim.psc.write(|w| w.psc().bits(psc) );

                    // TODO: Remove this `allow` once this field is made safe for stm32f100
                    #[allow(unused_unsafe)]
                    self.tim.arr.write(|w| unsafe { w.arr().bits(arr) });

                    // Trigger an update event to load the prescaler value to the clock
                    self.reset();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Retrieves the content of the prescaler register. The real prescaler is this value + 1.
                pub fn psc(&self) -> u16 {
                    self.tim.psc.read().psc().bits()
                }

                /// Retrieves the value of the auto-reload register.
                pub fn arr(&self) -> u16 {
                    self.tim.arr.read().arr().bits()
                }

                /// Retrieves the current timer counter value.
                pub fn cnt(&self) -> u16 {
                    self.tim.cnt.read().cnt().bits()
                }

                /// Stops the timer
                pub fn stop(self) -> Timer<$TIMX> {
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

                /// Returns the number of microseconds since the last update event.
                /// *NOTE:* This method is not a very good candidate to keep track of time, because
                /// it is very easy to lose an update event.
                pub fn micros_since(&self) -> u32 {
                    let timer_clock = self.clk.0;
                    let psc = u32(self.tim.psc.read().psc().bits());

                    // freq_divider is always bigger than 0, since (psc + 1) is always less than
                    // timer_clock
                    let freq_divider = u64(timer_clock / (psc + 1));
                    let cnt = u64(self.tim.cnt.read().cnt().bits());

                    // It is safe to make this cast, because the maximum timer period in this HAL is
                    // 1s (1Hz), then 1 second < (2^32 - 1) microseconds
                    u32(1_000_000 * cnt / freq_divider).unwrap()
                }

                /// Resets the counter
                pub fn reset(&mut self) {
                    // Sets the URS bit to prevent an interrupt from being triggered by
                    // the UG bit
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
                    let (psc, arr) = compute_arr_presc(timeout.into().0, self.clk.0);
                    self.restart_raw(psc, arr);
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

            impl Cancel for CountDownTimer<$TIMX>
            {
                type Error = Error;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    let is_counter_enabled = self.tim.cr1.read().cen().is_enabled();
                    if !is_counter_enabled {
                        return Err(Self::Error::Canceled);
                    }

                    // disable counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    Ok(())
                }
            }

            impl Periodic for CountDownTimer<$TIMX> {}
        )+
    }
}

#[inline(always)]
fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u16) {
    let ticks = clock / freq;
    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
    let arr = u16(ticks / u32(psc + 1)).unwrap();
    (psc, arr)
}

hal! {
    TIM2: (tim2, APB1, dbg_tim2_stop, tim2),
    TIM3: (tim3, APB1, dbg_tim3_stop, tim2),
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
hal! {
    TIM1: (tim1, APB2, dbg_tim1_stop, tim1),
}

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity",))]
hal! {
    TIM6: (tim6, APB1, dbg_tim6_stop, tim6),
}

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103",),),
    any(feature = "stm32f100", feature = "connectivity",)
))]
hal! {
    TIM7: (tim7, APB1, dbg_tim7_stop, tim6),
}

#[cfg(feature = "stm32f100")]
hal! {
    TIM15: (tim15, APB2, dbg_tim15_stop),
    TIM16: (tim16, APB2, dbg_tim16_stop),
    TIM17: (tim17, APB2, dbg_tim17_stop),
}

#[cfg(feature = "medium")]
hal! {
    TIM4: (tim4, APB1, dbg_tim4_stop, tim2),
}

#[cfg(any(feature = "high", feature = "connectivity"))]
hal! {
    TIM5: (tim5, APB1, dbg_tim5_stop, tim2),
}

#[cfg(all(feature = "stm32f103", feature = "high",))]
hal! {
    TIM8: (tim8, APB2, dbg_tim8_stop, tim1),
}

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
