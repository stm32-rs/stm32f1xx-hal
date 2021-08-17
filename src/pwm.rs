/*!
  # Pulse width modulation

  The general purpose timers (`TIM2`, `TIM3`, and `TIM4`) can be used to output
  pulse width modulated signals on some pins. The timers support up to 4
  simultaneous pwm outputs in separate `Channels`

  ## Usage for pre-defined channel combinations

  This crate only defines basic channel combinations for default AFIO remappings,
  where all the channels are enabled. Start by setting all the pins for the
  timer you want to use to alternate push pull pins:

  ```rust
  let gpioa = ..; // Set up and split GPIOA
  // Select the pins you want to use
  let pins = (
      gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
  );

  // Set up the timer as a PWM output. If selected pins may correspond to different remap options,
  // then you must specify the remap generic parameter. Otherwise, if there is no such ambiguity,
  // the remap generic parameter can be omitted without complains from the compiler.
  let (c1, c2, c3, c4) = Timer::tim2(p.TIM2, &clocks)
      .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 1.khz())
      .3;

  // Start using the channels
  c1.set_duty(c1.get_max_duty());
  // ...
  ```

  Then call the `pwm` function on the corresponding timer.

  NOTE: In some cases you need to specify remap you need, especially for TIM2
  (see [Alternate function remapping](super::timer)):

  ```
    let device: pac::Peripherals = ..;

    // Put the timer in PWM mode using the specified pins
    // with a frequency of 100 hz.
    let (c0, c1, c2, c3) = Timer::tim2(device.TIM2, &clocks)
        .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 100.hz());

    // Set the duty cycle of channel 0 to 50%
    c0.set_duty(c0.get_max_duty() / 2);
    // PWM outputs are disabled by default
    c0.enable()
  ```
*/

use core::marker::Copy;
use core::marker::PhantomData;

use crate::hal;
#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
use crate::pac::TIM1;
#[cfg(feature = "medium")]
use crate::pac::TIM4;
use crate::pac::{TIM2, TIM3};
use cast::{u16, u32};

use crate::afio::MAPR;
use crate::bb;
use crate::gpio::{self, Alternate};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::Timer;

pub trait Pins<REMAP, P> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels;

    fn check_used(c: Channel) -> Channel {
        if (c == Channel::C1 && Self::C1)
            || (c == Channel::C2 && Self::C2)
            || (c == Channel::C3 && Self::C3)
            || (c == Channel::C4 && Self::C4)
        {
            c
        } else {
            panic!("Unused channel")
        }
    }

    fn split() -> Self::Channels;
}

#[derive(Clone, Copy, PartialEq)]
pub enum Channel {
    C1,
    C2,
    C3,
    C4,
}

use crate::timer::sealed::{Ch1, Ch2, Ch3, Ch4, Remap};
macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($TRAIT:ident),+ ), ( $($ENCHX:ident),+ ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<TIM, REMAP, OUTMODE, $($PINX,)+> Pins<REMAP, ($($ENCHX),+)> for ($($PINX),+)
            where
                REMAP: Remap<Periph = TIM>,
                $($PINX: $TRAIT<REMAP> + gpio::PinExt<Mode=Alternate<OUTMODE>>,)+
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(PwmChannel<TIM, $ENCHX>),+);
                fn split() -> Self::Channels {
                    ($(PwmChannel::<TIM, $ENCHX> { _channel: PhantomData, _tim: PhantomData }),+)
                }
            }
        )+
    };
}

pins_impl!(
    (P1, P2, P3, P4), (Ch1, Ch2, Ch3, Ch4), (C1, C2, C3, C4);
    (P2, P3, P4), (Ch2, Ch3, Ch4), (C2, C3, C4);
    (P1, P3, P4), (Ch1, Ch3, Ch4), (C1, C3, C4);
    (P1, P2, P4), (Ch1, Ch2, Ch4), (C1, C2, C4);
    (P1, P2, P3), (Ch1, Ch2, Ch3), (C1, C2, C3);
    (P3, P4), (Ch3, Ch4), (C3, C4);
    (P2, P4), (Ch2, Ch4), (C2, C4);
    (P2, P3), (Ch2, Ch3), (C2, C3);
    (P1, P4), (Ch1, Ch4), (C1, C4);
    (P1, P3), (Ch1, Ch3), (C1, C3);
    (P1, P2), (Ch1, Ch2), (C1, C2);
    (P1), (Ch1), (C1);
    (P2), (Ch2), (C2);
    (P3), (Ch3), (C3);
    (P4), (Ch4), (C4);
);

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
impl Timer<TIM1> {
    pub fn pwm<REMAP, P, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> Pwm<TIM1, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP, P>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

        // TIM1 has a break function that deactivates the outputs, this bit automatically activates
        // the output when no break input is present
        self.tim.bdtr.modify(|_, w| w.aoe().set_bit());

        let Self { tim, clk } = self;
        tim1(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM2> {
    pub fn pwm<REMAP, P, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> Pwm<TIM2, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP, P>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim2(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM3> {
    pub fn pwm<REMAP, P, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> Pwm<TIM3, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP, P>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim3(tim, _pins, freq.into(), clk)
    }
}

#[cfg(feature = "medium")]
impl Timer<TIM4> {
    pub fn pwm<REMAP, P, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> Pwm<TIM4, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP, P>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

        let Self { tim, clk } = self;
        tim4(tim, _pins, freq.into(), clk)
    }
}

pub struct Pwm<TIM, REMAP, P, PINS>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    clk: Hertz,
    _pins: PhantomData<(TIM, REMAP, P, PINS)>,
}

impl<TIM, REMAP, P, PINS> Pwm<TIM, REMAP, P, PINS>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }
}

pub struct PwmChannel<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident),)+) => {
        $(
            fn $timX<REMAP, P, PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clk: Hertz,
            ) -> Pwm<$TIMX, REMAP, P, PINS>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP, P>,
            {
                if PINS::C1 {
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1() );
                }

                if PINS::C2 {
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1() );
                }

                if PINS::C3 {
                    tim.ccmr2_output()
                        .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1() );
                }

                if PINS::C4 {
                    tim.ccmr2_output()
                        .modify(|_, w| w.oc4pe().set_bit().oc4m().pwm_mode1() );
                }
                let ticks = clk.0 / freq.0;
                let psc = u16(ticks / (1 << 16)).unwrap();
                tim.psc.write(|w| w.psc().bits(psc) );
                let arr = u16(ticks / u32(psc + 1)).unwrap();
                tim.arr.write(|w| w.arr().bits(arr));

                // The psc register is buffered, so we trigger an update event to update it
                // Sets the URS bit to prevent an interrupt from being triggered by the UG bit
                tim.cr1.modify(|_, w| w.urs().set_bit());
                tim.egr.write(|w| w.ug().set_bit());
                tim.cr1.modify(|_, w| w.urs().clear_bit());

                tim.cr1.write(|w|
                    w.cms()
                        .bits(0b00)
                        .dir()
                        .clear_bit()
                        .opm()
                        .clear_bit()
                        .cen()
                        .set_bit()
                );

                Pwm {
                    clk,
                    _pins: PhantomData
                }
            }

        /*
        The following implemention of the embedded_hal::Pwm uses Hertz as a time type.  This was choosen
        because of the timescales of operations being on the order of nanoseconds and not being able to
        efficently represent a float on the hardware.  It might be possible to change the time type to
        a different time based using such as the nanosecond.  The issue with doing so is that the max
        delay would then be at just a little over 2 seconds because of the 32 bit depth of the number.
        Using milliseconds is also an option, however, using this as a base unit means that only there
        could be resolution issues when trying to get a specific value, because of the integer nature.

        To find a middle ground, the Hertz type is used as a base here and the Into trait has been
        defined for several base time units.  This will allow for calling the set_period method with
        something that is natural to both the MCU and the end user.
        */
        impl<REMAP, P, PINS> hal::Pwm for Pwm<$TIMX, REMAP, P, PINS> where
            REMAP: Remap<Periph = $TIMX>,
            PINS: Pins<REMAP, P>,
            {
                type Channel = Channel;
                type Duty = u16;
                type Time = Hertz;

                fn enable(&mut self, channel: Self::Channel) {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 0) },
                        Channel::C2 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 4) },
                        Channel::C3 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 8) },
                        Channel::C4 => unsafe { bb::set(&(*$TIMX::ptr()).ccer, 12) }
                    }
                }

                fn disable(&mut self, channel: Self::Channel) {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 0) },
                        Channel::C2 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 4) },
                        Channel::C3 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 8) },
                        Channel::C4 => unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 12) },
                    }
                }

                fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).ccr1.read().ccr().bits() },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).ccr2.read().ccr().bits() },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).ccr3.read().ccr().bits() },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).ccr4.read().ccr().bits() },
                    }
                }

                fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { (*$TIMX::ptr()).ccr1.write(|w| w.ccr().bits(duty)) },
                        Channel::C2 => unsafe { (*$TIMX::ptr()).ccr2.write(|w| w.ccr().bits(duty)) },
                        Channel::C3 => unsafe { (*$TIMX::ptr()).ccr3.write(|w| w.ccr().bits(duty)) },
                        Channel::C4 => unsafe { (*$TIMX::ptr()).ccr4.write(|w| w.ccr().bits(duty)) },
                    }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn get_period(&self) -> Self::Time {
                    let clk = self.clk;
                    let psc: u16 = unsafe{(*$TIMX::ptr()).psc.read().psc().bits()};
                    let arr: u16 = unsafe{(*$TIMX::ptr()).arr.read().arr().bits()};

                    // Length in ms of an internal clock pulse
                    (clk.0 / u32(psc * arr)).hz()
                }

                fn set_period<T>(&mut self, period: T) where
                    T: Into<Self::Time> {
                        let clk = self.clk;

                        let ticks = clk.0 / period.into().0;
                        let psc = u16(ticks / (1 << 16)).unwrap();
                        let arr = u16(ticks / u32(psc + 1)).unwrap();
                        unsafe {
                            (*$TIMX::ptr()).psc.write(|w| w.psc().bits(psc));
                            (*$TIMX::ptr()).arr.write(|w| w.arr().bits(arr));
                        }
                }
            }

            impl hal::PwmPin for PwmChannel<$TIMX, C1> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 0) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 0) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr1.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr1.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for PwmChannel<$TIMX, C2> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 4) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 4) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr2.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr2.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for PwmChannel<$TIMX, C3> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 8) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 8) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr3.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr3.write(|w| w.ccr().bits(duty)) }
                }
            }

            impl hal::PwmPin for PwmChannel<$TIMX, C4> {
                type Duty = u16;

                fn disable(&mut self) {
                    unsafe { bb::clear(&(*$TIMX::ptr()).ccer, 12) }
                }

                fn enable(&mut self) {
                    unsafe { bb::set(&(*$TIMX::ptr()).ccer, 12) }
                }

                fn get_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).ccr4.read().ccr().bits() }
                }

                fn get_max_duty(&self) -> u16 {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn set_duty(&mut self, duty: u16) {
                    unsafe { (*$TIMX::ptr()).ccr4.write(|w| w.ccr().bits(duty)) }
                }
            }
        )+
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
hal! {
    TIM1: (tim1),
}

hal! {
    TIM2: (tim2),
    TIM3: (tim3),
}

#[cfg(feature = "medium")]
hal! {
    TIM4: (tim4),
}
