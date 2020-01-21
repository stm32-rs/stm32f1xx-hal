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
  let (c1, c2, c3, c4) = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
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
    let (c0, c1, c2, c3) = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
        .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 100.hz());

    // Set the duty cycle of channel 0 to 50%
    c0.set_duty(c0.get_max_duty() / 2);
    // PWM outputs are disabled by default
    c0.enable()
  ```
*/

use core::marker::PhantomData;
use core::marker::{Copy};
use core::mem;
use core::ops::{Deref, DerefMut};
use embedded_hal::PwmPin;

use cast::{u16, u32};
use crate::hal;
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
use crate::bb;
use crate::gpio::{self, Alternate, PushPull};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::timer::Timer;

pub trait Pins<REMAP, P> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels;
}

use crate::timer::sealed::{Remap, Ch1, Ch2, Ch3, Ch4};
macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($TRAIT:ident),+ ), ( $($ENCHX:ident),* ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<TIM, REMAP, $($PINX,)+> Pins<REMAP, ($($ENCHX),+)> for ($($PINX),+)
            where
                REMAP: Remap<Periph = TIM>,
                $($PINX: $TRAIT<REMAP> + gpio::Mode<Alternate<PushPull>>,)+
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(PwmChannel<TIM, $ENCHX>),+);
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

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
impl Timer<TIM1> {
    pub fn pwm<REMAP, P, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> Pwm<TIM1, PINS::Channels>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP, P>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

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
    ) -> Pwm<TIM2, PINS::Channels>
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
    ) -> Pwm<TIM3, PINS::Channels>
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
    ) -> Pwm<TIM4, PINS::Channels>
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

pub struct Pwm<TIM, PWMCHANNELS> {
    clk: Hertz,
    _channels: PhantomData<PWMCHANNELS>,
    _tim: PhantomData<TIM>,
}

impl<TIM, PWMCHANNELS> Deref for Pwm<TIM, PWMCHANNELS> {
    type Target = PWMCHANNELS;

    fn deref(&self) -> &Self::Target {
        unsafe { mem::MaybeUninit::uninit().assume_init() }
    }
}

impl<TIM, PWMCHANNELS> DerefMut for Pwm<TIM, PWMCHANNELS> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { mem::MaybeUninit::uninit().assume_init() }
    }
}

#[derive(Copy, Clone)]
pub struct PwmChannel<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

/*
  The following implemention of the embedded_hal::Pwm uses Hertz as a time type.  This was choosen 
  because of the timescales of operationsbeing on the order of nanoseconds and not being able to 
  efficently represent a float on the hardware.  It might be possible to change the time type to
  a different time based using such as the nanosecond.  The issue with doing so is that the max 
  delay would then be at just a little over 2 seconds because of the 32 bit depth of the number.
  Using milliseconds is also an option, however, using this as a base unit means that only there 
  could be resolution issues when trying to get a specific value, because of the integer nature.

  To find a middle ground, the Hertz type is used as a base here and the Into trait has been 
  defined for several base time units.  This will allow for calling the set_period method with 
  something that is natural to both the MCU and the end user.
*/
macro_rules! pwm_impl {
    ( $( $TIMX:ident, $timX:ident, ( $($CHNUM:pat),+ ), ( $($ENUMNUM:tt),+ ), ( $($ENCHX:ident),+ ); )+ )  => {
            $(
            #[allow(unused_parens)]
            impl hal::Pwm for Pwm<$TIMX, ($(PwmChannel<$TIMX, $ENCHX>),+,)> {
                type Channel = u8;
                type Duty = u16;
                type Time = Hertz;

                fn enable(&mut self, channel: Self::Channel) {
                    match channel {
                        $( $CHNUM => self.$ENUMNUM.enable(),)+
                        _ => {}
                    }
                }

                fn disable(&mut self, channel: Self::Channel) {
                    match channel {
                        $( $CHNUM => self.$ENUMNUM.disable(),)+
                        _ => {}
                    }
                }

                fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                    match channel {
                        $( $CHNUM => self.$ENUMNUM.get_duty(),)+
                        _ => 0
                    }
                }

                fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                    match channel {
                        $( $CHNUM => self.$ENUMNUM.set_duty(duty),)+
                        _ => {}
                    }
                }

                fn get_max_duty(&self) -> Self::Duty {
                    unsafe { (*$TIMX::ptr()).arr.read().arr().bits() }
                }

                fn get_period(&self) -> Self::Time {
                    let clk = self.clk;
                    let mut psc: u16 = 0;
                    let mut arr: u16 = 0;
                    unsafe {
                        psc = (*$TIMX::ptr()).psc.read().psc().bits();
                        arr = (*$TIMX::ptr()).arr.read().arr().bits();
                    }

                    // Length in ms of an internal clock pulse
                    (clk.0 / u32(psc * arr)).hz()
//                    (((psc as u32) / clk.0) * (1_000_000 as u32) * (arr as u32)).ms()
                }

                fn set_period<P>(&mut self, period: P) where
                    P: Into<Self::Time> {
                        let clk = self.clk;

//                        let freq = u16(1 / period.into());

                        let ticks = clk.0 / period.into().0;
                        let psc = u16(ticks / (1 << 16)).unwrap();
                        let arr = u16(ticks / u32(psc + 1)).unwrap();
                        unsafe {
                            (*$TIMX::ptr()).psc.write(|w| w.psc().bits(psc));
                            (*$TIMX::ptr()).arr.write(|w| w.arr().bits(arr));
                        }
                }
            }
        )+
    };
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident),)+) => {
        $(
            fn $timX<REMAP, P, PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clk: Hertz,
            ) -> Pwm<$TIMX, PINS::Channels>
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
                    clk: clk,
                    _tim: PhantomData,
                    _channels: PhantomData
                }
            }
            
            pwm_impl!(
                $TIMX, $timX, (0, 1, 2, 3), (0, 1, 2, 3), (C1, C2, C3, C4);
                $TIMX, $timX, (0, 1, 2), (0, 1, 2), (C1, C2, C3);
                $TIMX, $timX, (0, 1, 2), (0, 1, 2), (C1, C2, C4);
                $TIMX, $timX, (0, 1, 2), (0, 1, 2), (C2, C3, C4);
                $TIMX, $timX, (0, 1), (0, 1), (C1, C2);
                $TIMX, $timX, (0, 1), (0, 1), (C1, C3);
                $TIMX, $timX, (0, 1), (0, 1), (C1, C4);
                $TIMX, $timX, (0, 1), (0, 1), (C2, C3);
                $TIMX, $timX, (0, 1), (0, 1), (C2, C4);
                $TIMX, $timX, (0, 1), (0, 1), (C3, C4);
                $TIMX, $timX, (0), (0), (C1);
                $TIMX, $timX, (0), (0), (C2);
                $TIMX, $timX, (0), (0), (C3);
                $TIMX, $timX, (0), (0), (C4);
            );

            impl Copy for PwmChannel<$TIMX, C1> {}
            impl Clone for PwmChannel<$TIMX, C1> {
                fn clone(&self) -> Self {
                    unsafe { mem::MaybeUninit::uninit().assume_init() }
                }            
            }

            impl Copy for PwmChannel<$TIMX, C2> {}
            impl Clone for PwmChannel<$TIMX, C2> {
                fn clone(&self) -> Self {
                    unsafe { mem::MaybeUninit::uninit().assume_init() }
                }            
            }

            impl Copy for PwmChannel<$TIMX, C3> {}
            impl Clone for PwmChannel<$TIMX, C3> {
                fn clone(&self) -> Self {
                    unsafe { mem::MaybeUninit::uninit().assume_init() }
                }            
            }

            impl Copy for PwmChannel<$TIMX, C4> {}
            impl Clone for PwmChannel<$TIMX, C4> {
                fn clone(&self) -> Self {
                    unsafe { mem::MaybeUninit::uninit().assume_init() }
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

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
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
