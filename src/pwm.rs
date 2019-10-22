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
  let pins = (
      gpioa.pa0.into_alternate_push_pull(),
      gpioa.pa1.into_alternate_push_pull(),
      gpioa.pa2.into_alternate_push_pull(),
      gpioa.pa3.into_alternate_push_pull(),
  );
  ```

  Then call the `pwm` function on the corresponding timer:

  ```
    let device: pac::Peripherals = ..;

    // Put the timer in PWM mode using the specified pins
    // with a frequency of 100 hz.
    let (c0, c1, c2, c3) = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
        .pwm(pins, &mut afio.mapr, 100.hz());

    // Set the duty cycle of channel 0 to 50%
    c0.set_duty(c0.get_max_duty() / 2);
    // PWM outputs are disabled by default
    c0.enable()
  ```

  ## Usage for custom channel combinations

  Note that crate itself defines only basic channel combinations for default AFIO remappings,
  where all the channels are enabled. Meanwhile it is possible to configure PWM for any custom
  selection of channels. The `Pins` trait shows the mapping between timers, output pins and
  channels. So this trait needs to be implemented for the custom combination of channels and
  AFIO remappings. However minor additional efforts are needed since it is not possible to
  implement a foreign trait for a foreign type. The trick is to use the newtype pattern.

  The first example selects PB5 channel for TIM3 PWM output:

  ```
  struct MyChannels(PB5<Alternate<PushPull>>);

  impl Pins<TIM3>  for MyChannels {
    const REMAP: u8 = 0b10; // use TIM3 AFIO remapping for PB4, PB5, PB0, PB1 pins
    const C1: bool = false;
    const C2: bool = true;  // use channel C2
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM3, C2>;
  }
  ```

  The second example selects PC8 and PC9 channels for TIM3 PWM output:

  ```
  struct MyChannels(PC8<Alternate<PushPull>>, PC9<Alternate<PushPull>>);

  impl Pins<TIM3>  for MyChannels {
    const REMAP: u8 = 0b11; // use TIM3 AFIO remapping for PC6, PC7, PC8, PC9 pins
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = true;  // use channel C3
    const C4: bool = true;  // use channel C4
    type Channels = (Pwm<TIM3, C3>, Pwm<TIM3, C4>);
  }
  ```

  REMAP value and channel pins should be specified according to the stm32f1xx specification,
  e.g. the section 9.3.7 "Timer alternate function remapping" in RM0008 Rev 20.

  Finally, here is a complete example for two channels:

  ```
  use stm32f1xx_hal::stm32::TIM3;
  use stm32f1xx_hal::gpio::gpiob::{PB4, PB5};
  use stm32f1xx_hal::gpio::{Alternate, PushPull};
  use stm32f1xx_hal::pwm::{Pins, Pwm, C1, C2, C3, C4};

  struct MyChannels(PB4<Alternate<PushPull>>,  PB5<Alternate<PushPull>>);

  impl Pins<TIM3> for MyChannels
  {
    const REMAP: u8 = 0b10;
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = (Pwm<TIM3, C1>, Pwm<TIM3, C2>)
  }

  ...

  let gpiob = ..; // Set up and split GPIOB

  let p1 = gpiob.pb4.into_alternate_push_pull(&mut gpiob.crl);
  let p2 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

  ...

  let device: pac::Peripherals = ..;

  let (mut c1, mut c2) = device.TIM3.pwm(
      MyChannels(p1, p2),
      &mut afio.mapr,
      100.hz(),
      clocks,
      &mut rcc.apb1
  );

  // Set the duty cycle of channels C1 and C2 to 50% and 25% respectively
  c1.set_duty(c1.get_max_duty() / 2);
  c2.set_duty(c2.get_max_duty() / 4);

  // PWM outputs are disabled by default
  c1.enable()
  c2.enable()

  ```
*/

use core::marker::PhantomData;
use core::mem;

use cast::{u16, u32};
use crate::hal;
use crate::pac::{TIM2, TIM3, TIM4};

use crate::afio::MAPR;
use crate::bb;
use crate::gpio::{
    gpioa::{PA0, PA1, PA2, PA3, PA6, PA7, PA15},
    gpiob::{PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11},
    gpioc::{PC6, PC7, PC8, PC9},
    gpiod::{PD12, PD13, PD14, PD15},
    Alternate, PushPull
};
use crate::time::Hertz;
use crate::timer::Timer;

pub trait Pins<TIM> {
    const REMAP: u8;
    const C1: bool;
    const C2: bool;
    const C3: bool;
    const C4: bool;
    type Channels;
}

pub trait IsRemap {
    const REMAP: u8;
}

macro_rules! remap_impl {
    ( $($REMAP:ident: $BITS:tt,)+ ) => {
        $(
            pub struct $REMAP;

            impl IsRemap for $REMAP {
                const REMAP: u8 = $BITS;
            }
        )+
    }
}

remap_impl! {
    NoRemap: 0b0,
    Remap: 0b1,
    PartialRemap1: 0b01,
    PartialRemap2: 0b10,
    FullRemap: 0b11,
}

macro_rules! pins_impl {
    ( $($TIMX:ident: ( $REMAP:ident, $P1:ident, $P2:ident, $P3:ident, $P4:ident ),)+ ) => {
        $(
            pins_impl! {
                $TIMX: ($REMAP, $P1, $P2, $P3, $P4), (C1, C2, C3, C4), (),
                $TIMX: ($REMAP, $P2, $P3, $P4), (C2, C3, C4), (C1),
                $TIMX: ($REMAP, $P1, $P3, $P4), (C1, C3, C4), (C2),
                $TIMX: ($REMAP, $P1, $P2, $P4), (C1, C2, C4), (C3),
                $TIMX: ($REMAP, $P1, $P2, $P3), (C1, C2, C3), (C4),
                $TIMX: ($REMAP, $P3, $P4), (C3, C4), (C1, C2),
                $TIMX: ($REMAP, $P2, $P4), (C2, C4), (C1, C3),
                $TIMX: ($REMAP, $P2, $P3), (C2, C3), (C1, C4),
                $TIMX: ($REMAP, $P1, $P4), (C1, C4), (C2, C3),
                $TIMX: ($REMAP, $P1, $P3), (C1, C3), (C2, C4),
                $TIMX: ($REMAP, $P1, $P2), (C1, C2), (C3, C4),
                $TIMX: ($REMAP, $P1), (C1), (C2, C3, C4),
                $TIMX: ($REMAP, $P2), (C2), (C1, C3, C4),
                $TIMX: ($REMAP, $P3), (C3), (C1, C2, C4),
                $TIMX: ($REMAP, $P4), (C4), (C1, C2, C3),
            }
        )+
    };

    ( $($TIMX:ident: ( $REMAP:ident, $($PINX:ident),+ ), ( $($ENCHX:ident),+ ), ( $($DISCHX:ident),* ),)+ ) => {
        $(
            impl Pins<$TIMX>
                for (
                    $REMAP,
                    $($PINX<Alternate<PushPull>>,)+
                )
            {
                const REMAP: u8 = <$REMAP>::REMAP;
                $(const $ENCHX: bool = true;)+
                    $(const $DISCHX: bool = false;)*
                type Channels = ($(Pwm<$TIMX, $ENCHX>),+);
            }
        )+
    };
}

pins_impl! {
    TIM2: (NoRemap, PA0, PA1, PA2, PA3),
    TIM2: (PartialRemap1, PA15, PB3, PA2, PA3),
    TIM2: (PartialRemap2, PA0, PA1, PB10, PB11),
    TIM2: (FullRemap, PA15, PB3, PB10, PB11),

    TIM3: (NoRemap, PA6, PA7, PB0, PB1),
    TIM3: (PartialRemap2, PB4, PB5, PB0, PB1),
    TIM3: (FullRemap, PC6, PC7, PC8, PC9),

    TIM4: (NoRemap, PB6, PB7, PB8, PB9),
    TIM4: (Remap, PD12, PD13, PD14, PD15),
}

impl Timer<TIM2> {
    pub fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        PINS: Pins<TIM2>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(PINS::REMAP) });

        let Self { tim, clk } = self;
        tim2(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM3> {
    pub fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        PINS: Pins<TIM3>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(PINS::REMAP) });

        let Self { tim, clk } = self;
        tim3(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM4> {
    pub fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        PINS: Pins<TIM4>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(PINS::REMAP == 1));

        let Self { tim, clk } = self;
        tim4(tim, _pins, freq.into(), clk)
    }
}

pub struct Pwm<TIM, CHANNEL> {
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
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clk: Hertz,
            ) -> PINS::Channels
            where
                PINS: Pins<$TIMX>,
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

                unsafe { mem::uninitialized() }
            }

            impl hal::PwmPin for Pwm<$TIMX, C1> {
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

            impl hal::PwmPin for Pwm<$TIMX, C2> {
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

            impl hal::PwmPin for Pwm<$TIMX, C3> {
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

            impl hal::PwmPin for Pwm<$TIMX, C4> {
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

hal! {
    TIM2: (tim2),
    TIM3: (tim3),
    TIM4: (tim4),
}
