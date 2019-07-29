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
    let (c0, c1, c2, c3) = device.TIM2.pwm(
        pins,
        &mut afio.mapr,
        100.hz(),
        clocks,
        &mut rcc.apb1
    );

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
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA6, PA7};
use crate::gpio::gpiob::{PB0, PB1, PB6, PB7, PB8, PB9};
use crate::gpio::{Alternate, PushPull};
use crate::rcc::{Clocks, APB1};
use crate::time::Hertz;
use crate::timer::PclkSrc;

pub trait Pins<TIM> {
    const REMAP: u8;
    const C1: bool;
    const C2: bool;
    const C3: bool;
    const C4: bool;
    type Channels;
}

impl Pins<TIM2>
    for (
        PA0<Alternate<PushPull>>,
        PA1<Alternate<PushPull>>,
        PA2<Alternate<PushPull>>,
        PA3<Alternate<PushPull>>,
    )
{
    const REMAP: u8 = 0b00;
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM2, C1>, Pwm<TIM2, C2>, Pwm<TIM2, C3>, Pwm<TIM2, C4>);
}

impl Pins<TIM3>
    for (
        PA6<Alternate<PushPull>>,
        PA7<Alternate<PushPull>>,
        PB0<Alternate<PushPull>>,
        PB1<Alternate<PushPull>>,
    )
{
    const REMAP: u8 = 0b00;
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM3, C1>, Pwm<TIM3, C2>, Pwm<TIM3, C3>, Pwm<TIM3, C4>);
}

impl Pins<TIM4>
    for (
        PB6<Alternate<PushPull>>,
        PB7<Alternate<PushPull>>,
        PB8<Alternate<PushPull>>,
        PB9<Alternate<PushPull>>,
    )
{
    const REMAP: u8 = 0b0;
    const C1: bool = true;
    const C2: bool = true;
    const C3: bool = true;
    const C4: bool = true;
    type Channels = (Pwm<TIM4, C1>, Pwm<TIM4, C2>, Pwm<TIM4, C3>, Pwm<TIM4, C4>);
}

pub trait PwmExt: Sized {
    fn pwm<PINS, T>(
        self,
        _: PINS,
        mapr: &mut MAPR,
        frequency: T,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

impl PwmExt for TIM2 {
    fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        mapr.mapr()
            .modify(|_, w| unsafe { w.tim2_remap().bits(PINS::REMAP) });

        tim2(self, _pins, freq.into(), clocks, apb)
    }
}

impl PwmExt for TIM3 {
    fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        mapr.mapr()
            .modify(|_, w| unsafe { w.tim3_remap().bits(PINS::REMAP) });

        tim3(self, _pins, freq.into(), clocks, apb)
    }
}

impl PwmExt for TIM4 {
    fn pwm<PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> PINS::Channels
    where
        PINS: Pins<Self>,
        T: Into<Hertz>,
    {
        mapr.mapr()
            .modify(|_, w| w.tim4_remap().bit(PINS::REMAP == 1));

        tim4(self, _pins, freq.into(), clocks, apb)
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
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clocks: Clocks,
                apb: &mut APB1,
            ) -> PINS::Channels
            where
                PINS: Pins<$TIMX>,
            {
                apb.enr().modify(|_, w| w.$timXen().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

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
                let clk = $TIMX::get_clk(&clocks).0;
                let freq = freq.0;
                let ticks = clk / freq;
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
    TIM2: (tim2, tim2en, tim2rst),
    TIM3: (tim3, tim3en, tim3rst),
    TIM4: (tim4, tim4en, tim4rst),
}
