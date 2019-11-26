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
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA6, PA7, PA15};
use crate::gpio::gpiob::{PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
use crate::gpio::{Alternate, PushPull};
use crate::time::Hertz;
use crate::timer::Timer;


mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: u8;
    }
    pub trait Ch1<REMAP> {
        const ENABLED: bool;
    }
    pub trait Ch2<REMAP> {
        const ENABLED: bool;
    }
    pub trait Ch3<REMAP> {
        const ENABLED: bool;
    }
    pub trait Ch4<REMAP> {
        const ENABLED: bool;
    }
}
use sealed::{Remap, Ch1, Ch2, Ch3, Ch4};

pub trait Pins<TIM> {
    fn ch1_state() -> bool {
        false
    }
    fn ch2_state() -> bool {
        false
    }
    fn ch3_state() -> bool {
        false
    }
    fn ch4_state() -> bool {
        false
    }
    type Channels;
}
impl<TIM, REMAP, CH1, CH2, CH3, CH4> Pins<REMAP> for (CH1, CH2, CH3, CH4)
where
    REMAP: Remap<Periph = TIM>,
    CH1: Ch1<REMAP>,
    CH2: Ch2<REMAP>,
    CH3: Ch3<REMAP>,
    CH4: Ch4<REMAP> {
    type Channels = (Pwm<TIM, C1>, Pwm<TIM, C2>, Pwm<TIM, C3>, Pwm<TIM, C4>);
    fn ch1_state() -> bool {
        CH1::ENABLED
    }
    fn ch2_state() -> bool {
        CH2::ENABLED
    }
    fn ch3_state() -> bool {
        CH3::ENABLED
    }
    fn ch4_state() -> bool {
        CH4::ENABLED
    }
}

use crate::gpio::NoPin;
impl<REMAP> Ch1<REMAP> for NoPin {
    const ENABLED: bool = false;
}
impl<REMAP> Ch2<REMAP> for NoPin {
    const ENABLED: bool = false;
}
impl<REMAP> Ch3<REMAP> for NoPin {
    const ENABLED: bool = false;
}
impl<REMAP> Ch4<REMAP> for NoPin {
    const ENABLED: bool = false;
}

macro_rules! remap {
    ($name:ident, $TIMX:ident, $state:literal, $CH1:ident, $CH2:ident, $CH3:ident, $CH4:ident) => {
        pub struct $name;
        impl Remap for $name {
            type Periph = $TIMX;
            const REMAP: u8 = $state;
        }
        impl Ch1<$name> for $CH1<Alternate<PushPull>> {
            const ENABLED: bool = true;
        }
        impl Ch2<$name> for $CH2<Alternate<PushPull>> {
            const ENABLED: bool = true;
        }
        impl Ch3<$name> for $CH3<Alternate<PushPull>> {
            const ENABLED: bool = true;
        }
        impl Ch4<$name> for $CH4<Alternate<PushPull>> {
            const ENABLED: bool = true;
        }
    }
}

remap!(Tim2NoRemap, TIM2, 0b00, PA0, PA1, PA2, PA3);
remap!(Tim2PartialRemap1, TIM2, 0b01, PA15, PB3, PA2, PA3);
remap!(Tim2PartialRemap2, TIM2, 0b10, PA0, PA1, PB10, PB11);
remap!(Tim2FullRemap, TIM2, 0b11, PA15, PB3, PB10, PB11);
remap!(Tim3NoRemap, TIM3, 0b00, PA6, PA7, PB0, PB1);
remap!(Tim3PartialRemap, TIM3, 0b10, PB4, PB5, PB0, PB1);
remap!(Tim3FullRemap, TIM3, 0b11, PC6, PC7, PC8, PC9);
remap!(Tim4NoRemap, TIM4, 0b00, PB6, PB7, PB8, PB9);

impl Timer<TIM2> {
    pub fn pwm<REMAP, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim2(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM3> {
    pub fn pwm<REMAP, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        let Self { tim, clk } = self;
        tim3(tim, _pins, freq.into(), clk)
    }
}

impl Timer<TIM4> {
    pub fn pwm<REMAP, PINS, T>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: T,
    ) -> PINS::Channels
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP>,
        T: Into<Hertz>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

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
            fn $timX<REMAP, PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                clk: Hertz,
            ) -> PINS::Channels
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP>,
            {
                if PINS::ch1_state() {
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1() );
                }

                if PINS::ch2_state() {
                    tim.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1() );
                }

                if PINS::ch3_state() {
                    tim.ccmr2_output()
                        .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1() );
                }

                if PINS::ch4_state() {
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

                unsafe { mem::MaybeUninit::uninit().assume_init() }
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
