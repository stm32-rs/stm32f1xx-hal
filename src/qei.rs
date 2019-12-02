/**
  # Quadrature Encoder Interface

  NOTE: In some cases you need to specify remap you need, especially for TIM2
  (see [Alternate function remapping](super::timer)):
*/

use core::u16;

use core::marker::PhantomData;

use crate::hal::{self, Direction};
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

use crate::timer::{Timer, sealed::Remap};
use crate::pwm_input::Pins;

pub struct Qei<TIM, REMAP, PINS> {
    tim: TIM,
    pins: PINS,
    _remap: PhantomData<REMAP>,
}

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f103",
    feature = "stm32f105",
))]
impl Timer<TIM1> {
    pub fn qei<REMAP, PINS>(self, pins: PINS, mapr: &mut MAPR) -> Qei<TIM1, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim1(tim, pins)
    }
}

impl Timer<TIM2> {
    pub fn qei<REMAP, PINS>(self, pins: PINS, mapr: &mut MAPR) -> Qei<TIM2, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim2(tim, pins)
    }
}

impl Timer<TIM3> {
    pub fn qei<REMAP, PINS>(self, pins: PINS, mapr: &mut MAPR) -> Qei<TIM3, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim3(tim, pins)
    }
}

#[cfg(feature = "medium")]
impl Timer<TIM4> {
    pub fn qei<REMAP, PINS>(self, pins: PINS, mapr: &mut MAPR) -> Qei<TIM4, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

        let Self { tim, clk: _ } = self;
        Qei::_tim4(tim, pins)
    }
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl<REMAP, PINS> Qei<$TIMX, REMAP, PINS> {
                fn $timX(tim: $TIMX, pins: PINS) -> Self {
                    // Configure TxC1 and TxC2 as captures
                    tim.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti2());

                    // enable and configure to capture on rising edge
                    tim.ccer.write(|w| {
                        w.cc1e()
                            .set_bit()
                            .cc1p()
                            .clear_bit()
                            .cc2e()
                            .set_bit()
                            .cc2p()
                            .clear_bit()
                    });

                    // configure as quadrature encoder
                    tim.smcr.write(|w| w.sms().bits(3));

                    tim.arr.write(|w| w.arr().bits(u16::MAX));
                    tim.cr1.write(|w| w.cen().set_bit());

                    Qei { tim, pins, _remap: PhantomData }
                }

                pub fn release(self) -> ($TIMX, PINS) {
                    (self.tim, self.pins)
                }
            }

            impl<REMAP, PINS> hal::Qei for Qei<$TIMX, REMAP, PINS> {
                type Count = u16;

                fn count(&self) -> u16 {
                    self.tim.cnt.read().cnt().bits()
                }

                fn direction(&self) -> Direction {
                    if self.tim.cr1.read().dir().bit_is_clear() {
                        hal::Direction::Upcounting
                    } else {
                        hal::Direction::Downcounting
                    }
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
    TIM1: (_tim1, tim1en, tim1rst),
}
hal! {
    TIM2: (_tim2, tim2en, tim2rst),
    TIM3: (_tim3, tim3en, tim3rst),
}
#[cfg(feature = "medium")]
hal! {
    TIM4: (_tim4, tim4en, tim4rst),
}
