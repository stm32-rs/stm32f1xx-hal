//! # Quadrature Encoder Interface
use core::u16;

use crate::hal::{self, Direction};
#[cfg(feature = "tim2")]
use crate::pac::TIM2;
#[cfg(feature = "tim3")]
use crate::pac::TIM3;
#[cfg(feature = "tim4")]
use crate::pac::TIM4;

use crate::afio::MAPR;
#[cfg(feature = "gpioa")]
use crate::gpio::gpioa;
#[cfg(feature = "gpiob")]
use crate::gpio::gpiob;
use crate::gpio::{Floating, Input};
use crate::rcc::APB1;

pub trait Pins<TIM> {
    const REMAP: u8;
}

#[cfg(feature = "gpioa")]
#[cfg(feature = "tim2")]
impl Pins<TIM2> for (gpioa::PA0<Input<Floating>>, gpioa::PA1<Input<Floating>>) {
    const REMAP: u8 = 0b00;
}

#[cfg(feature = "gpioa")]
#[cfg(feature = "tim3")]
impl Pins<TIM3> for (gpioa::PA6<Input<Floating>>, gpioa::PA7<Input<Floating>>) {
    const REMAP: u8 = 0b00;
}

#[cfg(feature = "gpiob")]
#[cfg(feature = "tim4")]
impl Pins<TIM4> for (gpiob::PB6<Input<Floating>>, gpiob::PB7<Input<Floating>>) {
    const REMAP: u8 = 0b00;
}

pub struct Qei<TIM, PINS> {
    tim: TIM,
    pins: PINS,
}

#[cfg(feature = "tim2")]
impl<PINS> Qei<TIM2, PINS> {
    pub fn tim2(tim: TIM2, pins: PINS, mapr: &mut MAPR, apb: &mut APB1) -> Self
    where
        PINS: Pins<TIM2>,
    {
        mapr.mapr()
            .modify(|_, w| unsafe { w.tim2_remap().bits(PINS::REMAP) });

        Qei::_tim2(tim, pins, apb)
    }
}

#[cfg(feature = "tim3")]
impl<PINS> Qei<TIM3, PINS> {
    pub fn tim3(tim: TIM3, pins: PINS, mapr: &mut MAPR, apb: &mut APB1) -> Self
    where
        PINS: Pins<TIM3>,
    {
        mapr.mapr()
            .modify(|_, w| unsafe { w.tim3_remap().bits(PINS::REMAP) });

        Qei::_tim3(tim, pins, apb)
    }
}

#[cfg(feature = "tim4")]
impl<PINS> Qei<TIM4, PINS> {
    pub fn tim4(tim: TIM4, pins: PINS, mapr: &mut MAPR, apb: &mut APB1) -> Self
    where
        PINS: Pins<TIM4>,
    {
        mapr.mapr()
            .modify(|_, w| w.tim4_remap().bit(PINS::REMAP == 1));

        Qei::_tim4(tim, pins, apb)
    }
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl<PINS> Qei<$TIMX, PINS> {
                fn $timX(tim: $TIMX, pins: PINS, apb: &mut APB1) -> Self {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    // Configure TxC1 and TxC2 as captures
                    tim.ccmr1_output
                        .write(|w| unsafe { w.bits({ (0b01 << 0) | (0b01 << 8) }) });

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
                    tim.smcr.write(|w| unsafe { w.sms().bits(3) });

                    tim.arr.write(|w| w.arr().bits(u16::MAX));
                    tim.cr1.write(|w| w.cen().set_bit());

                    Qei { tim, pins }
                }

                pub fn release(self) -> ($TIMX, PINS) {
                    (self.tim, self.pins)
                }
            }

            impl<PINS> hal::Qei for Qei<$TIMX, PINS> {
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

#[cfg(feature = "tim2")]
hal! { TIM2: (_tim2, tim2en, tim2rst), }
#[cfg(feature = "tim3")]
hal! { TIM3: (_tim3, tim3en, tim3rst), }
#[cfg(feature = "tim4")]
hal! { TIM4: (_tim4, tim4en, tim4rst), }
