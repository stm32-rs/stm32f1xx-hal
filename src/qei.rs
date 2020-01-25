/**
  # Quadrature Encoder Interface

  NOTE: In some cases you need to specify remap you need, especially for TIM2
  (see [Alternate function remapping](super::timer)):
*/
use core::u16;

use core::marker::PhantomData;

use crate::hal::{self, Direction};
#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "stm32f105",))]
use crate::pac::TIM1;
#[cfg(feature = "medium")]
use crate::pac::TIM4;
use crate::pac::{TIM2, TIM3};

use crate::afio::MAPR;

use crate::pwm_input::Pins;
use crate::timer::{sealed::Remap, Timer};

/// SMS (Slave Mode Selection) register
#[derive(Copy, Clone, Debug)]
pub enum SlaveMode {
    /// Counter counts up/down on TI2FP1 edge depending on TI1FP2 level.
    EncoderMode1 = 0b001,
    /// Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level.
    EncoderMode2 = 0b010,
    /// Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the
    /// level of the other input.
    EncoderMode3 = 0b011,
    /// Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and
    /// generates an update of the registers.
    ResetMode = 0b100,
    /// Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not
    /// reset). Only the start of the counter is controlled.
    TriggerMode = 0b110,
    /// External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.
    ExternalClockMode1 = 0b111,
}

/// Quadrature Encoder Interface (QEI) options
///
/// The `Default` implementation provides a configuration for a 4-count pulse which counts from
/// 0-65535. The counter wraps back to 0 on overflow.
#[derive(Copy, Clone, Debug)]
pub struct QeiOptions {
    /// Encoder slave mode
    pub slave_mode: SlaveMode,

    /// Autoreload value
    ///
    /// This value allows the maximum count to be configured, up to 65535. Setting a lower value
    /// will overflow the counter to 0 sooner.
    pub auto_reload_value: u16,
}

impl Default for QeiOptions {
    fn default() -> Self {
        Self {
            slave_mode: SlaveMode::EncoderMode3,
            auto_reload_value: u16::MAX,
        }
    }
}

pub struct Qei<TIM, REMAP, PINS> {
    tim: TIM,
    pins: PINS,
    _remap: PhantomData<REMAP>,
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "stm32f105",))]
impl Timer<TIM1> {
    pub fn qei<REMAP, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        options: QeiOptions,
    ) -> Qei<TIM1, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim1(tim, pins, options)
    }
}

impl Timer<TIM2> {
    pub fn qei<REMAP, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        options: QeiOptions,
    ) -> Qei<TIM2, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim2(tim, pins, options)
    }
}

impl Timer<TIM3> {
    pub fn qei<REMAP, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        options: QeiOptions,
    ) -> Qei<TIM3, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        let Self { tim, clk: _ } = self;
        Qei::_tim3(tim, pins, options)
    }
}

#[cfg(feature = "medium")]
impl Timer<TIM4> {
    pub fn qei<REMAP, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        options: QeiOptions,
    ) -> Qei<TIM4, REMAP, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

        let Self { tim, clk: _ } = self;
        Qei::_tim4(tim, pins, options)
    }
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl<REMAP, PINS> Qei<$TIMX, REMAP, PINS> {
                fn $timX(tim: $TIMX, pins: PINS, options: QeiOptions) -> Self {
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
                    tim.smcr.write(|w| w.sms().bits(options.slave_mode as u8));

                    tim.arr.write(|w| w.arr().bits(options.auto_reload_value));
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

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "stm32f105",))]
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
