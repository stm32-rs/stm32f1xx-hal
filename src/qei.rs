/**
  # Quadrature Encoder Interface

  NOTE: In some cases you need to specify remap you need, especially for TIM2
  (see [Alternate function remapping](super::timer)):
*/
use crate::pac;
use embedded_hal_02 as hal;
pub use hal::Direction;

use crate::rcc::Clocks;
use crate::timer::{InputPins, Timer};

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

pub struct Qei<TIM: InputPins> {
    tim: TIM,
    pins: TIM::Channels12,
}

pub trait QeiExt: Sized + InputPins {
    fn qei(
        self,
        pins: impl Into<<Self as InputPins>::Channels12>,
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self>;
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl QeiExt for pac::TIM1 {
    fn qei(
        self,
        pins: impl Into<<Self as InputPins>::Channels12>,
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self> {
        Timer::new(self, clocks).qei(pins, options)
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl Timer<pac::TIM1> {
    pub fn qei(
        self,
        pins: impl Into<<pac::TIM1 as InputPins>::Channels12>,
        options: QeiOptions,
    ) -> Qei<pac::TIM1> {
        let Self { tim, clk: _ } = self;
        Qei::_tim1(tim, pins, options)
    }
}

impl QeiExt for pac::TIM2 {
    fn qei(
        self,
        pins: impl Into<<Self as InputPins>::Channels12>,
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self> {
        Timer::new(self, clocks).qei(pins, options)
    }
}

impl Timer<pac::TIM2> {
    pub fn qei(
        self,
        pins: impl Into<<pac::TIM2 as InputPins>::Channels12>,
        options: QeiOptions,
    ) -> Qei<pac::TIM2> {
        let Self { tim, clk: _ } = self;
        Qei::_tim2(tim, pins, options)
    }
}

impl QeiExt for pac::TIM3 {
    fn qei(
        self,
        pins: impl Into<<Self as InputPins>::Channels12>,
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self> {
        Timer::new(self, clocks).qei(pins, options)
    }
}

impl Timer<pac::TIM3> {
    pub fn qei(
        self,
        pins: impl Into<<pac::TIM3 as InputPins>::Channels12>,
        options: QeiOptions,
    ) -> Qei<pac::TIM3> {
        let Self { tim, clk: _ } = self;
        Qei::_tim3(tim, pins, options)
    }
}

#[cfg(feature = "medium")]
impl QeiExt for pac::TIM4 {
    fn qei(
        self,
        pins: impl Into<<Self as InputPins>::Channels12>,
        options: QeiOptions,
        clocks: &Clocks,
    ) -> Qei<Self> {
        Timer::new(self, &clocks).qei(pins, options)
    }
}

#[cfg(feature = "medium")]
impl Timer<pac::TIM4> {
    pub fn qei(
        self,
        pins: impl Into<<pac::TIM4 as InputPins>::Channels12>,
        options: QeiOptions,
    ) -> Qei<pac::TIM4> {
        let Self { tim, clk: _ } = self;
        Qei::_tim4(tim, pins, options)
    }
}

macro_rules! hal {
    ($TIMX:ty: $timX:ident, $timXen:ident, $timXrst:ident) => {
        impl Qei<$TIMX> {
            fn $timX(
                tim: $TIMX,
                pins: impl Into<<$TIMX as InputPins>::Channels12>,
                options: QeiOptions,
            ) -> Self {
                let pins = pins.into();

                // Configure TxC1 and TxC2 as captures
                tim.ccmr1_input().write(|w| w.cc1s().ti1().cc2s().ti2());

                // enable and configure to capture on rising edge
                tim.ccer().write(|w| {
                    w.cc1e().set_bit();
                    w.cc1p().clear_bit();
                    w.cc2e().set_bit();
                    w.cc2p().clear_bit()
                });

                // configure as quadrature encoder
                tim.smcr().write(|w| w.sms().set(options.slave_mode as u8));

                tim.arr().write(|w| w.arr().set(options.auto_reload_value));
                tim.cr1().write(|w| w.cen().set_bit());

                Qei { tim, pins }
            }

            pub fn release(self) -> ($TIMX, <$TIMX as InputPins>::Channels12) {
                (self.tim, self.pins)
            }
        }

        impl hal::Qei for Qei<$TIMX> {
            type Count = u16;

            fn count(&self) -> u16 {
                self.tim.cnt().read().cnt().bits()
            }

            fn direction(&self) -> Direction {
                if self.tim.cr1().read().dir().bit_is_clear() {
                    Direction::Upcounting
                } else {
                    Direction::Downcounting
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
hal!(pac::TIM1: _tim1, tim1en, tim1rst);

hal!(pac::TIM2: _tim2, tim2en, tim2rst);
hal!(pac::TIM3: _tim3, tim3en, tim3rst);

#[cfg(feature = "medium")]
hal!(pac::TIM4: _tim4, tim4en, tim4rst);
