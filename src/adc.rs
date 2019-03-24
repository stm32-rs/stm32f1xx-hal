//! # API for the Analog to Digital converter

use embedded_hal::adc::{Channel, OneShot};

use crate::gpio::Analog;
use crate::gpio::{gpioa, gpiob, gpioc};

use crate::rcc::APB2;

use crate::stm32::ADC1;

/// Analog to Digital converter interface
pub struct Adc {
    rb: ADC1,
    sample_time: AdcSampleTime,
    align: AdcAlign,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
pub enum AdcSampleTime {
    /// 1.5 cycles sampling time
    T_1,
    /// 7.5 cycles sampling time
    T_7,
    /// 13.5 cycles sampling time
    T_13,
    /// 28.5 cycles sampling time
    T_28,
    /// 41.5 cycles sampling time
    T_41,
    /// 55.5 cycles sampling time
    T_55,
    /// 71.5 cycles sampling time
    T_71,
    /// 239.5 cycles sampling time
    T_239,
}

impl AdcSampleTime {
    /// Get the default sample time (currently 239.5 cycles)
    pub fn default() -> Self {
        AdcSampleTime::T_28
    }
}

impl From<AdcSampleTime> for u8 {
    fn from(val: AdcSampleTime) -> Self {
        match val {
            AdcSampleTime::T_1 => 0,
            AdcSampleTime::T_7 => 1,
            AdcSampleTime::T_13 => 2,
            AdcSampleTime::T_28 => 3,
            AdcSampleTime::T_41 => 4,
            AdcSampleTime::T_55 => 5,
            AdcSampleTime::T_71 => 6,
            AdcSampleTime::T_239 => 7,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// ADC Result Alignment
pub enum AdcAlign {
    /// Left aligned results (most significant bits)
    Left,
    /// Right aligned results (least significant bits)
    Right,
}

impl AdcAlign {
    /// Get the default alignment (currently right aligned)
    pub fn default() -> Self {
        AdcAlign::Right
    }
}

impl From<AdcAlign> for u8 {
    fn from(val: AdcAlign) -> Self {
        match val {
            AdcAlign::Left => 1,
            AdcAlign::Right => 0,
        }
    }
}

impl From<AdcAlign> for bool {
    fn from(val: AdcAlign) -> Self {
        match val {
            AdcAlign::Left => true,
            AdcAlign::Right => false,
        }
    }
}

macro_rules! adc_pins {
    ($($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<Adc> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

adc_pins!(
    gpioa::PA0<Analog> => 0_u8,
    gpioa::PA1<Analog> => 1_u8,
    gpioa::PA2<Analog> => 2_u8,
    gpioa::PA3<Analog> => 3_u8,
    gpioa::PA4<Analog> => 4_u8,
    gpioa::PA5<Analog> => 5_u8,
    gpioa::PA6<Analog> => 6_u8,
    gpioa::PA7<Analog> => 7_u8,
    gpiob::PB0<Analog> => 8_u8,
    gpiob::PB1<Analog> => 9_u8,
    gpioc::PC0<Analog> => 10_u8,
    gpioc::PC1<Analog> => 11_u8,
    gpioc::PC2<Analog> => 12_u8,
    gpioc::PC3<Analog> => 13_u8,
    gpioc::PC4<Analog> => 14_u8,
    gpioc::PC5<Analog> => 15_u8,
);

/// A stored ADC config, can be restored by using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, AdcAlign);

impl Adc {
    /// Init a new Adc
    ///
    /// Sets all configurable parameters to defaults and performs a boot time
    /// calibration. As such this method may take an appreciable time to run.
    pub fn new(adc: ADC1, apb2: &mut APB2) -> Self {
        let mut s = Self {
            rb: adc,
            sample_time: AdcSampleTime::default(),
            align: AdcAlign::default(),
        };
        s.enable_clock(apb2);
        s.power_down();
        s.reset(apb2);
        s.setup_oneshot();
        s.power_up();
        s.calibrate();
        s
    }

    /// Saves a copy of the current ADC config
    pub fn save_cfg(&mut self) -> StoredConfig {
        StoredConfig(self.sample_time, self.align)
    }

    /// Restores a stored config
    pub fn restore_cfg(&mut self, cfg: StoredConfig) {
        self.sample_time = cfg.0;
        self.align = cfg.1;
    }

    /// Resets the ADC config to default, returning the existing config as
    /// a stored config.
    pub fn default_cfg(&mut self) -> StoredConfig {
        let cfg = self.save_cfg();
        self.sample_time = AdcSampleTime::default();
        self.align = AdcAlign::default();
        cfg
    }

    /// Set the Adc sampling time
    ///
    /// Options can be found in [AdcSampleTime](crate::adc::AdcSampleTime).
    pub fn set_sample_time(&mut self, t_samp: AdcSampleTime) {
        self.sample_time = t_samp;
    }

    /// Set the Adc result alignment
    ///
    /// Options can be found in [AdcAlign](crate::adc::AdcAlign).
    pub fn set_align(&mut self, align: AdcAlign) {
        self.align = align;
    }

    /// Returns the largest possible sample value for the current settings
    pub fn max_sample(&self) -> u16 {
        match self.align {
            AdcAlign::Left => u16::max_value(),
            AdcAlign::Right => (1 << 12) - 1,
        }
    }

    fn power_up(&mut self) {
        self.rb.cr2.modify(|_, w| w.adon().set_bit());
    }

    fn power_down(&mut self) {
        self.rb.cr2.modify(|_, w| w.adon().clear_bit());
    }

    fn reset(&mut self, apb2: &mut APB2) {
        apb2.rstr().modify(|_, w| w.adc1rst().set_bit());
        apb2.rstr().modify(|_, w| w.adc1rst().clear_bit());
    }

    fn enable_clock(&mut self, apb2: &mut APB2) {
        apb2.enr().modify(|_, w| w.adc1en().set_bit());
    }

    fn calibrate(&mut self) {
        /* reset calibration */
        self.rb.cr2.modify(|_, w| w.rstcal().set_bit());
        while self.rb.cr2.read().rstcal().bit_is_set() {}

        /* calibrate */
        self.rb.cr2.modify(|_, w| w.cal().set_bit());
        while self.rb.cr2.read().cal().bit_is_set() {}
    }

    fn setup_oneshot(&mut self) {
        self.rb.cr2.modify(|_, w| w.cont().clear_bit());
        self.rb.cr2.modify(|_, w| w.exttrig().set_bit());
        self.rb.cr2.modify(|_, w| unsafe { w.extsel().bits(0b111) });

        self.rb.cr1.modify(|_, w| w.scan().clear_bit());
        self.rb.cr1.modify(|_, w| w.discen().set_bit());

        self.rb.sqr1.modify(|_, w| unsafe { w.l().bits(0b0) });
    }

    fn set_chan_smps(&mut self, chan: u8) {
        match chan {
            0 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp0().bits(self.sample_time.into()) }),
            1 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp1().bits(self.sample_time.into()) }),
            2 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp2().bits(self.sample_time.into()) }),
            3 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp3().bits(self.sample_time.into()) }),
            4 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp4().bits(self.sample_time.into()) }),
            5 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp5().bits(self.sample_time.into()) }),
            6 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp6().bits(self.sample_time.into()) }),
            7 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp7().bits(self.sample_time.into()) }),
            8 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp8().bits(self.sample_time.into()) }),
            9 => self
                .rb
                .smpr2
                .modify(|_, w| unsafe { w.smp9().bits(self.sample_time.into()) }),

            10 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp10().bits(self.sample_time.into()) }),
            11 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp11().bits(self.sample_time.into()) }),
            12 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp12().bits(self.sample_time.into()) }),
            13 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp13().bits(self.sample_time.into()) }),
            14 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp14().bits(self.sample_time.into()) }),
            15 => self
                .rb
                .smpr1
                .modify(|_, w| unsafe { w.smp15().bits(self.sample_time.into()) }),

            _ => unreachable!(),
        }

        return;
    }

    fn convert(&mut self, chan: u8) -> u16 {
        self.rb.cr2.modify(|_, w| w.align().bit(self.align.into()));
        self.set_chan_smps(chan);
        self.rb.sqr3.modify(|_, w| unsafe { w.sq1().bits(chan) });

        // ADC start conversion of regular sequence
        self.rb.cr2.modify(|_, w| w.swstart().set_bit());
        while self.rb.cr2.read().swstart().bit_is_set() {}
        // ADC wait for conversion results
        while self.rb.sr.read().eoc().bit_is_clear() {}

        let res = self.rb.dr.read().data().bits();
        res
    }
}

impl<WORD, PIN> OneShot<Adc, WORD, PIN> for Adc
where
    WORD: From<u16>,
    PIN: Channel<Adc, ID = u8>,
{
    type Error = ();

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        self.power_up();
        let res = self.convert(PIN::channel());
        self.power_down();
        Ok(res.into())
    }
}
