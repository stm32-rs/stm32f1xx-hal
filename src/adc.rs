//! # API for the Analog to Digital converter

use core::marker::PhantomData;
use embedded_hal::adc::{Channel, OneShot};

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl",),))]
use crate::dma::dma2;
use crate::dma::{dma1, CircBuffer, Receive, RxDma, Transfer, TransferPayload, W};
#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl",),))]
use crate::gpio::gpiof;
use crate::gpio::Analog;
use crate::gpio::{gpioa, gpiob, gpioc};
use crate::rcc::{Clocks, Enable, Reset};
use core::sync::atomic::{self, Ordering};
use cortex_m::asm::delay;
use embedded_dma::StaticWriteBuffer;

#[cfg(feature = "stm32f103")]
use crate::pac::ADC2;
#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl")))]
use crate::pac::ADC3;
use crate::pac::{ADC1, RCC};

/// Continuous mode
pub struct Continuous;
/// Scan mode
pub struct Scan;

/// ADC configuration
pub struct Adc<ADC> {
    rb: ADC,
    sample_time: SampleTime,
    align: Align,
    clocks: Clocks,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
pub enum SampleTime {
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

impl Default for SampleTime {
    /// Get the default sample time (currently 28.5 cycles)
    fn default() -> Self {
        SampleTime::T_28
    }
}

impl From<SampleTime> for u8 {
    fn from(val: SampleTime) -> Self {
        use SampleTime::*;
        match val {
            T_1 => 0,
            T_7 => 1,
            T_13 => 2,
            T_28 => 3,
            T_41 => 4,
            T_55 => 5,
            T_71 => 6,
            T_239 => 7,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// ADC data register alignment
pub enum Align {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl Default for Align {
    /// Default: right alignment
    fn default() -> Self {
        Align::Right
    }
}

impl From<Align> for bool {
    fn from(val: Align) -> Self {
        match val {
            Align::Right => false,
            Align::Left => true,
        }
    }
}

macro_rules! adc_pins {
    ($ADC:ident, $($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

adc_pins!(ADC1,
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

#[cfg(any(feature = "stm32f103",))]
adc_pins!(ADC2,
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

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl",),))]
adc_pins!(ADC3,
    gpioa::PA0<Analog> => 0_u8,
    gpioa::PA1<Analog> => 1_u8,
    gpioa::PA2<Analog> => 2_u8,
    gpioa::PA3<Analog> => 3_u8,
    gpiof::PF6<Analog> => 4_u8,
    gpiof::PF7<Analog> => 5_u8,
    gpiof::PF8<Analog> => 6_u8,
    gpiof::PF9<Analog> => 7_u8,
    gpiof::PF10<Analog> => 8_u8,
    gpioc::PC0<Analog> => 10_u8,
    gpioc::PC1<Analog> => 11_u8,
    gpioc::PC2<Analog> => 12_u8,
    gpioc::PC3<Analog> => 13_u8,
);

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct StoredConfig(SampleTime, Align);

macro_rules! adc_hal {
    ($(
        $ADC:ident: ($adc:ident),
    )+) => {
        $(

            impl Adc<$ADC> {
                /// Init a new Adc
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn $adc(adc: $ADC, clocks: Clocks) -> Self {
                    let mut s = Self {
                        rb: adc,
                        sample_time: SampleTime::default(),
                        align: Align::default(),
                        clocks,
                    };
                    s.enable_clock();
                    s.power_down();
                    s.reset();
                    s.setup_oneshot();
                    s.power_up();

                    // The manual states that we need to wait two ADC clocks cycles after power-up
                    // before starting calibration, we already delayed in the power-up process, but
                    // if the adc clock is too low that was not enough.
                    if s.clocks.adcclk().0 < 2_500_000 {
                        let two_adc_cycles = s.clocks.sysclk().0 / s.clocks.adcclk().0 *2;
                        let already_delayed = s.clocks.sysclk().0 / 800_000;
                        if two_adc_cycles > already_delayed {
                            delay(two_adc_cycles - already_delayed);
                        }
                    }
                    s.calibrate();
                    s
                }

                /// Save current ADC config
                pub fn save_cfg(&mut self) -> StoredConfig {
                    StoredConfig(self.sample_time, self.align)
                }

                /// Restore saved ADC config
                pub fn restore_cfg(&mut self, cfg: StoredConfig) {
                    self.sample_time = cfg.0;
                    self.align = cfg.1;
                }

                /// Reset the ADC config to default, return existing config
                pub fn default_cfg(&mut self) -> StoredConfig {
                    let cfg = self.save_cfg();
                    self.sample_time = SampleTime::default();
                    self.align = Align::default();
                    cfg
                }

                /// Set ADC sampling time
                ///
                /// Options can be found in [SampleTime](crate::adc::SampleTime).
                pub fn set_sample_time(&mut self, t_samp: SampleTime) {
                    self.sample_time = t_samp;
                }

                /// Set the Adc result alignment
                ///
                /// Options can be found in [Align](crate::adc::Align).
                pub fn set_align(&mut self, align: Align) {
                    self.align = align;
                }

                /// Returns the largest possible sample value for the current settings
                pub fn max_sample(&self) -> u16 {
                    match self.align {
                        Align::Left => u16::max_value(),
                        Align::Right => (1 << 12) - 1,
                    }
                }

                #[inline(always)]
                pub fn set_external_trigger(&mut self, trigger: crate::pac::$adc::cr2::EXTSEL_A) {
                    self.rb.cr2.modify(|_, w| w.extsel().variant(trigger))
                }

                fn power_up(&mut self) {
                    self.rb.cr2.modify(|_, w| w.adon().set_bit());

                    // The reference manual says that a stabilization time is needed after power_up,
                    // this time can be found in the datasheets.
                    // Here we are delaying for approximately 1us, considering 1.25 instructions per
                    // cycle. Do we support a chip which needs more than 1us ?
                    delay(self.clocks.sysclk().0 / 800_000);
                }

                fn power_down(&mut self) {
                    self.rb.cr2.modify(|_, w| w.adon().clear_bit());
                }

                fn reset(&mut self) {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $ADC::reset(rcc);
                }

                fn enable_clock(&mut self) {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $ADC::enable(rcc);
                }

                fn disable_clock(&mut self) {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $ADC::disable(rcc);
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
                    self.rb.cr2.modify(|_, w| w
                        .cont().clear_bit()
                        .exttrig().set_bit()
                        .extsel().bits(0b111)
                    );

                    self.rb.cr1.modify(|_, w| w
                        .scan().clear_bit()
                        .discen().set_bit()
                    );

                    self.rb.sqr1.modify(|_, w| w.l().bits(0b0));
                }

                fn set_channel_sample_time(&mut self, chan: u8, sample_time: SampleTime) {
                    let sample_time = sample_time.into();
                    match chan {
                        0 => self.rb.smpr2.modify(|_, w| w.smp0().bits(sample_time)),
                        1 => self.rb.smpr2.modify(|_, w| w.smp1().bits(sample_time)),
                        2 => self.rb.smpr2.modify(|_, w| w.smp2().bits(sample_time)),
                        3 => self.rb.smpr2.modify(|_, w| w.smp3().bits(sample_time)),
                        4 => self.rb.smpr2.modify(|_, w| w.smp4().bits(sample_time)),
                        5 => self.rb.smpr2.modify(|_, w| w.smp5().bits(sample_time)),
                        6 => self.rb.smpr2.modify(|_, w| w.smp6().bits(sample_time)),
                        7 => self.rb.smpr2.modify(|_, w| w.smp7().bits(sample_time)),
                        8 => self.rb.smpr2.modify(|_, w| w.smp8().bits(sample_time)),
                        9 => self.rb.smpr2.modify(|_, w| w.smp9().bits(sample_time)),

                        10 => self.rb.smpr1.modify(|_, w| w.smp10().bits(sample_time)),
                        11 => self.rb.smpr1.modify(|_, w| w.smp11().bits(sample_time)),
                        12 => self.rb.smpr1.modify(|_, w| w.smp12().bits(sample_time)),
                        13 => self.rb.smpr1.modify(|_, w| w.smp13().bits(sample_time)),
                        14 => self.rb.smpr1.modify(|_, w| w.smp14().bits(sample_time)),
                        15 => self.rb.smpr1.modify(|_, w| w.smp15().bits(sample_time)),
                        16 => self.rb.smpr1.modify(|_, w| w.smp16().bits(sample_time)),
                        17 => self.rb.smpr1.modify(|_, w| w.smp17().bits(sample_time)),
                        _ => unreachable!(),
                    }
                }

                fn set_regular_sequence (&mut self, channels: &[u8]) {
                    let len = channels.len();
                    let bits = channels.iter().take(6).enumerate().fold(0u32, |s, (i, c)|
                        s | ((*c as u32) << (i * 5))
                    );
                    self.rb.sqr3.write(|w| unsafe { w
                        .bits( bits )
                    });
                    if len > 6 {
                        let bits = channels.iter().skip(6).take(6).enumerate().fold(0u32, |s, (i, c)|
                            s | ((*c as u32) << (i * 5))
                        );
                        self.rb.sqr2.write(|w| unsafe { w
                            .bits( bits )
                        });
                    }
                    if len > 12 {
                        let bits = channels.iter().skip(12).take(4).enumerate().fold(0u32, |s, (i, c)|
                            s | ((*c as u32) << (i * 5))
                        );
                        self.rb.sqr1.write(|w| unsafe { w
                            .bits( bits )
                        });
                    }
                    self.rb.sqr1.modify(|_, w| w.l().bits((len-1) as u8));
                }

                fn set_continuous_mode(&mut self, continuous: bool) {
                    self.rb.cr2.modify(|_, w| w.cont().bit(continuous));
                }

                fn set_discontinuous_mode(&mut self, channels_count: Option<u8>) {
                    self.rb.cr1.modify(|_, w| match channels_count {
                        Some(count) => w.discen().set_bit().discnum().bits(count),
                        None => w.discen().clear_bit(),
                    });
                }

                /**
                  Performs an ADC conversion

                  NOTE: Conversions can be started by writing a 1 to the ADON
                  bit in the `CR2` while it is already 1, and no other bits
                  are being written in the same operation. This means that
                  the EOC bit *might* be set already when entering this function
                  which can cause a read of stale values

                  The check for `cr2.swstart.bit_is_set` *should* fix it, but
                  does not. Therefore, ensure you do not do any no-op modifications
                  to `cr2` just before calling this function
                */
                fn convert(&mut self, chan: u8) -> u16 {
                    // Dummy read in case something accidentally triggered
                    // a conversion by writing to CR2 without changing any
                    // of the bits
                    self.rb.dr.read().data().bits();

                    self.set_channel_sample_time(chan, self.sample_time);
                    self.rb.sqr3.modify(|_, w| unsafe { w.sq1().bits(chan) });

                    // ADC start conversion of regular sequence
                    self.rb.cr2.modify(|_, w|
                        w
                            .swstart().set_bit()
                            .align().bit(self.align.into())
                    );
                    while self.rb.cr2.read().swstart().bit_is_set() {}
                    // ADC wait for conversion results
                    while self.rb.sr.read().eoc().bit_is_clear() {}

                    let res = self.rb.dr.read().data().bits();
                    res
                }

                /// Powers down the ADC, disables the ADC clock and releases the ADC Peripheral
                pub fn release(mut self) -> $ADC {
                    self.power_down();
                    self.disable_clock();
                    self.rb
                }
            }

            impl ChannelTimeSequence for Adc<$ADC> {
                #[inline(always)]
                fn set_channel_sample_time(&mut self, chan: u8, sample_time: SampleTime) {
                    self.set_channel_sample_time(chan, sample_time);
                }
                #[inline(always)]
                fn set_regular_sequence (&mut self, channels: &[u8]) {
                    self.set_regular_sequence(channels);
                }
                #[inline(always)]
                fn set_continuous_mode(&mut self, continuous: bool) {
                    self.set_continuous_mode(continuous);
                }
                #[inline(always)]
                fn set_discontinuous_mode(&mut self, channels: Option<u8>) {
                    self.set_discontinuous_mode(channels);
                }
            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC>
            where
                WORD: From<u16>,
                PIN: Channel<$ADC, ID = u8>,
                {
                    type Error = ();

                    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                        let res = self.convert(PIN::channel());
                        Ok(res.into())
                    }
                }

        )+
    }
}

impl Adc<ADC1> {
    fn read_aux(&mut self, chan: u8) -> u16 {
        let tsv_off = if self.rb.cr2.read().tsvrefe().bit_is_clear() {
            self.rb.cr2.modify(|_, w| w.tsvrefe().set_bit());

            // The reference manual says that a stabilization time is needed after the powering the
            // sensor, this time can be found in the datasheets.
            // Here we are delaying for approximately 10us, considering 1.25 instructions per
            // cycle. Do we support a chip which needs more than 10us ?
            delay(self.clocks.sysclk().0 / 80_000);
            true
        } else {
            false
        };

        let val = self.convert(chan);

        if tsv_off {
            self.rb.cr2.modify(|_, w| w.tsvrefe().clear_bit());
        }

        val
    }

    /// Temperature sensor is connected to channel 16 on ADC1. This sensor can be used
    /// to measure ambient temperature of the device. However note that the returned
    /// value is not an absolute temperature value.
    ///
    /// In particular, according to section 11.10 from Reference Manual RM0008 Rev 20:
    /// "The temperature sensor output voltage changes linearly with temperature. The offset
    /// of this line varies from chip to chip due to process variation (up to 45 °C from one
    /// chip to another). The internal temperature sensor is more suited to applications
    /// that detect temperature variations instead of absolute temperatures. If accurate
    /// temperature readings are needed, an external temperature sensor part should be used."
    ///
    /// Formula to calculate temperature value is also taken from the section 11.10.
    pub fn read_temp(&mut self) -> i32 {
        /// According to section 5.3.18 "Temperature sensor characteristics"
        /// from STM32F1xx datasheets, TS constants values are as follows:
        ///   AVG_SLOPE - average slope
        ///   V_25 - temperature sensor ADC voltage at 25°C
        const AVG_SLOPE: i32 = 43;
        const V_25: i32 = 1430;

        let prev_cfg = self.save_cfg();

        // recommended ADC sampling for temperature sensor is 17.1 usec,
        // so use the following approximate settings
        // to support all ADC frequencies
        let sample_time = match self.clocks.adcclk().0 {
            0..=1_200_000 => SampleTime::T_1,
            1_200_001..=1_500_000 => SampleTime::T_7,
            1_500_001..=2_400_000 => SampleTime::T_13,
            2_400_001..=3_100_000 => SampleTime::T_28,
            3_100_001..=4_000_000 => SampleTime::T_41,
            4_000_001..=5_000_000 => SampleTime::T_55,
            5_000_001..=14_000_000 => SampleTime::T_71,
            _ => SampleTime::T_239,
        };

        self.set_sample_time(sample_time);
        let val_temp: i32 = self.read_aux(16u8).into();
        let val_vref: i32 = self.read_aux(17u8).into();
        let v_sense = val_temp * 1200 / val_vref;

        self.restore_cfg(prev_cfg);

        (V_25 - v_sense) * 10 / AVG_SLOPE + 25
    }

    /// Internal reference voltage Vrefint is connected to channel 17 on ADC1.
    /// According to section 5.3.4 "Embedded reference voltage" from STM32F1xx
    /// datasheets, typical value of this reference voltage is 1200 mV.
    ///
    /// This value is useful when ADC readings need to be converted into voltages.
    /// For instance, reading from any ADC channel can be converted into voltage (mV)
    /// using the following formula:
    ///     v_chan = adc.read(chan) * 1200 / adc.read_vref()
    pub fn read_vref(&mut self) -> u16 {
        self.read_aux(17u8)
    }
}

adc_hal! {
    ADC1: (adc1),
}

#[cfg(feature = "stm32f103")]
adc_hal! {
    ADC2: (adc2),
}

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl",),))]
adc_hal! {
    ADC3: (adc3),
}

pub struct AdcPayload<ADC, PINS, MODE> {
    adc: Adc<ADC>,
    pins: PINS,
    _mode: PhantomData<MODE>,
}

pub trait ChannelTimeSequence {
    /// Set ADC sampling time for particular channel
    fn set_channel_sample_time(&mut self, chan: u8, sample_time: SampleTime);
    /// ADC Set a Regular Channel Conversion Sequence
    ///
    /// Define a sequence of channels to be converted as a regular group.
    fn set_regular_sequence(&mut self, channels: &[u8]);
    /// Set ADC continuous conversion
    ///
    /// When continuous conversion is enabled conversion does not stop at the last selected group channel but continues again from the first selected group channel.
    fn set_continuous_mode(&mut self, continuous: bool);
    /// Set ADC discontinuous mode
    ///
    /// It can be used to convert a short sequence of conversions (up to 8) which is a part of the regular sequence of conversions.
    fn set_discontinuous_mode(&mut self, channels_count: Option<u8>);
}

/// Set channel sequence and sample times for custom pins
///
/// Example:
/// ```rust, ignore
/// pub struct AdcPins(PA0<Analog>, PA2<Analog>);
/// impl SetChannels<AdcPins> for Adc<ADC1> {
///     fn set_samples(&mut self) {
///         self.set_channel_sample_time(0, adc::SampleTime::T_28);
///         self.set_channel_sample_time(2, adc::SampleTime::T_28);
///     }
///     fn set_sequence(&mut self) {
///         self.set_regular_sequence(&[0, 2, 0, 2]);
///         // Optionally we can set continuous scan mode
///         self.set_continuous_mode(true);
///         // Also we can use discontinuous conversion (3 channels per conversion)
///         self.set_discontinuous_mode(Some(3));
///     }
/// }
/// ```
pub trait SetChannels<PINS>: ChannelTimeSequence {
    fn set_samples(&mut self);
    fn set_sequence(&mut self);
}

pub type AdcDma<ADC, PINS, MODE, CHANNEL> = RxDma<AdcPayload<ADC, PINS, MODE>, CHANNEL>;

macro_rules! adcdma {
    ($ADCX:ident: (
        $rxdma:ident,
        $dmarxch:ty,
    )) => {
        pub type $rxdma<PINS, MODE> = AdcDma<$ADCX, PINS, MODE, $dmarxch>;

        impl<PINS, MODE> Receive for AdcDma<$ADCX, PINS, MODE, $dmarxch> {
            type RxChannel = $dmarxch;
            type TransmittedWord = u16;
        }

        impl<PINS> TransferPayload for AdcDma<$ADCX, PINS, Continuous, $dmarxch> {
            fn start(&mut self) {
                self.channel.start();
                self.payload.adc.rb.cr2.modify(|_, w| w.cont().set_bit());
                self.payload.adc.rb.cr2.modify(|_, w| w.adon().set_bit());
            }
            fn stop(&mut self) {
                self.channel.stop();
                self.payload.adc.rb.cr2.modify(|_, w| w.cont().clear_bit());
            }
        }

        impl<PINS> TransferPayload for AdcDma<$ADCX, PINS, Scan, $dmarxch> {
            fn start(&mut self) {
                self.channel.start();
                self.payload.adc.rb.cr2.modify(|_, w| w.adon().set_bit());
            }
            fn stop(&mut self) {
                self.channel.stop();
            }
        }

        impl Adc<$ADCX> {
            pub fn with_dma<PIN>(
                mut self,
                pins: PIN,
                dma_ch: $dmarxch,
            ) -> AdcDma<$ADCX, PIN, Continuous, $dmarxch>
            where
                PIN: Channel<$ADCX, ID = u8>,
            {
                self.rb.cr1.modify(|_, w| w.discen().clear_bit());
                self.rb.cr2.modify(|_, w| w.align().bit(self.align.into()));
                self.set_channel_sample_time(PIN::channel(), self.sample_time);
                self.rb
                    .sqr3
                    .modify(|_, w| unsafe { w.sq1().bits(PIN::channel()) });
                self.rb.cr2.modify(|_, w| w.dma().set_bit());

                let payload = AdcPayload {
                    adc: self,
                    pins,
                    _mode: PhantomData,
                };
                RxDma {
                    payload,
                    channel: dma_ch,
                }
            }

            pub fn with_scan_dma<PINS>(
                mut self,
                pins: PINS,
                dma_ch: $dmarxch,
            ) -> AdcDma<$ADCX, PINS, Scan, $dmarxch>
            where
                Self: SetChannels<PINS>,
            {
                self.rb.cr2.modify(|_, w| {
                    w.adon()
                        .clear_bit()
                        .dma()
                        .clear_bit()
                        .cont()
                        .clear_bit()
                        .align()
                        .bit(self.align.into())
                });
                self.rb
                    .cr1
                    .modify(|_, w| w.scan().set_bit().discen().clear_bit());
                self.set_samples();
                self.set_sequence();
                self.rb
                    .cr2
                    .modify(|_, w| w.dma().set_bit().adon().set_bit());

                let payload = AdcPayload {
                    adc: self,
                    pins,
                    _mode: PhantomData,
                };
                RxDma {
                    payload,
                    channel: dma_ch,
                }
            }
        }

        impl<PINS> AdcDma<$ADCX, PINS, Continuous, $dmarxch>
        where
            Self: TransferPayload,
        {
            pub fn split(mut self) -> (Adc<$ADCX>, PINS, $dmarxch) {
                self.stop();

                let AdcDma { payload, channel } = self;
                payload.adc.rb.cr2.modify(|_, w| w.dma().clear_bit());
                payload.adc.rb.cr1.modify(|_, w| w.discen().set_bit());

                (payload.adc, payload.pins, channel)
            }
        }

        impl<PINS> AdcDma<$ADCX, PINS, Scan, $dmarxch>
        where
            Self: TransferPayload,
        {
            pub fn split(mut self) -> (Adc<$ADCX>, PINS, $dmarxch) {
                self.stop();

                let AdcDma { payload, channel } = self;
                payload.adc.rb.cr2.modify(|_, w| w.dma().clear_bit());
                payload.adc.rb.cr1.modify(|_, w| w.discen().set_bit());
                payload.adc.rb.cr1.modify(|_, w| w.scan().clear_bit());

                (payload.adc, payload.pins, channel)
            }
        }

        impl<B, PINS, MODE> crate::dma::CircReadDma<B, u16> for AdcDma<$ADCX, PINS, MODE, $dmarxch>
        where
            Self: TransferPayload,
            &'static mut [B; 2]: StaticWriteBuffer<Word = u16>,
            B: 'static,
        {
            fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*$ADCX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);

                self.channel.ch().cr.modify(|_, w| {
                    w.mem2mem()
                        .clear_bit()
                        .pl()
                        .medium()
                        .msize()
                        .bits16()
                        .psize()
                        .bits16()
                        .circ()
                        .set_bit()
                        .dir()
                        .clear_bit()
                });

                self.start();

                CircBuffer::new(buffer, self)
            }
        }

        impl<B, PINS, MODE> crate::dma::ReadDma<B, u16> for AdcDma<$ADCX, PINS, MODE, $dmarxch>
        where
            Self: TransferPayload,
            B: StaticWriteBuffer<Word = u16>,
        {
            fn read(mut self, mut buffer: B) -> Transfer<W, B, Self> {
                // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                // until the end of the transfer.
                let (ptr, len) = unsafe { buffer.static_write_buffer() };
                self.channel.set_peripheral_address(
                    unsafe { &(*$ADCX::ptr()).dr as *const _ as u32 },
                    false,
                );
                self.channel.set_memory_address(ptr as u32, true);
                self.channel.set_transfer_length(len);

                atomic::compiler_fence(Ordering::Release);
                self.channel.ch().cr.modify(|_, w| {
                    w.mem2mem()
                        .clear_bit()
                        .pl()
                        .medium()
                        .msize()
                        .bits16()
                        .psize()
                        .bits16()
                        .circ()
                        .clear_bit()
                        .dir()
                        .clear_bit()
                });
                self.start();

                Transfer::w(buffer, self)
            }
        }
    };
}

adcdma! {
    ADC1: (
        AdcDma1,
        dma1::C1,
    )
}

#[cfg(all(feature = "stm32f103", any(feature = "high", feature = "xl",),))]
adcdma! {
    ADC3: (
        AdcDma3,
        dma2::C5,
    )
}
