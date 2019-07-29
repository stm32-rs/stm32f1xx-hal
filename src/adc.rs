//! # API for the Analog to Digital converter

use embedded_hal::adc::{Channel, OneShot};

use crate::gpio::Analog;
use crate::gpio::{gpioa, gpiob, gpioc};
use crate::rcc::{APB2, Clocks};
use crate::dma::{Receive, TransferPayload, dma1::C1, CircBuffer, Transfer, W, RxDma};
use core::sync::atomic::{self, Ordering};
use cortex_m::asm::delay;

use crate::stm32::ADC1;
#[cfg(any(
    feature = "stm32f103",
))]
use crate::stm32::ADC2;

/// ADC configuration
pub struct Adc<ADC> {
    rb: ADC,
    sample_time: AdcSampleTime,
    align: AdcAlign,
    clocks: Clocks
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
    /// Get the default sample time (currently 28.5 cycles)
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
/// ADC data register alignment
pub enum AdcAlign {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl AdcAlign {
    /// Default: right alignment
    pub fn default() -> Self {
        AdcAlign::Right
    }
}

impl From<AdcAlign> for u8 {
    fn from(val: AdcAlign) -> Self {
        match val {
            AdcAlign::Right => 0,
            AdcAlign::Left => 1,
        }
    }
}

impl From<AdcAlign> for bool {
    fn from(val: AdcAlign) -> Self {
        match val {
            AdcAlign::Right => false,
            AdcAlign::Left => true,
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

#[cfg(any(
    feature = "stm32f103",
))]
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

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, AdcAlign);

macro_rules! adc_hal {
    ($(
        $ADC:ident: (
            $init:ident,
            $adcxen:ident,
            $adcxrst:ident
        ),
    )+) => {
        $(

            impl Adc<$ADC> {
                /// Init a new Adc
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn $init(adc: $ADC, apb2: &mut APB2, clocks: Clocks) -> Self {
                    let mut s = Self {
                        rb: adc,
                        sample_time: AdcSampleTime::default(),
                        align: AdcAlign::default(),
                        clocks,
                    };
                    s.enable_clock(apb2);
                    s.power_down();
                    s.reset(apb2);
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
                    self.sample_time = AdcSampleTime::default();
                    self.align = AdcAlign::default();
                    cfg
                }

                /// Set ADC sampling time
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

                    // The reference manual says that a stabilization time is needed after power_up,
                    // this time can be found in the datasheets.
                    // Here we are delaying for approximately 1us, considering 1.25 instructions per
                    // cycle. Do we support a chip which needs more than 1us ?
                    delay(self.clocks.sysclk().0 / 800_000);
                }

                fn power_down(&mut self) {
                    self.rb.cr2.modify(|_, w| w.adon().clear_bit());
                }

                fn reset(&mut self, apb2: &mut APB2) {
                    apb2.rstr().modify(|_, w| w.$adcxrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$adcxrst().clear_bit());
                }

                fn enable_clock(&mut self, apb2: &mut APB2) {
                    apb2.enr().modify(|_, w| w.$adcxen().set_bit());
                }

                fn disable_clock(&mut self, apb2: &mut APB2) {
                    apb2.enr().modify(|_, w| w.$adcxen().clear_bit());
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
                        16 => self
                            .rb
                            .smpr1
                            .modify(|_, w| unsafe { w.smp16().bits(self.sample_time.into()) }),
                        17 => self
                            .rb
                            .smpr1
                            .modify(|_, w| unsafe { w.smp17().bits(self.sample_time.into()) }),
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

                /// Powers down the ADC, disables the ADC clock and releases the ADC Peripheral
                pub fn release(mut self, apb2: &mut APB2) -> $ADC {
                    self.power_down();
                    self.disable_clock(apb2);
                    self.rb
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
            0 ..= 1_200_000 => AdcSampleTime::T_1,
            1_200_001 ..= 1_500_000 => AdcSampleTime::T_7,
            1_500_001 ..= 2_400_000 => AdcSampleTime::T_13,
            2_400_001 ..= 3_100_000 => AdcSampleTime::T_28,
            3_100_001 ..= 4_000_000 => AdcSampleTime::T_41,
            4_000_001 ..= 5_000_000 => AdcSampleTime::T_55,
            5_000_001 ..= 14_000_000 => AdcSampleTime::T_71,
            _ => AdcSampleTime::T_239,
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

#[cfg(any(
    feature = "stm32f100",
    feature = "stm32f101",
))]
adc_hal! {
    ADC1: (
        adc1,
        adc1en,
        adc1rst
    ),
}

#[cfg(any(
    feature = "stm32f103",
))]
adc_hal! {
    ADC1: (
        adc1,
        adc1en,
        adc1rst
    ),
    ADC2: (
        adc2,
        adc2en,
        adc2rst
    ),
}

pub struct AdcPayload<PIN: Channel<ADC1>> {
    adc: Adc<ADC1>,
    pin: PIN,
}

pub type AdcDma<PIN> = RxDma<AdcPayload<PIN>, C1>;

impl<PIN> Receive for AdcDma<PIN> where PIN: Channel<ADC1> {
    type RxChannel = C1;
    type TransmittedWord = u16;
}

impl<PIN> TransferPayload for AdcDma<PIN> where PIN: Channel<ADC1> {
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

impl Adc<ADC1> {
    pub fn with_dma<PIN>(mut self, pin: PIN, dma_ch: C1) -> AdcDma<PIN>
    where
        PIN: Channel<ADC1, ID = u8>,
    {
        self.rb.cr1.modify(|_, w| w.discen().clear_bit());
        self.rb.cr2.modify(|_, w| w.align().bit(self.align.into()));
        self.set_chan_smps(PIN::channel());
        self.rb.sqr3.modify(|_, w| unsafe { w.sq1().bits(PIN::channel()) });
        self.rb.cr2.modify(|_, w| w.dma().set_bit());

        let payload = AdcPayload {
            adc: self,
            pin,
        };
        RxDma {
            payload,
            channel: dma_ch,
        }
    }
}

impl<PIN> AdcDma<PIN> where PIN: Channel<ADC1> {
    pub fn split(mut self) -> (Adc<ADC1>, PIN, C1) {
        self.stop();

        let AdcDma {payload, channel} = self;
        payload.adc.rb.cr2.modify(|_, w| w.dma().clear_bit());
        payload.adc.rb.cr1.modify(|_, w| w.discen().set_bit());

        (payload.adc, payload.pin, channel)
    }
}

impl<B, PIN> crate::dma::CircReadDma<B, u16> for AdcDma<PIN>
where
    B: AsMut<[u16]>,
    PIN: Channel<ADC1>,
{
    fn circ_read(mut self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
        {
            let buffer = buffer[0].as_mut();
            self.channel.set_peripheral_address(unsafe{ &(*ADC1::ptr()).dr as *const _ as u32 }, false);
            self.channel.set_memory_address(buffer.as_ptr() as u32, true);
            self.channel.set_transfer_length(buffer.len() * 2);

            atomic::compiler_fence(Ordering::Release);

            self.channel.ch().cr.modify(|_, w| { w
                .mem2mem() .clear_bit()
                .pl()      .medium()
                .msize()   .bits16()
                .psize()   .bits16()
                .circ()    .set_bit()
                .dir()     .clear_bit()
            });
        }

        self.start();

        CircBuffer::new(buffer, self)
    }
}

impl<B, PIN> crate::dma::ReadDma<B, u16> for AdcDma<PIN>
where
    B: AsMut<[u16]>,
    PIN: Channel<ADC1>,
{
    fn read(mut self, buffer: &'static mut B) -> Transfer<W, &'static mut B, Self> {
        {
            let buffer = buffer.as_mut();
            self.channel.set_peripheral_address(unsafe{ &(*ADC1::ptr()).dr as *const _ as u32 }, false);
            self.channel.set_memory_address(buffer.as_ptr() as u32, true);
            self.channel.set_transfer_length(buffer.len());
        }
        atomic::compiler_fence(Ordering::Release);
        self.channel.ch().cr.modify(|_, w| { w
            .mem2mem() .clear_bit()
            .pl()      .medium()
            .msize()   .bits16()
            .psize()   .bits16()
            .circ()    .clear_bit()
            .dir()     .clear_bit()
        });
        self.start();

        Transfer::w(buffer, self)
    }
}
