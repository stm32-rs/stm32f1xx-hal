//! # Reset & Control Clock

use core::ops::{Deref, DerefMut};

use crate::pac::{
    rcc::{self, RegisterBlock as RccRB},
    BKP, DBGMCU, PWR, RCC,
};

use crate::flash::ACR;
#[cfg(any(feature = "stm32f103", feature = "connectivity"))]
use crate::time::MHz;
use fugit::{HertzU32 as Hertz, RateExtU32};

use crate::backup_domain::BackupDomain;

mod enable;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;

    /// Constrains the `RCC` peripheral and apply clock configuration
    fn freeze(self, rcc_cfg: impl Into<RawConfig>, acr: &mut ACR) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            rb: self,
            clocks: Clocks::default(),
        }
    }

    fn freeze(self, rcc_cfg: impl Into<RawConfig>, acr: &mut ACR) -> Rcc {
        self.constrain().freeze(rcc_cfg, acr)
    }
}

/// Constrained RCC peripheral
///
/// Aquired by calling the [constrain](../trait.RccExt.html#tymethod.constrain) method
/// on the Rcc struct from the `PAC`
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// ```
pub struct Rcc {
    pub clocks: Clocks,
    pub(crate) rb: RCC,
}

impl Deref for Rcc {
    type Target = RCC;
    fn deref(&self) -> &Self::Target {
        &self.rb
    }
}

impl DerefMut for Rcc {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.rb
    }
}

macro_rules! bus_struct {
    ($($busX:ident => ($EN:ident, $en:ident, $($RST:ident, $rst:ident,)? $doc:literal),)+) => {
        $(
            #[doc = $doc]
            #[non_exhaustive]
            pub struct $busX;

            impl $busX {
                pub(crate) fn enr(rcc: &RccRB) -> &rcc::$EN {
                    rcc.$en()
                }
                $(
                    pub(crate) fn rstr(rcc: &RccRB) -> &rcc::$RST {
                        rcc.$rst()
                    }
                )?
            }
        )+
    };
}

bus_struct! {
    APB1 => (APB1ENR, apb1enr, APB1RSTR, apb1rstr, "Advanced Peripheral Bus 1 (APB1) registers"),
    APB2 => (APB2ENR, apb2enr, APB2RSTR, apb2rstr, "Advanced Peripheral Bus 2 (APB2) registers"),
    AHB => (AHBENR, ahbenr, "Advanced High-performance Bus (AHB) registers"),
}

const HSI: u32 = 8_000_000; // Hz

/// Clock configuration
///
/// Used to configure the frequencies of the clocks present in the processor.
///
/// After setting all frequencies, call the [freeze](#method.freeze) function to
/// apply the configuration.
///
/// **NOTE**: Currently, it is not guaranteed that the exact frequencies selected will be
/// used, only frequencies close to it.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Config {
    hse: Option<u32>,
    hse_bypass: bool,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl Config {
    pub const DEFAULT: Self = Self {
        hse: None,
        hse_bypass: false,
        hclk: None,
        pclk1: None,
        pclk2: None,
        sysclk: None,
        adcclk: None,
    };

    pub fn hsi() -> Self {
        Self::DEFAULT
    }

    pub fn hse(freq: Hertz) -> Self {
        Self::DEFAULT.use_hse(freq)
    }

    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator
    #[inline(always)]
    pub fn use_hse(mut self, freq: Hertz) -> Self {
        self.hse = Some(freq.raw());
        self
    }

    /// Bypasses the high-speed external oscillator and uses an external clock input on the OSC_IN
    /// pin.
    ///
    /// For this configuration, the OSC_IN pin should be connected to a clock source with a
    /// frequency specified in the call to use_hse(), and the OSC_OUT pin should not be connected.
    ///
    /// This function has no effect unless use_hse() is also called.
    pub fn bypass_hse_oscillator(self) -> Self {
        Self {
            hse_bypass: true,
            ..self
        }
    }

    /// Sets the desired frequency for the HCLK clock
    #[inline(always)]
    pub fn hclk(mut self, freq: Hertz) -> Self {
        self.hclk = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the PCKL1 clock
    #[inline(always)]
    pub fn pclk1(mut self, freq: Hertz) -> Self {
        self.pclk1 = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the PCLK2 clock
    #[inline(always)]
    pub fn pclk2(mut self, freq: Hertz) -> Self {
        self.pclk2 = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the SYSCLK clock
    #[inline(always)]
    pub fn sysclk(mut self, freq: Hertz) -> Self {
        self.sysclk = Some(freq.raw());
        self
    }

    /// Sets the desired frequency for the ADCCLK clock
    #[inline(always)]
    pub fn adcclk(mut self, freq: Hertz) -> Self {
        self.adcclk = Some(freq.raw());
        self
    }
}

impl Rcc {
    /// Applies the clock configuration and returns a `Clocks` struct that signifies that the
    /// clocks are frozen, and contains the frequencies used. After this function is called,
    /// the clocks can not change
    ///
    /// Usage:
    ///
    /// ```rust
    /// let dp = pac::Peripherals::take().unwrap();
    /// let mut flash = dp.FLASH.constrain();
    /// let mut rcc = dp.RCC.constrain();
    /// let clocks = rcc.freeze(rcc::Config::default(), &mut flash.acr);
    /// ```
    #[inline(always)]
    #[allow(unused)]
    pub fn freeze(self, cfg: impl Into<RawConfig>, acr: &mut ACR) -> Self {
        let cfg = cfg.into();
        let clocks = cfg.get_clocks();
        // adjust flash wait states
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        unsafe {
            acr.acr().write(|w| {
                w.latency().bits(if clocks.sysclk <= MHz(24) {
                    0b000
                } else if clocks.sysclk <= MHz(48) {
                    0b001
                } else {
                    0b010
                })
            });
        }

        let rcc = unsafe { &*RCC::ptr() };

        if cfg.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr().modify(|_, w| {
                if cfg.hse_bypass {
                    w.hsebyp().bypassed();
                }
                w.hseon().set_bit()
            });

            while rcc.cr().read().hserdy().bit_is_clear() {}
        }

        if let Some(pllmul_bits) = cfg.pllmul {
            // enable PLL and wait for it to be ready

            #[allow(unused_unsafe)]
            rcc.cfgr().modify(|_, w| unsafe {
                w.pllmul().bits(pllmul_bits).pllsrc().bit(cfg.hse.is_some())
            });

            rcc.cr().modify(|_, w| w.pllon().set_bit());

            while rcc.cr().read().pllrdy().bit_is_clear() {}
        }

        // set prescalers and clock source
        #[cfg(feature = "connectivity")]
        rcc.cfgr().modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2().bits(cfg.ppre2 as u8);
            w.ppre1().bits(cfg.ppre1 as u8);
            w.hpre().bits(cfg.hpre as u8);
            w.otgfspre().variant(cfg.usbpre);
            w.sw().bits(if cfg.pllmul.is_some() {
                // PLL
                0b10
            } else if cfg.hse.is_some() {
                // HSE
                0b1
            } else {
                // HSI
                0b0
            })
        });

        #[cfg(feature = "stm32f103")]
        rcc.cfgr().modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2().bits(cfg.ppre2 as u8);
            w.ppre1().bits(cfg.ppre1 as u8);
            w.hpre().bits(cfg.hpre as u8);
            w.usbpre().variant(cfg.usbpre);
            w.sw().bits(if cfg.pllmul.is_some() {
                // PLL
                0b10
            } else {
                // HSE or HSI
                u8::from(cfg.hse.is_some())
            })
        });

        #[cfg(any(feature = "stm32f100", feature = "stm32f101"))]
        rcc.cfgr().modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2().bits(cfg.ppre2 as u8);
            w.ppre1().bits(cfg.ppre1 as u8);
            w.hpre().bits(cfg.hpre as u8);
            w.sw().bits(if cfg.pllmul.is_some() {
                // PLL
                0b10
            } else if cfg.hse.is_some() {
                // HSE
                0b1
            } else {
                // HSI
                0b0
            })
        });

        Self {
            rb: self.rb,
            clocks,
        }
    }
}

pub trait BkpExt {
    /// Enables write access to the registers in the backup domain
    fn constrain(self, pwr: &mut PWR, rcc: &mut RCC) -> BackupDomain;
}

impl BkpExt for BKP {
    fn constrain(self, pwr: &mut PWR, rcc: &mut RCC) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        BKP::enable(rcc);
        PWR::enable(rcc);

        // Enable access to the backup registers
        pwr.cr().modify(|_r, w| w.dbp().set_bit());

        BackupDomain { _regs: self }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
///
/// To acquire it, use the freeze function on the `rcc.cfgr` register. If desired, you can adjust
/// the frequencies using the methods on [cfgr](struct.CFGR.html) before calling freeze.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// let mut flash = dp.FLASH.constrain();
///
/// let clocks = rcc.freeze(rcc::Config::default(), &mut flash.acr);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
    adcclk: Hertz,
    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    usbclk_valid: bool,
}

impl Default for Clocks {
    fn default() -> Clocks {
        let freq = HSI.Hz();
        Clocks {
            hclk: freq,
            pclk1: freq,
            pclk2: freq,
            ppre1: 1,
            ppre2: 1,
            sysclk: freq,
            adcclk: freq / 2,
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbclk_valid: false,
        }
    }
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub const fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub const fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub const fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the APB1 Timers
    pub const fn pclk1_tim(&self) -> Hertz {
        Hertz::from_raw(self.pclk1.raw() * if self.ppre1() == 1 { 1 } else { 2 })
    }

    /// Returns the frequency of the APB2 Timers
    pub const fn pclk2_tim(&self) -> Hertz {
        Hertz::from_raw(self.pclk2.raw() * if self.ppre2() == 1 { 1 } else { 2 })
    }

    pub(crate) const fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) const fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub const fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the adc clock frequency
    pub const fn adcclk(&self) -> Hertz {
        self.adcclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    pub const fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

/// Common trait for most of peripherals
pub trait Instance:
    crate::Ptr + crate::Steal + Enable + Reset + RccBus<Bus: BusClock> + Deref<Target = Self::RB>
{
}

/// Frequency on bus that peripheral is connected in
pub trait BusClock {
    /// Calculates frequency depending on `Clock` state
    fn clock(clocks: &Clocks) -> Hertz;
}

/// Frequency on bus that timer is connected in
pub trait BusTimerClock {
    /// Calculates base frequency of timer depending on `Clock` state
    fn timer_clock(clocks: &Clocks) -> Hertz;
}

impl BusClock for AHB {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl BusClock for APB1 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
}

impl BusClock for APB2 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
}

impl BusTimerClock for APB1 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1_tim()
    }
}

impl BusTimerClock for APB2 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        clocks.pclk2_tim()
    }
}

/// Bus associated to peripheral
pub trait RccBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub trait Enable: RccBus {
    /// Enables peripheral
    fn enable(rcc: &mut RCC);

    /// Disables peripheral
    fn disable(rcc: &mut RCC);

    /// Check if peripheral enabled
    fn is_enabled() -> bool;

    /// Check if peripheral disabled
    #[inline]
    fn is_disabled() -> bool {
        !Self::is_enabled()
    }

    /// # Safety
    ///
    /// Enables peripheral. Takes access to RCC internally
    unsafe fn enable_unchecked() {
        let mut rcc = RCC::steal();
        Self::enable(&mut rcc);
    }

    /// # Safety
    ///
    /// Disables peripheral. Takes access to RCC internally
    unsafe fn disable_unchecked() {
        let mut rcc = RCC::steal();
        Self::disable(&mut rcc);
    }
}

/// Reset peripheral
pub trait Reset: RccBus {
    /// Resets peripheral
    fn reset(rcc: &mut RCC);

    /// # Safety
    ///
    /// Resets peripheral. Takes access to RCC internally
    unsafe fn reset_unchecked() {
        let mut rcc = RCC::steal();
        Self::reset(&mut rcc);
    }
}

/// Stop peripheral when Core is halted
pub trait StopInDebug {
    fn stop_in_debug(&mut self, dbg: &mut DBGMCU, state: bool);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawConfig {
    pub hse: Option<u32>,
    pub hse_bypass: bool,
    pub pllmul: Option<u8>,
    pub hpre: HPre,
    pub ppre1: PPre,
    pub ppre2: PPre,
    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    pub usbpre: UsbPre,
    pub adcpre: AdcPre,
    pub allow_overclock: bool,
}

impl Default for RawConfig {
    fn default() -> Self {
        Self {
            hse: None,
            hse_bypass: false,
            pllmul: None,
            hpre: HPre::Div1,
            ppre1: PPre::Div1,
            ppre2: PPre::Div1,
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbpre: UsbPre::Div1_5,
            adcpre: AdcPre::Div2,
            allow_overclock: false,
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HPre {
    /// SYSCLK not divided
    Div1 = 7,
    /// SYSCLK divided by 2
    Div2 = 8,
    /// SYSCLK divided by 4
    Div4 = 9,
    /// SYSCLK divided by 8
    Div8 = 10,
    /// SYSCLK divided by 16
    Div16 = 11,
    /// SYSCLK divided by 64
    Div64 = 12,
    /// SYSCLK divided by 128
    Div128 = 13,
    /// SYSCLK divided by 256
    Div256 = 14,
    /// SYSCLK divided by 512
    Div512 = 15,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PPre {
    /// HCLK not divided
    Div1 = 3,
    /// HCLK divided by 2
    Div2 = 4,
    /// HCLK divided by 4
    Div4 = 5,
    /// HCLK divided by 8
    Div8 = 6,
    /// HCLK divided by 16
    Div16 = 7,
}

#[cfg(feature = "stm32f103")]
pub type UsbPre = rcc::cfgr::USBPRE;
#[cfg(feature = "connectivity")]
pub type UsbPre = rcc::cfgr::OTGFSPRE;
pub type AdcPre = rcc::cfgr::ADCPRE;

impl From<Config> for RawConfig {
    #[inline(always)]
    fn from(cfgr: Config) -> Self {
        Self::from_cfgr(cfgr)
    }
}

impl RawConfig {
    pub const fn from_cfgr(cfgr: Config) -> Self {
        let hse = cfgr.hse;
        let hse_bypass = cfgr.hse_bypass;
        let pllsrcclk = if let Some(hse) = hse { hse } else { HSI / 2 };

        let pllmul = if let Some(sysclk) = cfgr.sysclk {
            sysclk / pllsrcclk
        } else {
            1
        };

        let (pllmul_bits, sysclk) = if pllmul == 1 {
            (None, if let Some(hse) = hse { hse } else { HSI })
        } else {
            #[cfg(not(feature = "connectivity"))]
            let pllmul = match pllmul {
                1..=16 => pllmul,
                0 => 1,
                _ => 16,
            };

            #[cfg(feature = "connectivity")]
            let pllmul = match pllmul {
                4..=9 => pllmul,
                0..=3 => 4,
                _ => 9,
            };

            (Some(pllmul as u8 - 2), pllsrcclk * pllmul)
        };

        let hpre_bits = if let Some(hclk) = cfgr.hclk {
            match sysclk / hclk {
                0..=1 => HPre::Div1,
                2 => HPre::Div2,
                3..=5 => HPre::Div4,
                6..=11 => HPre::Div8,
                12..=39 => HPre::Div16,
                40..=95 => HPre::Div64,
                96..=191 => HPre::Div128,
                192..=383 => HPre::Div256,
                _ => HPre::Div512,
            }
        } else {
            HPre::Div1
        };

        let hclk = if hpre_bits as u8 >= 0b1100 {
            sysclk / (1 << (hpre_bits as u8 - 0b0110))
        } else {
            sysclk / (1 << (hpre_bits as u8 - 0b0111))
        };

        let pclk1 = if let Some(pclk1) = cfgr.pclk1 {
            pclk1
        } else if hclk < 36_000_000 {
            hclk
        } else {
            36_000_000
        };
        let ppre1_bits = match (hclk + pclk1 - 1) / pclk1 {
            0 | 1 => PPre::Div1,
            2 => PPre::Div2,
            3..=5 => PPre::Div4,
            6..=11 => PPre::Div8,
            _ => PPre::Div16,
        };

        let ppre2_bits = if let Some(pclk2) = cfgr.pclk2 {
            match hclk / pclk2 {
                0..=1 => PPre::Div1,
                2 => PPre::Div2,
                3..=5 => PPre::Div4,
                6..=11 => PPre::Div8,
                _ => PPre::Div16,
            }
        } else {
            PPre::Div1
        };

        let ppre2 = 1 << (ppre2_bits as u8 - 0b011);
        let pclk2 = hclk / (ppre2 as u32);

        // usbpre == false: divide clock by 1.5, otherwise no division
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        let usbpre = match (hse, pllmul_bits, sysclk) {
            (Some(_), Some(_), 72_000_000) => UsbPre::Div1_5,
            _ => UsbPre::Div1,
        };

        let apre_bits = if let Some(adcclk) = cfgr.adcclk {
            match pclk2 / adcclk {
                0..=2 => AdcPre::Div2,
                3..=4 => AdcPre::Div4,
                5..=7 => AdcPre::Div6,
                _ => AdcPre::Div8,
            }
        } else {
            AdcPre::Div8
        };

        Self {
            hse,
            hse_bypass,
            pllmul: pllmul_bits,
            hpre: hpre_bits,
            ppre1: ppre1_bits,
            ppre2: ppre2_bits,
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbpre,
            adcpre: apre_bits,
            allow_overclock: false,
        }
    }

    // NOTE: to maintain the invariant that the existence of a Clocks
    // value implies frozen clocks, this function must not be pub.
    fn get_clocks(&self) -> Clocks {
        let sysclk = if let Some(pllmul_bits) = self.pllmul {
            let pllsrcclk = if let Some(hse) = self.hse {
                hse
            } else {
                HSI / 2
            };
            pllsrcclk * (pllmul_bits as u32 + 2)
        } else if let Some(hse) = self.hse {
            hse
        } else {
            HSI
        };

        let hclk = if self.hpre as u8 >= 0b1100 {
            sysclk / (1 << (self.hpre as u8 - 0b0110))
        } else {
            sysclk / (1 << (self.hpre as u8 - 0b0111))
        };

        let ppre1 = 1 << (self.ppre1 as u8 - 0b011);
        let pclk1 = hclk / (ppre1 as u32);

        let ppre2 = 1 << (self.ppre2 as u8 - 0b011);
        let pclk2 = hclk / (ppre2 as u32);

        let apre = (self.adcpre as u8 + 1) << 1;
        let adcclk = pclk2 / (apre as u32);

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        let usbclk_valid = matches!(
            (self.hse, self.pllmul, sysclk),
            (Some(_), Some(_), 72_000_000) | (Some(_), Some(_), 48_000_000)
        );

        assert!(
            self.allow_overclock
                || (sysclk <= 72_000_000
                    && hclk <= 72_000_000
                    && pclk1 <= 36_000_000
                    && pclk2 <= 72_000_000
                    && adcclk <= 14_000_000)
        );

        Clocks {
            hclk: hclk.Hz(),
            pclk1: pclk1.Hz(),
            pclk2: pclk2.Hz(),
            ppre1,
            ppre2,
            sysclk: sysclk.Hz(),
            adcclk: adcclk.Hz(),
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbclk_valid,
        }
    }
}

#[test]
fn rcc_config_usb() {
    let cfgr = Config::default()
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz());

    let config = RawConfig::from_cfgr(cfgr);
    let config_expected = RawConfig {
        hse: Some(8_000_000),
        hse_bypass: false,
        pllmul: Some(4),
        hpre: HPre::Div1,
        ppre1: PPre::Div2,
        ppre2: PPre::Div1,
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        usbpre: UsbPre::Div1,
        adcpre: AdcPre::Div8,
        allow_overclock: false,
    };
    assert_eq!(config, config_expected);

    let clocks = config.get_clocks();
    let clocks_expected = Clocks {
        hclk: 48.MHz(),
        pclk1: 24.MHz(),
        pclk2: 48.MHz(),
        ppre1: 2,
        ppre2: 1,
        sysclk: 48.MHz(),
        adcclk: 6.MHz(),
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        usbclk_valid: true,
    };
    assert_eq!(clocks, clocks_expected);
}
