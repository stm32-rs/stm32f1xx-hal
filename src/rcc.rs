//! # Reset & Control Clock

use crate::pac::{rcc, PWR, RCC};

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
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
                adcclk: None,
            },
            bkp: BKP { _0: () },
        }
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
    pub cfgr: CFGR,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    fn enr(rcc: &rcc::RegisterBlock) -> &rcc::AHBENR {
        &rcc.ahbenr
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    fn enr(rcc: &rcc::RegisterBlock) -> &rcc::APB1ENR {
        &rcc.apb1enr
    }

    fn rstr(rcc: &rcc::RegisterBlock) -> &rcc::APB1RSTR {
        &rcc.apb1rstr
    }
}

impl APB1 {
    /// Set power interface clock (PWREN) bit in RCC_APB1ENR
    pub fn set_pwren() {
        let rcc = unsafe { &*RCC::ptr() };
        PWR::enable(rcc);
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    fn enr(rcc: &rcc::RegisterBlock) -> &rcc::APB2ENR {
        &rcc.apb2enr
    }

    fn rstr(rcc: &rcc::RegisterBlock) -> &rcc::APB2RSTR {
        &rcc.apb2rstr
    }
}

const HSI: u32 = 8_000_000; // Hz

/// Clock configuration register (CFGR)
///
/// Used to configure the frequencies of the clocks present in the processor.
///
/// After setting all frequencies, call the [freeze](#method.freeze) function to
/// apply the configuration.
///
/// **NOTE**: Currently, it is not guaranteed that the exact frequencies selected will be
/// used, only frequencies close to it.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator
    #[inline(always)]
    pub fn use_hse(mut self, freq: Hertz) -> Self {
        self.hse = Some(freq.raw());
        self
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
    /// let clocks = rcc.cfgr.freeze(&mut flash.acr);
    /// ```

    #[inline(always)]
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let cfg = Config::from_cfgr(self);
        Self::_freeze_with_config(cfg, acr)
    }

    #[inline(always)]
    pub fn freeze_with_config(self, cfg: Config, acr: &mut ACR) -> Clocks {
        Self::_freeze_with_config(cfg, acr)
    }

    #[allow(unused_variables)]
    fn _freeze_with_config(cfg: Config, acr: &mut ACR) -> Clocks {
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
            })
        }

        let rcc = unsafe { &*RCC::ptr() };

        if cfg.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        if let Some(pllmul_bits) = cfg.pllmul {
            // enable PLL and wait for it to be ready

            #[allow(unused_unsafe)]
            rcc.cfgr.modify(|_, w| unsafe {
                w.pllmul().bits(pllmul_bits).pllsrc().bit(cfg.hse.is_some())
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        // set prescalers and clock source
        #[cfg(feature = "connectivity")]
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2()
                .bits(cfg.ppre2 as u8)
                .ppre1()
                .bits(cfg.ppre1 as u8)
                .hpre()
                .bits(cfg.hpre as u8)
                .otgfspre()
                .variant(cfg.usbpre)
                .sw()
                .bits(if cfg.pllmul.is_some() {
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
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2()
                .bits(cfg.ppre2 as u8)
                .ppre1()
                .bits(cfg.ppre1 as u8)
                .hpre()
                .bits(cfg.hpre as u8)
                .usbpre()
                .variant(cfg.usbpre)
                .sw()
                .bits(if cfg.pllmul.is_some() {
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

        #[cfg(any(feature = "stm32f100", feature = "stm32f101"))]
        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2()
                .bits(cfg.ppre2 as u8)
                .ppre1()
                .bits(cfg.ppre1 as u8)
                .hpre()
                .bits(cfg.hpre as u8)
                .sw()
                .bits(if cfg.pllmul.is_some() {
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

        clocks
    }
}

pub struct BKP {
    _0: (),
}

impl BKP {
    /// Enables write access to the registers in the backup domain
    pub fn constrain(self, bkp: crate::pac::BKP, pwr: &mut PWR) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        let rcc = unsafe { &(*RCC::ptr()) };
        crate::pac::BKP::enable(rcc);
        crate::pac::PWR::enable(rcc);

        // Enable access to the backup registers
        pwr.cr.modify(|_r, w| w.dbp().set_bit());

        BackupDomain { _regs: bkp }
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
/// let clocks = rcc.cfgr.freeze(&mut flash.acr);
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

impl<T> BusClock for T
where
    T: RccBus,
    T::Bus: BusClock,
{
    fn clock(clocks: &Clocks) -> Hertz {
        T::Bus::clock(clocks)
    }
}

impl<T> BusTimerClock for T
where
    T: RccBus,
    T::Bus: BusTimerClock,
{
    fn timer_clock(clocks: &Clocks) -> Hertz {
        T::Bus::timer_clock(clocks)
    }
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
    fn enable(rcc: &rcc::RegisterBlock);
    fn disable(rcc: &rcc::RegisterBlock);
}
/// Reset peripheral
pub trait Reset: RccBus {
    fn reset(rcc: &rcc::RegisterBlock);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub hse: Option<u32>,
    pub pllmul: Option<u8>,
    pub hpre: HPre,
    pub ppre1: PPre,
    pub ppre2: PPre,
    #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
    pub usbpre: UsbPre,
    pub adcpre: AdcPre,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            hse: None,
            pllmul: None,
            hpre: HPre::Div1,
            ppre1: PPre::Div1,
            ppre2: PPre::Div1,
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbpre: UsbPre::Div15,
            adcpre: AdcPre::Div2,
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
pub type UsbPre = rcc::cfgr::USBPRE_A;
#[cfg(feature = "connectivity")]
pub type UsbPre = rcc::cfgr::OTGFSPRE_A;
pub type AdcPre = rcc::cfgr::ADCPRE_A;

impl Config {
    pub const fn from_cfgr(cfgr: CFGR) -> Self {
        let hse = cfgr.hse;
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
            (Some(_), Some(_), 72_000_000) => UsbPre::Div15,
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
            pllmul: pllmul_bits,
            hpre: hpre_bits,
            ppre1: ppre1_bits,
            ppre2: ppre2_bits,
            #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
            usbpre,
            adcpre: apre_bits,
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
            sysclk <= 72_000_000
                && hclk <= 72_000_000
                && pclk1 <= 36_000_000
                && pclk2 <= 72_000_000
                && adcclk <= 14_000_000
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
    let cfgr = CFGR::default()
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz());

    let config = Config::from_cfgr(cfgr);
    let config_expected = Config {
        hse: Some(8_000_000),
        pllmul: Some(4),
        hpre: HPre::Div1,
        ppre1: PPre::Div2,
        ppre2: PPre::Div1,
        #[cfg(any(feature = "stm32f103", feature = "connectivity"))]
        usbpre: UsbPre::Div1,
        adcpre: AdcPre::Div8,
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
