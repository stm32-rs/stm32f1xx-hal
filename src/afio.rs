//! # Alternate Function I/Os
use crate::pac::{self, afio, AFIO, RCC};

use crate::rcc::{Enable, Reset};

use crate::gpio::{self, Alternate, Cr, Debugger, Floating, Input, OpenDrain, PushPull};
use crate::sealed::Sealed;

pub trait AfioExt {
    fn constrain(self) -> Parts;
}

impl AfioExt for AFIO {
    fn constrain(self) -> Parts {
        let rcc = unsafe { &(*RCC::ptr()) };
        AFIO::enable(rcc);
        AFIO::reset(rcc);

        Parts {
            evcr: EVCR,
            mapr: MAPR { jtag_enabled: true },
            exticr1: EXTICR1,
            exticr2: EXTICR2,
            exticr3: EXTICR3,
            exticr4: EXTICR4,
            mapr2: MAPR2,
        }
    }
}

/// HAL wrapper around the AFIO registers
///
/// Aquired by calling [constrain](trait.AfioExt.html#constrain) on the [AFIO
/// registers](../pac/struct.AFIO.html)
///
/// ```rust
/// let p = pac::Peripherals::take().unwrap();
/// let mut rcc = p.RCC.constrain();
/// let mut afio = p.AFIO.constrain();
pub struct Parts {
    pub evcr: EVCR,
    pub mapr: MAPR,
    pub exticr1: EXTICR1,
    pub exticr2: EXTICR2,
    pub exticr3: EXTICR3,
    pub exticr4: EXTICR4,
    pub mapr2: MAPR2,
}

#[non_exhaustive]
pub struct EVCR;

impl EVCR {
    pub fn evcr(&mut self) -> &afio::EVCR {
        unsafe { (*AFIO::ptr()).evcr() }
    }
}

/// AF remap and debug I/O configuration register (MAPR)
///
/// Aquired through the [Parts](struct.Parts.html) struct.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// let mut afio = dp.AFIO.constrain();
/// function_using_mapr(&mut afio.mapr);
/// ```
#[non_exhaustive]
pub struct MAPR {
    jtag_enabled: bool,
}

impl MAPR {
    fn mapr(&mut self) -> &afio::MAPR {
        unsafe { (*AFIO::ptr()).mapr() }
    }

    pub fn modify_mapr<F>(&mut self, mod_fn: F)
    where
        F: for<'w> FnOnce(&afio::mapr::R, &'w mut afio::mapr::W) -> &'w mut afio::mapr::W,
    {
        let debug_bits = if self.jtag_enabled { 0b000 } else { 0b010 };
        self.mapr()
            .modify(unsafe { |r, w| mod_fn(r, w).swj_cfg().bits(debug_bits) });
    }

    /// Disables the JTAG to free up pa15, pb3 and pb4 for normal use
    #[allow(clippy::redundant_field_names, clippy::type_complexity)]
    pub fn disable_jtag(
        &mut self,
        pa15: gpio::PA15<Debugger>,
        pb3: gpio::PB3<Debugger>,
        pb4: gpio::PB4<Debugger>,
    ) -> (gpio::PA15, gpio::PB3, gpio::PB4) {
        self.jtag_enabled = false;
        // Avoid duplicating swj_cfg write code
        self.modify_mapr(|_, w| w);

        // NOTE(unsafe) The pins are now in the good state.
        unsafe { (pa15.activate(), pb3.activate(), pb4.activate()) }
    }
}

#[non_exhaustive]
pub struct EXTICR1;

impl EXTICR1 {
    pub fn exticr1(&mut self) -> &afio::EXTICR1 {
        unsafe { (*AFIO::ptr()).exticr1() }
    }
}

#[non_exhaustive]
pub struct EXTICR2;

impl EXTICR2 {
    pub fn exticr2(&mut self) -> &afio::EXTICR2 {
        unsafe { (*AFIO::ptr()).exticr2() }
    }
}

#[non_exhaustive]
pub struct EXTICR3;

impl EXTICR3 {
    pub fn exticr3(&mut self) -> &afio::EXTICR3 {
        unsafe { (*AFIO::ptr()).exticr3() }
    }
}

#[non_exhaustive]
pub struct EXTICR4;

impl EXTICR4 {
    pub fn exticr4(&mut self) -> &afio::EXTICR4 {
        unsafe { (*AFIO::ptr()).exticr4() }
    }
}

#[non_exhaustive]
pub struct MAPR2;

impl MAPR2 {
    pub fn mapr2(&mut self) -> &afio::MAPR2 {
        unsafe { (*AFIO::ptr()).mapr2() }
    }

    pub fn modify_mapr<F>(&mut self, mod_fn: F)
    where
        F: for<'w> FnOnce(&afio::mapr2::R, &'w mut afio::mapr2::W) -> &'w mut afio::mapr2::W,
    {
        self.mapr2().modify(|r, w| mod_fn(r, w));
    }
}

pub trait Remap: Sealed + Sized {
    type Mapr;
    #[doc(hidden)]
    fn rmp(mapr: &mut Self::Mapr, to: u8);
    fn remap<const R: u8>(self, mapr: &mut Self::Mapr) -> Rmp<Self, R> {
        Self::rmp(mapr, R);
        Rmp(self)
    }
}

macro_rules! remap {
    ($(
        $PER:ty: $MAPR:ident, $w:ident: $field:ident $(, { $allowed:pat })?;
    )+) => {
        $(
            remap!($PER: $MAPR, $w: $field $(, { $allowed })?);
        )+
    };
    ($PER:ty: $MAPR:ident, bool: $field:ident) => {
        impl Remap for $PER {
            type Mapr = $MAPR;
            fn rmp(mapr: &mut Self::Mapr, to: u8) {
                assert!(matches!(to, 0 | 1));
                mapr.modify_mapr(|_, w| w.$field().bit(to != 0));
            }
        }
    };
    ($PER:ty: $MAPR:ident, u8: $field:ident $(, {$allowed:pat })?) => {
        impl Remap for $PER {
            type Mapr = $MAPR;
            fn rmp(mapr: &mut Self::Mapr, to: u8) {
                $(assert!(matches!(to, $allowed)))?;
                mapr.modify_mapr(|_, w| unsafe { w.$field().bits(to) });
            }
        }
    };
}
use remap;

remap! {
    pac::SPI1: MAPR, bool: spi1_remap;
    pac::I2C1: MAPR, bool: i2c1_remap;
    pac::USART1: MAPR, bool: usart1_remap;
    pac::USART2: MAPR, bool: usart2_remap;
    pac::USART3: MAPR, u8: usart3_remap, { 0 | 1 | 3 };
    pac::TIM2: MAPR, u8: tim2_remap, { 0..=3 };
    pac::TIM3: MAPR, u8: tim3_remap, { 0 | 2 | 3 };
}

#[cfg(feature = "medium")]
remap! {
    pac::TIM4: MAPR, bool: tim4_remap;
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
remap! {
    pac::TIM1: MAPR, u8: tim1_remap, { 0 | 1 | 3 };
}

#[cfg(feature = "stm32f103")]
remap! {
    pac::CAN1: MAPR, u8: can_remap, { 0 | 2 | 3 };
}

#[cfg(feature = "connectivity")]
remap! {
    pac::CAN1: MAPR, u8: can1_remap, { 0 | 2 | 3 };
    //pac::ETHERNET_MAC: MAPR, bool: eth_remap;
    pac::CAN2: MAPR, bool: can2_remap;
    pac::SPI3: MAPR, bool: spi3_remap;
}

#[cfg(feature = "xl")]
remap! {
    pac::TIM9: MAPR2, bool: tim9_remap;
    pac::TIM10: MAPR2, bool: tim10_remap;
    pac::TIM11: MAPR2, bool: tim11_remap;
}
#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
remap! {
    pac::TIM13: MAPR2, bool: tim13_remap;
    pac::TIM14: MAPR2, bool: tim14_remap;
}

pub struct Rmp<T, const R: u8>(pub(crate) T);

pub trait RFrom<T, const R: u8> {
    fn rfrom(value: T) -> Self;
}

impl<T, const R: u8> RFrom<T, R> for T {
    fn rfrom(value: T) -> Self {
        value
    }
}

pub trait RInto<T, const R: u8> {
    fn rinto(self) -> T;
}

impl<S, T, const R: u8> RInto<T, R> for S
where
    T: RFrom<Self, R>,
{
    fn rinto(self) -> T {
        T::rfrom(self)
    }
}

// REMAPPING, see: https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
// Section 9.3

/// CAN pins
pub trait CanCommon {
    /// Transmit
    ///
    /// Alternate function push-pull
    type Tx;
    /// Receive
    ///
    /// Input floating / Input pull-up
    type Rx<PULL>;
}

#[cfg(feature = "has-can")]
pub mod can1 {
    use super::*;

    pin! {
        <Tx, Alternate<PushPull>> for [
            PA12: [0],
            PB9:  [2],
            PD1:  [3],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PA11: [0],
            PB8:  [2],
            PD0:  [3],
        ],
    }

    impl CanCommon for pac::CAN1 {
        type Tx = Tx;
        type Rx<PULL> = Rx<PULL>;
    }
}

#[cfg(feature = "connectivity")]
pub mod can2 {
    use super::*;

    pin! {
        <Tx, Alternate<PushPull>> for [
            PB6:  [0],
            PB13: [1],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PB5:  [0],
            PB12: [1],
        ],
    }

    impl CanCommon for pac::CAN2 {
        type Tx = Tx;
        type Rx<PULL> = Rx<PULL>;
    }
}

/// I2C pins
pub trait I2cCommon {
    /// Serial Clock
    ///
    /// Alternate function open drain
    type Scl;
    /// Serial Data
    ///
    /// Alternate function open drain
    type Sda;
    /// SMBus
    type Smba;
}

pub mod i2c1 {
    use super::*;

    pin! {
        <Scl, Alternate<OpenDrain>> for [
            PB6: [0],
            PB8: [1],
        ],
        <Sda, Alternate<OpenDrain>> for [
            PB7: [0],
            PB9: [1],
        ],
        <Smba, Alternate<OpenDrain>> for [
            PB5: [0, 1],
        ],
    }

    impl I2cCommon for pac::I2C1 {
        type Scl = Scl;
        type Sda = Sda;
        type Smba = Smba;
    }
}
pub mod i2c2 {
    use super::*;

    pin! {
        <Scl, Alternate<OpenDrain>> for [
            PB10: [0],
        ],
        <Sda, Alternate<OpenDrain>> for [
            PB11: [0],
        ],
        <Smba, Alternate<OpenDrain>> for [
            PB12: [0],
        ],
    }

    impl I2cCommon for pac::I2C2 {
        type Scl = Scl;
        type Sda = Sda;
        type Smba = Smba;
    }
}

/// SPI pins
pub trait SpiCommon {
    /// Master Serial Clock
    ///
    /// Alternate function push-pull
    type MSck;
    /// Slave Serial Clock
    ///
    /// Input floating
    type SSck;
    /// Master In
    ///
    /// Input floating / Input pull-up
    type Mi<PULL>;
    /// Slave Out
    ///
    /// Alternate function push-pull / open drain
    type So<Otype>;
    /// Master Out
    ///
    /// Alternate function push-pull
    type Mo;
    /// Slave In
    ///
    /// Input floating / Input pull-up
    type Si<PULL>;
    /// HW Slave Select (output)
    ///
    /// Alternate function push-pull
    type Nss;
    /// HW Slave Select (input)
    ///
    /// Input floating/ Input pull-up / Input pull-down
    type Ss<PULL>;
}

pub mod spi1 {
    use super::*;

    pin! {
        <Si, Input> default:Floating && <Mo, Alternate<PushPull>> for [
            PA7: [0],
            PB5: [1],
        ],
        <Ss, Input> default:Floating && <Nss, Alternate<PushPull>>  for [
            PA4: [0],
            PA15: [1],
        ],
    }
    pin! {
        <SSck, Input<Floating>> && <MSck, Alternate<PushPull>> for [
            PA5: [0],
            PB3: [1],
        ],
    }
    pin! {
        <Mi, Input> default:Floating && <So, Alternate> default:PushPull for [
            PA6: [0],
            PB4: [1],
        ],
    }

    impl SpiCommon for pac::SPI1 {
        type MSck = MSck;
        type SSck = SSck;
        type Mi<PULL> = Mi<PULL>;
        type So<Otype> = So<Otype>;
        type Mo = Mo;
        type Si<PULL> = Si<PULL>;
        type Nss = Nss;
        type Ss<PULL> = Ss<PULL>;
    }
}

pub mod spi2 {
    use super::*;

    pin! {
        <Si, Input> default:Floating && <Mo, Alternate<PushPull>> for [
            PB15: [0],
        ],
        <Ss, Input> default:Floating && <Nss, Alternate<PushPull>> for [
            PB12: [0],
        ],
    }
    pin! {
        <SSck, Input<Floating>> && <MSck, Alternate<PushPull>> for [
            PB13: [0],
        ],
    }
    pin! {
        <Mi, Input> default:Floating && <So, Alternate> default:PushPull for [
            PB14: [0],
        ],
    }

    impl SpiCommon for pac::SPI2 {
        type MSck = MSck;
        type SSck = SSck;
        type Mi<PULL> = Mi<PULL>;
        type So<Otype> = So<Otype>;
        type Mo = Mo;
        type Si<PULL> = Si<PULL>;
        type Nss = Nss;
        type Ss<PULL> = Ss<PULL>;
    }
}
#[cfg(any(feature = "high", feature = "connectivity"))]
pub mod spi3 {
    use super::*;

    #[cfg(not(feature = "connectivity"))]
    pin! {
        <Si, Input> default:Floating && <Mo, Alternate<PushPull>> for [
            PB5: [0],
        ],
        <Ss, Input> default:Floating && <Nss, Alternate<PushPull>> for [
            PA15: [0],
        ],
    }
    #[cfg(not(feature = "connectivity"))]
    pin! {
        <SSck, Input<Floating>> && <MSck, Alternate<PushPull>> for [
            PB3: [0],
        ],
    }
    #[cfg(not(feature = "connectivity"))]
    pin! {
        <Mi, Input> default:Floating && <So, Alternate> default:PushPull for [
            PB4: [0],
        ],
    }

    #[cfg(feature = "connectivity")]
    pin! {
        <Si, Input> default:Floating && <Mo, Alternate<PushPull>> for [
            PB5: [0],
            PC12: [1],
        ],
        <Ss, Input> default:Floating && <Nss, Alternate<PushPull>> for [
            PA15: [0],
            PA4: [1],
        ],
    }
    #[cfg(feature = "connectivity")]
    pin! {
        <SSck, Input<Floating>> && <MSck, Alternate<PushPull>> for [
            PB3: [0],
            PC10: [1],
        ],
    }
    #[cfg(feature = "connectivity")]
    pin! {
        <Mi, Input> default:Floating && <So, Alternate> default:PushPull for [
            PB4: [0],
            PC11: [1],
        ],
    }

    impl SpiCommon for pac::SPI3 {
        type MSck = MSck;
        type SSck = SSck;
        type Mi<PULL> = Mi<PULL>;
        type So<Otype> = So<Otype>;
        type Mo = Mo;
        type Si<PULL> = Si<PULL>;
        type Nss = Nss;
        type Ss<PULL> = Ss<PULL>;
    }
}

// Serial pins
pub trait SerialAsync {
    /// Receive
    ///
    /// Input floating / Input pull-up
    type Rx<PULL>;
    /// Transmit
    ///
    /// Alternate function push-pull / open drain
    type Tx<Otype>;
}
/// Synchronous mode
///
/// Alternate function push-pull
pub trait SerialSync {
    type Ck;
}
/// Hardware flow control (RS232)
pub trait SerialFlowControl {
    /// "Clear To Send" blocks the data transmission at the end of the current transfer when high
    ///
    /// Input floating/ Input pull-up
    type Cts<PULL>;
    /// "Request to send" indicates that the USART is ready to receive a data (when low)
    ///
    /// Alternate function push-pull
    type Rts;
}

pub mod usart1 {
    use super::*;

    pin! {
        <Tx, Alternate> default:PushPull for [
            PA9: [0],
            PB6: [1],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PA10: [0],
            PB7:  [1],
        ],
        <Cts, Input> default:Floating for [
            PA11: [0, 1],
        ],
    }

    pin! {
        <Ck, Alternate<PushPull>> for [
            PA8: [0, 1],
        ],
        <Rts, Alternate<PushPull>> for [
            PA12: [0, 1],
        ],
    }

    impl SerialAsync for pac::USART1 {
        type Rx<PULL> = Rx<PULL>;
        type Tx<Otype> = Tx<Otype>;
    }

    impl SerialSync for pac::USART1 {
        type Ck = Ck;
    }

    impl SerialFlowControl for pac::USART1 {
        type Cts<PULL> = Cts<PULL>;
        type Rts = Rts;
    }
}

pub mod usart2 {
    use super::*;

    pin! {
        <Tx, Alternate> default:PushPull for [
            PA2: [0],
            PD5: [1],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PA3: [0],
            PD6:  [1],
        ],
        <Cts, Input> default:Floating for [
            PA0: [0],
            PD3: [1],
        ],
    }

    pin! {
        <Ck, Alternate<PushPull>> for [
            PA4: [0],
            PD7: [1],
        ],
        <Rts, Alternate<PushPull>> for [
            PA1: [0],
            PD4:  [1],
        ],
    }

    impl SerialAsync for pac::USART2 {
        type Rx<PULL> = Rx<PULL>;
        type Tx<Otype> = Tx<Otype>;
    }

    impl SerialSync for pac::USART2 {
        type Ck = Ck;
    }

    impl SerialFlowControl for pac::USART2 {
        type Cts<PULL> = Cts<PULL>;
        type Rts = Rts;
    }
}

pub mod usart3 {
    use super::*;

    pin! {
        <Tx, Alternate> default:PushPull for [
            PB10: [0],
            PC10: [1],
            PD8:  [3],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PB11: [0],
            PC11: [1],
            PD9:  [3],
        ],
        <Cts, Input> default:Floating for [
            PB13: [0, 1],
            PD11: [3],
        ],
    }

    pin! {
        <Ck, Alternate<PushPull>> for [
            PB12: [0],
            PC12: [1],
            PD10: [3],
        ],
        <Rts, Alternate<PushPull>> for [
            PB14: [0, 1],
            PD12: [3],
        ],
    }

    impl SerialAsync for pac::USART3 {
        type Rx<PULL> = Rx<PULL>;
        type Tx<Otype> = Tx<Otype>;
    }

    impl SerialSync for pac::USART3 {
        type Ck = Ck;
    }

    impl SerialFlowControl for pac::USART3 {
        type Cts<PULL> = Cts<PULL>;
        type Rts = Rts;
    }
}

#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
pub mod uart4 {
    use super::*;

    pin! {
        <Tx, Alternate> default:PushPull for [
            PC10: [0],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PC11: [0],
        ],
    }

    impl SerialAsync for pac::UART4 {
        type Rx<PULL> = Rx<PULL>;
        type Tx<Otype> = Tx<Otype>;
    }
}

#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
pub mod uart5 {
    use super::*;

    pin! {
        <Tx, Alternate> default:PushPull for [
            PC12: [0],
        ],
    }
    pin! {
        <Rx, Input> default:Floating for [
            PD2: [0],
        ],
    }

    impl SerialAsync for pac::UART5 {
        type Rx<PULL> = Rx<PULL>;
        type Tx<Otype> = Tx<Otype>;
    }
}

/// Input capture / Output compare channel `C`
pub trait TimC<const C: u8> {
    /// Input capture channel
    ///
    /// Input floating
    type In;
    /// Output compare channel
    ///
    /// Alternate function push-pull
    type Out;
}

/// Complementary output channel `C`
///
/// Alternate function push-pull
pub trait TimNC<const C: u8> {
    type ChN;
}

/// Break input
///
/// Input floating
pub trait TimBkin {
    type Bkin;
}

/// External trigger timer input
///
/// Input floating
pub trait TimEtr {
    type Etr;
}

pub mod tim1 {
    use super::*;

    pin! {
        <Etr, Input<Floating>> for [
            PA12: [0, 1],
            PE7:  [3],
        ],
        <Bkin, Input<Floating>> for [
            PB12: [0],
            PA6:  [1],
            PE15: [3],
        ],
    }
    pin! {
        <Ch1In, Input<Floating>> && <Ch1Out, Alternate<PushPull>> for [
            PA8:  [0, 1],
            PE9:  [3],
        ],
        <Ch2In, Input<Floating>> && <Ch2Out, Alternate<PushPull>> for [
            PA9:  [0, 1],
            PE11: [3],
        ],
        <Ch3In, Input<Floating>> && <Ch3Out, Alternate<PushPull>> for [
            PA10: [0, 1],
            PE13: [3],
        ],
        <Ch4In, Input<Floating>> && <Ch4Out, Alternate<PushPull>> for [
            PA11: [0, 1],
            PE14: [3],
        ],
    }

    pin! {
        <Ch1N, Alternate<PushPull>> for [
            PB13: [0],
            PA7:  [1],
            PE8:  [3],
        ],
        <Ch2N, Alternate<PushPull>> for [
            PB14: [0],
            PB0:  [1],
            PE10: [3],
        ],
        <Ch3N, Alternate<PushPull>> for [
            PB15: [0],
            PB1:  [1],
            PE12: [3],
        ],
    }

    use pac::TIM1 as TIM;
    impl TimEtr for TIM {
        type Etr = Etr;
    }
    impl TimBkin for TIM {
        type Bkin = Bkin;
    }
    impl TimC<0> for TIM {
        type In = Ch1In;
        type Out = Ch1Out;
    }
    impl TimC<1> for TIM {
        type In = Ch2In;
        type Out = Ch2Out;
    }
    impl TimC<2> for TIM {
        type In = Ch3In;
        type Out = Ch3Out;
    }
    impl TimC<3> for TIM {
        type In = Ch4In;
        type Out = Ch4Out;
    }
    impl TimNC<0> for TIM {
        type ChN = Ch1N;
    }
    impl TimNC<1> for TIM {
        type ChN = Ch2N;
    }
    impl TimNC<2> for TIM {
        type ChN = Ch3N;
    }
}

pub mod tim2 {
    use super::*;

    pin! {
        <Etr, Input<Floating>> for [
            PA0:  [0, 2],
            PA15: [1, 3],
        ],
    }
    pin! {
        <Ch1In, Input<Floating>> && <Ch1Out, Alternate<PushPull>> for [
            PA0:  [0, 2],
            PA15: [1, 3],
        ],
        <Ch2In, Input<Floating>> && <Ch2Out, Alternate<PushPull>> for [
            PA1:  [0, 2],
            PB3:  [1, 3],
        ],
        <Ch3In, Input<Floating>> && <Ch3Out, Alternate<PushPull>> for [
            PA2:  [0, 1],
            PB10: [2, 3],
        ],
        <Ch4In, Input<Floating>> && <Ch4Out, Alternate<PushPull>> for [
            PA3:  [0, 1],
            PB11: [2, 3],
        ],
    }

    use pac::TIM2 as TIM;
    impl TimEtr for TIM {
        type Etr = Etr;
    }
    impl TimC<0> for TIM {
        type In = Ch1In;
        type Out = Ch1Out;
    }
    impl TimC<1> for TIM {
        type In = Ch2In;
        type Out = Ch2Out;
    }
    impl TimC<2> for TIM {
        type In = Ch3In;
        type Out = Ch3Out;
    }
    impl TimC<3> for TIM {
        type In = Ch4In;
        type Out = Ch4Out;
    }
}

pub mod tim3 {
    use super::*;

    pin! {
        <Etr, Input<Floating>> for [
            PD2:  [0, 2, 3],
        ],
    }
    pin! {
        <Ch1In, Input<Floating>> && <Ch1Out, Alternate<PushPull>> for [
            PA6:  [0],
            PB4:  [2],
            PC6:  [3],
        ],
        <Ch2In, Input<Floating>> && <Ch2Out, Alternate<PushPull>> for [
            PA7:  [0],
            PB5:  [2],
            PC7:  [3],
        ],
        <Ch3In, Input<Floating>> && <Ch3Out, Alternate<PushPull>> for [
            PB0:  [0, 2],
            PC8:  [3],
        ],
        <Ch4In, Input<Floating>> && <Ch4Out, Alternate<PushPull>> for [
            PB1:  [0, 2],
            PC9:  [3],
        ],
    }

    use pac::TIM3 as TIM;
    impl TimEtr for TIM {
        type Etr = Etr;
    }
    impl TimC<0> for TIM {
        type In = Ch1In;
        type Out = Ch1Out;
    }
    impl TimC<1> for TIM {
        type In = Ch2In;
        type Out = Ch2Out;
    }
    impl TimC<2> for TIM {
        type In = Ch3In;
        type Out = Ch3Out;
    }
    impl TimC<3> for TIM {
        type In = Ch4In;
        type Out = Ch4Out;
    }
}

pub mod tim4 {
    use super::*;

    pin! {
        <Etr, Input<Floating>> for [
            PE0:  [0, 1],
        ],
    }
    pin! {
        <Ch1In, Input<Floating>> && <Ch1Out, Alternate<PushPull>> for [
            PB6:  [0],
            PD12: [1],
        ],
        <Ch2In, Input<Floating>> && <Ch2Out, Alternate<PushPull>> for [
            PB7:  [0],
            PD13: [1],
        ],
        <Ch3In, Input<Floating>> && <Ch3Out, Alternate<PushPull>> for [
            PB8:  [0],
            PD14: [1],
        ],
        <Ch4In, Input<Floating>> && <Ch4Out, Alternate<PushPull>> for [
            PB9:  [0],
            PD15: [1],
        ],
    }

    use pac::TIM4 as TIM;
    impl TimEtr for TIM {
        type Etr = Etr;
    }
    impl TimC<0> for TIM {
        type In = Ch1In;
        type Out = Ch1Out;
    }
    impl TimC<1> for TIM {
        type In = Ch2In;
        type Out = Ch2Out;
    }
    impl TimC<2> for TIM {
        type In = Ch3In;
        type Out = Ch3Out;
    }
    impl TimC<3> for TIM {
        type In = Ch4In;
        type Out = Ch4Out;
    }
}

macro_rules! pin_mode {
    ( $($(#[$docs:meta])* <$name:ident, $MODE:ty> for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        $(
            //#[derive(Debug)]
            $(#[$docs])*
            pub enum $name {
                $(
                    $PX(gpio::$PX<$MODE>),
                )*
            }

            impl crate::Sealed for $name { }

            $(
                $(
                    impl RFrom<gpio::$PX<$MODE>, $remap> for $name {
                        fn rfrom(p: gpio::$PX<$MODE>) -> Self {
                            Self::$PX(p)
                        }
                    }
                )+

                impl<MODE> TryFrom<$name> for gpio::$PX<MODE>
                where
                    MODE: $crate::gpio::PinMode,
                    $MODE: $crate::gpio::PinMode,
                {
                    type Error = ();

                    fn try_from(a: $name) -> Result<Self, Self::Error> {
                        #[allow(irrefutable_let_patterns)]
                        if let $name::$PX(p) = a {
                            Ok(p.into_mode(&mut Cr))
                        } else {
                            Err(())
                        }
                    }
                }
            )*
        )*
    };
}
use pin_mode;

macro_rules! pin_default_mode {
    ( $($(#[$docs:meta])* <$name:ident, $M:ident> default:$DefaultMode:ident for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        $(
            //#[derive(Debug)]
            $(#[$docs])*
            pub enum $name<MODE = $DefaultMode> {
                $(
                    $PX(gpio::$PX<$M<MODE>>),
                )*
            }

            impl<MODE> crate::Sealed for $name<MODE> { }

            $(
                $(
                    impl<MODE> RFrom<gpio::$PX<$M<MODE>>, $remap> for $name<MODE> {
                        fn rfrom(p: gpio::$PX<$M<MODE>>) -> Self {
                            Self::$PX(p)
                        }
                    }
                )+

                impl<OUTMODE> TryFrom<$name> for gpio::$PX<OUTMODE>
                where
                    OUTMODE: $crate::gpio::PinMode,
                {
                    type Error = ();

                    fn try_from(a: $name) -> Result<Self, Self::Error> {
                        #[allow(irrefutable_let_patterns)]
                        if let $name::$PX(p) = a {
                            Ok(p.into_mode(&mut Cr))
                        } else {
                            Err(())
                        }
                    }
                }
            )*
        )*
    };
}
use pin_default_mode;

macro_rules! pin {
    ( $($(#[$docs:meta])* <$name:ident, Input<$MODE:ident>> && <$name2:ident, Alternate<$MODE2:ident>> for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin! {
            $($(#[$docs])* <$name, Input<$MODE>> for [$(
                $PX: [$($remap),+],
            )*],)*
        }
        pin! {
            $($(#[$docs])* <$name2, Alternate<$MODE2>> for [$(
                $PX: [$($remap),+],
            )*],)*
        }
    };

    ( $($(#[$docs:meta])* <$name:ident, Input> default:$DefaultMode:ident && <$name2:ident, Alternate<$MODE2:ident>> for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin! {
            $($(#[$docs])* <$name, Input> default:$DefaultMode for [$(
                $PX: [$($remap),+],
            )*],)*
        }
        pin! {
            $($(#[$docs])* <$name2, Alternate<$MODE2>> for [$(
                $PX: [$($remap),+],
            )*],)*
        }
    };

    ( $($(#[$docs:meta])* <$name:ident, Input> default:$DefaultMode:ident && <$name2:ident, Alternate> default:$DefaultMode2:ident for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin! {
            $($(#[$docs])* <$name, Input> default:$DefaultMode for [$(
                $PX: [$($remap),+],
            )*],)*
        }
        pin! {
            $($(#[$docs])* <$name2, Alternate> default:$DefaultMode2 for [$(
                $PX: [$($remap),+],
            )*],)*
        }
    };

    ( $($(#[$docs:meta])* <$name:ident, Input<$MODE:ident>> for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin_mode! {
            $($(#[$docs])* <$name, Input<$MODE>> for [$(
                $PX: [$($remap),+],
            )*],)*
        }
    };
    ( $($(#[$docs:meta])* <$name:ident, Alternate<$MODE:ident>> for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin_mode! {
            $($(#[$docs])* <$name, Alternate<$MODE>> for [$(
                $PX: [$($remap),+],
            )*],)*
        }

        $(
            $(
                $(
                    impl RFrom<gpio::$PX, $remap> for $name {
                        fn rfrom(p: gpio::$PX) -> Self {
                            Self::$PX(p.into_mode(&mut Cr))
                        }
                    }
                )+
            )*
        )*
    };
    ( $($(#[$docs:meta])* <$name:ident, Input> default:$DefaultMode:ident for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin_default_mode! {
            $($(#[$docs])* <$name, Input> default:$DefaultMode for [$(
                $PX: [$($remap),+],
            )*],)*
        }
    };
    ( $($(#[$docs:meta])* <$name:ident, Alternate> default:$DefaultMode:ident for [$(
        $PX:ident: [$($remap:literal),+],
    )*],)*) => {
        pin_default_mode! {
            $($(#[$docs])* <$name, Alternate> default:$DefaultMode for [$(
                $PX: [$($remap),+],
            )*],)*
        }
        $(
            $(
                $(
                    impl<MODE> RFrom<gpio::$PX, $remap> for $name<MODE>
                    where Alternate<MODE>: $crate::gpio::PinMode,
                    {
                        fn rfrom(p: gpio::$PX) -> Self {
                            Self::$PX(p.into_mode(&mut Cr))
                        }
                    }
                )+
            )*
        )*
    };
}
use pin;
