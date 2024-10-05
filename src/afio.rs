//! # Alternate Function I/Os
use crate::pac::{afio, AFIO, RCC};

use crate::rcc::{Enable, Reset};

use crate::gpio::{
    Debugger, Floating, Input, PA15, {PB3, PB4},
};

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
        pa15: PA15<Debugger>,
        pb3: PB3<Debugger>,
        pb4: PB4<Debugger>,
    ) -> (
        PA15<Input<Floating>>,
        PB3<Input<Floating>>,
        PB4<Input<Floating>>,
    ) {
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
}
