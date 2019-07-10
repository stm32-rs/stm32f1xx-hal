//! # Alternate Function I/Os
use crate::pac::{afio, AFIO};

use crate::rcc::APB2;

use crate::gpio::{
    Debugger,
    Input,
    Floating,
    gpioa::PA15,
    gpiob::{PB3, PB4},
};

pub trait AfioExt {
    fn constrain(self, apb2: &mut APB2) -> Parts;
}

impl AfioExt for AFIO {
    fn constrain(self, apb2: &mut APB2) -> Parts {
        apb2.enr().modify(|_, w| w.afioen().set_bit());
        apb2.rstr().modify(|_, w| w.afiorst().set_bit());
        apb2.rstr().modify(|_, w| w.afiorst().clear_bit());

        Parts {
            evcr: EVCR { _0: () },
            mapr: MAPR { _0: () },
            exticr1: EXTICR1 { _0: () },
            exticr2: EXTICR2 { _0: () },
            exticr3: EXTICR3 { _0: () },
            exticr4: EXTICR4 { _0: () },
            mapr2: MAPR2 { _0: () },
        }
    }
}

pub struct Parts {
    pub evcr: EVCR,
    pub mapr: MAPR,
    pub exticr1: EXTICR1,
    pub exticr2: EXTICR2,
    pub exticr3: EXTICR3,
    pub exticr4: EXTICR4,
    pub mapr2: MAPR2,
}

pub struct EVCR {
    _0: (),
}

impl EVCR {
    pub fn evcr(&mut self) -> &afio::EVCR {
        unsafe { &(*AFIO::ptr()).evcr }
    }
}

pub struct MAPR {
    _0: (),
}

impl MAPR {
    pub fn mapr(&mut self) -> &afio::MAPR {
        unsafe { &(*AFIO::ptr()).mapr }
    }

    /// Disables the JTAG to free up pa15, pb3 and pb4 for normal use
    pub fn disable_jtag(
        &mut self,
        pa15: PA15<Debugger>,
        pb3: PB3<Debugger>,
        pb4: PB4<Debugger>
    ) -> (
        PA15<Input<Floating>>,
        PB3<Input<Floating>>,
        PB4<Input<Floating>>,
    ) {
        self.mapr()
            .modify(|_, w| unsafe { w.swj_cfg().bits(0b010) });

        // NOTE(unsafe) The pins are now in the good state.
        unsafe { (pa15.activate(), pb3.activate(), pb4.activate()) }
    }
}

pub struct EXTICR1 {
    _0: (),
}

impl EXTICR1 {
    pub fn exticr1(&mut self) -> &afio::EXTICR1 {
        unsafe { &(*AFIO::ptr()).exticr1 }
    }
}

pub struct EXTICR2 {
    _0: (),
}

impl EXTICR2 {
    pub fn exticr2(&mut self) -> &afio::EXTICR2 {
        unsafe { &(*AFIO::ptr()).exticr2 }
    }
}

pub struct EXTICR3 {
    _0: (),
}

impl EXTICR3 {
    pub fn exticr3(&mut self) -> &afio::EXTICR3 {
        unsafe { &(*AFIO::ptr()).exticr3 }
    }
}

pub struct EXTICR4 {
    _0: (),
}

impl EXTICR4 {
    pub fn exticr4(&mut self) -> &afio::EXTICR4 {
        unsafe { &(*AFIO::ptr()).exticr4 }
    }
}

pub struct MAPR2 {
    _0: (),
}

impl MAPR2 {
    pub fn mapr2(&mut self) -> &afio::MAPR2 {
        unsafe { &(*AFIO::ptr()).mapr2 }
    }
}
