//! # Controller Area Network (CAN) Interface
//!
//! ## Alternate function remapping
//!
//! TX: Alternate Push-Pull Output
//! RX: Input
//!
//! ### CAN1
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PA12    | PB9   |
//! | RX       | PA11    | PB8   |
//!
//! ### CAN2
//!
//! | Function | NoRemap | Remap |
//! |----------|---------|-------|
//! | TX       | PB6     | PB13  |
//! | RX       | PB5     | PB12  |

use crate::afio::MAPR;
#[cfg(feature = "connectivity")]
use crate::gpio::gpiob::{PB12, PB13, PB5, PB6};
use crate::gpio::{
    gpioa::{PA11, PA12},
    gpiob::{PB8, PB9},
    Alternate, Input,
};
#[cfg(feature = "connectivity")]
use crate::pac::CAN2;
#[cfg(not(feature = "connectivity"))]
use crate::pac::USB;
use crate::pac::{CAN1, RCC};

pub trait Pins: crate::Sealed {
    type Instance;
    fn remap(mapr: &mut MAPR);
}

impl<INMODE, OUTMODE> crate::Sealed for (PA12<Alternate<OUTMODE>>, PA11<Input<INMODE>>) {}
impl<INMODE, OUTMODE> Pins for (PA12<Alternate<OUTMODE>>, PA11<Input<INMODE>>) {
    type Instance = CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0) });
    }
}

impl<INMODE, OUTMODE> crate::Sealed for (PB9<Alternate<OUTMODE>>, PB8<Input<INMODE>>) {}
impl<INMODE, OUTMODE> Pins for (PB9<Alternate<OUTMODE>>, PB8<Input<INMODE>>) {
    type Instance = CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0b10) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0b10) });
    }
}

#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> crate::Sealed for (PB13<Alternate<OUTMODE>>, PB12<Input<INMODE>>) {}
#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> Pins for (PB13<Alternate<OUTMODE>>, PB12<Input<INMODE>>) {
    type Instance = CAN2;

    fn remap(mapr: &mut MAPR) {
        mapr.modify_mapr(|_, w| w.can2_remap().clear_bit());
    }
}

#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> crate::Sealed for (PB6<Alternate<OUTMODE>>, PB5<Input<INMODE>>) {}
#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> Pins for (PB6<Alternate<OUTMODE>>, PB5<Input<INMODE>>) {
    type Instance = CAN2;

    fn remap(mapr: &mut MAPR) {
        mapr.modify_mapr(|_, w| w.can2_remap().set_bit());
    }
}

/// Interface to the CAN peripheral.
pub struct Can<Instance> {
    _peripheral: Instance,
}

impl<Instance> Can<Instance>
where
    Instance: crate::rcc::Enable,
{
    /// Creates a CAN interaface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    #[cfg(not(feature = "connectivity"))]
    pub fn new(can: Instance, _usb: USB) -> Can<Instance> {
        let rcc = unsafe { &(*RCC::ptr()) };
        Instance::enable(rcc);

        Can { _peripheral: can }
    }

    /// Creates a CAN interaface.
    #[cfg(feature = "connectivity")]
    pub fn new(can: Instance) -> Can<Instance> {
        let rcc = unsafe { &(*RCC::ptr()) };
        Instance::enable(rcc);

        Can { _peripheral: can }
    }

    /// Routes CAN TX signals and RX signals to pins.
    pub fn assign_pins<P>(&self, _pins: P, mapr: &mut MAPR)
    where
        P: Pins<Instance = Instance>,
    {
        P::remap(mapr);
    }
}

unsafe impl bxcan::Instance for Can<CAN1> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

#[cfg(feature = "connectivity")]
unsafe impl bxcan::Instance for Can<CAN2> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN2::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can<CAN1> {
    const NUM_FILTER_BANKS: u8 = 28;
}

#[cfg(feature = "connectivity")]
unsafe impl bxcan::MasterInstance for Can<CAN1> {}
