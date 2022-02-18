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
use crate::gpio::{self, Alternate, Input};
use crate::pac::{self, RCC};

pub trait Pins: crate::Sealed {
    type Instance;
    fn remap(mapr: &mut MAPR);
}

impl<INMODE, OUTMODE> crate::Sealed
    for (gpio::PA12<Alternate<OUTMODE>>, gpio::PA11<Input<INMODE>>)
{
}
impl<INMODE, OUTMODE> Pins for (gpio::PA12<Alternate<OUTMODE>>, gpio::PA11<Input<INMODE>>) {
    type Instance = pac::CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0) });
    }
}

impl<INMODE, OUTMODE> crate::Sealed for (gpio::PB9<Alternate<OUTMODE>>, gpio::PB8<Input<INMODE>>) {}
impl<INMODE, OUTMODE> Pins for (gpio::PB9<Alternate<OUTMODE>>, gpio::PB8<Input<INMODE>>) {
    type Instance = pac::CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0b10) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0b10) });
    }
}

#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> crate::Sealed
    for (gpio::PB13<Alternate<OUTMODE>>, gpio::PB12<Input<INMODE>>)
{
}
#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> Pins for (gpio::PB13<Alternate<OUTMODE>>, gpio::PB12<Input<INMODE>>) {
    type Instance = pac::CAN2;

    fn remap(mapr: &mut MAPR) {
        mapr.modify_mapr(|_, w| w.can2_remap().clear_bit());
    }
}

#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> crate::Sealed for (gpio::PB6<Alternate<OUTMODE>>, gpio::PB5<Input<INMODE>>) {}
#[cfg(feature = "connectivity")]
impl<INMODE, OUTMODE> Pins for (gpio::PB6<Alternate<OUTMODE>>, gpio::PB5<Input<INMODE>>) {
    type Instance = pac::CAN2;

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
    pub fn new(can: Instance, _usb: pac::USB) -> Can<Instance> {
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

unsafe impl bxcan::Instance for Can<pac::CAN1> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN1::ptr() as *mut _;
}

#[cfg(feature = "connectivity")]
unsafe impl bxcan::Instance for Can<pac::CAN2> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN2::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can<pac::CAN1> {
    const NUM_FILTER_BANKS: u8 = 28;
}

#[cfg(feature = "connectivity")]
unsafe impl bxcan::MasterInstance for Can<pac::CAN1> {}
