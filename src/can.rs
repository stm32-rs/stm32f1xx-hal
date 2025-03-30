//! # Controller Area Network (CAN) Interface
//!
//! ## Alternate function remapping
//!
//! ### CAN1
//!
//! | Function \ Remap | 0 (default) | 1    |
//! |------------------|-------------|------|
//! | TX (A-PP)        | PA12        | PB9  |
//! | RX (I-F/PU)      | PA11        | PB8  |
//!
//! ### CAN2
//!
//! | Function \ Remap | 0 (default) | 1    |
//! |------------------|-------------|------|
//! | TX (A-PP)        | PB6         | PB13 |
//! | RX (I-F/PU)      | PB5         | PB12 |

use crate::afio::{self, RInto, Rmp};
use crate::gpio::{Floating, UpMode};
use crate::pac::{self, RCC};

pub trait CanExt: Sized + Instance {
    fn can<PULL: UpMode>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: (impl RInto<Self::Tx, 0>, impl RInto<Self::Rx<PULL>, 0>),
    ) -> Can<Self, PULL>;
    fn can_loopback(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, Floating>;
}

impl<CAN: Instance> CanExt for CAN {
    fn can<PULL: UpMode>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: (impl RInto<Self::Tx, 0>, impl RInto<Self::Rx<PULL>, 0>),
    ) -> Can<Self, PULL> {
        Can::new(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
            pins,
        )
    }
    fn can_loopback(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, Floating> {
        Can::new_loopback(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
        )
    }
}

pub trait Instance: crate::rcc::Enable + afio::CanCommon {}
#[cfg(not(feature = "connectivity"))]
use pac::CAN as CAN1;
#[cfg(feature = "connectivity")]
use pac::CAN1;

impl Instance for CAN1 {}
#[cfg(feature = "connectivity")]
impl Instance for pac::CAN2 {}

/// Interface to the CAN peripheral.
#[allow(unused)]
pub struct Can<CAN: Instance, PULL = Floating> {
    can: CAN,
    pins: (Option<CAN::Tx>, Option<CAN::Rx<PULL>>),
}

impl<CAN: Instance, const R: u8> Rmp<CAN, R> {
    pub fn can<PULL: UpMode>(
        self,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
        pins: (impl RInto<CAN::Tx, R>, impl RInto<CAN::Rx<PULL>, R>),
    ) -> Can<CAN, PULL> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        let pins = (Some(pins.0.rinto()), Some(pins.1.rinto()));
        Can { can: self.0, pins }
    }
    pub fn can_loopback(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<CAN, Floating> {
        Can::new_loopback(
            self.0,
            #[cfg(not(feature = "connectivity"))]
            usb,
        )
    }
}

impl<CAN: Instance, PULL: UpMode> Can<CAN, PULL> {
    /// Creates a CAN interface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    pub fn new<const R: u8>(
        can: impl Into<Rmp<CAN, R>>,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
        pins: (impl RInto<CAN::Tx, R>, impl RInto<CAN::Rx<PULL>, R>),
    ) -> Can<CAN, PULL> {
        can.into().can(
            #[cfg(not(feature = "connectivity"))]
            _usb,
            pins,
        )
    }

    /// Creates a CAN interface in loopback mode
    pub fn new_loopback(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
    ) -> Can<CAN, PULL> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        Can {
            can,
            pins: (None, None),
        }
    }
}

unsafe impl<PULL> bxcan::Instance for Can<CAN1, PULL> {
    const REGISTERS: *mut bxcan::RegisterBlock = CAN1::ptr() as *mut _;
}

#[cfg(feature = "connectivity")]
unsafe impl<PULL> bxcan::Instance for Can<pac::CAN2, PULL> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN2::ptr() as *mut _;
}

unsafe impl<PULL> bxcan::FilterOwner for Can<CAN1, PULL> {
    const NUM_FILTER_BANKS: u8 = 28;
}

#[cfg(feature = "connectivity")]
unsafe impl<PULL> bxcan::MasterInstance for Can<CAN1, PULL> {}
