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
use crate::gpio::{self, Alternate, Cr, Floating, Input, PinMode, PullUp};
use crate::pac::{self, RCC};

pub trait InMode {}
impl InMode for Floating {}
impl InMode for PullUp {}

pub mod can1 {
    use super::*;

    remap! {
        Pins: [
            #[cfg(not(feature = "connectivity"))]
            All, Tx, Rx, PA12, PA11  => { |_, w| unsafe { w.can_remap().bits(0) } };
            #[cfg(feature = "connectivity")]
            All, Tx, Rx, PA12, PA11  => { |_, w| unsafe { w.can1_remap().bits(0) } };
            #[cfg(not(feature = "connectivity"))]
            Remap, RemapTx, RemapRx, PB9, PB8  => { |_, w| unsafe { w.can_remap().bits(10) } };
            #[cfg(feature = "connectivity")]
            Remap, RemapTx, RemapRx, PB9, PB8  => { |_, w| unsafe { w.can1_remap().bits(10) } };
        ]
    }
}

#[cfg(feature = "connectivity")]
pub mod can2 {
    use super::*;

    remap! {
        Pins: [
            All, Tx, Rx, PB6, PB5  => { |_, w| w.can2_remap().bit(false) };
            Remap, RemapTx, RemapRx, PB13, PB12  => { |_, w| w.can2_remap().bit(true) };
        ]
    }
}

macro_rules! remap {
    ($name:ident: [
        $($(#[$attr:meta])* $rname:ident, $txonly:ident, $rxonly:ident, $TX:ident, $RX:ident => { $remapex:expr };)+
    ]) => {
        pub enum $name<PULL> {
            $(
                $(#[$attr])*
                $rname { tx: gpio::$TX<Alternate>, rx: gpio::$RX<Input<PULL>> },
                $(#[$attr])*
                $txonly { tx: gpio::$TX<Alternate> },
                $(#[$attr])*
                $rxonly { rx: gpio::$RX<Input<PULL>> },
            )+
        }

        $(
            $(#[$attr])*
            impl<PULL: InMode> From<(gpio::$TX<Alternate>, gpio::$RX<Input<PULL>>, &mut MAPR)> for $name<PULL> {
                fn from(p: (gpio::$TX<Alternate>, gpio::$RX<Input<PULL>>, &mut MAPR)) -> Self {
                    p.2.modify_mapr($remapex);
                    Self::$rname { tx: p.0, rx: p.1 }
                }
            }

            $(#[$attr])*
            impl<PULL> From<(gpio::$TX, gpio::$RX, &mut MAPR)> for $name<PULL>
            where
                Input<PULL>: PinMode,
                PULL: InMode,
            {
                fn from(p: (gpio::$TX, gpio::$RX, &mut MAPR)) -> Self {
                    let mut cr = Cr;
                    let tx = p.0.into_mode(&mut cr);
                    let rx = p.1.into_mode(&mut cr);
                    p.2.modify_mapr($remapex);
                    Self::$rname { tx, rx }
                }
            }

            $(#[$attr])*
            impl From<(gpio::$TX, &mut MAPR)> for $name<Floating> {
                fn from(p: (gpio::$TX, &mut MAPR)) -> Self {
                    let tx = p.0.into_mode(&mut Cr);
                    p.1.modify_mapr($remapex);
                    Self::$txonly { tx }
                }
            }

            $(#[$attr])*
            impl<PULL> From<(gpio::$RX, &mut MAPR)> for $name<PULL>
            where
                Input<PULL>: PinMode,
                PULL: InMode,
            {
                fn from(p: (gpio::$RX, &mut MAPR)) -> Self {
                    let rx = p.0.into_mode(&mut Cr);
                    p.1.modify_mapr($remapex);
                    Self::$rxonly { rx }
                }
            }
        )+
    }
}
use remap;

pub trait CanExt: Sized + Instance {
    fn can<PULL>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Self::Pins<PULL>>,
    ) -> Can<Self, PULL>;
    fn can_loopback<PULL>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, PULL>;
}

impl<CAN: Instance> CanExt for CAN {
    fn can<PULL>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Self::Pins<PULL>>,
    ) -> Can<Self, PULL> {
        Can::new(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
            pins,
        )
    }
    fn can_loopback<PULL>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, PULL> {
        Can::new_loopback(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
        )
    }
}

pub trait Instance: crate::rcc::Enable {
    type Pins<PULL>;
}
impl Instance for pac::CAN1 {
    type Pins<PULL> = can1::Pins<PULL>;
}
#[cfg(feature = "connectivity")]
impl Instance for pac::CAN2 {
    type Pins<PULL> = can2::Pins<PULL>;
}

/// Interface to the CAN peripheral.
#[allow(unused)]
pub struct Can<CAN: Instance, PULL = Floating> {
    can: CAN,
    pins: Option<CAN::Pins<PULL>>,
}

impl<CAN: Instance, PULL> Can<CAN, PULL> {
    /// Creates a CAN interface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    pub fn new(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
        pins: impl Into<CAN::Pins<PULL>>,
    ) -> Can<CAN, PULL> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        let pins = Some(pins.into());
        Can { can, pins }
    }

    /// Creates a CAN interface in loopback mode
    pub fn new_loopback(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
    ) -> Can<CAN, PULL> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        Can { can, pins: None }
    }
}

unsafe impl<PULL> bxcan::Instance for Can<pac::CAN1, PULL> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN1::ptr() as *mut _;
}

#[cfg(feature = "connectivity")]
unsafe impl<PULL> bxcan::Instance for Can<pac::CAN2, PULL> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN2::ptr() as *mut _;
}

unsafe impl<PULL> bxcan::FilterOwner for Can<pac::CAN1, PULL> {
    const NUM_FILTER_BANKS: u8 = 28;
}

#[cfg(feature = "connectivity")]
unsafe impl<PULL> bxcan::MasterInstance for Can<pac::CAN1, PULL> {}
