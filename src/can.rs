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
        pub enum $name<INMODE> {
            $(
                $(#[$attr])*
                $rname { tx: gpio::$TX<Alternate>, rx: gpio::$RX<Input<INMODE>> },
                $(#[$attr])*
                $txonly { tx: gpio::$TX<Alternate> },
                $(#[$attr])*
                $rxonly { rx: gpio::$RX<Input<INMODE>> },
            )+
        }

        $(
            $(#[$attr])*
            impl<INMODE: InMode> From<(gpio::$TX<Alternate>, gpio::$RX<Input<INMODE>>, &mut MAPR)> for $name<INMODE> {
                fn from(p: (gpio::$TX<Alternate>, gpio::$RX<Input<INMODE>>, &mut MAPR)) -> Self {
                    p.2.modify_mapr($remapex);
                    Self::$rname { tx: p.0, rx: p.1 }
                }
            }

            $(#[$attr])*
            impl<INMODE> From<(gpio::$TX, gpio::$RX, &mut MAPR)> for $name<INMODE>
            where
                Input<INMODE>: PinMode,
                INMODE: InMode,
            {
                fn from(p: (gpio::$TX, gpio::$RX, &mut MAPR)) -> Self {
                    let mut cr = Cr::new();
                    let tx = p.0.into_mode(&mut cr);
                    let rx = p.1.into_mode(&mut cr);
                    p.2.modify_mapr($remapex);
                    Self::$rname { tx, rx }
                }
            }

            $(#[$attr])*
            impl From<(gpio::$TX, &mut MAPR)> for $name<Floating> {
                fn from(p: (gpio::$TX, &mut MAPR)) -> Self {
                    let tx = p.0.into_mode(&mut Cr::new());
                    p.1.modify_mapr($remapex);
                    Self::$txonly { tx }
                }
            }

            $(#[$attr])*
            impl<INMODE> From<(gpio::$RX, &mut MAPR)> for $name<INMODE>
            where
                Input<INMODE>: PinMode,
                INMODE: InMode,
            {
                fn from(p: (gpio::$RX, &mut MAPR)) -> Self {
                    let rx = p.0.into_mode(&mut Cr::new());
                    p.1.modify_mapr($remapex);
                    Self::$rxonly { rx }
                }
            }
        )+
    }
}
use remap;

pub trait CanExt: Sized + Instance {
    fn can<INMODE>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Self::Pins<INMODE>>,
    ) -> Can<Self, INMODE>;
    fn can_loopback<INMODE>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, INMODE>;
}

impl<CAN: Instance> CanExt for CAN {
    fn can<INMODE>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Self::Pins<INMODE>>,
    ) -> Can<Self, INMODE> {
        Can::new(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
            pins,
        )
    }
    fn can_loopback<INMODE>(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
    ) -> Can<Self, INMODE> {
        Can::new_loopback(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
        )
    }
}

pub trait Instance: crate::rcc::Enable {
    type Pins<INMODE>;
}
impl Instance for pac::CAN1 {
    type Pins<INMODE> = can1::Pins<INMODE>;
}
#[cfg(feature = "connectivity")]
impl Instance for pac::CAN2 {
    type Pins<INMODE> = can2::Pins<INMODE>;
}

/// Interface to the CAN peripheral.
#[allow(unused)]
pub struct Can<CAN: Instance, INMODE = Floating> {
    can: CAN,
    pins: Option<CAN::Pins<INMODE>>,
}

impl<CAN: Instance, INMODE> Can<CAN, INMODE> {
    /// Creates a CAN interface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    pub fn new(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
        pins: impl Into<CAN::Pins<INMODE>>,
    ) -> Can<CAN, INMODE> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        let pins = Some(pins.into());
        Can { can, pins }
    }

    /// Creates a CAN interface in loopback mode
    pub fn new_loopback(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
    ) -> Can<CAN, INMODE> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        Can { can, pins: None }
    }
}

unsafe impl<INMODE> bxcan::Instance for Can<pac::CAN1, INMODE> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN1::ptr() as *mut _;
}

#[cfg(feature = "connectivity")]
unsafe impl<INMODE> bxcan::Instance for Can<pac::CAN2, INMODE> {
    const REGISTERS: *mut bxcan::RegisterBlock = pac::CAN2::ptr() as *mut _;
}

unsafe impl<INMODE> bxcan::FilterOwner for Can<pac::CAN1, INMODE> {
    const NUM_FILTER_BANKS: u8 = 28;
}

#[cfg(feature = "connectivity")]
unsafe impl<INMODE> bxcan::MasterInstance for Can<pac::CAN1, INMODE> {}
