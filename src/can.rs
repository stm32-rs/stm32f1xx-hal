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

use crate::afio::Remap;
use crate::gpio::{self, Alternate, Cr, Input, NoPin, PushPull};
use crate::pac::{self, RCC};

pub struct Pins<TX, RX> {
    pub tx: TX,
    pub rx: RX,
}

impl<TX, RX> From<(TX, RX)> for Pins<TX, RX> {
    fn from(value: (TX, RX)) -> Self {
        Self {
            tx: value.0,
            rx: value.1,
        }
    }
}

pub mod can1 {
    use super::*;

    remap! {
        pac::CAN1: [
            PA12, PA11 => 0;
            PB9, PB8 => 2;
        ]
    }
}

#[cfg(feature = "connectivity")]
pub mod can2 {
    use super::*;

    remap! {
        pac::CAN2: [
            PB6, PB5 => 0;
            PB13, PB12 => 1;
        ]
    }
}

macro_rules! remap {
    ($PER:ty: [$($TX:ident, $RX:ident => $remap:literal;)+]) => {
        pub enum Tx {
            $(
                $TX(gpio::$TX<Alternate>),
            )+
            None(NoPin<PushPull>),
        }
        pub enum Rx {
            $(
                $RX(gpio::$RX<Input>),
            )+
            None(NoPin<PushPull>),
        }

        $(
            impl From<(gpio::$TX<Alternate>, gpio::$RX, &mut <$PER as Remap>::Mapr)> for Pins<Tx, Rx> {
                fn from(p: (gpio::$TX<Alternate>, gpio::$RX, &mut <$PER as Remap>::Mapr)) -> Self {
                    <$PER>::remap(p.2, $remap);
                    Self { tx: Tx::$TX(p.0), rx: Rx::$RX(p.1) }
                }
            }

            impl From<(gpio::$TX, gpio::$RX, &mut <$PER as Remap>::Mapr)> for Pins<Tx, Rx> {
                fn from(p: (gpio::$TX, gpio::$RX, &mut <$PER as Remap>::Mapr)) -> Self {
                    let mut cr = Cr;
                    let tx = p.0.into_mode(&mut cr);
                    let rx = p.1;
                    <$PER>::remap(p.2, $remap);
                    Self { tx: Tx::$TX(tx), rx: Rx::$RX(rx) }
                }
            }

            impl From<(gpio::$TX, &mut <$PER as Remap>::Mapr)> for Pins<Tx, Rx> {
                fn from(p: (gpio::$TX, &mut <$PER as Remap>::Mapr)) -> Self {
                    let tx = p.0.into_mode(&mut Cr);
                    <$PER>::remap(p.1, $remap);
                    Self { tx: Tx::$TX(tx), rx: Rx::None(NoPin::new()) }
                }
            }

            impl From<(gpio::$RX, &mut <$PER as Remap>::Mapr)> for Pins<Tx, Rx> {
                fn from(p: (gpio::$RX, &mut <$PER as Remap>::Mapr)) -> Self {
                    let rx = p.0;
                    <$PER>::remap(p.1, $remap);
                    Self { tx: Tx::None(NoPin::new()), rx: Rx::$RX(rx) }
                }
            }
        )+
    }
}
use remap;

pub trait CanExt: Sized + Instance {
    fn can(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Pins<Self::Tx, Self::Rx>>,
    ) -> Can<Self>;
    fn can_loopback(self, #[cfg(not(feature = "connectivity"))] usb: pac::USB) -> Can<Self>;
}

impl<CAN: Instance> CanExt for CAN {
    fn can(
        self,
        #[cfg(not(feature = "connectivity"))] usb: pac::USB,
        pins: impl Into<Pins<Self::Tx, Self::Rx>>,
    ) -> Can<Self> {
        Can::new(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
            pins,
        )
    }
    fn can_loopback(self, #[cfg(not(feature = "connectivity"))] usb: pac::USB) -> Can<Self> {
        Can::new_loopback(
            self,
            #[cfg(not(feature = "connectivity"))]
            usb,
        )
    }
}

pub trait Instance: crate::rcc::Enable {
    type Tx;
    type Rx;
}
impl Instance for pac::CAN1 {
    type Tx = can1::Tx;
    type Rx = can1::Rx;
}
#[cfg(feature = "connectivity")]
impl Instance for pac::CAN2 {
    type Tx = can2::Tx;
    type Rx = can2::Rx;
}

/// Interface to the CAN peripheral.
#[allow(unused)]
pub struct Can<CAN: Instance> {
    can: CAN,
    pins: Option<Pins<CAN::Tx, CAN::Rx>>,
}

impl<CAN: Instance> Can<CAN> {
    /// Creates a CAN interface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    pub fn new(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
        pins: impl Into<Pins<CAN::Tx, CAN::Rx>>,
    ) -> Can<CAN> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        let pins = Some(pins.into());
        Can { can, pins }
    }

    /// Creates a CAN interface in loopback mode
    pub fn new_loopback(
        can: CAN,
        #[cfg(not(feature = "connectivity"))] _usb: pac::USB,
    ) -> Can<CAN> {
        let rcc = unsafe { &(*RCC::ptr()) };
        CAN::enable(rcc);

        Can { can, pins: None }
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
