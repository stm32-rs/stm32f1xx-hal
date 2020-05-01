//! # Controller Area Network (CAN) Interface
//!
//! ## Alternate function remapping
//!
//! TX: Alternate Push-Pull Output
//! RX: Input Floating Input
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
    Alternate, Floating, Input, PushPull,
};
use crate::pac::CAN1;
#[cfg(feature = "connectivity")]
use crate::pac::CAN2;
#[cfg(not(feature = "connectivity"))]
use crate::pac::USB;
use crate::rcc::sealed::RccBus;
use core::{convert::Infallible, marker::PhantomData};

/// Identifier of a CAN message.
///
/// Can be either a standard identifier (11bit, Range: 0..0x3FF)
/// or a extendended identifier (29bit , Range: 0..0x1FFFFFFF).
#[derive(Clone, Copy)]
pub struct Id(u32);

impl Id {
    const STANDARD_SHIFT: u32 = 21; // 11 valid bits. Mask: 0xFFE0_0000
    const EXTENDED_SHIFT: u32 = 3; // 29 valid bits. Mask: 0xFFFF_FFF8
    const EID_MASK: u32 = 0x0000_0004;
    const RTR_MASK: u32 = 0x0000_0002;

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// Ids outside the allowed range are silently truncated.
    pub fn new_standard(id: u32) -> Self {
        assert!(id < 0x7FF);
        Self(id << Self::STANDARD_SHIFT)
    }

    /// Creates a new extendended identifier (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// Ids outside the allowed range are silently truncated.
    pub fn new_extended(id: u32) -> Id {
        assert!(id < 0x1FFF_FFFF);
        Self(id << Self::EXTENDED_SHIFT | Self::EID_MASK)
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    fn with_rtr(&self) -> Id {
        Self(self.0 | Self::RTR_MASK)
    }

    /// Returns the identifier.
    ///
    /// It is up to the user to check if it is an standard or extended id.
    pub fn id(&self) -> u32 {
        if self.is_extended() {
            self.0 >> Self::EXTENDED_SHIFT
        } else {
            self.0 >> Self::STANDARD_SHIFT
        }
    }

    /// Returns `true` if the identifier is an extended identifier.
    pub fn is_extended(&self) -> bool {
        self.0 & Self::EID_MASK != 0
    }

    /// Returns `true` if the identifier is a standard identifier.
    pub fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    fn rtr(&self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
}

/// A CAN data or remote frame.
pub struct Frame {
    id: Id,
    dlc: usize,
    data: [u8; 8],
}

impl Frame {
    /// Creates a new data frame.
    pub fn new(id: Id, data: &[u8]) -> Self {
        assert!(!id.rtr());

        let mut frame = Self {
            id,
            dlc: data.len(),
            data: [0; 8],
        };
        frame.data[0..data.len()].copy_from_slice(data);
        frame
    }

    /// Creates ane new frame with a standard identifier.
    pub fn new_standard(id: u32, data: &[u8]) -> Self {
        Self::new(Id::new_standard(id), data)
    }

    /// Creates ane new frame with an extended identifier.
    pub fn new_extended(id: u32, data: &[u8]) -> Self {
        Self::new(Id::new_extended(id), data)
    }

    /// Marks this frame as a remote frame.
    pub fn set_remote(&mut self) -> &mut Self {
        self.id = self.id.with_rtr();
        self.dlc = 0;
        self
    }

    /// Returns true if this `Frame` is a extended frame
    pub fn is_extended(&self) -> bool {
        self.id.is_extended()
    }

    /// Returns true if this `Frame` is a standard frame
    pub fn is_standard(&self) -> bool {
        self.id.is_standard()
    }

    /// Returns true if this `Frame` is a remote frame
    pub fn is_remote_frame(&self) -> bool {
        self.id.rtr()
    }

    /// Returns true if this `Frame` is a data frame
    pub fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    /// Returns the frame identifier.
    pub fn id(&self) -> u32 {
        self.id.id()
    }

    // Returns the frame data.
    pub fn data(&self) -> &[u8] {
        &self.data[0..self.dlc]
    }
}

// Seal the traits so that they cannot be implemented outside side this crate.
mod traits {
    pub trait Instance: crate::rcc::Enable {
        const REGISTERS: *const crate::pac::can1::RegisterBlock;
    }

    pub trait Pins {
        type CAN: Instance;
        fn remap(mapr: &mut super::MAPR);
    }
}

impl traits::Instance for CAN1 {
    const REGISTERS: *const crate::pac::can1::RegisterBlock = CAN1::ptr();
}

#[cfg(feature = "connectivity")]
impl traits::Instance for CAN2 {
    // Register blocks are the same except for the filter registers.
    // Those are only available on CAN1.
    const REGISTERS: *const crate::pac::can1::RegisterBlock = CAN2::ptr() as *const _;
}

impl traits::Pins for (PA12<Alternate<PushPull>>, PA11<Input<Floating>>) {
    type CAN = CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0) });
    }
}

impl traits::Pins for (PB9<Alternate<PushPull>>, PB8<Input<Floating>>) {
    type CAN = CAN1;

    fn remap(mapr: &mut MAPR) {
        #[cfg(not(feature = "connectivity"))]
        mapr.modify_mapr(|_, w| unsafe { w.can_remap().bits(0x10) });
        #[cfg(feature = "connectivity")]
        mapr.modify_mapr(|_, w| unsafe { w.can1_remap().bits(0x10) });
    }
}

#[cfg(feature = "connectivity")]
impl traits::Pins for (PB13<Alternate<PushPull>>, PB12<Input<Floating>>) {
    type CAN = CAN2;

    fn remap(mapr: &mut MAPR) {
        mapr.modify_mapr(|_, w| w.can2_remap().clear_bit());
    }
}

#[cfg(feature = "connectivity")]
impl traits::Pins for (PB6<Alternate<PushPull>>, PB5<Input<Floating>>) {
    type CAN = CAN2;

    fn remap(mapr: &mut MAPR) {
        mapr.modify_mapr(|_, w| w.can2_remap().set_bit());
    }
}

/// Interface to the CAN peripheral.
pub struct Can<Instance> {
    _can: PhantomData<Instance>,
}

impl<Instance> Can<Instance>
where
    Instance: traits::Instance,
{
    /// Creates a CAN interaface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    #[cfg(not(feature = "connectivity"))]
    pub fn new<Pins>(
        _can: Instance,
        _pins: Pins,
        mapr: &mut MAPR,
        apb: &mut <Instance as RccBus>::Bus,
        _usb: USB,
    ) -> Can<Instance>
    where
        Pins: traits::Pins<CAN = Instance>,
    {
        Pins::remap(mapr);
        Self::new_internal(apb)
    }

    /// Creates a CAN interaface.
    #[cfg(feature = "connectivity")]
    pub fn new<Pins>(
        _can: Instance,
        _pins: Pins,
        mapr: &mut MAPR,
        apb: &mut <Instance as RccBus>::Bus,
    ) -> Can<Instance>
    where
        Pins: traits::Pins<CAN = Instance>,
    {
        Pins::remap(mapr);
        Self::new_internal(apb)
    }

    fn new_internal(apb: &mut <Instance as RccBus>::Bus) -> Can<Instance> {
        Instance::enable(apb);
        Can { _can: PhantomData }
    }

    /// Configures the bit timings.
    ///
    /// Use http://www.bittiming.can-wiki.info/ to calculate a safe parameter
    /// value. Puts the peripheral in sleep mode. `Can::enable()` must be called
    /// afterwards to start reception and transmission.
    pub unsafe fn set_bit_timing(&mut self, btr: u32) {
        let can = &*Instance::REGISTERS;

        can.mcr.modify(|_, w| w.sleep().clear_bit().inrq().set_bit());
        while can.msr.read().inak().bit_is_clear() {}
        can.btr.write(|w| w.bits(btr));
        self.sleep();
    }

    /// Start reception and transmission.
    ///
    /// Waits for 11 consecutive recessive bits to sync to the CAN bus.
    /// When automatic wakup functionality is not used this function must be
    /// called to enable the peripheral after a wakeup interrupt.
    pub fn enable(&mut self) -> nb::Result<(), Infallible> {
        let can = unsafe { &*Instance::REGISTERS };
        let msr = can.msr.read();
        if msr.slak().bit_is_set() {
            // TODO: Make automatic bus-off management configurable.
            // TODO: Make automatic wakeup configurable.
            can.mcr.modify(|_, w| w.abom().set_bit().sleep().clear_bit());
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    /// Puts the peripheral in a sleep mode to save power.
    ///
    /// Reception and transmission is disabled.
    pub fn sleep(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        can.mcr.modify(|_, w| w.sleep().set_bit().inrq().clear_bit());
        while can.msr.read().slak().bit_is_clear() {}
    }
}
