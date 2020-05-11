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
use crate::bb;
#[cfg(feature = "connectivity")]
use crate::gpio::gpiob::{PB12, PB13, PB5, PB6};
use crate::gpio::{
    gpioa::{PA11, PA12},
    gpiob::{PB8, PB9},
    Alternate, Floating, Input, PushPull,
};
#[cfg(feature = "connectivity")]
use crate::pac::CAN2;
#[cfg(not(feature = "connectivity"))]
use crate::pac::USB;
use crate::pac::{
    can1::{RFR, RX, TX},
    CAN1,
};
use crate::rcc::APB1;
use core::{
    cmp::{Ord, Ordering},
    convert::{Infallible, TryInto},
    marker::PhantomData,
};

/// Identifier of a CAN message.
///
/// Can be either a standard identifier (11bit, Range: 0..0x3FF) or a
/// extendended identifier (29bit , Range: 0..0x1FFFFFFF).
///
/// The `Ord` trait is can be used to determine the frame’s priority this ID
/// belongs to. This works because the EID and RTR flags are included in the
/// underlying integer representaiton.
/// Lower identifier values mean higher priority. Additionally standard frames
/// have a higher priority than extended frames and data frames have a higher
/// priority than remote frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

    fn from_register(reg: u32) -> Id {
        Self(reg & 0xFFFF_FFFE)
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    fn with_rtr(self) -> Id {
        Self(self.0 | Self::RTR_MASK)
    }

    /// Returns the identifier.
    ///
    /// It is up to the user to check if it is an standard or extended id.
    pub fn as_u32(self) -> u32 {
        if self.is_extended() {
            self.0 >> Self::EXTENDED_SHIFT
        } else {
            self.0 >> Self::STANDARD_SHIFT
        }
    }

    /// Returns `true` if the identifier is an extended identifier.
    pub fn is_extended(self) -> bool {
        self.0 & Self::EID_MASK != 0
    }

    /// Returns `true` if the identifier is a standard identifier.
    pub fn is_standard(self) -> bool {
        !self.is_extended()
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    fn rtr(self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
}

impl Ord for Id {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self.is_standard(), other.is_standard()) {
            (true, false) => Ordering::Less,
            (false, true) => Ordering::Greater,
            // Ordering of the data/remote frames implicitly gives by the bit layout.
            _ => self.0.cmp(&other.0),
        }
    }
}

impl PartialOrd for Id {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A CAN data or remote frame.
#[derive(Clone, Debug)]
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
    pub fn id(&self) -> Id {
        self.id
    }

    /// Returns the frame data (0..8 bytes in length).
    pub fn data(&self) -> &[u8] {
        &self.data[0..self.dlc]
    }
}

impl Ord for Frame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.id().cmp(&other.id())
    }
}

impl PartialOrd for Frame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Frame {
    fn eq(&self, other: &Self) -> bool {
        self.id() == other.id()
    }
}

impl Eq for Frame {}

// Seal the traits so that they cannot be implemented outside side this crate.
mod traits {
    pub trait Instance: crate::rcc::Enable {
        const REGISTERS: *const crate::pac::can1::RegisterBlock;
        const FILTER_BANK_START: usize;
        const FILTER_BANK_STOP: usize;
    }

    pub trait Pins {
        type CAN: Instance;
        fn remap(mapr: &mut super::MAPR);
    }
}

impl traits::Instance for CAN1 {
    const REGISTERS: *const crate::pac::can1::RegisterBlock = CAN1::ptr();
    const FILTER_BANK_START: usize = 0;
    const FILTER_BANK_STOP: usize = 14;
}

#[cfg(feature = "connectivity")]
impl traits::Instance for CAN2 {
    // Register blocks are the same except for the filter registers.
    // Those are only available on CAN1.
    const REGISTERS: *const crate::pac::can1::RegisterBlock = CAN2::ptr() as *const _;
    const FILTER_BANK_START: usize = CAN1::FILTER_BANK_STOP;
    const FILTER_BANK_STOP: usize = 28;
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
    tx: Option<Tx<Instance>>,
    rx: Option<Rx<Instance>>,
}

impl<Instance> Can<Instance>
where
    Instance: traits::Instance<Bus = APB1>,
{
    /// Creates a CAN interaface.
    ///
    /// CAN shares SRAM with the USB peripheral. Take ownership of USB to
    /// prevent accidental shared usage.
    #[cfg(not(feature = "connectivity"))]
    pub fn new(_can: Instance, apb: &mut APB1, _usb: USB) -> Can<Instance> {
        Self::new_internal(apb)
    }

    /// Creates a CAN interaface.
    #[cfg(feature = "connectivity")]
    pub fn new(_can: Instance, apb: &mut APB1) -> Can<Instance> {
        Self::new_internal(apb)
    }

    fn new_internal(apb: &mut APB1) -> Can<Instance> {
        Instance::enable(apb);

        Can {
            _can: PhantomData,
            tx: Some(Tx { _can: PhantomData }),
            rx: Some(Rx { _can: PhantomData }),
        }
    }

    /// Routes CAN TX signals and RX signals to pins.
    pub fn assign_pins<Pins>(&self, _pins: Pins, mapr: &mut MAPR)
    where
        Pins: traits::Pins<CAN = Instance>,
    {
        Pins::remap(mapr);
    }

    /// Configures the bit timings.
    ///
    /// Use http://www.bittiming.can-wiki.info/ to calculate the `btr` parameter.
    /// Puts the peripheral in sleep mode. `Can::enable()` must be called afterwards
    /// to start reception and transmission.
    pub fn set_bit_timing(&mut self, btr: u32) {
        let can = unsafe { &*Instance::REGISTERS };

        can.mcr
            .modify(|_, w| w.sleep().clear_bit().inrq().set_bit());
        while can.msr.read().inak().bit_is_clear() {}
        can.btr.write(|w| unsafe { w.bits(btr) });
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
            can.mcr
                .modify(|_, w| w.abom().set_bit().sleep().clear_bit());
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
        can.mcr
            .modify(|_, w| w.sleep().set_bit().inrq().clear_bit());
        while can.msr.read().slak().bit_is_clear() {}
    }

    /// Returns the transmitter interface.
    ///
    /// Only the first calls returns a valid transmitter. Subsequent calls
    /// return `None`.
    pub fn take_tx(&mut self) -> Option<Tx<Instance>> {
        self.tx.take()
    }

    /// Returns the receiver interface.
    ///
    /// Takes ownership of filters which must be otained by `Can::split_filters()`.
    /// Only the first calls returns a valid receiver. Subsequent calls return `None`.
    pub fn take_rx(&mut self, _filters: Filters<Instance>) -> Option<Rx<Instance>> {
        self.rx.take()
    }
}

impl Can<CAN1> {
    /// Returns the filter part of the CAN peripheral.
    ///
    /// Filters are required for the receiver to accept any messages at all.
    #[cfg(not(feature = "connectivity"))]
    pub fn split_filters(&mut self) -> Option<Filters<CAN1>> {
        self.split_filters_internal()?;
        Some(Filters(PhantomData))
    }

    /// Returns the filter part of the CAN peripheral.
    ///
    /// Filters are required for the receiver to accept any messages at all.
    #[cfg(feature = "connectivity")]
    pub fn split_filters(&mut self) -> Option<(Filters<CAN1>, Filters<CAN2>)> {
        self.split_filters_internal()?;
        Some((Filters(PhantomData), Filters(PhantomData)))
    }

    fn split_filters_internal(&mut self) -> Option<()> {
        let can = unsafe { &*CAN1::ptr() };

        if can.fmr.read().finit().bit_is_clear() {
            return None;
        }

        // Same configuration for all filters banks.
        can.fm1r.write(|w| unsafe { w.bits(0x0000_0000) }); // Mask mode
        can.fs1r.write(|w| unsafe { w.bits(0xFFFF_FFFF) }); // 32bit scale

        // Filters are alternating between between the FIFO0 and FIFO1.
        can.ffa1r.write(|w| unsafe { w.bits(0xAAAA_AAAA) });

        // Init filter banks. Each used filter must still be enabled individually.
        #[allow(unused_unsafe)]
        can.fmr.modify(|_, w| unsafe {
            #[cfg(feature = "connectivity")]
            w.can2sb()
                .bits(<CAN1 as traits::Instance>::FILTER_BANK_STOP as u8);
            w.finit().clear_bit()
        });

        Some(())
    }
}

/// Interface to the filter banks of a CAN peripheral.
pub struct Filters<Instance>(PhantomData<Instance>);

impl<Instance> Filters<Instance>
where
    Instance: traits::Instance,
{
    /// Enables a filter that accepts all messages.
    pub fn accept_all(&mut self) {
        let can = unsafe { &*CAN1::ptr() };

        // For more details on the quirks of bxCAN see:
        // https://github.com/UAVCAN/libcanard/blob/8ee343c4edae0e0e4e1c040852aa3d8430f7bf76/drivers/stm32/canard_stm32.c#L471-L513
        let idx = Instance::FILTER_BANK_START;
        can.fb[idx].fr1.write(|w| unsafe { w.bits(0xFFFF_FFFE) });
        can.fb[idx].fr2.write(|w| unsafe { w.bits(0) });
        bb::set(&can.fa1r, idx as u8);
    }
}

/// Interface to the CAN transmitter part.
pub struct Tx<Instance> {
    _can: PhantomData<Instance>,
}

const fn ok_mask(idx: usize) -> u32 {
    0x02 << (8 * idx)
}

const fn abort_mask(idx: usize) -> u32 {
    0x80 << (8 * idx)
}

impl<Instance> Tx<Instance>
where
    Instance: traits::Instance,
{
    /// Puts a CAN frame in a free transmit mailbox for transmission on the bus.
    ///
    /// Frames are transmitted to the bus based on their priority (identifier).
    /// Transmit order is preserved for frames with identical identifiers.
    /// If all transmit mailboxes are full a higher priority frame replaces the
    /// lowest priority frame which is returned as `Ok(Some(frame))`.
    pub fn transmit(&mut self, frame: &Frame) -> nb::Result<Option<Frame>, Infallible> {
        let can = unsafe { &*Instance::REGISTERS };

        // Get the index of free mailbox or of one with the lowest priority.
        let tsr = can.tsr.read();
        let idx = tsr.code().bits() as usize;
        let mb = unsafe { &can.tx.get_unchecked(idx) };

        let empty_flags = (tsr.bits() >> 26) & 0b111;
        let pending_frame = if empty_flags == 0b111 {
            // All mailboxes are available: Send frame without performing any checks.
            None
        } else {
            // High priority frames are transmitted first by the mailbox system.
            // Frames with identical identifier shall be transmitted in FIFO order.
            // The controller schedules pending frames of same priority based on the
            // mailbox index instead. As a workaround check all pending mailboxes
            // and only accept higher priority frames.
            Self::check_priority(&can.tx[0], frame.id)?;
            Self::check_priority(&can.tx[1], frame.id)?;
            Self::check_priority(&can.tx[2], frame.id)?;

            if empty_flags != 0b000 {
                // There was a free mailbox.
                None
            } else {
                // No free mailbox is available. This can only happen when three frames with
                // descending priority were requested for transmission and all of them are
                // blocked by bus traffic with even higher priority.
                // To prevent a priority inversion abort and replace the lowest priority frame.
                if Self::abort(idx) {
                    // Read back the pending frame.
                    let mut pending_frame = Frame {
                        id: Id(mb.tir.read().bits()),
                        dlc: mb.tdtr.read().dlc().bits() as usize,
                        data: [0; 8],
                    };
                    pending_frame.data[0..4].copy_from_slice(&mb.tdlr.read().bits().to_ne_bytes());
                    pending_frame.data[4..8].copy_from_slice(&mb.tdhr.read().bits().to_ne_bytes());

                    Some(pending_frame)
                } else {
                    // Abort request failed because the frame was already sent (or being sent) on
                    // the bus. All mailboxes are now free. This can happen for small prescaler
                    // values (e.g. 1MBit/s bit timing with a source clock of 8MHz) or when an ISR
                    // has preemted the execution.
                    None
                }
            }
        };

        Self::write_tx_mailbox(mb, frame);
        Ok(pending_frame)
    }

    /// Tries to abort a pending frame. Returns `true` when aborted.
    fn abort(idx: usize) -> bool {
        let can = unsafe { &*Instance::REGISTERS };

        can.tsr.write(|w| unsafe { w.bits(abort_mask(idx)) });

        // Wait for the abort request to be finished.
        loop {
            let tsr = can.tsr.read().bits();
            if tsr & abort_mask(idx) == 0 {
                break tsr & ok_mask(idx) == 0;
            }
        }
    }

    /// Returns `Ok` when the mailbox is free or has a lower priority than
    /// identifier than `id`.
    fn check_priority(mb: &TX, id: Id) -> nb::Result<(), Infallible> {
        // Read the pending frame's id to check its priority.
        let tir = mb.tir.read();

        // Check the priority by comparing the identifiers. But first make sure the
        // frame has not finished transmission (`TXRQ` == 0) in the meantime.
        if tir.txrq().bit_is_set() && id >= Id::from_register(tir.bits()) {
            // There's a mailbox whose priority is higher or equal
            // the priority of the new frame.
            return Err(nb::Error::WouldBlock);
        }

        Ok(())
    }

    fn write_tx_mailbox(tx_mb: &crate::pac::can1::TX, frame: &Frame) {
        tx_mb
            .tdtr
            .write(|w| unsafe { w.dlc().bits(frame.dlc as u8) });
        tx_mb
            .tdlr
            .write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[0..4].try_into().unwrap())) });
        tx_mb
            .tdhr
            .write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[4..8].try_into().unwrap())) });
        tx_mb
            .tir
            .write(|w| unsafe { w.bits(frame.id.0).txrq().set_bit() });
    }

    /// Enables the transmit interrupt CANn_TX.
    ///
    /// The interrupt flags must be cleared with `Tx::clear_interrupt_flags()`.
    pub fn enable_interrupt(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        bb::set(&can.ier, 0); // TMEIE
    }

    /// Disables the transmit interrupt.
    pub fn disable_interrupt(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        bb::clear(&can.ier, 0); // TMEIE
    }

    /// Clears the request complete flag for all mailboxes.
    pub fn clear_interrupt_flags(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        can.tsr
            .write(|w| w.rqcp2().set_bit().rqcp1().set_bit().rqcp0().set_bit());
    }
}

/// Interface to the CAN receiver part.
pub struct Rx<Instance> {
    _can: PhantomData<Instance>,
}

impl<Instance> Rx<Instance>
where
    Instance: traits::Instance,
{
    /// Returns a received frame if available.
    pub fn receive(&mut self) -> nb::Result<Frame, Infallible> {
        let can = unsafe { &*Instance::REGISTERS };

        Self::receive_fifo(&can.rfr[0], &can.rx[0])
            .or_else(|_| Self::receive_fifo(&can.rfr[1], &can.rx[1]))
    }

    fn receive_fifo(rfr: &RFR, rx: &RX) -> nb::Result<Frame, Infallible> {
        // Check if a frame is available in the mailbox.
        if rfr.read().fmp().bits() == 0 {
            return Err(nb::Error::WouldBlock);
        }

        // Read the frame.
        let mut frame = Frame {
            id: Id(rx.rir.read().bits()),
            dlc: rx.rdtr.read().dlc().bits() as usize,
            data: [0; 8],
        };
        frame.data[0..4].copy_from_slice(&rx.rdlr.read().bits().to_ne_bytes());
        frame.data[4..8].copy_from_slice(&rx.rdhr.read().bits().to_ne_bytes());

        // Release the mailbox.
        rfr.write(|w| w.rfom().set_bit());

        Ok(frame)
    }

    /// Enables the receive interrupts CANn_RX0 and CANn_RX1.
    ///
    /// Make sure to register interrupt handlers for both.
    /// The interrupt flags are cleared by reading frames with `Rx::receive()`.
    pub fn enable_interrupts(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        bb::set(&can.ier, 1); // FMPIE0
        bb::set(&can.ier, 4); // FMPIE1
    }

    /// Disables the receive interrupts.
    pub fn disable_interrupts(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        bb::clear(&can.ier, 1); // FMPIE0
        bb::clear(&can.ier, 4); // FMPIE1
    }
}
