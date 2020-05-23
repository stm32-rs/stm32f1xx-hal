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
use crate::pac::CAN1;
#[cfg(feature = "connectivity")]
use crate::pac::CAN2;
#[cfg(not(feature = "connectivity"))]
use crate::pac::USB;
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
/// The `Ord` trait can be used to determine the frame’s priority this ID
/// belongs to.
/// Lower identifier values have a higher priority. Additionally standard frames
/// have a higher priority than extended frames and data frames have a higher
/// priority than remote frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Id(u32);

impl Id {
    const STANDARD_SHIFT: u32 = 21;
    const STANDARD_MASK: u32 = 0x7FF << Self::STANDARD_SHIFT;

    const EXTENDED_SHIFT: u32 = 3;
    const EXTENDED_MASK: u32 = 0x1FFF_FFFF << Self::EXTENDED_SHIFT;

    const IDE_MASK: u32 = 0x0000_0004;

    const RTR_MASK: u32 = 0x0000_0002;

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// IDs outside the allowed range are silently truncated.
    pub fn new_standard(id: u32) -> Self {
        assert!(id < 0x7FF);
        Self(id << Self::STANDARD_SHIFT)
    }

    /// Creates a new extendended identifier (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// IDs outside the allowed range are silently truncated.
    pub fn new_extended(id: u32) -> Id {
        assert!(id < 0x1FFF_FFFF);
        Self(id << Self::EXTENDED_SHIFT | Self::IDE_MASK)
    }

    fn from_register(reg: u32) -> Id {
        Self(reg & 0xFFFF_FFFE)
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    fn with_rtr(self, rtr: bool) -> Id {
        if rtr {
            Self(self.0 | Self::RTR_MASK)
        } else {
            Self(self.0 & !Self::RTR_MASK)
        }
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
        self.0 & Self::IDE_MASK != 0
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

    /// Creates a new frame with a standard identifier.
    pub fn new_standard(id: u32, data: &[u8]) -> Self {
        Self::new(Id::new_standard(id), data)
    }

    /// Creates a new frame with an extended identifier.
    pub fn new_extended(id: u32, data: &[u8]) -> Self {
        Self::new(Id::new_extended(id), data)
    }

    /// Marks the frame as a remote frame with configurable data length code (DLC).
    ///
    /// Remote frames do not contain any data, even if the frame was created with a
    /// non-empty data buffer.
    pub fn with_rtr(&mut self, dlc: usize) -> &mut Self {
        self.id = self.id.with_rtr(true);
        self.dlc = dlc;
        self
    }

    /// Returns true if this frame is an extended frame
    pub fn is_extended(&self) -> bool {
        self.id.is_extended()
    }

    /// Returns true if this frame is a standard frame
    pub fn is_standard(&self) -> bool {
        self.id.is_standard()
    }

    /// Returns true if this frame is a remote frame
    pub fn is_remote_frame(&self) -> bool {
        self.id.rtr()
    }

    /// Returns true if this frame is a data frame
    pub fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    /// Returns the frame identifier.
    pub fn id(&self) -> Id {
        self.id
    }

    /// Returns the data length code (DLC) which is in the range 0..8.
    ///
    /// For data frames the DLC value always matches the lenght of the data.
    /// Remote frames no not carry any data, yet the DLC can be greater than 0.
    pub fn dlc(&self) -> usize {
        self.dlc
    }

    /// Returns the frame data (0..8 bytes in length).
    pub fn data(&self) -> &[u8] {
        if self.is_data_frame() {
            &self.data[0..self.dlc]
        } else {
            &[]
        }
    }
}

// Ordering is based on the ID and can be used to sort frames by priority.
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

// The Equality traits compare the identifier and the data.
impl PartialEq for Frame {
    fn eq(&self, other: &Self) -> bool {
        self.id() == other.id() && self.data[0..self.dlc] == other.data[0..other.dlc]
    }
}

impl Eq for Frame {}

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

/// Number of supported filter banks.
#[cfg(not(feature = "connectivity"))]
pub const NUM_FILTER_BANKS: usize = 14;

/// Number of supported filter banks.
#[cfg(feature = "connectivity")]
pub const NUM_FILTER_BANKS: usize = 28;

/// Configuration proxy to be used with `Can::configure()`.
pub struct CanConfig<Instance> {
    _can: PhantomData<Instance>,
}

impl<Instance> CanConfig<Instance>
where
    Instance: traits::Instance,
{
    /// Configures the bit timings.
    ///
    /// Use http://www.bittiming.can-wiki.info/ to calculate the `btr` parameter.
    pub fn set_bit_timing(&mut self, btr: u32) {
        let can = unsafe { &*Instance::REGISTERS };
        can.btr.modify(|r, w| unsafe {
            let mode_bits = r.bits() & 0xC000_0000;
            w.bits(mode_bits | btr)
        });
    }

    /// Enables or disables loopback mode: Internally connects the TX and RX
    /// signals together.
    pub fn set_loopback(&mut self, enabled: bool) {
        let can = unsafe { &*Instance::REGISTERS };
        can.btr.modify(|_, w| w.lbkm().bit(enabled));
    }

    /// Enables or disables silent mode: Disconnects the TX signal from the pin.
    pub fn set_silent(&mut self, enabled: bool) {
        let can = unsafe { &*Instance::REGISTERS };
        can.btr.modify(|_, w| w.silm().bit(enabled));
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

    /// Configure bit timings and silent/loop-back mode.
    ///
    /// Acutal configuration happens on the `CanConfig` that is passed to the
    /// closure. It must be done this way because those configuration bits can
    /// only be set if the CAN controller is in a special init mode.
    /// Puts the peripheral in sleep mode afterwards. `Can::enable()` must be
    /// called to exit sleep mode and start reception and transmission.
    pub fn configure<F>(&mut self, f: F)
    where
        F: FnOnce(&mut CanConfig<Instance>),
    {
        let can = unsafe { &*Instance::REGISTERS };

        // Enter init mode.
        can.mcr
            .modify(|_, w| w.sleep().clear_bit().inrq().set_bit());
        while can.msr.read().inak().bit_is_clear() {}

        let mut config = CanConfig { _can: PhantomData };
        f(&mut config);

        // Leave init mode: go back to sleep.
        self.sleep();
    }

    /// Configures the automatic wake-up feature.
    pub fn set_automatic_wakeup(&mut self, enabled: bool) {
        let can = unsafe { &*Instance::REGISTERS };
        can.mcr.modify(|_, w| w.awum().bit(enabled));
    }

    /// Start reception and transmission.
    ///
    /// Waits for 11 consecutive recessive bits to sync to the CAN bus.
    pub fn enable(&mut self) -> nb::Result<(), Infallible> {
        let can = unsafe { &*Instance::REGISTERS };
        let msr = can.msr.read();
        if msr.slak().bit_is_set() {
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

    /// Enables the wake-up state change interrupt (CANn_SCE).
    ///
    /// Call `Can::enable()` in the ISR when the automatic wake-up is not enabled.
    pub fn enable_wakeup_interrupt(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        bb::set(&can.ier, 16); // WKUIE
    }

    /// Clears all state-change interrupt flags.
    pub fn clear_interrupt_flags(&mut self) {
        let can = unsafe { &*Instance::REGISTERS };
        can.msr.write(|w| w.wkui().set_bit());
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
        // Set all filter banks to 32bit scale and mask mode.
        // Filters are alternating between between the FIFO0 and FIFO1 to share the
        // load equally.
        self.split_filters_internal(0x0000_0000, 0xFFFF_FFFF, 0xAAAA_AAAA, NUM_FILTER_BANKS)?;
        Some(Filters::new(0, NUM_FILTER_BANKS))
    }

    /// Returns the filter part of the CAN peripheral.
    ///
    /// Filters are required for the receiver to accept any messages at all.
    /// `split_idx` can be in the range `0..NUM_FILTER_BANKS` and decides the number
    /// of filters assigned to each peripheral. A value of `0` means all filter
    /// banks are used for CAN2 while `NUM_FILTER_BANKS` reserves all filter banks
    /// for CAN1.
    #[cfg(feature = "connectivity")]
    pub fn split_filters(&mut self, split_idx: usize) -> Option<(Filters<CAN1>, Filters<CAN2>)> {
        // Set all filter banks to 32bit scale and mask mode.
        // Filters are alternating between between the FIFO0 and FIFO1 to share the
        // load equally.
        self.split_filters_internal(0x0000_0000, 0xFFFF_FFFF, 0xAAAA_AAAA, split_idx)?;
        Some((
            Filters::new(0, split_idx),
            Filters::new(split_idx, NUM_FILTER_BANKS),
        ))
    }

    /// Advanced version of `Can::split_filters()`.
    ///
    /// The additional parameters are bitmasks configure the filter banks.
    /// Bit 0 for filter bank 0, bit 1 for filter bank 1 and so on.
    /// `fm1r` in combination with `fs1r` sets the filter bank layout. The correct
    /// `Filters::add_*()` function must be used.
    /// `ffa1r` selects the FIFO the filter uses to store accepted messages.
    /// More details can be found in the reference manual of the device.
    #[cfg(not(feature = "connectivity"))]
    pub fn split_filters_advanced(
        &mut self,
        fm1r: u32,
        fs1r: u32,
        ffa1r: u32,
    ) -> Option<Filters<CAN1>> {
        self.split_filters_internal(fm1r, fs1r, ffa1r, NUM_FILTER_BANKS)?;
        Some(Filters::new(0, NUM_FILTER_BANKS))
    }

    /// Advanced version of `Can::split_filters()`.
    ///
    /// The additional parameters are bitmasks to configure the filter banks.
    /// Bit 0 for filter bank 0, bit 1 for filter bank 1 and so on.
    /// `fm1r` in combination with `fs1r` sets the filter bank layout. The correct
    /// `Filters::add_*()` function must be used.
    /// `ffa1r` selects the FIFO the filter uses to store accepted messages.
    /// More details can be found in the reference manual of the device.
    #[cfg(feature = "connectivity")]
    pub fn split_filters_advanced(
        &mut self,
        fm1r: u32,
        fs1r: u32,
        ffa1r: u32,
        split_idx: usize,
    ) -> Option<(Filters<CAN1>, Filters<CAN2>)> {
        self.split_filters_internal(fm1r, fs1r, ffa1r, split_idx)?;
        Some((
            Filters::new(0, split_idx),
            Filters::new(split_idx, NUM_FILTER_BANKS),
        ))
    }

    fn split_filters_internal(
        &mut self,
        fm1r: u32,
        fs1r: u32,
        ffa1r: u32,
        _split_idx: usize,
    ) -> Option<()> {
        let can = unsafe { &*CAN1::ptr() };

        if can.fmr.read().finit().bit_is_clear() {
            return None;
        }

        can.fm1r.write(|w| unsafe { w.bits(fm1r) });
        can.fs1r.write(|w| unsafe { w.bits(fs1r) });
        can.ffa1r.write(|w| unsafe { w.bits(ffa1r) });

        // Init filter banks. Each used filter must still be enabled individually.
        #[allow(unused_unsafe)]
        can.fmr.modify(|_, w| unsafe {
            #[cfg(feature = "connectivity")]
            w.can2sb().bits(_split_idx as u8);
            w.finit().clear_bit()
        });

        Some(())
    }
}

/// Filter with an optional mask.
pub struct Filter {
    id: u32,
    mask: u32,
}

impl Filter {
    /// Creates a filter that accepts all messages.
    pub fn accept_all() -> Self {
        Self { id: 0, mask: 0 }
    }

    /// Creates a filter that accepts frames with the specified standard identifier.
    pub fn new_standard(id: u32) -> Self {
        Self {
            id: id << Id::STANDARD_SHIFT,
            mask: Id::STANDARD_MASK | Id::IDE_MASK | Id::RTR_MASK,
        }
    }

    /// Creates a filter that accepts frames with the extended standard identifier.
    pub fn new_extended(id: u32) -> Self {
        Self {
            id: id << Id::EXTENDED_SHIFT | Id::IDE_MASK,
            mask: Id::EXTENDED_MASK | Id::IDE_MASK | Id::RTR_MASK,
        }
    }

    /// Only look at the bits of the indentifier which are set to 1 in the mask.
    ///
    /// A mask of 0 accepts all identifiers.
    pub fn with_mask(&mut self, mask: u32) -> &mut Self {
        if self.is_extended() {
            self.mask = (self.mask & !Id::EXTENDED_MASK) | (mask << Id::EXTENDED_SHIFT);
        } else {
            self.mask = (self.mask & !Id::STANDARD_MASK) | (mask << Id::STANDARD_SHIFT);
        }
        self
    }

    /// Makes this filter accept both data and remote frames.
    pub fn allow_remote(&mut self) -> &mut Self {
        self.mask &= !Id::RTR_MASK;
        self
    }

    /// Makes this filter accept only remote frames.
    pub fn only_remote(&mut self) -> &mut Self {
        self.id |= Id::RTR_MASK;
        self.mask |= Id::RTR_MASK;
        self
    }

    fn is_extended(&self) -> bool {
        self.id & Id::IDE_MASK != 0
    }

    fn matches_single_id(&self) -> bool {
        ((self.mask & (Id::IDE_MASK | Id::RTR_MASK)) == (Id::IDE_MASK | Id::RTR_MASK))
            && if self.is_extended() {
                (self.mask & Id::EXTENDED_MASK) == Id::EXTENDED_MASK
            } else {
                (self.mask & Id::STANDARD_MASK) == Id::STANDARD_MASK
            }
    }

    fn reg_to_16bit(reg: u32) -> u32 {
        (reg & Id::STANDARD_MASK) >> 16 | (reg & Id::IDE_MASK) << 1 | (reg & Id::RTR_MASK) << 3
    }

    fn id_to_16bit(&self) -> u32 {
        Self::reg_to_16bit(self.id)
    }

    fn mask_to_16bit(&self) -> u32 {
        Self::reg_to_16bit(self.mask)
    }
}

/// Interface to the filter banks of a CAN peripheral.
pub struct Filters<Instance> {
    start_idx: usize,
    stop_idx: usize,
    count: usize,
    _can: PhantomData<Instance>,
}

impl<Instance> Filters<Instance>
where
    Instance: traits::Instance,
{
    fn new(start_idx: usize, stop_idx: usize) -> Self {
        Self {
            start_idx,
            stop_idx,
            count: 0,
            _can: PhantomData,
        }
    }

    /// Returns the number of available filters.
    ///
    /// This can number can be larger than the number of filter banks if
    /// `Can::split_filters_advanced()` was used.
    pub fn num_available(&self) -> usize {
        let can = unsafe { &*CAN1::ptr() };

        let mut filter_count = self.stop_idx - self.start_idx;

        let owned_bits = ((1 << filter_count) - 1) << self.start_idx;
        let mode_list = can.fm1r.read().bits() & owned_bits;
        let scale_16bit = !can.fs1r.read().bits() & owned_bits;

        filter_count += mode_list.count_ones() as usize;
        filter_count += scale_16bit.count_ones() as usize;
        filter_count += (mode_list & scale_16bit).count_ones() as usize;
        filter_count
    }

    /// Adds a filter. Returns `Err` if the maximum number of filters was reached.
    pub fn add(&mut self, filter: &Filter) -> Result<(), ()> {
        let can = unsafe { &*CAN1::ptr() };

        let idx = self.start_idx + self.count;
        if idx >= self.stop_idx {
            return Err(());
        }

        let mode_list = (can.fm1r.read().bits() & (1 << idx)) != 0;
        let scale_16bit = (can.fs1r.read().bits() & (1 << idx)) == 0;
        let bank_enabled = (can.fa1r.read().bits() & (1 << idx)) != 0;

        // Make sure the filter is supported by the filter bank configuration.
        if (mode_list && !filter.matches_single_id()) || (scale_16bit && filter.is_extended()) {
            return Err(());
        }

        // Disable the filter bank so it can be modified.
        bb::clear(&can.fa1r, idx as u8);

        let filter_bank = &can.fb[idx];
        let fr1 = filter_bank.fr1.read().bits();
        let fr2 = filter_bank.fr2.read().bits();
        let (fr1, fr2) = match (mode_list, scale_16bit, bank_enabled) {
            // 29bit id + mask
            (false, false, _) => {
                self.count += 1;
                (filter.id, filter.mask)
            }
            // 2x 29bit id
            (true, false, false) => (filter.id, filter.id),
            (true, false, true) => {
                self.count += 1;
                (fr1, filter.id)
            }
            // 2x 11bit id + mask
            (false, true, false) => (
                filter.mask_to_16bit() << 16 | filter.id_to_16bit(),
                filter.mask_to_16bit() << 16 | filter.id_to_16bit(),
            ),
            (false, true, true) => {
                self.count += 1;
                (fr1, filter.mask_to_16bit() << 16 | filter.id_to_16bit())
            }
            // 4x 11bit id
            (true, true, false) => (
                filter.id_to_16bit() << 16 | filter.id_to_16bit(),
                filter.id_to_16bit() << 16 | filter.id_to_16bit(),
            ),
            (true, true, true) => {
                let f = [fr1 & 0xFFFF, fr1 >> 16, fr2 & 0xFFFF, fr2 >> 16];

                if f[0] == f[1] {
                    // One filter available, add the second.
                    (filter.id_to_16bit() << 16 | f[0], fr2)
                } else if f[0] == f[2] {
                    // Two filters available, add the third.
                    (fr1, f[0] << 16 | filter.id_to_16bit())
                } else if f[0] == f[3] {
                    // Three filters available, add the fourth.
                    self.count += 1;
                    (fr1, filter.id_to_16bit() << 16 | f[2])
                } else {
                    unreachable!()
                }
            }
        };

        filter_bank.fr1.write(|w| unsafe { w.bits(fr1) });
        filter_bank.fr2.write(|w| unsafe { w.bits(fr2) });
        bb::set(&can.fa1r, idx as u8); // Enable the filter bank
        Ok(())
    }

    /// Disables all enabled filter banks.
    pub fn clear(&mut self) {
        let can = unsafe { &*CAN1::ptr() };

        assert!(self.start_idx + self.count <= self.stop_idx);
        for i in self.start_idx..(self.start_idx + self.count) {
            // Bitbanding required because the filters are shared between CAN1 and CAN2
            bb::clear(&can.fa1r, i as u8);
        }
        self.count = 0;
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
    /// Transmit order is preserved for frames with of identifiers.
    /// If all transmit mailboxes are full, a higher priority frame replaces the
    /// lowest priority frame, which is returned as `Ok(Some(frame))`.
    pub fn transmit(&mut self, frame: &Frame) -> nb::Result<Option<Frame>, Infallible> {
        let can = unsafe { &*Instance::REGISTERS };

        // Get the index of the next free mailbox or the one with the lowest priority.
        let tsr = can.tsr.read();
        let idx = tsr.code().bits() as usize;

        let frame_is_pending =
            tsr.tme0().bit_is_clear() || tsr.tme1().bit_is_clear() || tsr.tme2().bit_is_clear();
        let pending_frame = if frame_is_pending {
            // High priority frames are transmitted first by the mailbox system.
            // Frames with identical identifier shall be transmitted in FIFO order.
            // The controller schedules pending frames of same priority based on the
            // mailbox index instead. As a workaround check all pending mailboxes
            // and only accept higher priority frames.
            self.check_priority(0, frame.id)?;
            self.check_priority(1, frame.id)?;
            self.check_priority(2, frame.id)?;

            let all_frames_are_pending =
                tsr.tme0().bit_is_clear() && tsr.tme1().bit_is_clear() && tsr.tme2().bit_is_clear();
            if all_frames_are_pending {
                // No free mailbox is available. This can only happen when three frames with
                // descending priority were requested for transmission and all of them are
                // blocked by bus traffic with even higher priority.
                // To prevent a priority inversion abort and replace the lowest priority frame.
                self.read_pending_mailbox(idx)
            } else {
                // There was a free mailbox.
                None
            }
        } else {
            // All mailboxes are available: Send frame without performing any checks.
            None
        };

        self.write_mailbox(idx, frame);
        Ok(pending_frame)
    }

    /// Returns `Ok` when the mailbox is free or has a lower priority than
    /// identifier than `id`.
    fn check_priority(&self, idx: usize, id: Id) -> nb::Result<(), Infallible> {
        let can = unsafe { &*Instance::REGISTERS };

        // Read the pending frame's id to check its priority.
        assert!(idx < 3);
        let tir = &can.tx[idx].tir.read();

        // Check the priority by comparing the identifiers. But first make sure the
        // frame has not finished transmission (`TXRQ` == 0) in the meantime.
        if tir.txrq().bit_is_set() && id >= Id::from_register(tir.bits()) {
            // There's a mailbox whose priority is higher or equal
            // the priority of the new frame.
            return Err(nb::Error::WouldBlock);
        }

        Ok(())
    }

    fn write_mailbox(&mut self, idx: usize, frame: &Frame) {
        let can = unsafe { &*Instance::REGISTERS };

        debug_assert!(idx < 3);
        let mb = unsafe { &can.tx.get_unchecked(idx) };

        mb.tdtr.write(|w| unsafe { w.dlc().bits(frame.dlc as u8) });
        mb.tdlr
            .write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[0..4].try_into().unwrap())) });
        mb.tdhr
            .write(|w| unsafe { w.bits(u32::from_ne_bytes(frame.data[4..8].try_into().unwrap())) });
        mb.tir
            .write(|w| unsafe { w.bits(frame.id.0).txrq().set_bit() });
    }

    fn read_pending_mailbox(&mut self, idx: usize) -> Option<Frame> {
        let can = unsafe { &*Instance::REGISTERS };

        debug_assert!(idx < 3);
        let mb = unsafe { &can.tx.get_unchecked(idx) };

        if self.abort(idx) {
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

    /// Tries to abort a pending frame. Returns `true` when aborted.
    fn abort(&mut self, idx: usize) -> bool {
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

    /// Returns `true` if no frame is pending for transmission.
    pub fn is_idle(&self) -> bool {
        let can = unsafe { &*Instance::REGISTERS };
        let tsr = can.tsr.read();
        tsr.tme0().bit_is_set() && tsr.tme1().bit_is_set() && tsr.tme2().bit_is_set()
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
    ///
    /// Returns `Err` when a frame was lost due to buffer overrun.
    pub fn receive(&mut self) -> nb::Result<Frame, ()> {
        match self.receive_fifo(0) {
            Err(nb::Error::WouldBlock) => self.receive_fifo(1),
            result => result,
        }
    }

    fn receive_fifo(&mut self, fifo_nr: usize) -> nb::Result<Frame, ()> {
        let can = unsafe { &*Instance::REGISTERS };

        assert!(fifo_nr < 2);
        let rfr = &can.rfr[fifo_nr];
        let rx = &can.rx[fifo_nr];

        // Check if a frame is available in the mailbox.
        let rfr_read = rfr.read();
        if rfr_read.fmp().bits() == 0 {
            return Err(nb::Error::WouldBlock);
        }

        // Check for RX FIFO overrun.
        if rfr_read.fovr().bit_is_set() {
            rfr.write(|w| w.fovr().set_bit());
            return Err(nb::Error::Other(()));
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
