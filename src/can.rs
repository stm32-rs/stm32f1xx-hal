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
