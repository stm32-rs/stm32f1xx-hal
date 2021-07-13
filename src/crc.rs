//! CRC

use crate::pac::{CRC, RCC};
use crate::rcc::Enable;

/// Extension trait to constrain the CRC peripheral
pub trait CrcExt {
    /// Constrains the CRC peripheral to play nicely with the other abstractions
    #[allow(clippy::wrong_self_convention, clippy::new_ret_no_self)]
    fn new(self) -> Crc;
}

impl CrcExt for CRC {
    fn new(self) -> Crc {
        let rcc = unsafe { &(*RCC::ptr()) };
        CRC::enable(rcc);

        Crc { crc: self }
    }
}

/// Constrained CRC peripheral
pub struct Crc {
    crc: CRC,
}

impl Crc {
    pub fn read(&self) -> u32 {
        self.crc.dr.read().bits()
    }

    pub fn write(&mut self, val: u32) {
        self.crc.dr.write(|w| w.dr().bits(val))
    }

    pub fn reset(&self) {
        self.crc.cr.write(|w| w.reset().set_bit());
        // calling CRC::dr::write() just after CRC::cr::reset() will not work as expected, and
        // inserting single nop() seems to solve the problem.
        cortex_m::asm::nop();
    }
}
