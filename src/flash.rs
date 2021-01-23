//! Flash memory

// See ST document PM0075 for more information

use crate::pac::{flash, FLASH};

pub const FLASH_START: u32 = 0x0800_0000;
pub const FLASH_END: u32 = 0x080F_FFFF;

const OPT_BYTES_BASE: u32 = 0x1FFF_F800;
const RDPRT_KEY: u8 = 0xA5;
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

pub const SZ_1K: u16 = 1024;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum Error {
    AddressLargerThanFlash,
    AddressMisaligned,
    LengthNotMultiple2,
    LengthTooLong,
    EraseError,
    ProgrammingError,
    WriteError,
    VerifyError,
    UnlockError,
    UnlockOptError,
    LockError,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum SectorSize {
    Sz1K = 1,
    Sz2K = 2,
    Sz4K = 4,
}
impl SectorSize {
    const fn kbytes(self) -> u16 {
        SZ_1K * self as u16
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum FlashSize {
    Sz16K = 16,
    Sz32K = 32,
    Sz64K = 64,
    Sz128K = 128,
    Sz256K = 256,
    Sz384K = 384,
    Sz512K = 512,
    Sz768K = 768,
    Sz1M = 1024,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum ProtectionStatus {
    Unprotected,
    Read,
    Write,
    ReadWrite,
}

impl FlashSize {
    const fn kbytes(self) -> u32 {
        SZ_1K as u32 * self as u32
    }
}

pub struct FlashWriter<'a> {
    flash: &'a mut Parts,
    sector_sz: SectorSize,
    flash_sz: FlashSize,
    verify: bool,
}
impl<'a> FlashWriter<'a> {
    fn unlock(&mut self) -> Result<()> {
        // Wait for any ongoing operations
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // NOTE(unsafe) write Keys to the key register. This is safe because the
        // only side effect of these writes is to unlock the flash control
        // register, which is the intent of this function. Do not rearrange the
        // order of these writes or the control register will be permanently
        // locked out until reset.
        unsafe {
            self.flash.keyr.keyr().write(|w| w.key().bits(KEY1));
        }
        unsafe {
            self.flash.keyr.keyr().write(|w| w.key().bits(KEY2));
        }

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_clear() {
            true => Ok(()),
            false => Err(Error::UnlockError),
        }
    }

    fn lock(&mut self) -> Result<()> {
        //Wait for ongoing flash operations
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Set lock bit
        self.flash.cr.cr().modify(|_, w| w.lock().set_bit());

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_set() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn unlock_opt(&mut self) -> Result<()> {
        // First we must unlock the FPEC
        self.unlock()?;

        // Wait for any ongoing operations
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // NOTE(unsafe) write Keys to the OPTKEYR register. This is safe because the
        // only side effect of these writes is to unlock the option bytes
        // register, which is the intent of this function. Do not rearrange the
        // order of these writes.
        self.flash
            .optkeyr
            .optkeyr()
            .write(|w| unsafe { w.optkey().bits(KEY1) });

        self.flash
            .optkeyr
            .optkeyr()
            .write(|w| unsafe { w.optkey().bits(KEY2) });

        // Verify success
        if self.flash.cr.cr().read().optwre().bit_is_set() {
            Ok(())
        } else {
            Err(Error::UnlockOptError)
        }
    }

    fn lock_opt(&mut self) -> Result<()> {
        // Reset the OPTWRE bit before relocking flash generally
        self.flash.cr.cr().write(|w| w.optwre().clear_bit());
        self.lock()
    }

    /// Erases the Option Bytes. Requires Option Bytes registers to be unlocked.
    fn erase_opt(&mut self) -> Result<u32> {
        // Ensure OPTWRE is set
        if self.flash.cr.cr().read().optwre().bit_is_set() {
            let orig_val = unsafe { *(OPT_BYTES_BASE as *const u32) };
            // Erase the option bytes and wait
            self.flash.cr.cr().modify(|_, w| w.opter().set_bit());
            self.flash.cr.cr().modify(|_, w| w.strt().set_bit());
            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // Clear EOP bit if set
            if self.flash.sr.sr().read().eop().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.eop().clear_bit());
            }
            // Clear the OPTER bit
            self.flash.cr.cr().modify(|_, w| w.opter().clear_bit());
            Ok(orig_val)
        } else {
            Err(Error::UnlockOptError)
        }
    }

    fn set_rdp(&mut self, val: u8) -> Result<()> {
        // First we need to unlock the Option Byte programming
        self.unlock_opt()?;

        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // The documentation is somewhat vague here, but if we don't
        // first erase the Option Bytes, the subsequent write fails with
        // a set PGERR bit.
        self.erase_opt()?;

        // Now set the OPTPG bit to enable write access to Option Bytes
        self.flash.cr.cr().modify(|_, w| w.optpg().set_bit());

        let intended_rdp = val as u16;
        let opt_ptr = OPT_BYTES_BASE as *mut u16;
        unsafe {
            core::ptr::write_volatile(opt_ptr, intended_rdp);
        }

        // // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Clear OPTPG bit
        self.flash.cr.cr().modify(|_, w| w.optpg().clear_bit());

        // Check for an error condition
        if self.flash.sr.sr().read().pgerr().bit_is_set() {
            // Clear the error bit
            self.flash.sr.sr().modify(|_, w| w.pgerr().clear_bit());
            // Before returning, re-lock flash.
            self.lock_opt()?;
            return Err(Error::ProgrammingError);
        }

        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Now we should re-lock the Option Bytes. NOTE: We intentionally
        // do not do this before checking PEGERR in case of side-effects.
        self.lock_opt()?;

        // If the operation was successful, the lower half-word of Option Bytes
        // should be composed of our RDP value and its complement as the lower and
        // upper bytes respectively.
        if unsafe { *opt_ptr } == intended_rdp | !(intended_rdp & 0xFF) << 8 {
            Ok(())
        } else {
            Err(Error::ProgrammingError)
        }
    }

    fn valid_address(&self, offset: u32) -> Result<()> {
        if FLASH_START + offset > FLASH_END {
            Err(Error::AddressLargerThanFlash)
        } else if offset & 0x1 != 0 {
            Err(Error::AddressMisaligned)
        } else {
            Ok(())
        }
    }

    fn valid_length(&self, offset: u32, length: usize) -> Result<()> {
        if offset + length as u32 > self.flash_sz.kbytes() as u32 {
            Err(Error::LengthTooLong)
        } else if length & 0x1 != 0 {
            Err(Error::LengthNotMultiple2)
        } else {
            Ok(())
        }
    }

    /// Erase sector which contains `start_offset`
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        // Unlock Flash
        self.unlock()?;

        // Set Page Erase
        self.flash.cr.cr().modify(|_, w| w.per().set_bit());

        // Write address bits
        // NOTE(unsafe) This sets the page address in the Address Register.
        // The side-effect of this write is that the page will be erased when we
        // set the STRT bit in the CR below. The address is validated by the
        // call to self.valid_address() above.
        unsafe {
            self.flash
                .ar
                .ar()
                .write(|w| w.far().bits(FLASH_START + start_offset));
        }

        // Start Operation
        self.flash.cr.cr().modify(|_, w| w.strt().set_bit());

        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        // Check for errors
        let sr = self.flash.sr.sr().read();

        // Remove Page Erase Operation bit
        self.flash.cr.cr().modify(|_, w| w.per().clear_bit());

        // Re-lock flash
        self.lock()?;

        if sr.wrprterr().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.wrprterr().clear_bit());
            Err(Error::EraseError)
        } else {
            if self.verify {
                // By subtracting 1 from the sector size and masking with
                // start_offset, we make 'start' point to the beginning of the
                // page. We do this because the entire page should have been
                // erased, regardless of where in the page the given
                // 'start_offset' was.
                let size = self.sector_sz.kbytes() as u32;
                let start = start_offset & !(size - 1);
                for idx in start..start + size {
                    let write_address = (FLASH_START + idx as u32) as *const u16;
                    let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                    if verify != 0xFFFF {
                        return Err(Error::VerifyError);
                    }
                }
            }

            Ok(())
        }
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.valid_length(start_offset, length)?;

        // Erase every sector touched by start_offset + length
        for offset in
            (start_offset..start_offset + length as u32).step_by(self.sector_sz.kbytes() as usize)
        {
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Retrieve a slice of data from `FLASH_START + offset`
    pub fn read(&self, offset: u32, length: usize) -> Result<&[u8]> {
        self.valid_address(offset)?;

        if offset + length as u32 > self.flash_sz.kbytes() as u32 {
            return Err(Error::LengthTooLong);
        }

        let address = (FLASH_START + offset) as *const _;

        Ok(
            // NOTE(unsafe) read with no side effects. The data returned will
            // remain valid for its lifetime because we take an immutable
            // reference to this FlashWriter, and any operation that would
            // invalidate the data returned would first require taking a mutable
            // reference to this FlashWriter.
            unsafe { core::slice::from_raw_parts(address, length) },
        )
    }

    /// Write data to `FLASH_START + offset`
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        self.valid_length(offset, data.len())?;

        // Unlock Flash
        self.unlock()?;

        for idx in (0..data.len()).step_by(2) {
            self.valid_address(offset + idx as u32)?;

            let write_address = (FLASH_START + offset + idx as u32) as *mut u16;

            // Set Page Programming to 1
            self.flash.cr.cr().modify(|_, w| w.pg().set_bit());

            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // Flash is written 16 bits at a time, so combine two bytes to get a
            // half-word
            let hword: u16 = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address, hword) };

            // Wait for write
            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // Set Page Programming to 0
            self.flash.cr.cr().modify(|_, w| w.pg().clear_bit());

            // Check for errors
            if self.flash.sr.sr().read().pgerr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.pgerr().clear_bit());

                self.lock()?;
                return Err(Error::ProgrammingError);
            } else if self.flash.sr.sr().read().wrprterr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.wrprterr().clear_bit());

                self.lock()?;
                return Err(Error::WriteError);
            } else if self.verify {
                // Verify written WORD
                // NOTE(unsafe) read with no side effects within FLASH area
                let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                if verify != hword {
                    self.lock()?;
                    return Err(Error::VerifyError);
                }
            }
        }

        // Lock Flash and report success
        self.lock()?;
        Ok(())
    }

    /// Enable/disable verifying that each erase or write operation completed
    /// successfuly.
    ///
    /// When enabled, after each erase operation every address is read to make
    /// sure it contains the erase value of 0xFFFF. After each write operation,
    /// every address written is read and compared to the value that should have
    /// been written. If any address does not contain the expected value, the
    /// function will return Err.
    /// When disabled, no verification is performed, erase/write operations are
    /// assumed to have succeeded.
    pub fn change_verification(&mut self, verify: bool) {
        self.verify = verify;
    }

    /// Reads the OBR and WRPR registers to determine protection status of flash
    pub fn protection_status(&mut self) -> ProtectionStatus {
        let read_prot = self.flash.obr.obr().read().rdprt().bit_is_set();
        let write_prot = self.flash.wrpr.wrpr().read().wrp().bits() == 0;
        match (read_prot, write_prot) {
            (false, false) => ProtectionStatus::Unprotected,
            (true, false) => ProtectionStatus::Read,
            (false, true) => ProtectionStatus::Write,
            (true, true) => ProtectionStatus::ReadWrite,
        }
    }

    /// Enables Read Protection by setting RDP Option Byte. Will not take efect until
    /// MCU has been reset (e.g. via `cortex_m::peripheral::SCB::sys_reset()`)
    pub fn protect_read(&mut self) -> Result<()> {
        // We can write any value that is not RDPRT_KEY to enable read protection.
        // Arbitrarily choose to write 0xBB
        self.set_rdp(0xBB)
    }

    /// NOTE(unsafe) This will cause flash to be erased! Un-protects flash from read.
    pub unsafe fn unprotect_read(&mut self) -> Result<()> {
        self.set_rdp(RDPRT_KEY)
    }
}

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Parts {
        Parts {
            acr: ACR { _0: () },
            ar: AR { _0: () },
            cr: CR { _0: () },
            keyr: KEYR { _0: () },
            obr: OBR { _0: () },
            optkeyr: OPTKEYR { _0: () },
            sr: SR { _0: () },
            wrpr: WRPR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub acr: ACR,

    /// Opaque AR register
    pub(crate) ar: AR,

    /// Opaque CR register
    pub(crate) cr: CR,

    /// Opaque KEYR register
    pub(crate) keyr: KEYR,

    /// Opaque OBR register
    pub(crate) obr: OBR,

    /// Opaque OPTKEYR register
    pub(crate) optkeyr: OPTKEYR,

    /// Opaque SR register
    pub(crate) sr: SR,

    /// Opaque WRPR register
    pub(crate) wrpr: WRPR,
}
impl Parts {
    pub fn writer(&mut self, sector_sz: SectorSize, flash_sz: FlashSize) -> FlashWriter {
        FlashWriter {
            flash: self,
            sector_sz,
            flash_sz,
            verify: true,
        }
    }
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

#[allow(dead_code)]
impl ACR {
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }
}

/// Opaque AR register
pub struct AR {
    _0: (),
}

#[allow(dead_code)]
impl AR {
    pub(crate) fn ar(&mut self) -> &flash::AR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).ar }
    }
}

/// Opaque CR register
pub struct CR {
    _0: (),
}

#[allow(dead_code)]
impl CR {
    pub(crate) fn cr(&mut self) -> &flash::CR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).cr }
    }
}

/// Opaque KEYR register
pub struct KEYR {
    _0: (),
}

#[allow(dead_code)]
impl KEYR {
    pub(crate) fn keyr(&mut self) -> &flash::KEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).keyr }
    }
}

/// Opaque OBR register
pub struct OBR {
    _0: (),
}

#[allow(dead_code)]
impl OBR {
    pub(crate) fn obr(&mut self) -> &flash::OBR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).obr }
    }
}

/// Opaque OPTKEYR register
pub struct OPTKEYR {
    _0: (),
}

#[allow(dead_code)]
impl OPTKEYR {
    pub(crate) fn optkeyr(&mut self) -> &flash::OPTKEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).optkeyr }
    }
}

/// Opaque SR register
pub struct SR {
    _0: (),
}

#[allow(dead_code)]
impl SR {
    pub(crate) fn sr(&mut self) -> &flash::SR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).sr }
    }
}

/// Opaque WRPR register
pub struct WRPR {
    _0: (),
}

#[allow(dead_code)]
impl WRPR {
    pub(crate) fn wrpr(&mut self) -> &flash::WRPR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).wrpr }
    }
}
