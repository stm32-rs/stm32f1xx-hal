//! Flash memory

use crate::pac::{flash, FLASH};

pub const FLASH_START: u32 = 0x0800_0000;
pub const FLASH_END:   u32 = 0x080F_FFFF;

const RDPRT_KEY: u16 = 0x00A5;
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

pub const SZ_1K: u16 = 1024;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum Error {
    AddressLargedThanFlash,
    AddressMisAligned,
    LengthNotMultiple2,
    LengthTooLong,
    EraseError,
    ProgrammingError,
    WriteError,
    VerifyError,
    UnLockError,
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
    Sz16K  = 16,
    Sz32K  = 32,
    Sz64K  = 64,
    Sz128K = 128,
    Sz256K = 256,
    Sz384K = 384,
    Sz512K = 512,
    Sz768K = 768,
    Sz1M   = 1024,
}
impl FlashSize {
    const fn kbytes(self) -> u32 {
        SZ_1K as u32 * self as u32
    }
}

pub struct FlashReaderGuard<'a> {
    _flash: &'a FlashWriter<'a>,
    data: &'a [u8],
}

impl<'a> FlashReaderGuard<'a> {
    pub fn data(&self) -> &[u8] {
        self.data
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

        // NOTE(unsafe) write Keys to the key register
        unsafe { self.flash.keyr.keyr().write(|w| w.key().bits(KEY1)); }
        unsafe { self.flash.keyr.keyr().write(|w| w.key().bits(KEY2)); }

        // Verify success
        match self.flash.cr.cr().read().lock().bit_is_clear() {
            true => Ok(()),
            false => Err(Error::UnLockError),
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

    fn valid_address(&self, offset: u32) -> Result<()> {
        if FLASH_START + offset > FLASH_END {
            Err(Error::AddressLargedThanFlash)
        }
        else if offset & 0x1 != 0 {
            Err(Error::AddressMisAligned)
        }
        else {
            Ok(())
        }
    }

    fn valid_length(&self, offset: u32, length: usize) -> Result<()> {
        if offset + length as u32 > self.flash_sz.kbytes() as u32 {
            Err(Error::LengthTooLong)
        }
        else if length & 0x1 != 0 {
            Err(Error::LengthNotMultiple2)
        }
        else {
            Ok(())
        }
    }

    /// Erase sector which contains start_offset
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        // Unlock Flash
        self.unlock()?;

        // Set Page Erase
        self.flash.cr.cr().modify(|_, w| w.per().set_bit() );

        // Write address bits
        // NOTE(unsafe) This sets the sector address in the Address Register
        unsafe { self.flash.ar.ar().write(|w| w.far().bits(FLASH_START + start_offset) ); }

        // Start Operation
        self.flash.cr.cr().modify(|_, w| w.strt().set_bit() );

        // Wait for operation to finish
        while self.flash.sr.sr().read().bsy().bit_is_set() { }

        // Check for errors
        let sr = self.flash.sr.sr().read();

        // Remove Page Erase Operation bit
        self.flash.cr.cr().modify(|_, w| w.per().clear_bit() );

        // Re-lock flash
        self.lock()?;

        if sr.wrprterr().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.wrprterr().clear_bit() );
            Err(Error::EraseError)
        } else {
            if self.verify {
                let start = start_offset & !(self.sector_sz.kbytes() as u32 -1);
                let end = start + self.sector_sz.kbytes() as u32 -1;
                for idx in start..end {
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

    /// Erase the Flash Sectors from FLASH_START + start_offset to length
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.valid_length(start_offset, length)?;

        // Erase every sector touched by start_offset + length
        for offset in (start_offset..start_offset + length as u32).step_by(self.sector_sz.kbytes() as usize) {
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Retrieve an slice of data from FLASH_START +offset
    pub fn read(&self, offset: u32, length: usize) -> Result<FlashReaderGuard> {
        self.valid_address(offset)?;

        if offset + length as u32 > self.flash_sz.kbytes() as u32 {
            return Err(Error::LengthTooLong);
        }

        let address = (FLASH_START + offset) as *const _;

        Ok(FlashReaderGuard {
            _flash: self,

            // NOTE(unsafe) read with no side effects safe-guarded by FlashReaderGuard.
            data: unsafe { core::slice::from_raw_parts::<'static, u8>(address, length) }
        })
    }

    /// Write data to FLASH_START +offset
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        self.valid_length(offset, data.len() )?;

        // Unlock Flash
        self.unlock()?;

        for idx in (0..data.len() ).step_by(2) {
            self.valid_address(offset + idx as u32)?;

            let hword: u16 =
                  (data[idx]   as u16)
                | (data[idx+1] as u16) << 8;
            let write_address = (FLASH_START + offset + idx as u32) as *mut u16;

            // Set Page Programming to 1
            self.flash.cr.cr().modify(|_, w| w.pg().set_bit() );

            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address, hword) };

            // Wait for write
            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // Set Page Programming to 0
            self.flash.cr.cr().modify(|_, w| w.pg().clear_bit() );

            // Check for errors
            if self.flash.sr.sr().read().pgerr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.pgerr().clear_bit() );

                self.lock()?;
                return Err(Error::ProgrammingError);
            }
            else if self.flash.sr.sr().read().wrprterr().bit_is_set() {
                self.flash.sr.sr().modify(|_, w| w.wrprterr().clear_bit() );

                self.lock()?;
                return Err(Error::WriteError);
            }
            else if self.verify {
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

    pub fn change_verification(&mut self, verify: bool) {
        self.verify = verify;
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
            acr:     ACR     { _0: () },
            ar:      AR      { _0: () },
            cr:      CR      { _0: () },
            keyr:    KEYR    { _0: () },
            obr:     OBR     { _0: () },
            optkeyr: OPTKEYR { _0: () },
            sr:      SR      { _0: () },
            wrpr:    WRPR    { _0: () },
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
