use stm32::{RTC};

use crate::rcc::Lse;
use crate::backup_domain::BackupDomain;


/*
    Configuring RTC registers requires the following process:
    poll RTOFF, make sure it is 1
    set CNF to enter config mode
    write to the registers
    clear CNF
    poll RTOFF to make sure it's back to 1
    The frequency of the rtc is calculated by
    f_rtcclk / (PRL[19:0] + 1)
    f_rtcclk can probably be configured to be the main clock. Then we can use
    rcc to figure out what that frequency is
    both vbat and vdd might have to be connected for the clock to keep counting
*/

/**
  Interface to the real time clock. This struct can not be instanciated directly,
  instead, it is created by the `BackupDomain`. This ensures that the rtc registers
  are writeable and that the clock has been configured correctly.
*/
pub struct Rtc {
    regs: RTC,
    _lse: Lse,
}


impl Rtc {
    /**
      Initialises the RTC, this should only be called if access to the backup
      domain has been enabled
    */
    pub fn rtc(regs: RTC, lse: Lse, _bkp: &BackupDomain) -> Self {
        // Set the prescaler to make it count up once every second
        // The manual on page 490 says that the prescaler value for this should be 7fffh
        let mut result = Rtc {
            regs,
            _lse: lse,
        };

        let freq = lse.freq().0;
        let prl = freq - 1;
        assert!(prl < 1 << 20);
        result.perform_write(|s| {
            s.regs.prlh.write(|w| unsafe { w.bits(prl >> 16) });
            s.regs.prll.write(|w| unsafe { w.bits(prl as u16 as u32) });
        });

        result
    }

    /// Set the current rtc value to the specified amount of counts
    pub fn set_cnt(&mut self, counts: u32) {
        self.perform_write(|s| {
            s.regs.cnth.write(|w| unsafe{w.bits(counts >> 16)});
        });
        self.perform_write(|s| {
            s.regs.cntl.write(|w| unsafe{w.bits(counts)});
        });
    }

    /// Sets the time at which an alarm will be triggered
    pub fn set_alarm(&mut self, counts: u32) {
        // Set alarm time
        // See section 18.3.5 for explanation
        let alarm_value = counts - 1;
        self.perform_write(|s| {
            s.regs.alrh.write(|w| unsafe{w.alrh().bits((alarm_value >> 16) as u16)});
        });
        self.perform_write(|s| {
            s.regs.alrl.write(|w| unsafe{w.alrl().bits((alarm_value & 0x0000ffff) as u16)});
        });
    }

    /// Enables the RTCALARM interrupt
    pub fn listen_alarm(&mut self) {
        // Enable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.alrie().set_bit());
        })
    }

    /// Disables the RTCALARM interrupt
    pub fn unlisten_alarm(&mut self) {
        // Disable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.alrie().clear_bit());
        })
    }

    /// Reads the current time
    pub fn read_counts(&self) -> u32 {
        // Wait for the APB1 interface to be ready
        while self.regs.crl.read().rsf().bit() == false {}

        ((self.regs.cnth.read().bits() << 16) as u32) + (self.regs.cntl.read().bits() as u32)
    }

    /// Enables the RTC second interrupt
    pub fn listen_seconds(&mut self) {
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.secie().set_bit())
        })
    }

    /// Disables the RTC second interrupt
    pub fn unlisten_seconds(&mut self) {
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.secie().clear_bit())
        })
    }

    /// Clears the RTC second interrupt flag
    pub fn clear_second_flag(&mut self) {
        self.perform_write(|s| {
            s.regs.crl.modify(|_, w| w.secf().clear_bit())
        })
    }

    /// Clears the RTC alarm interrupt flag
    pub fn clear_alarm_flag(&mut self) {
        self.perform_write(|s| {
            s.regs.crl.modify(|_, w| w.alrf().clear_bit())
        })
    }


    /**
      The RTC registers can not be written to at any time as documented on page
      485 of the manual. Performing writes using this function ensures that
      the writes are done correctly.
    */
    fn perform_write(&mut self, func: impl Fn(&mut Self)) {
        // Wait for the last write operation to be done
        while self.regs.crl.read().rtoff().bit() == false {}
        // Put the clock into config mode
        self.regs.crl.modify(|_, w| w.cnf().set_bit());

        // Perform the write opertaion
        func(self);

        // Take the device out of config mode
        self.regs.crl.modify(|_, w| w.cnf().clear_bit());
        // Wait for the write to be done
        while !self.regs.crl.read().rtoff().bit() {}
    }
}
