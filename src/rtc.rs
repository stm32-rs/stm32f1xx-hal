/*!
  Real time clock

  A continuously running clock that counts seconds. It is part of the backup domain which means
  that the counter is not affected by system resets or standby mode. If Vbat is connected, it is
  not reset even if the rest of the device is powered off. This allows it to be used to wake the
  CPU when it is in low power mode.

  Since it is part of the backup domain, write access to it must be enabled before the RTC can be
  used. See `backup_domain` for more details.

  See examples/rtc.rs and examples/blinky_rtc.rs for usage examples.
*/

use stm32::{RTC};

use crate::rcc::Lse;
use crate::backup_domain::BackupDomain;

use nb;
use void::Void;


/**
  Interface to the real time clock
*/
pub struct Rtc {
    regs: RTC,
    _lse: Lse,
}


impl Rtc {
    /**
      Initialises the RTC. The `Lse` and `BackupDomain` structs are created by
      `Rcc.lse.freeze()` and `Rcc.bkp.constrain()` respectively
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

    /// Set the current rtc value to the specified amount of seconds
    pub fn set_seconds(&mut self, seconds: u32) {
        self.perform_write(|s| {
            s.regs.cnth.write(|w| unsafe{w.bits(seconds >> 16)});
        });
        self.perform_write(|s| {
            s.regs.cntl.write(|w| unsafe{w.bits(seconds)});
        });
    }

    /**
      Sets the time at which an alarm will be triggered

      This also clears the alarm flag if it is set
    */
    pub fn set_alarm(&mut self, seconds: u32) {
        // Set alarm time
        // See section 18.3.5 for explanation
        let alarm_value = seconds - 1;
        self.perform_write(|s| {
            s.regs.alrh.write(|w| unsafe{w.alrh().bits((alarm_value >> 16) as u16)});
        });
        self.perform_write(|s| {
            s.regs.alrl.write(|w| unsafe{w.alrl().bits((alarm_value & 0x0000ffff) as u16)});
        });

        self.clear_alarm_flag();
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
    pub fn read_seconds(&self) -> u32 {
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
      Return `Ok(())` if the alarm flag is set, `Err(nb::WouldBlock)` otherwise.

      **Note**: Does not clear the alarm flag

      ```rust
      use nb::block;

      rtc.set_alarm(rtc.read_counts() + 5);
      // NOTE: Safe unwrap because Void can't be returned
      block!(rtc.wait_alarm()).unwrap();
      ```
    */
    pub fn wait_alarm(&mut self) -> nb::Result<(), Void> {
        if self.regs.crl.read().alrf().bit() == true {
            Ok(())
        }
        else {
            Err(nb::Error::WouldBlock)
        }
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
