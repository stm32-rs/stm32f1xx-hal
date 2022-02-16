/*!
  Real time clock
*/
use crate::pac::{RCC, RTC};

use crate::backup_domain::BackupDomain;
use crate::time::{Hertz, Hz};

use core::convert::Infallible;
use core::marker::PhantomData;

// The LSE runs at at 32 768 hertz unless an external clock is provided
const LSE_HERTZ: Hertz = Hz(32_768);
const LSI_HERTZ: Hertz = Hz(40_000);

/// RTC clock source HSE clock divided by 128 (type state)
pub struct RtcClkHseDiv128;
/// RTC clock source LSE oscillator clock (type state)
pub struct RtcClkLse;
/// RTC clock source LSI oscillator clock (type state)
pub struct RtcClkLsi;

/**
  Real time clock

  A continuously running clock that counts seconds¹. It is part of the backup domain which means
  that the counter is not affected by system resets or standby mode. If Vbat is connected, it is
  not reset even if the rest of the device is powered off. This allows it to be used to wake the
  CPU when it is in low power mode.


  See [examples/rtc.rs] and [examples/blinky_rtc.rs] for usage examples.

  1: Unless configured to another frequency using [select_frequency](struct.Rtc.html#method.select_frequency)

  [examples/rtc.rs]: https://github.com/stm32-rs/stm32f1xx-hal/blob/v0.7.0/examples/rtc.rs
  [examples/blinky_rtc.rs]: https://github.com/stm32-rs/stm32f1xx-hal/blob/v0.7.0/examples/blinky_rtc.rs
*/

pub struct Rtc<CS = RtcClkLse> {
    regs: RTC,
    _clock_source: PhantomData<CS>,
}

impl Rtc<RtcClkLse> {
    /**
      Initialises the RTC. The `BackupDomain` struct is created by
      `Rcc.bkp.constrain()`.

      The frequency is set to 1 Hz.

      Since the RTC is part of the backup domain, The RTC counter is not reset by normal resets or
      power cycles where (VBAT) still has power. Use [set_time](#method.set_time) if you want to
      reset the counter.
    */
    pub fn new(regs: RTC, bkp: &mut BackupDomain) -> Self {
        let mut result = Rtc {
            regs,
            _clock_source: PhantomData,
        };

        Self::enable_rtc(bkp);

        // Set the prescaler to make it count up once every second.
        let prl = LSE_HERTZ.raw() - 1;
        assert!(prl < 1 << 20);
        result.perform_write(|s| {
            s.regs.prlh.write(|w| unsafe { w.bits(prl >> 16) });
            s.regs.prll.write(|w| unsafe { w.bits(prl as u16 as u32) });
        });

        result
    }

    /// Enables the RTC device with the lse as the clock
    fn enable_rtc(_bkp: &mut BackupDomain) {
        // NOTE: Safe RCC access because we are only accessing bdcr
        // and we have a &mut on BackupDomain
        let rcc = unsafe { &*RCC::ptr() };
        rcc.bdcr.modify(|_, w| {
            w
                // start the LSE oscillator
                .lseon()
                .set_bit()
                // Enable the RTC
                .rtcen()
                .set_bit()
                // Set the source of the RTC to LSE
                .rtcsel()
                .lse()
        })
    }
}

impl Rtc<RtcClkLsi> {
    pub fn new_lsi(regs: RTC, bkp: &mut BackupDomain) -> Self {
        let mut result = Rtc {
            regs,
            _clock_source: PhantomData,
        };

        Self::enable_rtc(bkp);

        // Set the prescaler to make it count up once every second.
        let prl = LSI_HERTZ.raw() - 1;
        assert!(prl < 1 << 20);
        result.perform_write(|s| {
            s.regs.prlh.write(|w| unsafe { w.bits(prl >> 16) });
            s.regs.prll.write(|w| unsafe { w.bits(prl as u16 as u32) });
        });

        result
    }

    /// Enables the RTC device with the lsi as the clock
    fn enable_rtc(_bkp: &mut BackupDomain) {
        // NOTE: Safe RCC access because we are only accessing bdcr
        // and we have a &mut on BackupDomain
        let rcc = unsafe { &*RCC::ptr() };
        rcc.csr.modify(|_, w| {
            w
                // start the LSI oscillator
                .lsion()
                .set_bit()
        });
        rcc.bdcr.modify(|_, w| {
            w
                // Enable the RTC
                .rtcen()
                .set_bit()
                // Set the source of the RTC to LSI
                .rtcsel()
                .lsi()
        })
    }
}

impl Rtc<RtcClkHseDiv128> {
    pub fn new_hse(regs: RTC, bkp: &mut BackupDomain, hse: Hertz) -> Self {
        let mut result = Rtc {
            regs,
            _clock_source: PhantomData,
        };

        Self::enable_rtc(bkp);

        // Set the prescaler to make it count up once every second.
        let prl = hse.raw() / 128 - 1;
        assert!(prl < 1 << 20);
        result.perform_write(|s| {
            s.regs.prlh.write(|w| unsafe { w.bits(prl >> 16) });
            s.regs.prll.write(|w| unsafe { w.bits(prl as u16 as u32) });
        });

        result
    }

    /// Enables the RTC device with the lsi as the clock
    fn enable_rtc(_bkp: &mut BackupDomain) {
        // NOTE: Safe RCC access because we are only accessing bdcr
        // and we have a &mut on BackupDomain
        let rcc = unsafe { &*RCC::ptr() };
        if rcc.cr.read().hserdy().bit_is_clear() {
            panic!("HSE oscillator not ready");
        }
        rcc.bdcr.modify(|_, w| {
            w
                // Enable the RTC
                .rtcen()
                .set_bit()
                // Set the source of the RTC to HSE/128
                .rtcsel()
                .hse()
        })
    }
}

impl<CS> Rtc<CS> {
    /// Selects the frequency of the RTC Timer
    /// NOTE: Maximum frequency of 16384 Hz using the internal LSE
    pub fn select_frequency(&mut self, frequency: Hertz) {
        // The manual says that the zero value for the prescaler is not recommended, thus the
        // minimum division factor is 2 (prescaler + 1)
        assert!(frequency <= LSE_HERTZ / 2);

        let prescaler = LSE_HERTZ / frequency - 1;
        self.perform_write(|s| {
            s.regs.prlh.write(|w| unsafe { w.bits(prescaler >> 16) });
            s.regs
                .prll
                .write(|w| unsafe { w.bits(prescaler as u16 as u32) });
        });
    }

    /// Set the current RTC counter value to the specified amount
    pub fn set_time(&mut self, counter_value: u32) {
        self.perform_write(|s| {
            s.regs
                .cnth
                .write(|w| unsafe { w.bits(counter_value >> 16) });
            s.regs
                .cntl
                .write(|w| unsafe { w.bits(counter_value as u16 as u32) });
        });
    }

    /**
      Sets the time at which an alarm will be triggered

      This also clears the alarm flag if it is set
    */
    pub fn set_alarm(&mut self, counter_value: u32) {
        // Set alarm time
        // See section 18.3.5 for explanation
        let alarm_value = counter_value - 1;

        // TODO: Remove this `allow` once these fields are made safe for stm32f100
        #[allow(unused_unsafe)]
        self.perform_write(|s| {
            s.regs
                .alrh
                .write(|w| unsafe { w.alrh().bits((alarm_value >> 16) as u16) });
            s.regs
                .alrl
                .write(|w| unsafe { w.alrl().bits(alarm_value as u16) });
        });

        self.clear_alarm_flag();
    }

    /// Enables the RTC interrupt to trigger when the counter reaches the alarm value. In addition,
    /// if the EXTI controller has been set up correctly, this function also enables the RTCALARM
    /// interrupt.
    pub fn listen_alarm(&mut self) {
        // Enable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.alrie().set_bit());
        })
    }

    /// Stops the RTC alarm from triggering the RTC and RTCALARM interrupts
    pub fn unlisten_alarm(&mut self) {
        // Disable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh.modify(|_, w| w.alrie().clear_bit());
        })
    }

    /// Reads the current counter
    pub fn current_time(&self) -> u32 {
        // Wait for the APB1 interface to be ready
        while !self.regs.crl.read().rsf().bit() {}

        self.regs.cnth.read().bits() << 16 | self.regs.cntl.read().bits()
    }

    /// Enables triggering the RTC interrupt every time the RTC counter is increased
    pub fn listen_seconds(&mut self) {
        self.perform_write(|s| s.regs.crh.modify(|_, w| w.secie().set_bit()))
    }

    /// Disables the RTC second interrupt
    pub fn unlisten_seconds(&mut self) {
        self.perform_write(|s| s.regs.crh.modify(|_, w| w.secie().clear_bit()))
    }

    /// Clears the RTC second interrupt flag
    pub fn clear_second_flag(&mut self) {
        self.perform_write(|s| s.regs.crl.modify(|_, w| w.secf().clear_bit()))
    }

    /// Clears the RTC alarm interrupt flag
    pub fn clear_alarm_flag(&mut self) {
        self.perform_write(|s| s.regs.crl.modify(|_, w| w.alrf().clear_bit()))
    }

    /**
      Return `Ok(())` if the alarm flag is set, `Err(nb::WouldBlock)` otherwise.

      ```rust
      use nb::block;

      rtc.set_alarm(rtc.read_counts() + 5);
      // NOTE: Safe unwrap because Infallible can't be returned
      block!(rtc.wait_alarm()).unwrap();
      ```
    */
    pub fn wait_alarm(&mut self) -> nb::Result<(), Infallible> {
        if self.regs.crl.read().alrf().bit() {
            self.regs.crl.modify(|_, w| w.alrf().clear_bit());
            Ok(())
        } else {
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
        while !self.regs.crl.read().rtoff().bit() {}
        // Put the clock into config mode
        self.regs.crl.modify(|_, w| w.cnf().set_bit());

        // Perform the write operation
        func(self);

        // Take the device out of config mode
        self.regs.crl.modify(|_, w| w.cnf().clear_bit());
        // Wait for the write to be done
        while !self.regs.crl.read().rtoff().bit() {}
    }
}
