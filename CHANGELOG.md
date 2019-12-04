# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Added support for `ExtiPin` pin traits

## [v0.5.0] - 2019-12-03

### Added

- Added `Mode` marker trait for `gpio` pins that correspondent to pin mode.
- RCC `Bus` trait + private `Enable` and `Reset` traits
- Added `micros_since` and `reset` methods to timer
- Added `select_frequency` method to RTC
- Unidirectional DMA support for SPI (TX only)
- Added USB driver for `stm32f102` and `stm32f103` devices
- Added all timers for all variants as described by CubeMX. Commented out {TIM9, TIM10} for XL and {TIM12, TIM13, TIM14} for XL and F100-HIGH due to missing fields for those devices in stm32-rs.
- ADC measurement now can be run by timer

### Breaking changes

- Implement more pin combinations for PWM configuration, added PWM for TIM1 (API for custom PWM pins was removed as it's no more needed)
- Bump `stm32f1` dependency (`0.9.0`)
- `void::Void` replaced with `Infallible` where it is possible
- Change timer/pwm init API
- Remove `set_low` and `set_high` for pins in Alternate output mode
- Renames `set_seconds` and `seconds` methods on RTC to `set_time` and `current_time`, respectively
- Starting the timer does not generate interrupt requests anymore
- Make MAPR::mapr() private
- i2c mode now takes Hertz instead of a generic u32
- Timers that were previously incorrectly available without medium/high/xl density features may now be missing

### Fixed

- Fix some F1 variants crashing when modifying MAPR if JTAG is disabled
- Switched Timer stop_in_debug to modify cr instead of writing it to prevent it clobbering the rest of the register (was breaking ITM output when configuring pwm_input for example)

### Changed

- Pins can be passed in any order to SPI constructor,
  `NoSck`, `NoMiso` and `NoMosi` can be also passed instead of real pin
- DMA traits now require AsSlice instead of AsRef
- GPIO `downgrade` function now returns a `Pxx` instead of a type specific to a
  GPIO port

- AdcDma can process several pins at a time

## [v0.4.0] - 2019-08-09

### Added

- Added DMA support for ADC1.
- Added type aliases `Tx1` for `Tx<USART1>`, `RxDma1` for `RxDma<USART1, dma1::C5>`, etc.
- Add ADC1 reading functions for channels 16 (temperature) and 17 (internal reference voltage)
- Update existing ADC example according to ADC API changes
- Add new ADC example to read ambient temperature using ADC1 CH16
- Add `listen` and `unlisten` to `serial::Tx` and `serial::Rx`.
- Add methods `read_data_register` and `write_data_register` to
  `backup_domain::BackupDomain`, which allow read and write access to the Backup
  Data Register.

### Breaking changes

- Replace gpio traits with digital::v2
- Bump `stm32f1` dependency (`0.8.0`)
- ADC now requires the clock configuration for initialisation
- `disable_jtag` now transforms PA15, PB3 and PB4 to forbid their use without desactivating JTAG

### Changed

- Fix hclk miscalculation

## [v0.3.0] - 2019-04-27

### Added

- Added device-selected meta feature flag
- Restore and improve Serial DMA-based TX, RX and circular implementation
- Implement ADC example
- Implement ADC embedded_hal traits
- Implement ADC clock configuration
- Add feature for using STM32F101 chip
- Add gpio pins corresponding to LQFP-100 package
- Implement `core::fmt::Write` for `serial::Tx`
- Add methods `stop`, `release` and `clear_update_interrupt_flag` to `Timer` (`clear_update_interrupt_flag` does not apply to `Timer<SYST>`)
- Add timer interrupt example using RTFM
- Implement IndependentWatchdog for the IWDG peripheral
- Remove all PWM channel configurations except 'all the channels for default remapping' configuratons
- Update PWM documentation: clarify custom selection of channels
- Add PWM example for custom selection of channels

### Changed

- *Breaking change* Add additional configuration options to USART.
    - Baud rate now has to be set using configuration struct
- Now requires stm32f1 v0.7 (breaking change)
- enable PWM on stm32f100
- Fix gpio misconfiguration when using a timer in pwm input mode. Now the gpio has to be configured in floating input mode.

## [v0.2.1] - 2019-03-08

### Added

- Add basic backup domain support
- Add support for real time clock
- Add patches for using STM32F100 chip (PWM disabled)

### Changed

- Improve documentation

### Fixed

- Use correct clock for serial baudrate computation




## [v0.2.0] - 2019-02-10

### Added

- Add support for setting initial pin state
- Added ChangeLog

### Changed

- Add information about device features to readme
- Allow read-/write-only transactions in write_read
- Bumped dependency versions (breaking change)

### Fixed

- Fix link to docs.rs

## [v0.1.1] - 2018-12-17

### Added

- First tagged version

[Unreleased]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.5.0...HEAD
[v0.5.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.1...v0.3.0
[v0.2.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.1.1...v0.2.0
