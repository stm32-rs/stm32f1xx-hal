# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Implement ADC example
- Implement ADC embedded_hal traits
- Implement ADC clock configuration
- Add feature for using STM32F101 chip
- Add gpio pins corresponding to LQFP-100 package
- Implement `core::fmt::Write` for `serial::Tx`
- Add methods `stop`, `release` and `clear_update_interrupt_flag` to `Timer` (`clear_update_interrupt_flag` does not apply to `Timer<SYST>`)
- Add timer interrupt example using RTFM

### Changed

- enable PWM on stm32f100

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

[Unreleased]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.1...HEAD
[v0.2.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.1.1...v0.2.0
