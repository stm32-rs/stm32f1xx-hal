# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Added DMA support for ADC1.
- Added type aliases `Tx1` for `Tx<USART1>`, `RxDma1` for `RxDma<USART1, dma1::C5>`, etc.
- Add ADC1 reading functions for channels 16 (temperature) and 17 (internal reference voltage)
- Update existing ADC example according to ADC API changes
- Add new ADC example to read ambient temperature using ADC1 CH16
- Add `listen` and `unlisten` to `serial::Tx` and `serial::Rx`.


### Breaking changes

- Replace gpio traits with digital::v2
- Bump `stm32f1` dependency (`0.8.0`)
- ADC now requires the clock configuration for intialisation
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

[Unreleased]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.3.0...HEAD
[v0.3.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.1...v0.3.0
[v0.2.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.1.1...v0.2.0
