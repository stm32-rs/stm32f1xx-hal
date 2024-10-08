# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Breaking changes

- Relax pin type generics for `Serial`, `I2c`, `Spi`, `Can`. [#462]
  Use enums of pin tuples and `Enum::from<(tuple)>` for pin remap before passing to peripheral.
  Remove `RemapStruct`s. [#462] [#506] [#509]
- Use independent `Spi` and `SpiSlave` structures instead of `OP` generic [#462]
- Take `&Clocks` instead of `Clocks` [#498]
- Temporary replace `stm32f1` with `stm32f1-staging` [#503]

### Changed

- PWM timer auto reload value is now preloaded/buffered [#453]
- Move from bors/manual merge to GH merge queue [#467]
- Replace UB code by a legitimate pointer access [#480]
- Fix flash error flag clearing [#489]
- Clarify README for windows users [#496]
- Check "device selected" in `build.rs` [#502]
- Use gpio field enums internally [#506]
- Unmacro `dma.rs` [#505]
- Rework USART remap, 
- Updated `usb-device` and `usbd-serial` to latest versions

### Added

- Allow to set HSE bypass bit in `RCC` clock configuration register to use an external clock input on the `OSC_IN` pin [#485]
- initial support of `embedded-hal-1.0` [#416]
- Add tools/check.py python script for local check [#467]
- Add changelog check on PRs [#467]
- Reexport `Direction` from `qei` [#479]
- Add DAC [#483]
- Add an option to allow overclocking [#494]
- `new` on gpio mode [#506]
- Add `Serial` `rx`/`tx` constructors [#509]

[#416]: https://github.com/stm32-rs/stm32f1xx-hal/pull/416
[#453]: https://github.com/stm32-rs/stm32f1xx-hal/pull/453
[#462]: https://github.com/stm32-rs/stm32f1xx-hal/pull/462
[#467]: https://github.com/stm32-rs/stm32f1xx-hal/pull/467
[#479]: https://github.com/stm32-rs/stm32f1xx-hal/pull/479
[#480]: https://github.com/stm32-rs/stm32f1xx-hal/pull/480
[#483]: https://github.com/stm32-rs/stm32f1xx-hal/pull/483
[#485]: https://github.com/stm32-rs/stm32f1xx-hal/pull/485
[#489]: https://github.com/stm32-rs/stm32f1xx-hal/pull/489
[#494]: https://github.com/stm32-rs/stm32f1xx-hal/pull/494
[#496]: https://github.com/stm32-rs/stm32f1xx-hal/pull/496
[#498]: https://github.com/stm32-rs/stm32f1xx-hal/pull/498
[#502]: https://github.com/stm32-rs/stm32f1xx-hal/pull/502
[#503]: https://github.com/stm32-rs/stm32f1xx-hal/pull/503
[#505]: https://github.com/stm32-rs/stm32f1xx-hal/pull/505
[#506]: https://github.com/stm32-rs/stm32f1xx-hal/pull/506
[#509]: https://github.com/stm32-rs/stm32f1xx-hal/pull/509

## [v0.10.0] - 2022-12-12

- `Timer`: adds `get_interrupt` to `Timer`
- `gpio`: port and pin generics first, then mode,
  `PinMode` for modes instead of pins, `HL` trait, other cleanups
- `flash`: add one-cycle delay of reading `BSY` bit after setting `STRT` bit to
           fix errata.
- `PwmHz::get_period`: fix computation of return value, prevent division by zero
- return `i2c::Error::Timeout` instead of `nb::WouldBlock` when time is out

### Breaking changes

- Passing the `Clock` parameter to `Serial` by reference.
- `Serial::usart1/2/3` -> `Serial::new`.
- `Serial` implements `Write<WORD>` and `Read<WORD>` for `WORD` simultaneously as u8 and u16.
- Bump bxcan version to [v0.7.0](https://github.com/stm32-rs/bxcan/releases/tag/v0.7.0)

### Added

- Allow access to the `Tx` and `Rx` parts of the `Serial` without the need for splitting.
- Allow `Serial` reconfiguration by references to the `Tx` and `Rx` parts.
- Allow `Serial` release after splitting.
- `Spi::is_busy()`

### Fixed

- `CounterHz` and `Counter` do not `wait` properly after the second and subsequent `start`.

## [v0.9.0] - 2022-03-02

### Added

- Spi Slave mode
- Reexport gpio pins to `gpio` mod
- Added the ability to specify the word size (8 or 9 bits) for `Serial` (USART). When using parity, the parity bit is included in the number of bits of the word.
- `blocking::serial::Write` for `Tx` and `Serial`. `core::fmt::Write` for `Serial`
- `Instance` for Timer's, rtic-monotonic fugit impl
- Serial can now be reconfigured, allowing to change e.g. the baud rate after initialization.

### Changed

- Use `embedded-dma` 0.2.0
- Connectivity line devices configuration supports ADC2
- replace `GetBusFreq` with `BusClock` and `BusTimerClock`

## [v0.8.0] - 2021-12-29

### Breaking changes

- Bump `stm32f1` to `0.14.0`, `cortex-m-rtic` to `1.0.0`, `bxcan` to `0.6` and others
- Bump `stm32-usbd` dependency (`0.6.0`)
- Use bit-banding for Peripheral enable/reset.
  Don't require APBs in initializers.
- Rename `gpio::Edge::{RISING, FALLING, RISING_FALLING}` to `Rising`, `Falling`, `RisingFalling`, respectively

### Added

- RTC clock source can be selected.
- `rcc::Config` with prescalers for direct setting of clocks without calculating
- `From<Bps>` for `serial::Config`
- `From<Into<Hertz>>` for `i2c::Mode`
- `exti_rtic` example
- Support for OpenDrain pin configuration on CAN, SPI, UART, PWM output pins
- LSB/MSB bit format selection for `SPI`
- Support for CAN peripherals with the `bxcan` crate
- Add DAC, UART4, UART5 clock in RCC for the f103 high density line
- `start_raw` function and `arr`, `bsc` getters for more fine grained
  control over the timer.
- Added RxTxDma support support to the DMA infrastructure
- Added DMA receive support for `SPI`
- Added `release` functions to SPI DMA
- Add GPIOF/GPIOG support for high/xl density lines
- Allow using `Input<PullUp>` and `Input<PullDown>` for all alternate
  function inputs.
- Add `PartialOrd` derivation for `Bps`, `Hertz`, `KiloHertz`, and `MegaHertz`

### Fixed

- RTC not enable PWR clock
- USART2 remap
- Fix > 2 byte i2c reads
- Send stop after acknowledge errors on i2c
- Fix i2c interactions after errors
- Fix SPI3 alternate function remapping
- Do not enable UART DMA flags unconditionally
- Fix flash erase verification always failing
- Fix invalid 8-bit access to USART registers.

### Changed

- Move Tx & Rx in Serial. `Read` and `Write` now implemented on `Rx` and `Tx`
- USB driver is now enabled by default for all devices supporting it
- Updated `bxcan` dependency
- Change internal implementation of pins using const generics
- Use `cortex-m-rtic` instead of `cortex-m-rtfm` in the examples
- Renamed `serial`'s `RxDma`/`TxDma`'s `split` method into `release`
- Renamed I2C's `free` method into `release`
- Enable SPI DMA in `with_tx_dma`, not in `SpiTxDma::start`
- Use maximum frequency of 36 MHz on PCLK1
- Round up when calculating the PCLK1 prescaler

## [v0.7.0] - 2020-10-17

### Breaking changes

- MonoTimer now takes ownership of the DCB register
- SPI objects now have a `FrameSize` type field
- Bit banding functions (`bb::*`) are now correctly marked as unsafe
- Add missing remap to `spi3` constructor. Requires a new `mapr` argument.
- Change DMA API to use embedded-dma traits.

### Added

- Add 16 bit dataframe size for `SPI`
- Implement `timer::Cancel` trait for `CountDownTimer`
- Changing Output pin slew rates through the OutputSpeed trait
- Add support for ADC continuous conversion
- Add supoort for ADC discontinuous mode

### Fixed

- Fix MonoTimer not working in debug mode.
- Add missing TX DMA implementation for SPI3.

## [v0.6.1] - 2020-06-25

### Added

- Add runtime-reconfigurable GPIO pins
- Added support for writing/reading/erasing onboard flash

### Fixed

- Fix wrong frequency reported by `MonoTimer`
- Fix wrong timings generated by `Timer::syst`
- Fix period retrieval for timers

### Changed

- Use `Deref` for I2C generic implementations instead of macros
- Deprecate `Spi::free` and rename it to `Spi::release`
- Improve `SPI` documentation
- Improve `RCC` and `AFIO` register documentation

## [v0.6.0] - 2020-06-06

### Breaking changes

- Bump `stm32f1` dependency (`0.11.0`)
- Make traits `rcc::Enable` and `rcc::Reset` public, but `RccBus` sealed

### Added

- Extend the Pwm implementation to cover the full embedded_hal::Pwm API
- Add `QeiOptions` struct to configure slave mode and auto reload value of QEI interface
- Implement multiplication and division for frequency wrappers (#193)
- Add support for CRC

### Changed

- Support for connectivity line devices: `stm32f105xx` and `stm32f107xx`
- Consistently use PAC as `pac` and mark `device` and `stm32` informally as deprecated
- Replace default blocking spi Write implementation with an optimized one
- Use `Deref` for SPI generic implementations instead of macros

### Fixed

- Fix PWM on `TIM1`
- Fix ADC race condition causing incorrect reads at certain frequencies

## [v0.5.3] - 2020-01-20

- Add `InputPin` impl for generic open drain outputs
- Implement `Read<u8>` / `Write<u8>` for `Serial` (#171)
- Fix docs.rs build

## [v0.5.2] - 2019-12-15

- Fix USB module docs

## [v0.5.1] - 2019-12-14

### Added

- Added support for `ExtiPin` pin traits

### Fixed

- Fix SPI2 and 3 using the wrong frequency
- Fix some problems with I2C reads anad writes


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
- `disable_jtag` now transforms PA15, PB3 and PB4 to forbid their use without deactivating JTAG

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
- Remove all PWM channel configurations except 'all the channels for default remapping' configurations
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

[Unreleased]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.10.0...HEAD
[v0.10.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.9.0...v0.10.0
[v0.9.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.8.0...v0.9.0
[v0.8.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.7.0...v0.8.0
[v0.7.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.6.1...v0.7.0
[v0.6.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.6.0...v0.6.1
[v0.6.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.5.3...v0.6.0
[v0.5.3]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.5.2...v0.5.3
[v0.5.2]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.5.1...v0.5.2
[v0.5.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.5.0...v0.5.1
[v0.5.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.1...v0.3.0
[v0.2.1]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.1.1...v0.2.0
