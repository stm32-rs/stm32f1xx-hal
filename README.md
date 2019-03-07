# `stm32f1xx-hal`

> [HAL] for the STM32F1 family of microcontrollers

[HAL]: https://crates.io/crates/embedded-hal

[![crates.io](https://img.shields.io/crates/v/stm32f1xx-hal.svg)](https://crates.io/crates/stm32f1xx-hal)
[![Released API docs](https://docs.rs/stm32f1xx-hal/badge.svg)](https://docs.rs/stm32f1xx-hal)

## Usage

This crate supports multiple microcontrollers in the
stm32f1 family. Which specific microcontroller you want to build for has to be
specified with a feature, for example `stm32f103`.

If no microcontroller is specified, the crate will not compile.

### Building an Example

If you are compiling the crate on its own for development or running examples,
specify your microcontroller on the command line. For example:

```
cargo build --features stm32f103 --example led
```

### Using as a Dependency

When using this crate as a dependency in your project, the microcontroller can 
be specified as part of the `Cargo.toml` definition.

```
[dependencies.stm32f1xx-hal]
version = "0.2.0"
features = ["stm32f100", "rt"]
```

## Supported Microcontrollers

* STM32F100
* STM32F103

## Blinky example

The following example blinks an LED connected to pin PC13. For instructions on how set up a project and run the example, see the [documentation]

[documentation]: https://docs.rs/stm32f1xx-hal/

```rust
#![no_std]
#![no_main]

extern crate panic_halt;

use nb::block;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::Timer,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store
    // the frozen frequencies in `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, 1.hz(), clocks);

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();
    }
}
```

## Documentation

The documentation can be found at [docs.rs](https://docs.rs/stm32f1xx-hal/).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
