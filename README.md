# `stm32f1xx-hal`

> [HAL] for the STM32F1 family of microcontrollers

[HAL]: https://crates.io/crates/embedded-hal

## Usage

This crate will eventually contain support for multiple microcontrollers in the
stm32f1 family. Which specific microcontroller you want to build for has to be
specified with a feature, for example `stm32f103`.

```
cargo build --features stm32f103
```

If no device is specified, the crate does not compile.

Alternatively, adding the following to your project's `Cargo.toml` will target
your chip without needing the `--features` argument:

```
[features]
default = ["stm32f100", "rt"]
rt = ["stm32f1xx-hal/rt"]
stm32f100 = ["stm32f1xx-hal/stm32f100"]
```

## Supported Microcontrollers

* STM32F100
* STM32F103

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
