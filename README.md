# `stm32f1xx-hal`

> [HAL] for the STM32F1 family of microcontrollers

[HAL]: https://crates.io/crates/embedded-hal

## Usage

This crate will eventually contain support for multiple microcontrollers in the
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
