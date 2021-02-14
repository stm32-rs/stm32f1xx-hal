//! # General Purpose I/Os
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOC peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
//! ```
//!
//! See the documentation for [rcc::APB2](../rcc/struct.APB2.html) for details about the input parameter to
//! `split`.
//!
//! This gives you a struct containing two control registers `crl` and `crh`, and all the pins
//! `px0..px15`. These structs are what you use to interract with the pins to change their modes,
//! or their inputs or outputs. For example, to set `pa5` high, you would call
//!
//! ```rust
//! let output = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
//! output.set_high();
//! ```
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull up resistor. Will be high when nothing
//!     is connected
//!     - **PullDown**: Input connected to high with a weak pull up resistor. Will be low when nothing
//!     is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it do ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Debugger**: Some pins start out being used by the debugger. A pin in this mode can only be
//! used if the [JTAG peripheral has been turned off](#accessing-pa15-pb3-and-pb14).
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the `as_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode
//!
//! ## Accessing PA15, PB3, and PB14
//!
//! These pins are used by the JTAG peripheral by default. To use them in your program, you need to
//! disable that peripheral. This is done using the [afio::MAPR::disable_jtag](../afio/struct.MAPR.html#method.disable_jtag) function
//!
//! # Interfacing with v1 traits
//!
//! `embedded-hal` has two versions of the digital traits, `v2` which is used by this crate and
//! `v1` which is deprecated but still used by a lot of drivers.  If you want to use such a driver
//! with this crate, you need to convert the digital pins to the `v1` type.
//!
//! This is done using `embedded-hal::digital::v1_compat::OldOutputPin`. For example:
//!
//! ```rust
//! let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
//! let mut mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();
//! ```

use core::marker::PhantomData;

use crate::afio;
use crate::pac::EXTI;
use crate::rcc::APB2;

/// Slew rates available for Output and relevant AlternateMode Pins
///
/// See Table 21 "Output MODE bits" in the reference
pub enum IOPinSpeed {
    /// Slew at 10Mhz
    Mhz10 = 0b01, // (yes, this one is "less" then 2Mhz)
    /// Slew at 2Mhz
    Mhz2 = 0b10,
    /// Slew at 50Mhz
    Mhz50 = 0b11,
}

/// Allow setting of the slew rate of an IO pin
///
/// Initially all pins are set to the maximum slew rate
pub trait OutputSpeed<CR> {
    fn set_speed(&mut self, cr: &mut CR, speed: IOPinSpeed);
}

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, apb2: &mut APB2) -> Self::Parts;
}

/// Marker trait for pin mode detection.
pub trait Mode<MODE> {}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Input<MODE> {}

/// Used by the debugger (type state)
pub struct Debugger;
/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Output<MODE> {}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;
impl Active for Analog {}

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Alternate<MODE> {}

pub enum State {
    High,
    Low,
}

// Using SCREAMING_SNAKE_CASE to be consistent with other HALs
// see 59b2740 and #125 for motivation
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub enum Edge {
    RISING,
    FALLING,
    RISING_FALLING,
}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, afio: &mut afio::Parts);
    fn trigger_on_edge(&mut self, exti: &EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &EXTI);
    fn disable_interrupt(&mut self, exti: &EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&mut self) -> bool;
}

/// Tracks the current pin state for dynamic pins
pub enum Dynamic {
    InputFloating,
    InputPullUp,
    InputPullDown,
    OutputPushPull,
    OutputOpenDrain,
}

impl Active for Dynamic {}

#[derive(Debug, PartialEq)]
pub enum PinModeError {
    IncorrectMode,
}

impl Dynamic {
    fn is_input(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown | OutputOpenDrain => true,
            OutputPushPull => false,
        }
    }
    fn is_output(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown => false,
            OutputPushPull | OutputOpenDrain => true,
        }
    }
}

/// NOTE: This trait should ideally be private but must be pub in order to avoid
/// complaints from the compiler.
pub trait PinMode<CR> {
    unsafe fn set_mode(cr: &mut CR) -> Self;
}

// These impls are needed because a macro can not brace initialise a ty token
impl<MODE> Input<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Output<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl<MODE> Alternate<MODE> {
    const fn _new() -> Self {
        Self { _mode: PhantomData }
    }
}
impl Debugger {
    const fn _new() -> Self {
        Self {}
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $PXx:ident, $extigpionr:expr, $PX:ident, [
        $($CR:ident: [ $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $exticri:ident),)+ ],)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;

            use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use crate::pac::{$gpioy, $GPIOX};
            use crate::pac::EXTI;
            use crate::afio;

            use crate::rcc::{APB2, Enable, Reset};
            use super::{
                Alternate, Floating, GpioExt, Input,
                OpenDrain,
                Output,
                PullDown,
                PullUp,
                PushPull,
                Analog,
                State,
                Active,
                Debugger,
                Pxx,
                Mode,
                Edge,
                ExtiPin,
                PinMode,
                Dynamic,
                PinModeError,
                OutputSpeed,
                IOPinSpeed,
            };

            /// GPIO parts
            pub struct Parts {
                /// Opaque CRL register
                pub crl: CRL,
                /// Opaque CRH register
                pub crh: CRH,
                $(
                    $(
                        /// Pin
                        pub $pxi: $PX<$MODE, $i>,
                    )+
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, apb: &mut APB2) -> Parts {
                    $GPIOX::enable(apb);
                    $GPIOX::reset(apb);

                    Parts {
                        crl: CRL { _0: () },
                        crh: CRH { _0: () },
                        $(
                            $(
                                $pxi: $PX { mode: <$MODE>::_new() },
                            )+
                        )+
                    }
                }
            }

            /// Opaque CRL register
            pub struct CRL {
                _0: (),
            }

            impl CRL {
                // NOTE(allow) we get a warning on GPIOC because it only has 3 high pins
                #[allow(dead_code)]
                pub(crate) fn cr(&mut self) -> &$gpioy::CRL {
                    unsafe { &(*$GPIOX::ptr()).crl }
                }
            }

            /// Opaque CRH register
            pub struct CRH {
                _0: (),
            }

            impl CRH {
                pub(crate) fn cr(&mut self) -> &$gpioy::CRH {
                    unsafe { &(*$GPIOX::ptr()).crh }
                }
            }

            /// Partially erased pin. Only used in the Pxx enum
            pub struct Generic<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> Generic<MODE> {
                pub fn downgrade(self) -> Pxx<MODE> {
                    Pxx::$PXx(self)
                }
            }

            impl<MODE> OutputPin for Generic<Output<MODE>> {
                type Error = Infallible;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) })
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) })
                }
            }

            impl<MODE> InputPin for Generic<Input<MODE>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl<MODE> ExtiPin for Generic<Input<MODE>> {
                /// Make corresponding EXTI line sensitive to this pin
                fn make_interrupt_source(&mut self, afio: &mut afio::Parts) {
                    let offset = 4 * (self.i % 4);
                    match self.i {
                        0..=3 => {
                            afio.exticr1.exticr1().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        4..=7 => {
                            afio.exticr2.exticr2().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        8..=11 => {
                            afio.exticr3.exticr3().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        12..=15 => {
                            afio.exticr4.exticr4().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        _ => unreachable!(),
                    }
                }

                /// Generate interrupt on rising edge, falling edge or both
                fn trigger_on_edge(&mut self, exti: &EXTI, edge: Edge) {
                    match edge {
                        Edge::RISING => {
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                        },
                        Edge::FALLING => {
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                        },
                        Edge::RISING_FALLING => {
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                        }
                    }
                }

                /// Enable external interrupts from this pin.
                fn enable_interrupt(&mut self, exti: &EXTI) {
                    exti.imr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                }

                /// Disable external interrupts from this pin
                fn disable_interrupt(&mut self, exti: &EXTI) {
                    exti.imr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                }

                /// Clear the interrupt pending bit for this pin
                fn clear_interrupt_pending_bit(&mut self) {
                    unsafe { (*EXTI::ptr()).pr.write(|w| w.bits(1 << self.i) ) };
                }

                /// Reads the interrupt pending bit for this pin
                fn check_interrupt(&mut self) -> bool {
                    unsafe { ((*EXTI::ptr()).pr.read().bits() & (1 << self.i)) != 0 }
                }
            }

            impl <MODE> StatefulOutputPin for Generic<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl <MODE> toggleable::Default for Generic<Output<MODE>> {}

            impl InputPin for Generic<Output<OpenDrain>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            pub type $PXx<MODE> = Pxx<MODE>;

            impl<MODE> Mode<MODE> for Generic<MODE> {}

            /// Pin
            pub struct $PX<MODE, const N: u8> {
                mode: MODE,
            }
            impl<MODE, const N: u8> Mode<MODE> for $PX<MODE, N> {}
            impl<MODE, const N: u8> $PX<MODE, N> {
                const OFFSET: u32 = (4 * ({N} as u32)) % 32;
            }

            impl<MODE, const N: u8> $PX<MODE, N> where MODE: Active {
                /// Erases the pin number from the type
                #[inline]
                fn into_generic(self) -> Generic<MODE> {
                    Generic {
                        i: {N},
                        _mode: PhantomData,
                    }
                }

                /// Erases the pin number and port from the type
                ///
                /// This is useful when you want to collect the pins into an array where you
                /// need all the elements to have the same type
                pub fn downgrade(self) -> Pxx<MODE> {
                    self.into_generic().downgrade()
                }
            }

            impl<const N: u8> $PX<Debugger, N> {
                /// Put the pin in an active state. The caller
                /// must enforce that the pin is really in this
                /// state in the hardware.
                #[allow(dead_code)]
                pub(crate) unsafe fn activate(self) -> $PX<Input<Floating>, N> {
                    $PX { mode: Input::_new() }
                }
            }

            // Internal helper functions

            // NOTE: The functions in this impl block are "safe", but they
            // are callable when the pin is in modes where they don't make
            // sense.
            impl<MODE, const N: u8> $PX<MODE, N> {
                /**
                  Set the output of the pin regardless of its mode.
                  Primarily used to set the output value of the pin
                  before changing its mode to an output to avoid
                  a short spike of an incorrect value
                */
                fn set_state(&mut self, state: State) {
                    match state {
                        State::High => unsafe {
                            (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << {N}))
                        }
                        State::Low => unsafe {
                            (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + {N})))
                        }
                    }
                }

                fn _is_set_low(&self) -> bool {
                    unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << {N}) == 0 }
                }

                fn _is_low(&self) -> bool {
                    // NOTE(unsafe) atomic read with no side effects
                    unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << {N}) == 0 }
                }
            }

            // embedded_hal impls

            impl<MODE, const N: u8> OutputPin for $PX<Output<MODE>, N> {
                type Error = Infallible;
                #[inline]
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(self.set_state(State::High))
                }

                #[inline]
                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(self.set_state(State::Low))
                }
            }

            impl<MODE, const N: u8> StatefulOutputPin for $PX<Output<MODE>, N> {
                #[inline]
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                #[inline]
                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    Ok(self._is_set_low())
                }
            }

            impl<MODE, const N: u8> toggleable::Default for $PX<Output<MODE>, N> {}

            impl<MODE, const N: u8> InputPin for $PX<Input<MODE>, N> {
                type Error = Infallible;
                #[inline]
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                #[inline]
                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(self._is_low())
                }
            }

            impl<const N: u8> InputPin for $PX<Output<OpenDrain>, N> {
                type Error = Infallible;
                #[inline]
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                #[inline]
                fn is_low(&self) -> Result<bool, Self::Error> {
                    Ok(self._is_low())
                }
            }
            impl<const N: u8> OutputPin for $PX<Dynamic, N> {
                type Error = PinModeError;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    if self.mode.is_output() {
                        self.set_state(State::High);
                        Ok(())
                    }
                    else {
                        Err(PinModeError::IncorrectMode)
                    }
                }
                fn set_low(&mut self) -> Result<(), Self::Error> {
                    if self.mode.is_output() {
                        self.set_state(State::Low);
                        Ok(())
                    }
                    else {
                        Err(PinModeError::IncorrectMode)
                    }
                }
            }

            impl<const N: u8> InputPin for $PX<Dynamic, N> {
                type Error = PinModeError;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }
                fn is_low(&self) -> Result<bool, Self::Error> {
                    if self.mode.is_input() {
                        Ok(self._is_low())
                    }
                    else {
                        Err(PinModeError::IncorrectMode)
                    }
                }
            }

            impl<MODE, const N: u8> $PX<Input<MODE>, N> {
                #[inline(always)]
                fn _trigger_on_edge(&mut self, exti: &EXTI, edge: Edge) {
                    match edge {
                        Edge::RISING => {
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << {N})) });
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << {N})) });
                        },
                        Edge::FALLING => {
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << {N})) });
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << {N})) });
                        },
                        Edge::RISING_FALLING => {
                            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << {N})) });
                            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << {N})) });
                        }
                    }
                }
                #[inline(always)]
                fn _enable_interrupt(&mut self, exti: &EXTI) {
                    exti.imr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << {N})) });
                }

                #[inline(always)]
                fn _disable_interrupt(&mut self, exti: &EXTI) {
                    exti.imr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << {N})) });
                }
                #[inline(always)]
                fn _clear_interrupt_pending_bit(&mut self) {
                    unsafe { (*EXTI::ptr()).pr.write(|w| w.bits(1 << {N}) ) };
                }
                #[inline(always)]
                fn _check_interrupt(&mut self) -> bool {
                    unsafe { ((*EXTI::ptr()).pr.read().bits() & (1 << {N})) != 0 }
                }
            }

            $(
                $(
                    pub type $PXi<MODE> = $PX<MODE, $i>;

                    impl<MODE> $PX<MODE, $i> where MODE: Active {
                        /// Configures the pin to operate as an alternate function push-pull output
                        /// pin.
                        #[inline]
                        pub fn into_alternate_push_pull(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Alternate<PushPull>, $i> {
                            // Alternate function output push pull
                            const CNF: u32 = 0b10;
                            // Output mode, max speed 50 MHz
                            const MODE: u32 = 0b11;
                            const BITS: u32 = (CNF << 2) | MODE;

                            // input mode
                            cr
                                .cr()
                                .modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Alternate::_new() }
                        }

                        /// Configures the pin to operate as an alternate function open-drain output
                        /// pin.
                        #[inline]
                        pub fn into_alternate_open_drain(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Alternate<OpenDrain>, $i> {
                            // Alternate function output open drain
                            const CNF: u32 = 0b11;
                            // Output mode, max speed 50 MHz
                            const MODE: u32 = 0b11;
                            const BITS: u32 = (CNF << 2) | MODE;

                            // input mode
                            cr
                                .cr()
                                .modify(|r, w| unsafe {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Alternate::_new() }
                        }

                        /// Configures the pin to operate as a floating input pin
                        #[inline]
                        pub fn into_floating_input(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Input<Floating>, $i> {
                            unsafe {
                                $PX::<Input<Floating>, $i>::set_mode(cr)
                            }
                        }

                        /// Configures the pin to operate as a pulled down input pin
                        #[inline]
                        pub fn into_pull_down_input(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Input<PullDown>, $i> {
                            unsafe {
                                $PX::<Input<PullDown>, $i>::set_mode(cr)
                            }
                        }

                        /// Configures the pin to operate as a pulled up input pin
                        #[inline]
                        pub fn into_pull_up_input(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Input<PullUp>, $i> {
                            unsafe {
                                $PX::<Input<PullUp>, $i>::set_mode(cr)
                            }
                        }

                        /// Configures the pin to operate as an open-drain output pin.
                        /// Initial state will be low.
                        #[inline]
                        pub fn into_open_drain_output(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Output<OpenDrain>, $i> {
                            self.into_open_drain_output_with_state(cr, State::Low)
                        }

                        /// Configures the pin to operate as an open-drain output pin.
                        /// `initial_state` specifies whether the pin should be initially high or low.
                        #[inline]
                        pub fn into_open_drain_output_with_state(
                            mut self,
                            cr: &mut $CR,
                            initial_state: State,
                        ) -> $PX<Output<OpenDrain>, $i> {
                            self.set_state(initial_state);
                            unsafe {
                                $PX::<Output<OpenDrain>, $i>::set_mode(cr)
                            }
                        }
                        /// Configures the pin to operate as an push-pull output pin.
                        /// Initial state will be low.
                        #[inline]
                        pub fn into_push_pull_output(
                            self,
                            cr: &mut $CR,
                        ) -> $PX<Output<PushPull>, $i> {
                            self.into_push_pull_output_with_state(cr, State::Low)
                        }

                        /// Configures the pin to operate as an push-pull output pin.
                        /// `initial_state` specifies whether the pin should be initially high or low.
                        #[inline]
                        pub fn into_push_pull_output_with_state(
                            mut self,
                            cr: &mut $CR,
                            initial_state: State,
                        ) -> $PX<Output<PushPull>, $i> {
                            self.set_state(initial_state);
                            unsafe {
                                $PX::<Output<PushPull>, $i>::set_mode(cr)
                            }
                        }

                        /// Configures the pin to operate as an analog input pin
                        #[inline]
                        pub fn into_analog(self, cr: &mut $CR) -> $PX<Analog, $i> {
                            unsafe {
                                $PX::<Analog, $i>::set_mode(cr)
                            }
                        }

                        /// Configures the pin as a pin that can change between input
                        /// and output without changing the type. It starts out
                        /// as a floating input
                        #[inline]
                        pub fn into_dynamic(self, cr: &mut $CR) -> $PX<Dynamic, $i> {
                            self.into_floating_input(cr);
                            $PX::<Dynamic, $i>{mode: Dynamic::InputFloating}
                        }
                    }

                    // These macros are defined here instead of at the top level in order
                    // to be able to refer to macro variables from the outer layers.
                    macro_rules! impl_temp_output {
                        (
                            $fn_name:ident,
                            $stateful_fn_name:ident,
                            $mode:ty
                        ) => {
                            /**
                              Temporarily change the mode of the pin.

                              The value of the pin after conversion is undefined. If you
                              want to control it, use `$stateful_fn_name`
                            */
                            #[inline]
                            pub fn $fn_name(
                                &mut self,
                                cr: &mut $CR,
                                mut f: impl FnMut(&mut $PX<$mode, $i>)
                            ) {
                                let mut temp = unsafe { $PX::<$mode, $i>::set_mode(cr) };
                                f(&mut temp);
                                unsafe {
                                    Self::set_mode(cr);
                                }
                            }

                            /**
                              Temporarily change the mode of the pin.

                              Note that the new state is set slightly before conversion
                              happens. This can cause a short output glitch if switching
                              between output modes
                            */
                            #[inline]
                            pub fn $stateful_fn_name(
                                &mut self,
                                cr: &mut $CR,
                                state: State,
                                mut f: impl FnMut(&mut $PX<$mode, $i>)
                            ) {
                                self.set_state(state);
                                let mut temp = unsafe { $PX::<$mode, $i>::set_mode(cr) };
                                f(&mut temp);
                                unsafe {
                                    Self::set_mode(cr);
                                }
                            }
                        }
                    }
                    macro_rules! impl_temp_input {
                        (
                            $fn_name:ident,
                            $mode:ty
                        ) => {
                            /**
                              Temporarily change the mode of the pin.
                            */
                            #[inline]
                            pub fn $fn_name(
                                &mut self,
                                cr: &mut $CR,
                                mut f: impl FnMut(&mut $PX<$mode, $i>)
                            ) {
                                let mut temp = unsafe { $PX::<$mode, $i>::set_mode(cr) };
                                f(&mut temp);
                                unsafe {
                                    Self::set_mode(cr);
                                }
                            }
                        }
                    }

                    impl<MODE> $PX<MODE, $i> where MODE: Active, $PX<MODE, $i>: PinMode<$CR> {
                        impl_temp_output!(
                            as_push_pull_output,
                            as_push_pull_output_with_state,
                            Output<PushPull>
                        );
                        impl_temp_output!(
                            as_open_drain_output,
                            as_open_drain_output_with_state,
                            Output<OpenDrain>
                        );
                        impl_temp_input!(
                            as_floating_input,
                            Input<Floating>
                        );
                        impl_temp_input!(
                            as_pull_up_input,
                            Input<PullUp>
                        );
                        impl_temp_input!(
                            as_pull_down_input,
                            Input<PullDown>
                        );
                    }

                    impl<MODE> OutputSpeed<$CR> for $PX<Output<MODE>, $i> {
                        fn set_speed(&mut self, cr: &mut $CR, speed: IOPinSpeed){
                            cr.cr().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((speed as u32) << Self::OFFSET))
                            });
                        }
                    }

                    impl OutputSpeed<$CR> for $PX<Alternate<PushPull>, $i> {
                        fn set_speed(&mut self, cr: &mut $CR, speed: IOPinSpeed){
                            cr.cr().modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((speed as u32) << Self::OFFSET))
                            });
                        }
                    }

                    // Dynamic pin

                    impl $PX<Dynamic, $i> {
                        #[inline]
                        pub fn make_pull_up_input(&mut self, cr: &mut $CR) {
                            // NOTE(unsafe), we have a mutable reference to the current pin
                            unsafe { $PX::<Input<PullUp>, $i>::set_mode(cr) };
                            self.mode = Dynamic::InputPullUp;
                        }
                        #[inline]
                        pub fn make_pull_down_input(&mut self, cr: &mut $CR) {
                            // NOTE(unsafe), we have a mutable reference to the current pin
                            unsafe { $PX::<Input<PullDown>, $i>::set_mode(cr) };
                            self.mode = Dynamic::InputPullDown;
                        }
                        #[inline]
                        pub fn make_floating_input(&mut self, cr: &mut $CR) {
                            // NOTE(unsafe), we have a mutable reference to the current pin
                            unsafe { $PX::<Input<Floating>, $i>::set_mode(cr) };
                            self.mode = Dynamic::InputFloating;
                        }
                        #[inline]
                        pub fn make_push_pull_output(&mut self, cr: &mut $CR) {
                            // NOTE(unsafe), we have a mutable reference to the current pin
                            unsafe { $PX::<Output<PushPull>, $i>::set_mode(cr) };
                            self.mode = Dynamic::OutputPushPull;
                        }
                        #[inline]
                        pub fn make_open_drain_output(&mut self, cr: &mut $CR) {
                            // NOTE(unsafe), we have a mutable reference to the current pin
                            unsafe { $PX::<Output<OpenDrain>, $i>::set_mode(cr) };
                            self.mode = Dynamic::OutputOpenDrain;
                        }
                    }

                    // Exti pin impls

                    impl<MODE> ExtiPin for $PX<Input<MODE>, $i> {
                        /// Configure EXTI Line $i to trigger from this pin.
                        fn make_interrupt_source(&mut self, afio: &mut afio::Parts) {
                            let offset = 4 * ($i % 4);
                            afio.$exticri.$exticri().modify(|r, w| unsafe {
                                let mut exticr = r.bits();
                                exticr = (exticr & !(0xf << offset)) | ($extigpionr << offset);
                                w.bits(exticr)
                            });
                        }

                        /// Generate interrupt on rising edge, falling edge or both
                        fn trigger_on_edge(&mut self, exti: &EXTI, edge: Edge) {
                            self._trigger_on_edge(exti, edge);
                        }

                        /// Enable external interrupts from this pin.
                        #[inline]
                        fn enable_interrupt(&mut self, exti: &EXTI) {
                            self._enable_interrupt(exti);
                        }

                        /// Disable external interrupts from this pin
                        #[inline]
                        fn disable_interrupt(&mut self, exti: &EXTI) {
                            self._disable_interrupt(exti);
                        }

                        /// Clear the interrupt pending bit for this pin
                        #[inline]
                        fn clear_interrupt_pending_bit(&mut self) {
                            self._clear_interrupt_pending_bit();
                        }

                        /// Reads the interrupt pending bit for this pin
                        #[inline]
                        fn check_interrupt(&mut self) -> bool {
                            self._check_interrupt()
                        }
                    }

                    impl PinMode<$CR> for $PX<Input<Floating>, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // Floating input
                            const CNF: u32 = 0b01;
                            // Input mode
                            const MODE: u32 = 0b00;
                            const BITS: u32 = (CNF << 2) | MODE;

                            // input mode
                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Input::_new() }
                        }
                    }

                    impl PinMode<$CR> for $PX<Input<PullDown>, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // Pull up/down input
                            const CNF: u32 = 0b10;
                            // Input mode
                            const MODE: u32 = 0b00;
                            const BITS: u32 = (CNF << 2) | MODE;

                            //pull down:
                            // NOTE(unsafe) atomic write to a stateless register
                            (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i)));

                            // input mode
                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Input::_new() }
                        }
                    }


                    impl PinMode<$CR> for $PX<Input<PullUp>, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // Pull up/down input
                            const CNF: u32 = 0b10;
                            // Input mode
                            const MODE: u32 = 0b00;
                            const BITS: u32 = (CNF << 2) | MODE;

                            //pull up:
                            // NOTE(unsafe) atomic write to a stateless register
                            (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i));

                            // input mode
                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Input::_new() }
                        }
                    }

                    impl PinMode<$CR> for $PX<Output<OpenDrain>, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // General purpose output open-drain
                            const CNF: u32 = 0b01;
                            // Open-Drain Output mode, max speed 50 MHz
                            const MODE: u32 = 0b11;
                            const BITS: u32 = (CNF << 2) | MODE;

                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Output::_new() }
                        }
                    }

                    impl PinMode<$CR> for $PX<Output<PushPull>, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // General purpose output push-pull
                            const CNF: u32 = 0b00;
                            // Output mode, max speed 50 MHz
                            const MODE: u32 = 0b11;
                            const BITS: u32 = (CNF << 2) | MODE;


                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Output::_new() }
                        }
                    }

                    impl PinMode<$CR> for $PX<Analog, $i> {
                        unsafe fn set_mode(cr: &mut $CR) -> Self {
                            // Analog input
                            const CNF: u32 = 0b00;
                            // Input mode
                            const MODE: u32 = 0b00;
                            const BITS: u32 = (CNF << 2) | MODE;

                            // analog mode
                            cr
                                .cr()
                                .modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                                });

                            $PX { mode: Analog{} }
                        }
                    }
                )+
            )+
        }
    }
}

macro_rules! impl_pxx {
    ($(($port:ident :: $pin:ident)),*) => {
        use embedded_hal::digital::v2::{InputPin, StatefulOutputPin, OutputPin};
        use core::convert::Infallible;

        pub enum Pxx<MODE> {
            $(
                $pin($port::Generic<MODE>)
            ),*
        }

        impl<MODE> OutputPin for Pxx<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_high()),*
                }
            }

            fn set_low(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_low()),*
                }
            }
        }

        impl<MODE> StatefulOutputPin for Pxx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_high()),*
                }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> InputPin for Pxx<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl InputPin for Pxx<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl<MODE> ExtiPin for Pxx<Input<MODE>> {
            fn make_interrupt_source(&mut self, afio: &mut afio::Parts) {
                match self {
                    $(Pxx::$pin(pin) => pin.make_interrupt_source(afio)),*
                }
            }

            fn trigger_on_edge(&mut self, exti: &EXTI, level: Edge) {
                match self {
                    $(Pxx::$pin(pin) => pin.trigger_on_edge(exti, level)),*
                }
            }

            fn enable_interrupt(&mut self, exti: &EXTI) {
                match self {
                    $(Pxx::$pin(pin) => pin.enable_interrupt(exti)),*
                }
            }

            fn disable_interrupt(&mut self, exti: &EXTI) {
                match self {
                    $(Pxx::$pin(pin) => pin.disable_interrupt(exti)),*
                }
            }

            fn clear_interrupt_pending_bit(&mut self) {
                match self {
                    $(Pxx::$pin(pin) => pin.clear_interrupt_pending_bit()),*
                }
            }

            fn check_interrupt(&mut self) -> bool {
                match self {
                    $(Pxx::$pin(pin) => pin.check_interrupt()),*
                }
            }
        }
    }
}

impl_pxx! {
    (gpioa::PAx),
    (gpiob::PBx),
    (gpioc::PCx),
    (gpiod::PDx),
    (gpioe::PEx)
}

gpio!(GPIOA, gpioa, gpioa, PAx, 0, PA, [
    CRL: [
        PA0: (pa0, 0, Input<Floating>, exticr1),
        PA1: (pa1, 1, Input<Floating>, exticr1),
        PA2: (pa2, 2, Input<Floating>, exticr1),
        PA3: (pa3, 3, Input<Floating>, exticr1),
        PA4: (pa4, 4, Input<Floating>, exticr2),
        PA5: (pa5, 5, Input<Floating>, exticr2),
        PA6: (pa6, 6, Input<Floating>, exticr2),
        PA7: (pa7, 7, Input<Floating>, exticr2),
    ],
    CRH: [
        PA8: (pa8, 8, Input<Floating>, exticr3),
        PA9: (pa9, 9, Input<Floating>, exticr3),
        PA10: (pa10, 10, Input<Floating>, exticr3),
        PA11: (pa11, 11, Input<Floating>, exticr3),
        PA12: (pa12, 12, Input<Floating>, exticr4),
        PA13: (pa13, 13, Debugger, exticr4),
        PA14: (pa14, 14, Debugger, exticr4),
        PA15: (pa15, 15, Debugger, exticr4),
    ],
]);

gpio!(GPIOB, gpiob, gpioa, PBx, 1, PB, [
    CRL: [
        PB0: (pb0, 0, Input<Floating>, exticr1),
        PB1: (pb1, 1, Input<Floating>, exticr1),
        PB2: (pb2, 2, Input<Floating>, exticr1),
        PB3: (pb3, 3, Debugger, exticr1),
        PB4: (pb4, 4, Debugger, exticr2),
        PB5: (pb5, 5, Input<Floating>, exticr2),
        PB6: (pb6, 6, Input<Floating>, exticr2),
        PB7: (pb7, 7, Input<Floating>, exticr2),
    ],
    CRH: [
        PB8: (pb8, 8, Input<Floating>, exticr3),
        PB9: (pb9, 9, Input<Floating>, exticr3),
        PB10: (pb10, 10, Input<Floating>, exticr3),
        PB11: (pb11, 11, Input<Floating>, exticr3),
        PB12: (pb12, 12, Input<Floating>, exticr4),
        PB13: (pb13, 13, Input<Floating>, exticr4),
        PB14: (pb14, 14, Input<Floating>, exticr4),
        PB15: (pb15, 15, Input<Floating>, exticr4),
    ],
]);

gpio!(GPIOC, gpioc, gpioa, PCx, 2, PC, [
    CRL: [
        PC0: (pc0, 0, Input<Floating>, exticr1),
        PC1: (pc1, 1, Input<Floating>, exticr1),
        PC2: (pc2, 2, Input<Floating>, exticr1),
        PC3: (pc3, 3, Input<Floating>, exticr1),
        PC4: (pc4, 4, Input<Floating>, exticr2),
        PC5: (pc5, 5, Input<Floating>, exticr2),
        PC6: (pc6, 6, Input<Floating>, exticr2),
        PC7: (pc7, 7, Input<Floating>, exticr2),
    ],
    CRH: [
        PC8: (pc8, 8, Input<Floating>, exticr3),
        PC9: (pc9, 9, Input<Floating>, exticr3),
        PC10: (pc10, 10, Input<Floating>, exticr3),
        PC11: (pc11, 11, Input<Floating>, exticr3),
        PC12: (pc12, 12, Input<Floating>, exticr4),
        PC13: (pc13, 13, Input<Floating>, exticr4),
        PC14: (pc14, 14, Input<Floating>, exticr4),
        PC15: (pc15, 15, Input<Floating>, exticr4),
    ],
]);

gpio!(GPIOD, gpiod, gpioa, PDx, 3, PD, [
    CRL: [
        PD0: (pd0, 0, Input<Floating>, exticr1),
        PD1: (pd1, 1, Input<Floating>, exticr1),
        PD2: (pd2, 2, Input<Floating>, exticr1),
        PD3: (pd3, 3, Input<Floating>, exticr1),
        PD4: (pd4, 4, Input<Floating>, exticr2),
        PD5: (pd5, 5, Input<Floating>, exticr2),
        PD6: (pd6, 6, Input<Floating>, exticr2),
        PD7: (pd7, 7, Input<Floating>, exticr2),
    ],
    CRH: [
        PD8: (pd8, 8, Input<Floating>, exticr3),
        PD9: (pd9, 9, Input<Floating>, exticr3),
        PD10: (pd10, 10, Input<Floating>, exticr3),
        PD11: (pd11, 11, Input<Floating>, exticr3),
        PD12: (pd12, 12, Input<Floating>, exticr4),
        PD13: (pd13, 13, Input<Floating>, exticr4),
        PD14: (pd14, 14, Input<Floating>, exticr4),
        PD15: (pd15, 15, Input<Floating>, exticr4),
    ],
]);

gpio!(GPIOE, gpioe, gpioa, PEx, 4, PE, [
    CRL: [
        PE0: (pe0, 0, Input<Floating>, exticr1),
        PE1: (pe1, 1, Input<Floating>, exticr1),
        PE2: (pe2, 2, Input<Floating>, exticr1),
        PE3: (pe3, 3, Input<Floating>, exticr1),
        PE4: (pe4, 4, Input<Floating>, exticr2),
        PE5: (pe5, 5, Input<Floating>, exticr2),
        PE6: (pe6, 6, Input<Floating>, exticr2),
        PE7: (pe7, 7, Input<Floating>, exticr2),
    ],
    CRH: [
        PE8: (pe8, 8, Input<Floating>, exticr3),
        PE9: (pe9, 9, Input<Floating>, exticr3),
        PE10: (pe10, 10, Input<Floating>, exticr3),
        PE11: (pe11, 11, Input<Floating>, exticr3),
        PE12: (pe12, 12, Input<Floating>, exticr4),
        PE13: (pe13, 13, Input<Floating>, exticr4),
        PE14: (pe14, 14, Input<Floating>, exticr4),
        PE15: (pe15, 15, Input<Floating>, exticr4),
    ],
]);
