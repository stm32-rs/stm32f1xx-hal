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

use core::convert::Infallible;
use core::marker::PhantomData;

use crate::afio;
use crate::hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};
use crate::pac::{self, EXTI};
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

pub trait PinExt {
    type Mode;
    /// Return pin number
    fn pin_id(&self) -> u8;
    /// Return port number
    fn port_id(&self) -> u8;
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
    Rising,
    Falling,
    RisingFalling,
}

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
}

use sealed::Interruptable;
impl<MODE> Interruptable for Input<MODE> {}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, afio: &mut afio::Parts);
    fn trigger_on_edge(&mut self, exti: &EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &EXTI);
    fn disable_interrupt(&mut self, exti: &EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&mut self) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    fn make_interrupt_source(&mut self, afio: &mut afio::Parts) {
        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                afio.exticr1.exticr1().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                afio.exticr2.exticr2().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                afio.exticr3.exticr3().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                afio.exticr4.exticr4().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge or both
    fn trigger_on_edge(&mut self, exti: &EXTI, edge: Edge) {
        let i = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    fn enable_interrupt(&mut self, exti: &EXTI) {
        exti.imr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    fn disable_interrupt(&mut self, exti: &EXTI) {
        exti.imr
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*EXTI::ptr()).pr.write(|w| w.bits(1 << self.pin_id())) };
    }

    /// Reads the interrupt pending bit for this pin
    fn check_interrupt(&mut self) -> bool {
        unsafe { ((*EXTI::ptr()).pr.read().bits() & (1 << self.pin_id())) != 0 }
    }
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
pub trait PinMode {
    type CR;
    unsafe fn set_mode(cr: &mut Self::CR) -> Self;
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
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $port_id:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $CR:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{APB2, Enable, Reset};
            use super::{Active, Floating, GpioExt, Input, Generic, Pxx, Pin, CRL, CRH, Crl, Crh};
            #[allow(unused)]
            use super::Debugger;

            /// GPIO parts
            pub struct Parts {
                /// Opaque CRL register
                pub crl: Crl<$port_id>,
                /// Opaque CRH register
                pub crh: Crh<$port_id>,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            $(
                pub type $PXi<MODE> = Pin<MODE, $CR, $port_id, $i>;
            )+

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, apb: &mut APB2) -> Parts {
                    $GPIOX::enable(apb);
                    $GPIOX::reset(apb);

                    Parts {
                        crl: Crl::<$port_id> { _0: () },
                        crh: Crh::<$port_id> { _0: () },
                        $(
                            $pxi: $PXi::new(<$MODE>::_new()),
                        )+
                    }
                }
            }

            impl<MODE> Generic<MODE, $port_id> {
                pub fn downgrade(self) -> Pxx<MODE> {
                    Pxx::$PXx(self)
                }
            }

            impl<MODE, CR, const N: u8> Pin<MODE, CR, $port_id, N>
            where
                MODE: Active,
            {
                /// Erases the pin number and port from the type
                ///
                /// This is useful when you want to collect the pins into an array where you
                /// need all the elements to have the same type
                pub fn downgrade(self) -> Pxx<MODE> {
                    self.into_generic().downgrade()
                }
            }
        }
    }
}

/// Partially erased pin. Only used in the Pxx enum
pub struct Generic<MODE, const P: char> {
    i: u8,
    _mode: PhantomData<MODE>,
}

impl<MODE, const P: char> PinExt for Generic<MODE, P> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        self.i
    }
    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - 0x41
    }
}

impl<MODE, const P: char> OutputPin for Generic<Output<MODE>, P> {
    type Error = Infallible;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << self.i)) };
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe {
            (*Gpio::<P>::ptr())
                .bsrr
                .write(|w| w.bits(1 << (16 + self.i)))
        };
        Ok(())
    }
}

impl<MODE, const P: char> InputPin for Generic<Input<MODE>, P> {
    type Error = Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << self.i) == 0 })
    }
}

impl<MODE, const P: char> StatefulOutputPin for Generic<Output<MODE>, P> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|b| !b)
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << self.i) == 0 })
    }
}

impl<MODE, const P: char> toggleable::Default for Generic<Output<MODE>, P> {}

impl<const P: char> InputPin for Generic<Output<OpenDrain>, P> {
    type Error = Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << self.i) == 0 })
    }
}

/// Pin
pub struct Pin<MODE, CR, const P: char, const N: u8> {
    mode: MODE,
    _cr: PhantomData<CR>,
}

impl<MODE, CR, const P: char, const N: u8> Pin<MODE, CR, P, N> {
    const OFFSET: u32 = (4 * (N as u32)) % 32;

    const fn new(mode: MODE) -> Self {
        Self {
            mode,
            _cr: PhantomData,
        }
    }
}

impl<MODE, CR, const P: char, const N: u8> PinExt for Pin<MODE, CR, P, N> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        N
    }
    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - 0x41
    }
}

impl<CR, const P: char, const N: u8> Pin<Debugger, CR, P, N> {
    /// Put the pin in an active state. The caller
    /// must enforce that the pin is really in this
    /// state in the hardware.
    #[allow(dead_code)]
    pub(crate) unsafe fn activate(self) -> Pin<Input<Floating>, CR, P, N> {
        Pin::new(Input::_new())
    }
}

impl<CR, const P: char, const N: u8> OutputPin for Pin<Dynamic, CR, P, N> {
    type Error = PinModeError;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self.set_state(State::High);
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self.set_state(State::Low);
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
}

impl<CR, const P: char, const N: u8> InputPin for Pin<Dynamic, CR, P, N> {
    type Error = PinModeError;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        if self.mode.is_input() {
            Ok(self._is_low())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
}

// Internal helper functions

// NOTE: The functions in this impl block are "safe", but they
// are callable when the pin is in modes where they don't make
// sense.
impl<MODE, CR, const P: char, const N: u8> Pin<MODE, CR, P, N> {
    /**
      Set the output of the pin regardless of its mode.
      Primarily used to set the output value of the pin
      before changing its mode to an output to avoid
      a short spike of an incorrect value
    */
    fn set_state(&mut self, state: State) {
        match state {
            State::High => unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) },
            State::Low => unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << (16 + N))) },
        }
    }

    fn _is_set_low(&self) -> bool {
        unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << N) == 0 }
    }

    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << N) == 0 }
    }
}

impl<MODE, CR, const P: char, const N: u8> Pin<MODE, CR, P, N>
where
    MODE: Active,
{
    /// Erases the pin number from the type
    #[inline]
    fn into_generic(self) -> Generic<MODE, P> {
        Generic {
            i: N,
            _mode: PhantomData,
        }
    }
}

// embedded_hal impls
impl<MODE, CR, const P: char, const N: u8> OutputPin for Pin<Output<MODE>, CR, P, N> {
    type Error = Infallible;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        self.set_state(State::High);
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        self.set_state(State::Low);
        Ok(())
    }
}

impl<MODE, CR, const P: char, const N: u8> StatefulOutputPin for Pin<Output<MODE>, CR, P, N> {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|b| !b)
    }

    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_low())
    }
}

impl<MODE, CR, const P: char, const N: u8> toggleable::Default for Pin<Output<MODE>, CR, P, N> {}

impl<MODE, CR, const P: char, const N: u8> InputPin for Pin<Input<MODE>, CR, P, N> {
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

impl<CR, const P: char, const N: u8> InputPin for Pin<Output<OpenDrain>, CR, P, N> {
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

macro_rules! cr {
    ($CR:ident, $Cr:ident, $cr:ident) => {
        pub struct $CR;

        /// Opaque CR register
        pub struct $Cr<const P: char> {
            _0: (),
        }

        impl<const P: char> $Cr<P> {
            // NOTE(allow) we get a warning on GPIOC because it only has 3 high pins
            #[allow(dead_code)]
            pub(crate) fn cr(&mut self) -> &pac::gpioa::$CR {
                unsafe { &(*Gpio::<P>::ptr()).$cr }
            }
        }

        impl<MODE, const P: char, const N: u8> Pin<MODE, $CR, P, N>
        where
            MODE: Active,
        {
            /// Configures the pin to operate as an alternate function push-pull output
            /// pin.
            #[inline]
            pub fn into_alternate_push_pull(
                self,
                cr: &mut $Cr<P>,
            ) -> Pin<Alternate<PushPull>, $CR, P, N> {
                // Alternate function output push pull
                const CNF: u32 = 0b10;
                // Output mode, max speed 50 MHz
                const MODE: u32 = 0b11;
                const BITS: u32 = (CNF << 2) | MODE;

                // input mode
                cr.cr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Alternate::_new())
            }

            /// Configures the pin to operate as an alternate function open-drain output
            /// pin.
            #[inline]
            pub fn into_alternate_open_drain(
                self,
                cr: &mut $Cr<P>,
            ) -> Pin<Alternate<OpenDrain>, $CR, P, N> {
                // Alternate function output open drain
                const CNF: u32 = 0b11;
                // Output mode, max speed 50 MHz
                const MODE: u32 = 0b11;
                const BITS: u32 = (CNF << 2) | MODE;

                // input mode
                cr.cr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Alternate::_new())
            }

            /// Configures the pin to operate as a floating input pin
            #[inline]
            pub fn into_floating_input(self, cr: &mut $Cr<P>) -> Pin<Input<Floating>, $CR, P, N> {
                unsafe { Pin::<Input<Floating>, $CR, P, N>::set_mode(cr) }
            }

            /// Configures the pin to operate as a pulled down input pin
            #[inline]
            pub fn into_pull_down_input(self, cr: &mut $Cr<P>) -> Pin<Input<PullDown>, $CR, P, N> {
                unsafe { Pin::<Input<PullDown>, $CR, P, N>::set_mode(cr) }
            }

            /// Configures the pin to operate as a pulled up input pin
            #[inline]
            pub fn into_pull_up_input(self, cr: &mut $Cr<P>) -> Pin<Input<PullUp>, $CR, P, N> {
                unsafe { Pin::<Input<PullUp>, $CR, P, N>::set_mode(cr) }
            }

            /// Configures the pin to operate as an open-drain output pin.
            /// Initial state will be low.
            #[inline]
            pub fn into_open_drain_output(
                self,
                cr: &mut $Cr<P>,
            ) -> Pin<Output<OpenDrain>, $CR, P, N> {
                self.into_open_drain_output_with_state(cr, State::Low)
            }

            /// Configures the pin to operate as an open-drain output pin.
            /// `initial_state` specifies whether the pin should be initially high or low.
            #[inline]
            pub fn into_open_drain_output_with_state(
                mut self,
                cr: &mut $Cr<P>,
                initial_state: State,
            ) -> Pin<Output<OpenDrain>, $CR, P, N> {
                self.set_state(initial_state);
                unsafe { Pin::<Output<OpenDrain>, $CR, P, N>::set_mode(cr) }
            }
            /// Configures the pin to operate as an push-pull output pin.
            /// Initial state will be low.
            #[inline]
            pub fn into_push_pull_output(
                self,
                cr: &mut $Cr<P>,
            ) -> Pin<Output<PushPull>, $CR, P, N> {
                self.into_push_pull_output_with_state(cr, State::Low)
            }

            /// Configures the pin to operate as an push-pull output pin.
            /// `initial_state` specifies whether the pin should be initially high or low.
            #[inline]
            pub fn into_push_pull_output_with_state(
                mut self,
                cr: &mut $Cr<P>,
                initial_state: State,
            ) -> Pin<Output<PushPull>, $CR, P, N> {
                self.set_state(initial_state);
                unsafe { Pin::<Output<PushPull>, $CR, P, N>::set_mode(cr) }
            }

            /// Configures the pin to operate as an analog input pin
            #[inline]
            pub fn into_analog(self, cr: &mut $Cr<P>) -> Pin<Analog, $CR, P, N> {
                unsafe { Pin::<Analog, $CR, P, N>::set_mode(cr) }
            }

            /// Configures the pin as a pin that can change between input
            /// and output without changing the type. It starts out
            /// as a floating input
            #[inline]
            pub fn into_dynamic(self, cr: &mut $Cr<P>) -> Pin<Dynamic, $CR, P, N> {
                self.into_floating_input(cr);
                Pin::new(Dynamic::InputFloating)
            }
        }

        // These macros are defined here instead of at the top level in order
        // to be able to refer to macro variables from the outer layers.
        macro_rules! impl_temp_output {
            ($fn_name:ident, $stateful_fn_name:ident, $mode:ty) => {
                /// Temporarily change the mode of the pin.
                ///
                /// The value of the pin after conversion is undefined. If you
                /// want to control it, use `$stateful_fn_name`
                #[inline]
                pub fn $fn_name(
                    &mut self,
                    cr: &mut $Cr<P>,
                    mut f: impl FnMut(&mut Pin<$mode, $CR, P, N>),
                ) {
                    let mut temp = unsafe { Pin::<$mode, $CR, P, N>::set_mode(cr) };
                    f(&mut temp);
                    unsafe {
                        Self::set_mode(cr);
                    }
                }

                /// Temporarily change the mode of the pin.
                ///
                /// Note that the new state is set slightly before conversion
                /// happens. This can cause a short output glitch if switching
                /// between output modes
                #[inline]
                pub fn $stateful_fn_name(
                    &mut self,
                    cr: &mut $Cr<P>,
                    state: State,
                    mut f: impl FnMut(&mut Pin<$mode, $CR, P, N>),
                ) {
                    self.set_state(state);
                    let mut temp = unsafe { Pin::<$mode, $CR, P, N>::set_mode(cr) };
                    f(&mut temp);
                    unsafe {
                        Self::set_mode(cr);
                    }
                }
            };
        }
        macro_rules! impl_temp_input {
            ($fn_name:ident, $mode:ty) => {
                /// Temporarily change the mode of the pin.
                #[inline]
                pub fn $fn_name(
                    &mut self,
                    cr: &mut $Cr<P>,
                    mut f: impl FnMut(&mut Pin<$mode, $CR, P, N>),
                ) {
                    let mut temp = unsafe { Pin::<$mode, $CR, P, N>::set_mode(cr) };
                    f(&mut temp);
                    unsafe {
                        Self::set_mode(cr);
                    }
                }
            };
        }

        impl<MODE, const P: char, const N: u8> Pin<MODE, $CR, P, N>
        where
            MODE: Active,
            Self: PinMode<CR = $Cr<P>>,
        {
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
            impl_temp_input!(as_floating_input, Input<Floating>);
            impl_temp_input!(as_pull_up_input, Input<PullUp>);
            impl_temp_input!(as_pull_down_input, Input<PullDown>);
        }

        impl<MODE, const P: char, const N: u8> OutputSpeed<$Cr<P>>
            for Pin<Output<MODE>, $CR, P, N>
        {
            fn set_speed(&mut self, cr: &mut $Cr<P>, speed: IOPinSpeed) {
                cr.cr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((speed as u32) << Self::OFFSET))
                });
            }
        }

        impl<const P: char, const N: u8> OutputSpeed<$Cr<P>>
            for Pin<Alternate<PushPull>, $CR, P, N>
        {
            fn set_speed(&mut self, cr: &mut $Cr<P>, speed: IOPinSpeed) {
                cr.cr().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0b11 << Self::OFFSET)) | ((speed as u32) << Self::OFFSET))
                });
            }
        }

        // Dynamic pin

        impl<const P: char, const N: u8> Pin<Dynamic, $CR, P, N> {
            #[inline]
            pub fn make_pull_up_input(&mut self, cr: &mut $Cr<P>) {
                // NOTE(unsafe), we have a mutable reference to the current pin
                unsafe { Pin::<Input<PullUp>, $CR, P, N>::set_mode(cr) };
                self.mode = Dynamic::InputPullUp;
            }
            #[inline]
            pub fn make_pull_down_input(&mut self, cr: &mut $Cr<P>) {
                // NOTE(unsafe), we have a mutable reference to the current pin
                unsafe { Pin::<Input<PullDown>, $CR, P, N>::set_mode(cr) };
                self.mode = Dynamic::InputPullDown;
            }
            #[inline]
            pub fn make_floating_input(&mut self, cr: &mut $Cr<P>) {
                // NOTE(unsafe), we have a mutable reference to the current pin
                unsafe { Pin::<Input<Floating>, $CR, P, N>::set_mode(cr) };
                self.mode = Dynamic::InputFloating;
            }
            #[inline]
            pub fn make_push_pull_output(&mut self, cr: &mut $Cr<P>) {
                // NOTE(unsafe), we have a mutable reference to the current pin
                unsafe { Pin::<Output<PushPull>, $CR, P, N>::set_mode(cr) };
                self.mode = Dynamic::OutputPushPull;
            }
            #[inline]
            pub fn make_open_drain_output(&mut self, cr: &mut $Cr<P>) {
                // NOTE(unsafe), we have a mutable reference to the current pin
                unsafe { Pin::<Output<OpenDrain>, $CR, P, N>::set_mode(cr) };
                self.mode = Dynamic::OutputOpenDrain;
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Input<Floating>, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // Floating input
                const CNF: u32 = 0b01;
                // Input mode
                const MODE: u32 = 0b00;
                const BITS: u32 = (CNF << 2) | MODE;

                // input mode
                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Input::_new())
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Input<PullDown>, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // Pull up/down input
                const CNF: u32 = 0b10;
                // Input mode
                const MODE: u32 = 0b00;
                const BITS: u32 = (CNF << 2) | MODE;

                //pull down:
                // NOTE(unsafe) atomic write to a stateless register
                (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << (16 + N)));

                // input mode
                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Input::_new())
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Input<PullUp>, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // Pull up/down input
                const CNF: u32 = 0b10;
                // Input mode
                const MODE: u32 = 0b00;
                const BITS: u32 = (CNF << 2) | MODE;

                //pull up:
                // NOTE(unsafe) atomic write to a stateless register
                (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N));

                // input mode
                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Input::_new())
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Output<OpenDrain>, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // General purpose output open-drain
                const CNF: u32 = 0b01;
                // Open-Drain Output mode, max speed 50 MHz
                const MODE: u32 = 0b11;
                const BITS: u32 = (CNF << 2) | MODE;

                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Output::_new())
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Output<PushPull>, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // General purpose output push-pull
                const CNF: u32 = 0b00;
                // Output mode, max speed 50 MHz
                const MODE: u32 = 0b11;
                const BITS: u32 = (CNF << 2) | MODE;

                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Output::_new())
            }
        }

        impl<const P: char, const N: u8> PinMode for Pin<Analog, $CR, P, N> {
            type CR = $Cr<P>;

            unsafe fn set_mode(cr: &mut Self::CR) -> Self {
                // Analog input
                const CNF: u32 = 0b00;
                // Input mode
                const MODE: u32 = 0b00;
                const BITS: u32 = (CNF << 2) | MODE;

                // analog mode
                cr.cr().modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << Self::OFFSET)) | (BITS << Self::OFFSET))
                });

                Pin::new(Analog {})
            }
        }
    };
}

cr!(CRH, Crh, crh);
cr!(CRL, Crl, crl);

macro_rules! impl_pxx {
    ($(($port_id:literal :: $pin:ident)),*) => {
        pub enum Pxx<MODE> {
            $(
                $pin(Generic<MODE, $port_id>)
            ),*
        }

        impl<MODE> PinExt for Pxx<MODE> {
            type Mode = MODE;

            #[inline(always)]
            fn pin_id(&self) -> u8 {
                match self {
                    $(Pxx::$pin(pin) => pin.pin_id()),*
                }
            }
            #[inline(always)]
            fn port_id(&self) -> u8 {
                match self {
                    $(Pxx::$pin(pin) => pin.port_id()),*
                }
            }
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
    }
}

impl_pxx! {
    ('A'::PAx),
    ('B'::PBx),
    ('C'::PCx),
    ('D'::PDx),
    ('E'::PEx)
}

gpio!(GPIOA, gpioa, PAx, 'A', [
    PA0: (pa0, 0, Input<Floating>, CRL),
    PA1: (pa1, 1, Input<Floating>, CRL),
    PA2: (pa2, 2, Input<Floating>, CRL),
    PA3: (pa3, 3, Input<Floating>, CRL),
    PA4: (pa4, 4, Input<Floating>, CRL),
    PA5: (pa5, 5, Input<Floating>, CRL),
    PA6: (pa6, 6, Input<Floating>, CRL),
    PA7: (pa7, 7, Input<Floating>, CRL),
    PA8: (pa8, 8, Input<Floating>, CRH),
    PA9: (pa9, 9, Input<Floating>, CRH),
    PA10: (pa10, 10, Input<Floating>, CRH),
    PA11: (pa11, 11, Input<Floating>, CRH),
    PA12: (pa12, 12, Input<Floating>, CRH),
    PA13: (pa13, 13, Debugger, CRH),
    PA14: (pa14, 14, Debugger, CRH),
    PA15: (pa15, 15, Debugger, CRH),
]);

gpio!(GPIOB, gpiob, PBx, 'B', [
    PB0: (pb0, 0, Input<Floating>, CRL),
    PB1: (pb1, 1, Input<Floating>, CRL),
    PB2: (pb2, 2, Input<Floating>, CRL),
    PB3: (pb3, 3, Debugger, CRL),
    PB4: (pb4, 4, Debugger, CRL),
    PB5: (pb5, 5, Input<Floating>, CRL),
    PB6: (pb6, 6, Input<Floating>, CRL),
    PB7: (pb7, 7, Input<Floating>, CRL),
    PB8: (pb8, 8, Input<Floating>, CRH),
    PB9: (pb9, 9, Input<Floating>, CRH),
    PB10: (pb10, 10, Input<Floating>, CRH),
    PB11: (pb11, 11, Input<Floating>, CRH),
    PB12: (pb12, 12, Input<Floating>, CRH),
    PB13: (pb13, 13, Input<Floating>, CRH),
    PB14: (pb14, 14, Input<Floating>, CRH),
    PB15: (pb15, 15, Input<Floating>, CRH),
]);

gpio!(GPIOC, gpioc, PCx, 'C', [
    PC0: (pc0, 0, Input<Floating>, CRL),
    PC1: (pc1, 1, Input<Floating>, CRL),
    PC2: (pc2, 2, Input<Floating>, CRL),
    PC3: (pc3, 3, Input<Floating>, CRL),
    PC4: (pc4, 4, Input<Floating>, CRL),
    PC5: (pc5, 5, Input<Floating>, CRL),
    PC6: (pc6, 6, Input<Floating>, CRL),
    PC7: (pc7, 7, Input<Floating>, CRL),
    PC8: (pc8, 8, Input<Floating>, CRH),
    PC9: (pc9, 9, Input<Floating>, CRH),
    PC10: (pc10, 10, Input<Floating>, CRH),
    PC11: (pc11, 11, Input<Floating>, CRH),
    PC12: (pc12, 12, Input<Floating>, CRH),
    PC13: (pc13, 13, Input<Floating>, CRH),
    PC14: (pc14, 14, Input<Floating>, CRH),
    PC15: (pc15, 15, Input<Floating>, CRH),
]);

gpio!(GPIOD, gpiod, PDx, 'D', [
    PD0: (pd0, 0, Input<Floating>, CRL),
    PD1: (pd1, 1, Input<Floating>, CRL),
    PD2: (pd2, 2, Input<Floating>, CRL),
    PD3: (pd3, 3, Input<Floating>, CRL),
    PD4: (pd4, 4, Input<Floating>, CRL),
    PD5: (pd5, 5, Input<Floating>, CRL),
    PD6: (pd6, 6, Input<Floating>, CRL),
    PD7: (pd7, 7, Input<Floating>, CRL),
    PD8: (pd8, 8, Input<Floating>, CRH),
    PD9: (pd9, 9, Input<Floating>, CRH),
    PD10: (pd10, 10, Input<Floating>, CRH),
    PD11: (pd11, 11, Input<Floating>, CRH),
    PD12: (pd12, 12, Input<Floating>, CRH),
    PD13: (pd13, 13, Input<Floating>, CRH),
    PD14: (pd14, 14, Input<Floating>, CRH),
    PD15: (pd15, 15, Input<Floating>, CRH),
]);

gpio!(GPIOE, gpioe, PEx, 'E', [
    PE0: (pe0, 0, Input<Floating>, CRL),
    PE1: (pe1, 1, Input<Floating>, CRL),
    PE2: (pe2, 2, Input<Floating>, CRL),
    PE3: (pe3, 3, Input<Floating>, CRL),
    PE4: (pe4, 4, Input<Floating>, CRL),
    PE5: (pe5, 5, Input<Floating>, CRL),
    PE6: (pe6, 6, Input<Floating>, CRL),
    PE7: (pe7, 7, Input<Floating>, CRL),
    PE8: (pe8, 8, Input<Floating>, CRH),
    PE9: (pe9, 9, Input<Floating>, CRH),
    PE10: (pe10, 10, Input<Floating>, CRH),
    PE11: (pe11, 11, Input<Floating>, CRH),
    PE12: (pe12, 12, Input<Floating>, CRH),
    PE13: (pe13, 13, Input<Floating>, CRH),
    PE14: (pe14, 14, Input<Floating>, CRH),
    PE15: (pe15, 15, Input<Floating>, CRH),
]);

struct Gpio<const P: char>;
impl<const P: char> Gpio<P> {
    const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
        match P {
            'A' => crate::pac::GPIOA::ptr(),
            'B' => crate::pac::GPIOB::ptr() as _,
            'C' => crate::pac::GPIOC::ptr() as _,
            'D' => crate::pac::GPIOD::ptr() as _,
            'E' => crate::pac::GPIOE::ptr() as _,
            _ => crate::pac::GPIOA::ptr(),
        }
    }
}
