//! # General Purpose I/Os
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOC peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split();
//! ```
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
//! ## Modes
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull-up resistor. Will be high when nothing
//!       is connected
//!     - **PullDown**: Input connected to ground with a weak pull-down resistor. Will be low when nothing
//!       is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!       mode. Can be used as an input in the `open` configuration
//! - **Debugger**: Some pins start out being used by the debugger. A pin in this mode can only be
//!   used if the [JTAG peripheral has been turned off](#accessing-pa15-pb3-and-pb14).
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
use crate::pac::EXTI;

mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
mod erased;
pub use erased::{AnyPin, ErasedPin};

mod hal_02;
mod hal_1;

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

impl From<IOPinSpeed> for Mode {
    fn from(value: IOPinSpeed) -> Self {
        match value {
            IOPinSpeed::Mhz10 => Self::Output,
            IOPinSpeed::Mhz2 => Self::Output2,
            IOPinSpeed::Mhz50 => Self::Output50,
        }
    }
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
pub trait OutputSpeed: HL {
    fn set_speed(&mut self, cr: &mut Self::Cr, speed: IOPinSpeed);
}

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers.
    ///
    /// This resets the state of the GPIO block.
    fn split(self) -> Self::Parts;

    /// Splits the GPIO block into independent pins and registers without resetting its state.
    ///
    /// # Safety
    /// Make sure that all pins modes are set in reset state.
    unsafe fn split_without_reset(self) -> Self::Parts;
}

#[derive(Debug, Default)]
pub struct NoPin<MODE>(PhantomData<MODE>);
impl<MODE> NoPin<MODE> {
    pub fn new() -> Self {
        Self(PhantomData)
    }
}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
#[derive(Default)]
pub struct Input;

/// Pull setting for an input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    /// Floating
    None = 0,
    /// Pulled up
    Up = 1,
    /// Pulled down
    Down = 2,
}

#[derive(Default)]
struct PulledInput;

impl Active for Input {}

/// Used by the debugger (type state)
#[derive(Default)]
pub struct Debugger;

/// Output mode (type state)
#[derive(Default)]
pub struct Output<Otype = PushPull>(PhantomData<Otype>);

impl<MODE> Active for Output<MODE> {}

/// Push pull output (type state)
#[derive(Default)]
pub struct PushPull;

/// Open drain output (type state)
#[derive(Default)]
pub struct OpenDrain;

/// Analog mode (type state)
#[derive(Default)]
pub struct Analog;

impl Active for Analog {}

/// Alternate function
#[derive(Default)]
pub struct Alternate<Otype = PushPull>(PhantomData<Otype>);

impl<MODE> Active for Alternate<MODE> {}

/// Digital output pin state
pub use embedded_hal_02::digital::v2::PinState;

// Using SCREAMING_SNAKE_CASE to be consistent with other HALs
// see 59b2740 and #125 for motivation
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}

    pub trait PinMode: Default {
        const CNF: super::Cnf;
        const MODE: super::Mode;
    }
}

use crate::pac::gpioa::crl::{CNF0 as Cnf, MODE0 as Mode};

use sealed::Interruptable;
pub(crate) use sealed::PinMode;

impl Interruptable for Input {}
impl Interruptable for Dynamic {}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, afio: &mut afio::Parts);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    fn make_interrupt_source(&mut self, afio: &mut afio::Parts) {
        let pin_number = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (pin_number % 4);
        match pin_number {
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
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let pin_number = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
                exti.ftsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin_number)) });
            }
            Edge::Falling => {
                exti.rtsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin_number)) });
                exti.ftsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
            }
            Edge::RisingFalling => {
                exti.rtsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
                exti.ftsr()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*EXTI::ptr()).pr().write(|w| w.bits(1 << self.pin_id())) };
    }

    /// Reads the interrupt pending bit for this pin
    fn check_interrupt(&self) -> bool {
        unsafe { ((*EXTI::ptr()).pr().read().bits() & (1 << self.pin_id())) != 0 }
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

impl Default for Dynamic {
    fn default() -> Self {
        Dynamic::InputFloating
    }
}

impl Active for Dynamic {}

#[derive(Debug, PartialEq, Eq)]
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

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $port_id:expr, [
        $($PXi:ident: ($pxi:ident, $pin_number:expr $(, $MODE:ty)?),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::pac::{$GPIOX, RCC};
            use crate::rcc::{Enable, Reset};
            use super::{Active, GpioExt, Input, PartiallyErasedPin, ErasedPin, Pin, Cr};
            #[allow(unused)]
            use super::Debugger;

            /// GPIO parts
            pub struct Parts {
                /// Opaque CRL register
                pub crl: Cr<$port_id, false>,
                /// Opaque CRH register
                pub crh: Cr<$port_id, true>,
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            $(
                pub type $PXi<MODE = Input> = Pin<$port_id, $pin_number, MODE>;
            )+

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $GPIOX::enable(rcc);
                    $GPIOX::reset(rcc);

                    Parts {
                        crl: Cr::<$port_id, false>,
                        crh: Cr::<$port_id, true>,
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }

                unsafe fn split_without_reset(self) -> Parts {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $GPIOX::enable(rcc);

                    Parts {
                        crl: Cr::<$port_id, false>,
                        crh: Cr::<$port_id, true>,
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }

            impl<MODE> PartiallyErasedPin<$port_id, MODE> {
                pub fn erase(self) -> ErasedPin<MODE> {
                    ErasedPin::$PXx(self)
                }
            }

            impl<const N: u8, MODE> Pin<$port_id, N, MODE>
            where
                MODE: Active,
            {
                /// Erases the pin number and port from the type
                ///
                /// This is useful when you want to collect the pins into an array where you
                /// need all the elements to have the same type
                pub fn erase(self) -> ErasedPin<MODE> {
                    self.erase_number().erase()
                }
            }
        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

/// Generic pin type
///
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
pub struct Pin<const P: char, const N: u8, MODE = Input> {
    mode: MODE,
}

/// Represents high or low configuration register
pub trait HL {
    /// Configuration register associated to pin
    type Cr;
}

macro_rules! cr {
    ($cr_is_h:literal: [$($pin_number:literal),+]) => {
        $(
            impl<const P: char, MODE> HL for Pin<P, $pin_number, MODE> {
                type Cr = Cr<P, $cr_is_h>;
            }
        )+
    }
}

cr!(false: [0, 1, 2, 3, 4, 5, 6, 7]);
cr!(true: [8, 9, 10, 11, 12, 13, 14, 15]);

impl<const P: char, const N: u8, MODE: Default> Pin<P, N, MODE> {
    fn new() -> Self {
        Self {
            mode: Default::default(),
        }
    }
}

impl<const P: char, const N: u8, MODE> PinExt for Pin<P, N, MODE> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        N
    }

    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - b'A'
    }
}

impl<const P: char, const N: u8> Pin<P, N, Debugger> {
    /// Put the pin in an active state. The caller
    /// must enforce that the pin is really in this
    /// state in the hardware.
    #[allow(dead_code)]
    pub(crate) unsafe fn activate(self) -> Pin<P, N, Input> {
        Pin::new()
    }
}

// Internal helper functions

// NOTE: The functions in this impl block are "safe", but they
// are callable when the pin is in modes where they don't make
// sense.
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /**
      Set the output of the pin regardless of its mode.
      Primarily used to set the output value of the pin
      before changing its mode to an output to avoid
      a short spike of an incorrect value
    */

    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }

    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.bs(N).set_bit())
    }

    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.br(N).set_bit())
    }

    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.odr().read().odr(N).bit_is_clear()
    }

    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.idr().read().idr(N).bit_is_clear()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active,
{
    /// Erases the pin number from the type
    #[inline]
    pub fn erase_number(self) -> PartiallyErasedPin<P, MODE> {
        PartiallyErasedPin::new(N)
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    #[inline]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self._is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        self._set_state(state)
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self._is_set_low()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self._is_set_low() {
            self._set_high()
        } else {
            self._set_low()
        }
    }
}

impl<const P: char, const N: u8> Pin<P, N, Input> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    fn _set_pull_up(&mut self) {
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.bs(N).set_bit());
    }
    fn _pull_up(mut self) -> Self {
        self._set_pull_up();
        self
    }
    fn _set_pull_down(&mut self) {
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.br(N).set_bit());
    }
    fn _pull_down(mut self) -> Self {
        self._set_pull_down();
        self
    }
}
impl<const P: char, const N: u8> Pin<P, N, Input> {
    /// Set the internal pull-up and pull-down resistor
    pub fn set_internal_resistor(&mut self, pull: Pull) {
        match pull {
            Pull::None => {
                let gpio = unsafe { &(*gpiox::<P>()) };
                match N {
                    0..=7 => {
                        gpio.crl().modify(|_, w| w.cnf(N).variant(Cnf::OpenDrain));
                    }
                    8..=15 => {
                        gpio.crh()
                            .modify(|_, w| unsafe { w.cnf(N - 16).bits(Cnf::OpenDrain as u8) });
                    }
                    _ => unreachable!(),
                }
            }
            Pull::Up => {
                self.mode::<PulledInput>();
                self._set_pull_up()
            }
            Pull::Down => {
                self.mode::<PulledInput>();
                self._set_pull_down()
            }
        }
    }

    /// Set the internal pull-up and pull-down resistor
    pub fn internal_resistor(mut self, pull: Pull) -> Self {
        self.set_internal_resistor(pull);
        self
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

/// Opaque CR register
#[non_exhaustive]
pub struct Cr<const P: char, const H: bool>;

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active,
    Self: HL,
{
    /// Configures the pin to operate as an alternate function push-pull output
    /// pin.
    #[inline]
    pub fn into_alternate_push_pull(
        self,
        cr: &mut <Self as HL>::Cr,
    ) -> Pin<P, N, Alternate<PushPull>> {
        self.into_mode(cr)
    }

    /// Configures the pin to operate as an alternate function open-drain output
    /// pin.
    #[inline]
    pub fn into_alternate_open_drain(
        self,
        cr: &mut <Self as HL>::Cr,
    ) -> Pin<P, N, Alternate<OpenDrain>> {
        self.into_mode(cr)
    }

    /// Configures the pin to operate as a floating input pin
    #[inline]
    pub fn into_floating_input(self, cr: &mut <Self as HL>::Cr) -> Pin<P, N, Input> {
        self.into_mode(cr)
    }

    /// Configures the pin to operate as a pulled down input pin
    #[inline]
    pub fn into_pull_down_input(mut self, _cr: &mut <Self as HL>::Cr) -> Pin<P, N, Input> {
        self.mode::<PulledInput>();
        Pin::new()._pull_down()
    }

    /// Configures the pin to operate as a pulled up input pin
    #[inline]
    pub fn into_pull_up_input(mut self, _cr: &mut <Self as HL>::Cr) -> Pin<P, N, Input> {
        self.mode::<PulledInput>();
        Pin::new()._pull_up()
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// Initial state will be low.
    #[inline]
    pub fn into_open_drain_output(self, cr: &mut <Self as HL>::Cr) -> Pin<P, N, Output<OpenDrain>> {
        self.into_open_drain_output_with_state(cr, PinState::Low)
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    #[inline]
    pub fn into_open_drain_output_with_state(
        mut self,
        cr: &mut <Self as HL>::Cr,
        initial_state: PinState,
    ) -> Pin<P, N, Output<OpenDrain>> {
        self._set_state(initial_state);
        self.into_mode(cr)
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// Initial state will be low.
    #[inline]
    pub fn into_push_pull_output(self, cr: &mut <Self as HL>::Cr) -> Pin<P, N, Output<PushPull>> {
        self.into_push_pull_output_with_state(cr, PinState::Low)
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    #[inline]
    pub fn into_push_pull_output_with_state(
        mut self,
        cr: &mut <Self as HL>::Cr,
        initial_state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_state(initial_state);
        self.into_mode(cr)
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// The state will not be changed.
    #[inline]
    pub fn into_push_pull_output_with_current_state(
        self,
        cr: &mut <Self as HL>::Cr,
    ) -> Pin<P, N, Output<PushPull>> {
        self.into_mode(cr)
    }

    /// Configures the pin to operate as an analog input pin
    #[inline]
    pub fn into_analog(self, cr: &mut <Self as HL>::Cr) -> Pin<P, N, Analog> {
        self.into_mode(cr)
    }

    /// Configures the pin as a pin that can change between input
    /// and output without changing the type. It starts out
    /// as a floating input
    #[inline]
    pub fn into_dynamic(mut self, _cr: &mut <Self as HL>::Cr) -> Pin<P, N, Dynamic> {
        self.mode::<Input>();
        Pin::new()
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
            _cr: &mut <Self as HL>::Cr,
            mut f: impl FnMut(&mut Pin<P, N, $mode>),
        ) {
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, $mode>::new();
            f(&mut temp);
            temp.mode::<MODE>();
        }

        /// Temporarily change the mode of the pin.
        ///
        /// Note that the new state is set slightly before conversion
        /// happens. This can cause a short output glitch if switching
        /// between output modes
        #[inline]
        pub fn $stateful_fn_name(
            &mut self,
            _cr: &mut <Self as HL>::Cr,
            state: PinState,
            mut f: impl FnMut(&mut Pin<P, N, $mode>),
        ) {
            self._set_state(state);
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, $mode>::new();
            f(&mut temp);
            temp.mode::<MODE>();
        }
    };
}
macro_rules! impl_temp_input {
    ($fn_name:ident, $mode:ty $(, $pull:ident)?) => {
        /// Temporarily change the mode of the pin.
        #[inline]
        pub fn $fn_name(
            &mut self,
            _cr: &mut <Self as HL>::Cr,
            mut f: impl FnMut(&mut Pin<P, N, Input>),
        ) {
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, Input>::new() $(.$pull())?;
            f(&mut temp);
            temp.mode::<MODE>();
        }
    };
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active + PinMode,
    Self: HL,
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
    impl_temp_input!(as_floating_input, Input);
    impl_temp_input!(as_pull_up_input, PulledInput, _pull_up);
    impl_temp_input!(as_pull_down_input, PulledInput, _pull_down);
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    Self: HL,
{
    #[inline(always)]
    fn _set_speed(&mut self, _cr: &mut <Self as HL>::Cr, speed: IOPinSpeed) {
        let gpio = unsafe { &(*gpiox::<P>()) };

        match N {
            0..=7 => {
                gpio.crl().modify(|_, w| w.mode(N).variant(speed.into()));
            }
            8..=15 => {
                gpio.crh()
                    .modify(|_, w| unsafe { w.mode(N - 16).bits(speed as u8) });
            }
            _ => unreachable!(),
        }
    }
}

impl<const P: char, const N: u8, MODE> OutputSpeed for Pin<P, N, Output<MODE>>
where
    Self: HL,
{
    fn set_speed(&mut self, cr: &mut <Self as HL>::Cr, speed: IOPinSpeed) {
        self._set_speed(cr, speed)
    }
}

impl<const P: char, const N: u8> OutputSpeed for Pin<P, N, Alternate<PushPull>>
where
    Self: HL,
{
    fn set_speed(&mut self, cr: &mut <Self as HL>::Cr, speed: IOPinSpeed) {
        self._set_speed(cr, speed)
    }
}

// Dynamic pin

impl<const P: char, const N: u8> Pin<P, N, Dynamic>
where
    Self: HL,
{
    #[inline]
    pub fn make_pull_up_input(&mut self, _cr: &mut <Self as HL>::Cr) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<PulledInput>();
        self._set_pull_up();
        self.mode = Dynamic::InputPullUp;
    }

    #[inline]
    pub fn make_pull_down_input(&mut self, _cr: &mut <Self as HL>::Cr) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<PulledInput>();
        self._set_pull_down();
        self.mode = Dynamic::InputPullDown;
    }

    #[inline]
    pub fn make_floating_input(&mut self, _cr: &mut <Self as HL>::Cr) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Input>();
        self.mode = Dynamic::InputFloating;
    }

    #[inline]
    pub fn make_push_pull_output(&mut self, _cr: &mut <Self as HL>::Cr) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Output<PushPull>>();
        self.mode = Dynamic::OutputPushPull;
    }

    #[inline]
    pub fn make_open_drain_output(&mut self, _cr: &mut <Self as HL>::Cr) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Output<OpenDrain>>();
        self.mode = Dynamic::OutputOpenDrain;
    }
}

impl PinMode for Analog {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::PushPull;
}

impl PinMode for Input {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::OpenDrain;
}

impl PinMode for PulledInput {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::AltPushPull;
}

impl PinMode for Output<PushPull> {
    const MODE: Mode = Mode::Output50;
    const CNF: Cnf = Cnf::PushPull;
}

impl PinMode for Output<OpenDrain> {
    const MODE: Mode = Mode::Output50;
    const CNF: Cnf = Cnf::OpenDrain;
}

impl PinMode for Alternate<PushPull> {
    const MODE: Mode = Mode::Output50;
    const CNF: Cnf = Cnf::AltPushPull;
}

impl PinMode for Alternate<OpenDrain> {
    const MODE: Mode = Mode::Output50;
    const CNF: Cnf = Cnf::AltOpenDrain;
}

impl<const P: char, const N: u8, M> Pin<P, N, M> {
    fn mode<MODE: PinMode>(&mut self) {
        let gpio = unsafe { &(*gpiox::<P>()) };

        match N {
            0..=7 => {
                gpio.crl()
                    .modify(|_, w| w.mode(N).variant(MODE::MODE).cnf(N).variant(MODE::CNF));
            }
            8..=15 => {
                gpio.crh().modify(|_, w| unsafe {
                    w.mode(N - 16).bits(MODE::MODE as u8);
                    w.cnf(N - 16).bits(MODE::CNF as u8)
                });
            }
            _ => unreachable!(),
        }
    }
}

impl<const P: char, const N: u8, M> Pin<P, N, M>
where
    Self: HL,
{
    #[inline]
    pub(crate) fn into_mode<MODE: PinMode>(
        mut self,
        _cr: &mut <Self as HL>::Cr,
    ) -> Pin<P, N, MODE> {
        self.mode::<MODE>();
        Pin::new()
    }
}

impl Analog {
    pub fn new<const P: char, const N: u8, MODE>(
        pin: Pin<P, N, MODE>,
        cr: &mut <Pin<P, N, MODE> as HL>::Cr,
    ) -> Pin<P, N, Self>
    where
        Pin<P, N, MODE>: HL,
        Self: PinMode,
    {
        pin.into_mode(cr)
    }
}

impl Input {
    pub fn new<const P: char, const N: u8, MODE>(
        mut pin: Pin<P, N, MODE>,
        cr: &mut <Pin<P, N, MODE> as HL>::Cr,
        pull: Pull,
    ) -> Pin<P, N, Self>
    where
        Pin<P, N, MODE>: HL,
        Self: PinMode,
    {
        match pull {
            Pull::None => pin.into_mode(cr),
            Pull::Up => {
                pin.mode::<PulledInput>();
                Pin::new()._pull_up()
            }
            Pull::Down => {
                pin.mode::<PulledInput>();
                Pin::new()._pull_down()
            }
        }
    }
}

impl<Otype> Output<Otype> {
    pub fn new<const P: char, const N: u8, MODE>(
        mut pin: Pin<P, N, MODE>,
        cr: &mut <Pin<P, N, MODE> as HL>::Cr,
        state: PinState,
    ) -> Pin<P, N, Self>
    where
        Pin<P, N, MODE>: HL,
        Self: PinMode,
    {
        pin._set_state(state);
        pin.into_mode(cr)
    }
}

impl<Otype> Alternate<Otype> {
    pub fn new<const P: char, const N: u8, MODE>(
        pin: Pin<P, N, MODE>,
        cr: &mut <Pin<P, N, MODE> as HL>::Cr,
    ) -> Pin<P, N, Self>
    where
        Pin<P, N, MODE>: HL,
        Self: PinMode,
    {
        pin.into_mode(cr)
    }
}

gpio!(GPIOA, gpioa, PAx, 'A', [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13, Debugger),
    PA14: (pa14, 14, Debugger),
    PA15: (pa15, 15, Debugger),
]);

gpio!(GPIOB, gpiob, PBx, 'B', [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3, Debugger),
    PB4: (pb4, 4, Debugger),
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
    PB9: (pb9, 9),
    PB10: (pb10, 10),
    PB11: (pb11, 11),
    PB12: (pb12, 12),
    PB13: (pb13, 13),
    PB14: (pb14, 14),
    PB15: (pb15, 15),
]);

gpio!(GPIOC, gpioc, PCx, 'C', [
    PC0: (pc0, 0),
    PC1: (pc1, 1),
    PC2: (pc2, 2),
    PC3: (pc3, 3),
    PC4: (pc4, 4),
    PC5: (pc5, 5),
    PC6: (pc6, 6),
    PC7: (pc7, 7),
    PC8: (pc8, 8),
    PC9: (pc9, 9),
    PC10: (pc10, 10),
    PC11: (pc11, 11),
    PC12: (pc12, 12),
    PC13: (pc13, 13),
    PC14: (pc14, 14),
    PC15: (pc15, 15),
]);

gpio!(GPIOD, gpiod, PDx, 'D', [
    PD0: (pd0, 0),
    PD1: (pd1, 1),
    PD2: (pd2, 2),
    PD3: (pd3, 3),
    PD4: (pd4, 4),
    PD5: (pd5, 5),
    PD6: (pd6, 6),
    PD7: (pd7, 7),
    PD8: (pd8, 8),
    PD9: (pd9, 9),
    PD10: (pd10, 10),
    PD11: (pd11, 11),
    PD12: (pd12, 12),
    PD13: (pd13, 13),
    PD14: (pd14, 14),
    PD15: (pd15, 15),
]);

gpio!(GPIOE, gpioe, PEx, 'E', [
    PE0: (pe0, 0),
    PE1: (pe1, 1),
    PE2: (pe2, 2),
    PE3: (pe3, 3),
    PE4: (pe4, 4),
    PE5: (pe5, 5),
    PE6: (pe6, 6),
    PE7: (pe7, 7),
    PE8: (pe8, 8),
    PE9: (pe9, 9),
    PE10: (pe10, 10),
    PE11: (pe11, 11),
    PE12: (pe12, 12),
    PE13: (pe13, 13),
    PE14: (pe14, 14),
    PE15: (pe15, 15),
]);

#[cfg(any(feature = "xl", feature = "high"))]
gpio!(GPIOF, gpiof, PFx, 'F', [
    PF0:  (pf0, 0),
    PF1:  (pf1, 1),
    PF2:  (pf2, 2),
    PF3:  (pf3, 3),
    PF4:  (pf4, 4),
    PF5:  (pf5, 5),
    PF6:  (pf6, 6),
    PF7:  (pf7, 7),
    PF8:  (pf8, 8),
    PF9:  (pf9, 9),
    PF10: (pf10, 10),
    PF11: (pf11, 11),
    PF12: (pf12, 12),
    PF13: (pf13, 13),
    PF14: (pf14, 14),
    PF15: (pf15, 15),
]);

#[cfg(any(feature = "xl", feature = "high"))]
gpio!(GPIOG, gpiog, PGx, 'G', [
    PG0:  (pg0, 0),
    PG1:  (pg1, 1),
    PG2:  (pg2, 2),
    PG3:  (pg3, 3),
    PG4:  (pg4, 4),
    PG5:  (pg5, 5),
    PG6:  (pg6, 6),
    PG7:  (pg7, 7),
    PG8:  (pg8, 8),
    PG9:  (pg9, 9),
    PG10: (pg10, 10),
    PG11: (pg11, 11),
    PG12: (pg12, 12),
    PG13: (pg13, 13),
    PG14: (pg14, 14),
    PG15: (pg15, 15),
]);

const fn gpiox<const P: char>() -> *const crate::pac::gpioa::RegisterBlock {
    match P {
        'A' => crate::pac::GPIOA::ptr(),
        'B' => crate::pac::GPIOB::ptr() as _,
        'C' => crate::pac::GPIOC::ptr() as _,
        'D' => crate::pac::GPIOD::ptr() as _,
        'E' => crate::pac::GPIOE::ptr() as _,
        #[cfg(any(feature = "xl", feature = "high"))]
        'F' => crate::pac::GPIOF::ptr() as _,
        #[cfg(any(feature = "xl", feature = "high"))]
        'G' => crate::pac::GPIOG::ptr() as _,
        _ => unreachable!(),
    }
}
