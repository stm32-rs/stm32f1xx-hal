//! # General Purpose I/Os
//!
//! # Interfacing with v1 traits
//!
//! `embedded-hal` has two versions of the digital traits, `v2` which is used
//! by this crate and `v1` which is deprecated but still used by a lot of drivers.
//! If you want to use such a driver with this crate, you need to convert the digital pins to the `v1` type.
//!
//! This is done using `embedded-hal::digital::v1_compat::OldOutputPin`. For example:
//!
//! ```rust
//! let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
//! let mut mfrc522 = Mfrc522::new(spi, OldOutputPin::from(nss)).unwrap();
//! ```
//!

use core::marker::PhantomData;

use crate::rcc::APB2;
use crate::stm32::EXTI;
use crate::afio;

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

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $PXx:ident, $extigpionr:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $CR:ident, $exticri:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;

            use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use crate::pac::{$gpioy, $GPIOX};
            use crate::stm32::EXTI;
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
                ExtiPin
            };

            /// GPIO parts
            pub struct Parts {
                /// Opaque CRL register
                pub crl: CRL,
                /// Opaque CRH register
                pub crh: CRH,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
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
                            $pxi: $PXi { _mode: PhantomData },
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


            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> Mode<MODE> for $PXi<MODE> {}

                impl $PXi<Debugger> {
                    /// Put the pin in an active state. The caller
                    /// must enforce that the pin is really in this
                    /// state in the hardware.
                    #[allow(dead_code)]
                    pub(crate) unsafe fn activate(self) -> $PXi<Input<Floating>> {
                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Configures the pin to operate as an alternate function push-pull output
                    /// pin.
                    pub fn into_alternate_push_pull(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Alternate<PushPull>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output push pull
                        const CNF: u32 = 0b10;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an alternate function open-drain output
                    /// pin.
                    pub fn into_alternate_open_drain(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Alternate<OpenDrain>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output open drain
                        const CNF: u32 = 0b11;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Input<Floating>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Floating input
                        const CNF: u32 = 0b01;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Input<PullDown>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull down:
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) }

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Input<PullUp>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull up:
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) }

                        // input mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// Initial state will be low.
                    pub fn into_open_drain_output(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output_with_state(cr, State::Low)
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_open_drain_output_with_state(
                        self,
                        cr: &mut $CR,
                        initial_state: State,
                    ) -> $PXi<Output<OpenDrain>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output open-drain
                        const CNF: u32 = 0b01;
                        // Open-Drain Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        let mut res = $PXi { _mode: PhantomData };

                        match initial_state {
                            State::High => res.set_high(),
                            State::Low  => res.set_low(),
                        }.unwrap();

                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        res
                    }
                    /// Configures the pin to operate as an push-pull output pin.
                    /// Initial state will be low.
                    pub fn into_push_pull_output(
                        self,
                        cr: &mut $CR,
                    ) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output_with_state(cr, State::Low)
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_push_pull_output_with_state(
                        self,
                        cr: &mut $CR,
                        initial_state: State,
                    ) -> $PXi<Output<PushPull>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output push-pull
                        const CNF: u32 = 0b00;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        let mut res = $PXi { _mode: PhantomData };

                        match initial_state {
                            State::High => res.set_high(),
                            State::Low  => res.set_low(),
                        }.unwrap();

                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        res
                    }

                    /// Configures the pin to operate as an analog input pin
                    pub fn into_analog(self, cr: &mut $CR) -> $PXi<Analog> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Analog input
                        const CNF: u32 = 0b00;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // analog mode
                        cr
                            .cr()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Erases the pin number from the type
                    fn into_generic(self) -> Generic<MODE> {
                        Generic {
                            i: $i,
                            _mode: self._mode,
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

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Infallible;
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) })
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) })
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|b| !b)
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Infallible;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Infallible;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl<MODE> ExtiPin for $PXi<Input<MODE>> {
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
                        match edge {
                            Edge::RISING => {
                                exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                            },
                            Edge::FALLING => {
                                exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                            },
                            Edge::RISING_FALLING => {
                                exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                            }
                        }
                    }

                    /// Enable external interrupts from this pin.
                    fn enable_interrupt(&mut self, exti: &EXTI) {
                        exti.imr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                    }

                    /// Disable external interrupts from this pin
                    fn disable_interrupt(&mut self, exti: &EXTI) {
                        exti.imr.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    }

                    /// Clear the interrupt pending bit for this pin
                    fn clear_interrupt_pending_bit(&mut self) {
                        unsafe { (*EXTI::ptr()).pr.write(|w| w.bits(1 << $i) ) };
                    }

                    /// Reads the interrupt pending bit for this pin
                    fn check_interrupt(&mut self) -> bool {
                        unsafe { ((*EXTI::ptr()).pr.read().bits() & (1 << $i)) != 0 }
                    }
                }
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

impl_pxx!{
    (gpioa::PAx),
    (gpiob::PBx),
    (gpioc::PCx),
    (gpiod::PDx),
    (gpioe::PEx)
}

gpio!(GPIOA, gpioa, gpioa, PAx, 0, [
    PA0: (pa0, 0, Input<Floating>, CRL, exticr1),
    PA1: (pa1, 1, Input<Floating>, CRL, exticr1),
    PA2: (pa2, 2, Input<Floating>, CRL, exticr1),
    PA3: (pa3, 3, Input<Floating>, CRL, exticr1),
    PA4: (pa4, 4, Input<Floating>, CRL, exticr2),
    PA5: (pa5, 5, Input<Floating>, CRL, exticr2),
    PA6: (pa6, 6, Input<Floating>, CRL, exticr2),
    PA7: (pa7, 7, Input<Floating>, CRL, exticr2),
    PA8: (pa8, 8, Input<Floating>, CRH, exticr3),
    PA9: (pa9, 9, Input<Floating>, CRH, exticr3),
    PA10: (pa10, 10, Input<Floating>, CRH, exticr3),
    PA11: (pa11, 11, Input<Floating>, CRH, exticr3),
    PA12: (pa12, 12, Input<Floating>, CRH, exticr4),
    PA13: (pa13, 13, Debugger, CRH, exticr4),
    PA14: (pa14, 14, Debugger, CRH, exticr4),
    PA15: (pa15, 15, Debugger, CRH, exticr4),
]);

gpio!(GPIOB, gpiob, gpioa, PBx, 1, [
    PB0: (pb0, 0, Input<Floating>, CRL, exticr1),
    PB1: (pb1, 1, Input<Floating>, CRL, exticr1),
    PB2: (pb2, 2, Input<Floating>, CRL, exticr1),
    PB3: (pb3, 3, Debugger, CRL, exticr1),
    PB4: (pb4, 4, Debugger, CRL, exticr2),
    PB5: (pb5, 5, Input<Floating>, CRL, exticr2),
    PB6: (pb6, 6, Input<Floating>, CRL, exticr2),
    PB7: (pb7, 7, Input<Floating>, CRL, exticr2),
    PB8: (pb8, 8, Input<Floating>, CRH, exticr3),
    PB9: (pb9, 9, Input<Floating>, CRH, exticr3),
    PB10: (pb10, 10, Input<Floating>, CRH, exticr3),
    PB11: (pb11, 11, Input<Floating>, CRH, exticr3),
    PB12: (pb12, 12, Input<Floating>, CRH, exticr4),
    PB13: (pb13, 13, Input<Floating>, CRH, exticr4),
    PB14: (pb14, 14, Input<Floating>, CRH, exticr4),
    PB15: (pb15, 15, Input<Floating>, CRH, exticr4),
]);

gpio!(GPIOC, gpioc, gpioa, PCx, 2, [
    PC0: (pc0, 0, Input<Floating>, CRL, exticr1),
    PC1: (pc1, 1, Input<Floating>, CRL, exticr1),
    PC2: (pc2, 2, Input<Floating>, CRL, exticr1),
    PC3: (pc3, 3, Input<Floating>, CRL, exticr1),
    PC4: (pc4, 4, Input<Floating>, CRL, exticr2),
    PC5: (pc5, 5, Input<Floating>, CRL, exticr2),
    PC6: (pc6, 6, Input<Floating>, CRL, exticr2),
    PC7: (pc7, 7, Input<Floating>, CRL, exticr2),
    PC8: (pc8, 8, Input<Floating>, CRH, exticr3),
    PC9: (pc9, 9, Input<Floating>, CRH, exticr3),
    PC10: (pc10, 10, Input<Floating>, CRH, exticr3),
    PC11: (pc11, 11, Input<Floating>, CRH, exticr3),
    PC12: (pc12, 12, Input<Floating>, CRH, exticr4),
    PC13: (pc13, 13, Input<Floating>, CRH, exticr4),
    PC14: (pc14, 14, Input<Floating>, CRH, exticr4),
    PC15: (pc15, 15, Input<Floating>, CRH, exticr4),
]);

gpio!(GPIOD, gpiod, gpioa, PDx, 3, [
    PD0: (pd0, 0, Input<Floating>, CRL, exticr1),
    PD1: (pd1, 1, Input<Floating>, CRL, exticr1),
    PD2: (pd2, 2, Input<Floating>, CRL, exticr1),
    PD3: (pd3, 3, Input<Floating>, CRL, exticr1),
    PD4: (pd4, 4, Input<Floating>, CRL, exticr2),
    PD5: (pd5, 5, Input<Floating>, CRL, exticr2),
    PD6: (pd6, 6, Input<Floating>, CRL, exticr2),
    PD7: (pd7, 7, Input<Floating>, CRL, exticr2),
    PD8: (pd8, 8, Input<Floating>, CRH, exticr3),
    PD9: (pd9, 9, Input<Floating>, CRH, exticr3),
    PD10: (pd10, 10, Input<Floating>, CRH, exticr3),
    PD11: (pd11, 11, Input<Floating>, CRH, exticr3),
    PD12: (pd12, 12, Input<Floating>, CRH, exticr4),
    PD13: (pd13, 13, Input<Floating>, CRH, exticr4),
    PD14: (pd14, 14, Input<Floating>, CRH, exticr4),
    PD15: (pd15, 15, Input<Floating>, CRH, exticr4),
]);

gpio!(GPIOE, gpioe, gpioa, PEx, 4, [
    PE0: (pe0, 0, Input<Floating>, CRL, exticr1),
    PE1: (pe1, 1, Input<Floating>, CRL, exticr1),
    PE2: (pe2, 2, Input<Floating>, CRL, exticr1),
    PE3: (pe3, 3, Input<Floating>, CRL, exticr1),
    PE4: (pe4, 4, Input<Floating>, CRL, exticr2),
    PE5: (pe5, 5, Input<Floating>, CRL, exticr2),
    PE6: (pe6, 6, Input<Floating>, CRL, exticr2),
    PE7: (pe7, 7, Input<Floating>, CRL, exticr2),
    PE8: (pe8, 8, Input<Floating>, CRH, exticr3),
    PE9: (pe9, 9, Input<Floating>, CRH, exticr3),
    PE10: (pe10, 10, Input<Floating>, CRH, exticr3),
    PE11: (pe11, 11, Input<Floating>, CRH, exticr3),
    PE12: (pe12, 12, Input<Floating>, CRH, exticr4),
    PE13: (pe13, 13, Input<Floating>, CRH, exticr4),
    PE14: (pe14, 14, Input<Floating>, CRH, exticr4),
    PE15: (pe15, 15, Input<Floating>, CRH, exticr4),
]);
