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

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $iopxenr:ident, $iopxrst:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $CR:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use void::Void;
            use core::marker::PhantomData;

            use crate::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use crate::pac::{$gpioy, $GPIOX};

            use crate::rcc::APB2;
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

                fn split(self, apb2: &mut APB2) -> Parts {
                    apb2.enr().modify(|_, w| w.$iopxenr().set_bit());
                    apb2.rstr().modify(|_, w| w.$iopxrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$iopxrst().clear_bit());

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

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                type Error = Void;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) })
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) })
                }
            }

            impl<MODE> InputPin for $PXx<Input<MODE>> {
                type Error = Void;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl <MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl <MODE> toggleable::Default for $PXx<Output<MODE>> {}

            impl InputPin for $PXx<Output<OpenDrain>> {
                type Error = Void;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                }
            }

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

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
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<MODE> {
                        $PXx {
                            i: $i,
                            _mode: self._mode,
                        }
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Void;
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

                impl<MODE> OutputPin for $PXi<Alternate<MODE>> {
                    type Error = Void;
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << $i)) })
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + $i))) })
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Alternate<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|b| !b)
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Void;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Void;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << $i) == 0 })
                    }
                }
            )+
        }
    }
}

gpio!(GPIOA, gpioa, gpioa, iopaen, ioparst, PAx, [
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

gpio!(GPIOB, gpiob, gpioa, iopben, iopbrst, PBx, [
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

gpio!(GPIOC, gpioc, gpioa, iopcen, iopcrst, PCx, [
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

gpio!(GPIOD, gpiod, gpioa, iopden, iopdrst, PDx, [
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

gpio!(GPIOE, gpioe, gpioa, iopeen, ioperst, PEx, [
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
