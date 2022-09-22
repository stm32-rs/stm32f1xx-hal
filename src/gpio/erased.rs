use super::*;

pub type EPin<MODE> = ErasedPin<MODE>;

macro_rules! impl_pxx {
    ($(($port_id:literal :: $pin:ident)),*) => {
        /// Erased pin
        ///
        /// `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
        pub enum ErasedPin<MODE> {
            $(
                $pin(PartiallyErasedPin<$port_id, MODE>)
            ),*
        }

        impl<MODE> PinExt for ErasedPin<MODE> {
            type Mode = MODE;

            #[inline(always)]
            fn pin_id(&self) -> u8 {
                match self {
                    $(Self::$pin(pin) => pin.pin_id()),*
                }
            }

            #[inline(always)]
            fn port_id(&self) -> u8 {
                match self {
                    $(Self::$pin(pin) => pin.port_id()),*
                }
            }
        }

        impl<MODE> ErasedPin<Output<MODE>> {
            pub fn set_high(&mut self) {
                match self {
                    $(Self::$pin(pin) => pin.set_high()),*
                }
            }

            pub fn set_low(&mut self) {
                match self {
                    $(Self::$pin(pin) => pin.set_low()),*
                }
            }

            pub fn is_set_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_set_high()),*
                }
            }

            pub fn is_set_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> ErasedPin<Input<MODE>> {
            pub fn is_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_high()),*
                }
            }

            pub fn is_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl ErasedPin<Output<OpenDrain>> {
            pub fn is_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_high()),*
                }
            }

            pub fn is_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_low()),*
                }
            }
        }
    }
}

impl<MODE> ErasedPin<Output<MODE>> {
    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Infallible> {
        self.set_low();
        Ok(())
    }
}

impl<MODE> StatefulOutputPin for ErasedPin<Output<MODE>> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<MODE> InputPin for ErasedPin<Input<MODE>> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Infallible> {
        Ok(self.is_high())
    }

    fn is_low(&self) -> Result<bool, Infallible> {
        Ok(self.is_low())
    }
}

impl InputPin for ErasedPin<Output<OpenDrain>> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Infallible> {
        Ok(self.is_high())
    }

    fn is_low(&self) -> Result<bool, Infallible> {
        Ok(self.is_low())
    }
}

#[cfg(not(any(feature = "xl", feature = "high")))]
impl_pxx! {
    ('A'::PAx),
    ('B'::PBx),
    ('C'::PCx),
    ('D'::PDx),
    ('E'::PEx)
}

#[cfg(any(feature = "xl", feature = "high"))]
impl_pxx! {
    ('A'::PAx),
    ('B'::PBx),
    ('C'::PCx),
    ('D'::PDx),
    ('E'::PEx),
    ('F'::PFx),
    ('G'::PGx)
}
