use core::convert::Infallible;

use super::{Dynamic, ErasedPin, Input, OpenDrain, Output, PartiallyErasedPin, Pin, PinModeError};

pub use embedded_hal::digital::PinState;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

fn into_state(state: PinState) -> super::PinState {
    match state {
        PinState::Low => super::PinState::Low,
        PinState::High => super::PinState::High,
    }
}

// Implementations for `Pin`
impl<const P: char, const N: u8, MODE> ErrorType for Pin<P, N, Output<MODE>> {
    type Error = Infallible;
}
impl<const P: char, const N: u8, MODE> ErrorType for Pin<P, N, Input<MODE>> {
    type Error = Infallible;
}

impl embedded_hal::digital::Error for PinModeError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        match self {
            PinModeError::IncorrectMode => embedded_hal::digital::ErrorKind::Other,
        }
    }
}

impl<const P: char, const N: u8> ErrorType for Pin<P, N, Dynamic> {
    type Error = PinModeError;
}

impl<const P: char, const N: u8> OutputPin for Pin<P, N, Dynamic> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self._set_state(into_state(PinState::High));
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self._set_state(into_state(PinState::Low));
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
}

impl<const P: char, const N: u8> InputPin for Pin<P, N, Dynamic> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        if self.mode.is_input() {
            Ok(self._is_low())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
}

impl<const P: char, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> StatefulOutputPin for Pin<P, N, Output<MODE>> {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<const P: char, const N: u8, MODE> InputPin for Pin<P, N, Input<MODE>> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<const P: char, const N: u8> InputPin for Pin<P, N, Output<OpenDrain>> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

// PartiallyErasedPin

impl<const P: char, MODE> ErrorType for PartiallyErasedPin<P, MODE> {
    type Error = Infallible;
}

impl<const P: char, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<const P: char, MODE> StatefulOutputPin for PartiallyErasedPin<P, Output<MODE>> {
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<const P: char> InputPin for PartiallyErasedPin<P, Output<OpenDrain>> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<const P: char, MODE> InputPin for PartiallyErasedPin<P, Input<MODE>> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

// ErasedPin

impl<MODE> ErrorType for ErasedPin<MODE> {
    type Error = core::convert::Infallible;
}

impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
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
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<MODE> InputPin for ErasedPin<Input<MODE>> {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok((*self).is_low())
    }
}

impl InputPin for ErasedPin<Output<OpenDrain>> {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok((*self).is_low())
    }
}
