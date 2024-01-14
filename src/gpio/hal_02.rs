use super::*;
use embedded_hal_02::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

// Pin

impl<const P: char, const N: u8> OutputPin for Pin<P, N, Dynamic> {
    type Error = PinModeError;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self._set_state(PinState::High);
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        if self.mode.is_output() {
            self._set_state(PinState::Low);
            Ok(())
        } else {
            Err(PinModeError::IncorrectMode)
        }
    }
}

impl<const P: char, const N: u8> InputPin for Pin<P, N, Dynamic> {
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

impl<const P: char, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    type Error = Infallible;
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
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<const P: char, const N: u8, MODE> ToggleableOutputPin for Pin<P, N, Output<MODE>> {
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> InputPin for Pin<P, N, Input<MODE>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<const P: char, const N: u8> InputPin for Pin<P, N, Output<OpenDrain>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

// PartiallyErasedPin

impl<const P: char, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
    type Error = Infallible;

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
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<const P: char, MODE> ToggleableOutputPin for PartiallyErasedPin<P, Output<MODE>> {
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char> InputPin for PartiallyErasedPin<P, Output<OpenDrain>> {
    type Error = Infallible;

    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<const P: char, MODE> InputPin for PartiallyErasedPin<P, Input<MODE>> {
    type Error = Infallible;

    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

// ErasedPin

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
