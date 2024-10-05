use super::*;

pub type PEPin<const P: char, MODE> = PartiallyErasedPin<P, MODE>;

/// Partially erased pin
///
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
pub struct PartiallyErasedPin<const P: char, MODE> {
    pin_number: u8,
    _mode: PhantomData<MODE>,
}

impl<const P: char, MODE> PartiallyErasedPin<P, MODE> {
    pub(crate) fn new(pin_number: u8) -> Self {
        Self {
            pin_number,
            _mode: PhantomData,
        }
    }
}

impl<const P: char, MODE> PinExt for PartiallyErasedPin<P, MODE> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        self.pin_number
    }

    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - b'A'
    }
}

impl<const P: char, MODE> PartiallyErasedPin<P, Output<MODE>> {
    #[inline(always)]
    pub fn set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.bs(self.pin_number).set_bit());
    }

    #[inline(always)]
    pub fn set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.bsrr().write(|w| w.br(self.pin_number).set_bit());
    }

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
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    #[inline(always)]
    pub fn is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.odr().read().odr(self.pin_number).bit_is_clear()
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

impl<const P: char> PartiallyErasedPin<P, Output<OpenDrain>> {
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline(always)]
    pub fn is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.idr().read().idr(self.pin_number).bit_is_clear()
    }
}

impl<const P: char, MODE> PartiallyErasedPin<P, Input<MODE>> {
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline(always)]
    pub fn is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.idr().read().idr(self.pin_number).bit_is_clear()
    }
}
