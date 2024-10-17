use core::convert::Infallible;
use embedded_hal::delay::DelayNs;

use super::{Delay, ErasedChannel, Instance, PwmChannel, SysDelay, TimC, WithPwm};
use fugit::ExtU32Ceil;

impl DelayNs for SysDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay(ns.nanos_at_least());
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay(ms.millis_at_least());
    }
}

impl<TIM: Instance, const FREQ: u32> DelayNs for Delay<TIM, FREQ> {
    fn delay_ns(&mut self, ns: u32) {
        self.delay(ns.micros_at_least());
    }

    fn delay_us(&mut self, us: u32) {
        self.delay(us.micros_at_least());
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay(ms.millis_at_least());
    }
}

impl<TIM: Instance + WithPwm + TimC<C>, const C: u8, const COMP: bool> embedded_hal::pwm::ErrorType
    for PwmChannel<TIM, C, COMP>
{
    type Error = Infallible;
}

impl<TIM: Instance + WithPwm + TimC<C>, const C: u8, const COMP: bool>
    embedded_hal::pwm::SetDutyCycle for PwmChannel<TIM, C, COMP>
{
    fn max_duty_cycle(&self) -> u16 {
        self.get_max_duty()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty);
        Ok(())
    }
}

impl<TIM: Instance + WithPwm> embedded_hal::pwm::ErrorType for ErasedChannel<TIM> {
    type Error = Infallible;
}

impl<TIM: Instance + WithPwm> embedded_hal::pwm::SetDutyCycle for ErasedChannel<TIM> {
    fn max_duty_cycle(&self) -> u16 {
        self.get_max_duty()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty);
        Ok(())
    }
}
