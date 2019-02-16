//! Analog to digital converter

use stm32::{ADC1, ADC2, ADC3};

use hal::adc::{OneShot, Channel};

use nb;

use crate::gpio::Analog;
use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;

use crate::rcc::APB2;

pub struct Adc<ADC> {
    adc: ADC,
    current_channel: Option<u8>,
}

#[derive(Debug)]
pub enum AdcReadError {
    /// Another conversion is already being performed
    Busy
}

macro_rules! hal {
    ($(
        $ADC:ident: (
            $init:ident,
            $adcxen:ident,
            $adcxrst:ident
        ),
    )+) => {
        $(
            impl Adc<$ADC> {
                /// Powers up $ADC and blocks until it's ready
                pub fn $init(adc: $ADC, apb2: &mut APB2) -> Self {
                    // Reset and enable the ADC peripheral
                    apb2.rstr().modify(|_, w| w.$adcxrst().set_bit());
                    apb2.rstr().modify(|_, w| w.$adcxrst().clear_bit());
                    apb2.enr().modify(|_, w| w.$adcxen().set_bit());


                    adc.cr2.modify(|_, w| { w.cont().clear_bit()});

                    adc.cr2.modify(|_, w| { w.adon().set_bit()});

                    // Wait for the ADC to be ready
                    while adc.cr2.read().adon().bit_is_set() == false
                        {}

                    // Set the sequence length to 1
                    // Amount of conversions n-1

                    unsafe{adc.sqr1.modify(|_, w| w.l().bits(0))}
                    Self {
                        adc,
                        current_channel: None,
                    }
                }

                /// Make a single reading of the specified channel
                fn read_raw(&mut self, channel: u8) -> nb::Result<u16, AdcReadError> {
                    // Prevent starting another read before the previous one is done
                    if let Some(previous_channel) = self.current_channel {
                        if channel != previous_channel {
                            return Err(nb::Error::Other(AdcReadError::Busy))
                        }
                    }

                    // Select the channel to be converted
                    if self.adc.sr.read().strt().bit() == false {
                        unsafe{self.adc.sqr3.modify(|_, w| w.sq1().bits(channel))};
                        // Set ADON
                        // self.adc.cr2.modify(|_, w| w.swstart().set_bit());
                        self.adc.cr2.modify(|_, w| { w.adon().set_bit()});
                    }

                    // Check if the data is ready for reading
                    if self.adc.sr.read().eoc().bit() == false {
                        self.current_channel = Some(channel);
                        Err(nb::Error::WouldBlock)
                    }
                    else {
                        self.current_channel = None;
                        self.adc.sr.modify(|_, w| w.strt().clear_bit());
                        Ok(self.adc.dr.read().data().bits())
                    }
                }
            }


            impl<PIN> OneShot<$ADC, u16, PIN> for Adc<$ADC>
            where
                PIN: Channel<$ADC, ID=u8>,
            {
                type Error = AdcReadError;

                fn read(
                    &mut self,
                    _pin: &mut PIN
                ) -> nb::Result<u16, Self::Error>
                {
                    self.read_raw(PIN::channel())
                }
            }
        )+
    }
}


macro_rules! analog_pin_impls {
    ($($adc:ty: ($($pin:ident: $channel:expr),+)),+) =>
    {
        $(
            $(
                impl Channel<$adc> for $pin<Analog> {
                    type ID = u8;

                    fn channel() -> Self::ID {
                        $channel
                    }
                }
            )+
        )+
    }
}


hal! {
    ADC1: (
        adc1,
        adc1en,
        adc1rst
    ),
    ADC2: (
        adc2,
        adc2en,
        adc2rst
    ),
    ADC3: (
        adc3,
        adc3en,
        adc3rst
    ),
}

analog_pin_impls!{
    ADC1: (
        PA0: 0,
        PA1: 1,
        PA2: 2,
        PA3: 3,
        PA4: 4,
        PA5: 5,
        PA6: 6,
        PA7: 7,
        PB0: 8,
        PB1: 9
    ),
    ADC2: (
        PA0: 0,
        PA1: 1,
        PA2: 2,
        PA3: 3,
        PA4: 4,
        PA5: 5,
        PA6: 6,
        PA7: 7,
        PB0: 8,
        PB1: 9
    )
}
