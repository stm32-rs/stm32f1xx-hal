use crate::adc::{Adc, SampleTime};
use crate::pac::ADC1;
use embedded_hal::blocking::rng;

/// embedded-hal compatible rng implementation
pub struct Rng {
    adc: Adc<ADC1>,
}

impl Adc<ADC1> {
    pub fn into_rng(self) -> Rng {
        Rng { adc: self }
    }
}

impl Rng {
    pub fn release(self) -> Adc<ADC1> {
        self.adc
    }
}

impl rng::Read for Rng {
    type Error = ();
    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let prev_cfg = self.adc.save_cfg();
        // Set the minimum sampling time
        self.adc.set_sample_time(SampleTime::T_1);
        // Iterate all Bytes
        for buf in buffer {
            let mut random: u8 = 0;
            // Iterate bit0 to bit7
            for _i in 0..8 {
                // Record the times of the adc value is odd
                let mut num = 0;
                // The more iterations, the stronger randomness
                for _i in 0..5 {
                    // `num` add one if adc value is odd
                    // get random adc value beacuse of temperature drift.
                    num += self.adc.read_aux(16u8) & 0x01;
                }

                // Set bit to 1 if `num` is odd; Set bit to 0 if `num` is even.
                random = (random << 1) + (num & 0x01) as u8;
            }
            *buf = random;
        }
        self.adc.restore_cfg(prev_cfg);
        Ok(())
    }
}
