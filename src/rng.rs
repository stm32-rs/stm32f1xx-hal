use crate::adc::{Adc, SampleTime};
use crate::pac::ADC1;
use embedded_hal::blocking::rng;

pub struct Rng {
    adc: Adc<ADC1>,
}

impl Adc<ADC1> {
    pub fn as_rng(self) -> Rng {
        Rng { adc: self }
    }
}

impl rng::Read for Rng {
    type Error = ();
    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let prev_cfg = self.adc.save_cfg();
        self.adc.set_sample_time(SampleTime::T_1);
        for buf in buffer {
            let mut random: u8 = 0;
            for _i in 0..8 {
                let mut num = 0;
                for _i in 0..5 {
                    num += self.adc.read_aux(16u8) & 0x01;
                }
                random = (random << 1) + (num & 0x01) as u8;
            }
            *buf = random;
        }
        self.adc.restore_cfg(prev_cfg);
        Ok(())
    }
}
