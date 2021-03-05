//! Through buttons to control of LED
//!
//! This assumes that has tow leds, the red light is connected to *pa8* and the green light is connected to *pd2*.
//!
//! Meanwhile, it has tow buttons, we can call them *key_0* and *key_1*.
//!
//! The *key_0* is connected to *pc5*, and the *key_1* is connected to *pa15*.
//!
//! We need to set `into_pull_up_input` for *pc5* and *pa15*, for the reason that the *key_0* and *key_1* was connected to **GND**.
//!
//! Use *key_0* to control of red light, *key_1* to control of green light.
//!
//! Only press a button after release the button to turn on the led, again is turn down the led. And long press was nullity.

#![deny(unsafe_code)]
#![no_std]
#![no_main]
use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use panic_halt as _;
use stm32f1xx_hal::gpio::{gpioa::PA8, gpiod::PD2, Output, PushPull};
use stm32f1xx_hal::{delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // GPIOX
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut _gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    // The red_led and greed_led was connected to V3.3.
    let red_led = gpioa
        .pa8
        .into_push_pull_output_with_state(&mut gpioa.crh, stm32f1xx_hal::gpio::State::High);
    let greed_led = gpiod
        .pd2
        .into_push_pull_output_with_state(&mut gpiod.crl, stm32f1xx_hal::gpio::State::High);


    // The pa15 start out being used by the debugger.
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let (gpioa_pa15, _gpiob_pb3, _gpiob_pb4) =
        afio.mapr.disable_jtag(gpioa.pa15, _gpiob.pb3, _gpiob.pb4);

    let key_0 = gpioc.pc5.into_pull_up_input(&mut gpioc.crl);
    let key_1 = gpioa_pa15.into_pull_up_input(&mut gpioa.crh);
    
    // the key_up is record of key if press the button
    let mut key_up: bool = true;

    let mut delay = Delay::new(cp.SYST, clocks);

    // sava pins state
    let mut red_led = RedLedPin {
        state: true,
        pin: red_led,
    };
    let mut green_led = GreenLedPin {
        state: true,
        pin: greed_led,
    };


    loop {
        let key_result = (key_0.is_low().unwrap(), key_1.is_low().unwrap());
        if key_up && (key_result.0 || key_result.1) {
            key_up = false;
            delay.delay_ms(10u8);
            match key_result {
                (x, _) if x == true => red_led.switch().unwrap(),
                (_, y) if y == true => green_led.switch().unwrap(),
                (x, y) if x == true && y == true => {
                    red_led.switch().unwrap();
                    green_led.switch().unwrap();
                }
                (_, _) => (),
            }
        } else if !key_result.0 && !key_result.1 {
            key_up = true;
            delay.delay_ms(10u8);
        }
    }
}

struct RedLedPin {
    state: bool,
    pin: PA8<Output<PushPull>>,
}
struct GreenLedPin {
    state: bool,
    pin: PD2<Output<PushPull>>,
}

impl OutputPin for RedLedPin {
    type Error = ();
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        self.pin.set_low().unwrap();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        self.pin.set_high().unwrap();
        Ok(())
    }
}
impl OutputPin for GreenLedPin {
    type Error = ();
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        self.pin.set_low().unwrap();
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        self.pin.set_high().unwrap();
        Ok(())
    }
}

impl StatefulOutputPin for RedLedPin {
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }
}
impl StatefulOutputPin for GreenLedPin {
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }
}

/// Note: This trait shuold switch state of pin.
trait Switch {
    type Error;
    fn switch(&mut self) -> Result<(), Self::Error>;
}

impl Switch for RedLedPin {
    type Error = ();

    fn switch(&mut self) -> Result<(), Self::Error> {
        if self.is_set_high()? {
            self.set_low()
        } else {
            self.set_high()
        }
    }
}

impl Switch for GreenLedPin {
    type Error = ();

    fn switch(&mut self) -> Result<(), Self::Error> {
        if self.is_set_high()? {
            self.set_low()
        } else {
            self.set_high()
        }
    }
}
