//! Open loop motor control

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    prelude::*,
    pac::{self,TIM2},
    gpio::gpioa::PA0,
    gpio::{Alternate, PushPull},
    timer::Timer,
    serial::{Config, Serial},
    pwm::{Pins, Pwm, C1},
};
use embedded_hal::digital::v1_compat::OldOutputPin;

use nb::block;

use motor_driver::Motor;
use cortex_m_rt::{entry, exception, ExceptionFrame};

// Using PA0 channel for TIM2 PWM output
struct MyPwm(PA0<Alternate<PushPull>>);
impl Pins<TIM2> for MyPwm {
    const REMAP: u8 = 0b00;
    const C1: bool = true;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C1>;
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb2,
    );

    let mut rx = serial.split().1;


    let pwm = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
        .pwm(
            MyPwm(gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl)),
            &mut afio.mapr,
            1.khz()
        );

    let max_duty = pwm.get_max_duty() as i16;
    let mut motor = Motor::tb6612fng(
        OldOutputPin::from(gpioa.pa1.into_push_pull_output(&mut gpioa.crl)),
        OldOutputPin::from(gpioa.pa2.into_push_pull_output(&mut gpioa.crl)),
        pwm,
    );

    let mut duty = max_duty;
    let mut brake = true;

    motor.duty(duty as u16);

    hprintln!("{} {}", max_duty, brake).unwrap();
    loop {
        match block!(rx.read()).unwrap() {
            b'*' => duty *= 2,
            b'+' => duty += 1,
            b'-' => duty -= 1,
            b'/' => duty /= 2,
            b'r' => duty *= -1,
            b's' => brake = !brake,
            _ => continue,
        }

        if duty > max_duty {
            duty = max_duty;
        } else if duty < -max_duty {
            duty = -max_duty;
        }

        if brake {
            motor.brake();
        } else if duty > 0 {
            motor.cw();
        } else {
            motor.ccw();
        }

        motor.duty(duty.abs() as u16);

        hprintln!("{} {}", duty, brake).unwrap();
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
