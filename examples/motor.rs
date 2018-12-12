//! Open loop motor control

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate motor_driver;
extern crate panic_semihosting;
#[macro_use(block)]
extern crate nb;
extern crate stm32f1xx_hal as hal;

use core::fmt::Write;

use hal::prelude::*;
use hal::serial::Serial;
use hal::stm32f103xx;
use motor_driver::Motor;
use rt::{entry, exception, ExceptionFrame};
use sh::hio;

#[entry]
fn main() -> ! {
    let p = stm32f103xx::Peripherals::take().unwrap();

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
        115_200.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let mut rx = serial.split().1;

    let pwm = p.TIM2.pwm(
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
        &mut afio.mapr,
        1.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let max_duty = pwm.get_max_duty() as i16;
    let mut motor = Motor::tb6612fng(
        gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
        gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
        pwm,
    );

    let mut duty = max_duty;
    let mut brake = true;

    motor.duty(duty as u16);

    let mut hstdout = hio::hstdout().unwrap();
    writeln!(hstdout, "{} {}", max_duty, brake).unwrap();
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

        writeln!(hstdout, "{} {}", duty, brake).unwrap();
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
