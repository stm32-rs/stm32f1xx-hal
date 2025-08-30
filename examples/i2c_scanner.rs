//! Example of using I2C.
//! Scans available I2C devices on bus and print the result.

#![no_std]
#![no_main]

use panic_probe as _;

use rtt_target::{rprint, rprintln, rtt_init_print};

use cortex_m_rt::entry;

use stm32f1xx_hal::{self as hal, gpio::GpioExt, i2c::I2c, pac, prelude::*};

const VALID_ADDR_RANGE: core::ops::Range<u8> = 0x08..0x78;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let gpiob = dp.GPIOB.split(&mut rcc);

    // Configure I2C1
    let scl = gpiob.pb6;
    let sda = gpiob.pb7;
    let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        hal::i2c::Mode::standard(100.kHz()),
        &mut rcc,
    );

    rprintln!("Start i2c scanning...");
    rprintln!();

    for addr in 0x00_u8..0x80 {
        // Write the empty array and check the slave response.
        let byte: [u8; 1] = [0; 1];
        if VALID_ADDR_RANGE.contains(&addr) && i2c.write(addr, &byte).is_ok() {
            rprint!("{:02x}", addr);
        } else {
            rprint!("..");
        }
        if addr % 0x10 == 0x0F {
            rprintln!();
        } else {
            rprint!(" ");
        }
    }

    rprintln!();
    rprintln!("Done!");

    #[allow(clippy::empty_loop)]
    loop {}
}
