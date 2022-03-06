//! Reads a value from flash page 127, increments it, writes it back. Then, blinks n times,
//! where n: original value % 10.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use nb::block;

use panic_halt as _;

use cortex_m_rt::entry;
//use rtt_target::{rprint, rprintln, rtt_init_print};
use stm32f1xx_hal::flash::FlashWriter;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

use stm32f1xx_hal as _;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    //rtt_init_print!();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let writer = flash.writer(
        stm32f1xx_hal::flash::SectorSize::Sz1K,
        stm32f1xx_hal::flash::FlashSize::Sz128K,
    );
    //rprintln!("Reading page 127");
    let page = writer.read(127 * 1024, 16).unwrap();
    //rprintln!("{:?}", &page[..16]);
    let value: u32 = u32::from_le_bytes(page[..4].try_into().unwrap());
    //rprintln!("Value on page 127 is: {:?}", value);

    // Page must be erased before writing new value.
    //rprintln!("Erasing page");
    let mut writer = writer;
    writer.erase(127 * 1024, 1024);

    let new_value = value + 1;
    //rprintln!("Writing new value {}", new_value);
    writer.write(127 * 1024, &new_value.to_le_bytes()).unwrap();

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // LED should be off initially (active-low on blue pill board).
    led.set_high();

    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

    let blink_amount = value % 10;
    //rprintln!(
    //"Blinking 'value % 10' times. 'value % 10 = {}'",
    //blink_amount
    //);
    // Wait for the timer to trigger an update and change the state of the LED
    for _ in 0..blink_amount {
        block!(timer.wait()).unwrap();
        led.set_low();
        block!(timer.wait()).unwrap();
        led.set_high();
    }

    loop {
        continue;
    }
}
