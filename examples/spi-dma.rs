#![no_std]
#![no_main]

/**
  Transmits data over an SPI port using DMA
*/

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    spi::{Spi, Mode, Polarity, Phase},
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOA peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let pins = (
            gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb14.into_floating_input(&mut gpiob.crh),
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };
    let mut spi = Spi::spi2(
        dp.SPI2,
        pins,
        spi_mode,
        100.khz(),
        clocks,
        &mut rcc.apb1
    );

    // Set up the DMA device
    let dma = dp.DMA1.split(&mut rcc.ahb);

    // Connect the SPI device to the DMA
    let spi_dma = spi.with_tx_dma(dma.5);

    // Start a DMA transfer
    let transfer = spi_dma.write(b"hello, world");

    // Wait for it to finnish. The transfer takes ownership over the SPI device
    // and the data being sent anb those things are returned by transfer.wait
    let (_spi_dma, _buffer) = transfer.wait();

    loop {
    }
}
