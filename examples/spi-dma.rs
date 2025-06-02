//! Transmits data over an SPI port using DMA

#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    // Acquire the GPIOB peripheral
    let gpiob = dp.GPIOB.split(&mut rcc);

    let pins = (Some(gpiob.pb13), Some(gpiob.pb14), Some(gpiob.pb15));

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::new(dp.SPI2, pins, spi_mode, 100.kHz(), &mut rcc);

    // Set up the DMA device
    let dma = dp.DMA1.split(&mut rcc);

    // Connect the SPI device to the DMA
    let spi_dma = spi.with_tx_dma(dma.5);

    // Start a DMA transfer
    let transfer = spi_dma.write(b"hello, world");

    // Wait for it to finnish. The transfer takes ownership over the SPI device
    // and the data being sent anb those things are returned by transfer.wait
    let (_buffer, _spi_dma) = transfer.wait();

    loop {}
}
