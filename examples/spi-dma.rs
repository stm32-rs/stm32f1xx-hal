#![no_std]
#![no_main]

/**
  Transmits data over an SPI port using DMA
*/
use panic_halt as _;

use cortex_m::singleton;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi, SpiTxDma},
    dma::{R, Transfer, Transferable, WriteDma}
};

// Set size of data buffer to transmit via SPI DMA
const DATA_BUFFER_SIZE: usize = 8;

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
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 100.khz(), clocks, &mut rcc.apb1);

    // Set up the DMA device
    let dma = dp.DMA1.split(&mut rcc.ahb);

    // Connect the SPI device to the DMA
    let mut spi_dma = spi.with_tx_dma(dma.5);

    // Create a mutable data buffer, as we need to write data into it
    let data_buffer: &'static mut [u8; DATA_BUFFER_SIZE] =
            singleton!(: [u8; DATA_BUFFER_SIZE] = [0; DATA_BUFFER_SIZE]).unwrap();

    // Use a byte counter to generate some data for our buffer
    let mut data_byte = 0u8;
    

    // Do SPI transmission in an endless loop
    loop {
        // Fill the buffer with some data
        for ix in 0..DATA_BUFFER_SIZE {
            // Increase by 1, and insure it wraps
            data_byte = data_byte.wrapping_add(1u8);

            // Put the byte into the buffer
            data_buffer[ix] = data_byte;
        }

        // Call write function
        let transfer = write_spi_dma(spi_dma, data_buffer);

        // Wait for transfer to complete
        let (ret_buffer, ret_spidma) = transfer.wait();

        // Return ownership, so we can re-use these the next iteration
        data_buffer = ret_buffer;
        spi_dma = ret_spidma;
    }
}

/// The writing is done is a separate function, as this is typically how a driver would work
/// The Driver will take ownership of the SPI DMA for the duration of the Transfer Operation.
/// When complete, the wait() function returns the ownership of the buffer and SPI DMA, which
/// makes it usabe for something else. This way, one SPI can be shared among multiple drivers.
fn write_spi_dma<SPI, REMAP, PINS, CHANNEL>(spi_dma: SpiTxDma<SPI, REMAP, PINS, CHANNEL>, buffer: &'static mut [u8]) 
    -> Transfer<R, &'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>>
where
    SpiTxDma<SPI, REMAP, PINS, CHANNEL>: WriteDma<&'static mut [u8], u8>,
    Transfer<R, &'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>>:
        Transferable<&'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>> 
{
    // Simply call write, and return the Transfer instance
    spi_dma.write(buffer)
}
