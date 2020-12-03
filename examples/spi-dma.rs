#![no_std]
#![no_main]

/**
  Transmits data over an SPI port using DMA
*/
use panic_halt as _;

use nb::block;

use cortex_m::singleton;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi, SpiTxDma},
    dma::{Transfer, Transferable, WriteDma, R},
    timer::Timer
};

// The length of the data buffer for SPI transmission
const DATA_BUFFER_LEN: usize = 8;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();

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

    // Prepare a byte buffer for our transmissible data
    let mut data_buffer: &'static mut [u8] =
        singleton!(: [u8; DATA_BUFFER_LEN] = [0; DATA_BUFFER_LEN]).unwrap();

    // Configure the systick timer to trigger an update every second, used for a delay
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());

    // Simple counter to use to create data for SPI transmission
    let mut bcount: u8 = 1;

    // Now go into an infinite loop, which generates some data and transmits it
    loop {
        // Fill the buffer with data
        for i in 0..DATA_BUFFER_LEN {
            data_buffer[i] = bcount;
            bcount = bcount.wrapping_add(1);
        }

        // Call function to do the actual transmission, to demonstrate how (not exactly obvious!)
        let transfer = transfer_data_spidma(data_buffer, spi_dma);

        // Wait for the transfer to complete
        let (buffer_return, spi_dma_return) = transfer.wait();

        // Make sure the ownership is returned for the next loop iteration
        data_buffer = buffer_return;
        spi_dma = spi_dma_return;

        // Wait a short delay, before repeating
        block!(timer.wait()).unwrap();
    }
}

// Demonstrates how the SpiTxDma trait implementation can be passed to a function (or struct),
// and further how to return the Transfer instance back.
fn transfer_data_spidma<SPI, REMAP, PINS, CHANNEL>(
        data_buffer: &'static mut [u8],
        spi_dma: SpiTxDma<SPI, REMAP, PINS, CHANNEL>
    ) -> Transfer<R, &'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>>
where
    SpiTxDma<SPI, REMAP, PINS, CHANNEL>: WriteDma<&'static mut [u8], u8>,
    Transfer<R, &'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>>:
        Transferable<&'static mut [u8], SpiTxDma<SPI, REMAP, PINS, CHANNEL>>,
{
    // Simply call 'write' on the DMA instance, and return the 'Transfer'
    spi_dma.write(data_buffer)
}
