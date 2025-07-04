//! SPI slave mode test
//!
//! spi1 master <-> spi2 slave
//! PA5 <-SCK-> PB13
//! PA6 <-MISO-> PB14
//! PA7 <-MOSI-> PB15

#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use cortex_m::{asm, singleton};
use stm32f1xx_hal::spi::{Mode, Phase, Polarity};
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

use stm32f1xx_hal::{
    pac::{self, interrupt, Peripherals, SPI2},
    prelude::*,
    spi::{Event, SpiSlave},
};

static mut SPI2SLAVE: Option<SpiSlave<SPI2, u8>> = None;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // SPI1
    // Convert pins during SPI initialization
    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;

    let spi1 = dp.SPI1.spi(
        (Some(sck), Some(miso), Some(mosi)),
        MODE,
        10.kHz(),
        &mut rcc,
    );

    // SPI2
    // Convert pins before SPI initialization
    let sck = gpiob.pb13;
    let miso = gpiob.pb14;
    let mosi = gpiob.pb15;

    let spi2 = dp
        .SPI2
        .spi_slave((Some(sck), Some(miso), Some(mosi)), MODE, &mut rcc);

    // Set up the DMA device
    let dma = dp.DMA1.split(&mut rcc);

    let master_spi_dma = spi1.with_rx_tx_dma(dma.2, dma.3);
    let slave_spi_dma = spi2.with_rx_tx_dma(dma.4, dma.5);

    let master_buf = singleton!(: [u8; 12] = [0; 12]).unwrap();
    let slave_buf = singleton!(: [u8; 12] = [0; 12]).unwrap();

    // Make sure the buffers are the same length
    let slave_transfer = slave_spi_dma.read_write(slave_buf, b"hello,master");
    let master_transfer = master_spi_dma.read_write(master_buf, b"hello, slave");

    let (buffer, spi1_dma) = master_transfer.wait();
    let (_buffer, spi2_dma) = slave_transfer.wait();

    asm::bkpt();

    // test SPI with interrupts
    let (mut spi2, _, _) = spi2_dma.release();

    spi2.listen(Event::Rxne);
    spi2.listen(Event::Txe);
    spi2.listen(Event::Error);

    cortex_m::interrupt::free(|_| unsafe {
        SPI2SLAVE.replace(spi2);
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SPI2);
    }

    let master_transfer = spi1_dma.read_write(buffer.0, buffer.1);
    let (_buffer, _spi1_dma) = master_transfer.wait();

    loop {}
}

const R_BUFFER_LEN: usize = 16;
static mut R_BUFFER: &mut [u8; R_BUFFER_LEN] = &mut [0; R_BUFFER_LEN];
static mut RIDX: usize = 0;

const W_BUFFER_LEN: usize = 3;
static W_BUFFER: &[u8; W_BUFFER_LEN] = &[1, 2, 3];
static mut WIDX: usize = 0;

#[interrupt]
unsafe fn SPI2() {
    cortex_m::interrupt::free(|_| {
        if let Some(spi2) = SPI2SLAVE.as_mut() {
            if spi2.is_overrun() {
                // mcu processing speed is not enough
                asm::bkpt();
            }
            if spi2.is_rx_not_empty() {
                if let Ok(w) = nb::block!(spi2.read_nonblocking()) {
                    R_BUFFER[RIDX] = w;
                    RIDX += 1;
                    if RIDX >= R_BUFFER_LEN - 1 {
                        RIDX = 0;
                    }
                }
            }
            if spi2.is_tx_empty() {
                if let Ok(()) = nb::block!(spi2.write_nonblocking(W_BUFFER[WIDX])) {
                    WIDX += 1;
                    if WIDX >= W_BUFFER_LEN {
                        WIDX = 0;
                    }
                }
            }
        }
    })
}
