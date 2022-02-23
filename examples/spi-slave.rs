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
use embedded_hal::spi::{Mode, Phase, Polarity};
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

use stm32f1xx_hal::{
    gpio::{
        gpiob::{PB13, PB14, PB15},
        Alternate, Floating, Input, PushPull,
    },
    pac::{self, interrupt, Peripherals, SPI2},
    prelude::*,
    spi::{Event, Slave, Spi, Spi2NoRemap},
};

type SlaveSpi = Spi<
    SPI2,
    Spi2NoRemap,
    (
        PB13<Input<Floating>>,
        PB14<Alternate<PushPull>>,
        PB15<Input<Floating>>,
    ),
    u8,
    Slave,
>;

static mut SPI2SLAVE: Option<SlaveSpi> = None;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let spi1 = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        10.kHz(),
        clocks,
    );

    // SPI2
    let sck = gpiob.pb13;
    let miso = gpiob.pb14.into_alternate_push_pull(&mut gpiob.crh);
    let mosi = gpiob.pb15;

    let spi2 = Spi::spi2_slave(dp.SPI2, (sck, miso, mosi), MODE);

    // Set up the DMA device
    let dma = dp.DMA1.split();

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
                if let Ok(w) = nb::block!(spi2.read()) {
                    R_BUFFER[RIDX] = w;
                    RIDX += 1;
                    if RIDX >= R_BUFFER_LEN - 1 {
                        RIDX = 0;
                    }
                }
            }
            if spi2.is_tx_empty() {
                if let Ok(()) = nb::block!(spi2.send(W_BUFFER[WIDX])) {
                    WIDX += 1;
                    if WIDX >= W_BUFFER_LEN {
                        WIDX = 0;
                    }
                }
            }
        }
    })
}
