//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac::{self, interrupt, USART1},
    prelude::*,
    serial::{Rx, RxEvent, Tx, TxEvent},
};

static mut RX: Option<Rx<USART1>> = None;
static mut TX: Option<Tx<USART1>> = None;
#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain(&mut rcc);

    // Prepare the GPIOB peripheral
    let mut gpiob = p.GPIOB.split(&mut rcc);

    // USART1
    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;

    // Set up the usart device. Takes ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let (mut tx, mut rx) = p
        .USART1
        .remap(&mut afio.mapr)
        .serial((tx, rx), 115_200.bps(), &mut rcc)
        .split();
    tx.listen(TxEvent::TxEmpty);
    rx.listen(RxEvent::RxNotEmpty | RxEvent::Idle);

    cortex_m::interrupt::free(|_| unsafe {
        TX.replace(tx);
        RX.replace(rx);
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
    }

    loop {
        cortex_m::asm::wfi()
    }
}
const BUFFER_LEN: usize = 4096;
static mut BUFFER: &mut [u8; BUFFER_LEN] = &mut [0; BUFFER_LEN];
static mut WIDX: usize = 0;

unsafe fn write(buf: &[u8]) {
    if let Some(tx) = TX.as_mut() {
        buf.iter()
            .for_each(|w| if let Err(_err) = nb::block!(tx.write(*w)) {})
    }
}
#[interrupt]
unsafe fn USART1() {
    cortex_m::interrupt::free(|_| {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rx_not_empty() {
                if let Ok(w) = nb::block!(rx.read()) {
                    BUFFER[WIDX] = w;
                    WIDX += 1;
                    if WIDX >= BUFFER_LEN - 1 {
                        write(&BUFFER[..]);
                        WIDX = 0;
                    }
                }
                rx.listen(RxEvent::Idle);
            } else if rx.is_idle() {
                rx.unlisten(RxEvent::Idle);
                write(&BUFFER[0..WIDX]);
                WIDX = 0;
            }
        }
    })
}
