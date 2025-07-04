//! Testing 9 bits USART word length mode.
//!
//! This example demonstrates the use of the MSB bit (bit 8) to mark the beginning of a packet.
//! The first byte of the packet contains the address of the slave device.
//! The second byte of the packet contains the length of the message.
//! The remaining bytes of the packet contain the message itself.

//#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{
    gpio::PushPull,
    pac,
    prelude::*,
    serial::{self, Config, Error},
};

// The address of the slave device.
const SLAVE_ADDR: u8 = 123;

// Maximum possible message length.
const MSG_MAX_LEN: usize = u8::MAX as usize;

// Receives a message addressed to the slave device. Returns the size of the received message.
fn receive_msg<RX>(serial_rx: &mut RX, buf: &mut [u8; MSG_MAX_LEN]) -> usize
where
    RX: embedded_hal_02::serial::Read<u16, Error = serial::Error>,
{
    enum RxPhase {
        Start,
        Length,
        Data { len: usize, idx: usize },
    }

    let mut rx_phase = RxPhase::Start;

    loop {
        // Read the word that was just sent. Blocks until the read is complete.
        let received = block!(serial_rx.read()).unwrap();

        // If the beginning of the packet.
        if (received & 0x100) != 0 {
            rx_phase = if received as u8 == SLAVE_ADDR {
                RxPhase::Length
            } else {
                RxPhase::Start
            }
        } else {
            match rx_phase {
                RxPhase::Start => {}

                RxPhase::Length => {
                    if received == 0 {
                        return 0;
                    }
                    rx_phase = RxPhase::Data {
                        len: received as usize,
                        idx: 0,
                    };
                }

                RxPhase::Data { len, ref mut idx } => {
                    buf[*idx] = received as u8;
                    *idx += 1;
                    if *idx == len {
                        return len;
                    }
                }
            }
        }
    }
}

// Send message.
fn send_msg<TX>(serial_tx: &mut TX, msg: &[u8])
where
    TX: embedded_hal_02::serial::Write<u8, Error = Error>
        + embedded_hal_02::serial::Write<u16, Error = Error>,
{
    // Send address.
    block!(serial_tx.write(SLAVE_ADDR as u16 | 0x100)).unwrap();

    // Send message len.
    assert!(msg.len() <= MSG_MAX_LEN);
    block!(serial_tx.write(msg.len() as u8)).unwrap();

    // Send message.
    for &b in msg {
        block!(serial_tx.write(b)).unwrap();
    }
}

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate.
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    // Prepare the alternate function I/O registers.
    //let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral.
    let gpiob = p.GPIOB.split(&mut rcc);

    let tx_pin = gpiob.pb10;
    let rx_pin = gpiob.pb11;

    // Set up the usart device. Take ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    //
    //let serial = Serial::<_, PushPull, _>::new(p.USART3,
    // or shorter
    let serial = p.USART3.serial::<PushPull, _>(
        (tx_pin, rx_pin),
        Config::default()
            .baudrate(9600.bps())
            .wordlength_9bits()
            .parity_none(),
        &mut rcc,
    );

    // Split the serial struct into a transmitting and a receiving part.
    let (mut serial_tx, mut serial_rx) = serial.split();

    let mut buf = [0u8; MSG_MAX_LEN];

    // loopback
    loop {
        // Receive message from master device.
        let received_msg_len = receive_msg(&mut serial_rx, &mut buf);
        // Send the received message back.
        send_msg(&mut serial_tx, &buf[..received_msg_len]);
    }
}
