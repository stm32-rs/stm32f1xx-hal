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
    pac,
    prelude::*,
    serial::{Config, Rx3_16, Serial, Tx3_16},
};

// The address of the slave device.
const SLAVE_ADDR: u8 = 123;

// Maximum possible message length.
const MSG_MAX_LEN: usize = u8::MAX as usize;

// Receives a message addressed to the slave device. Returns the size of the received message.
fn receive_msg(serial_rx: &mut Rx3_16, buf: &mut [u8; MSG_MAX_LEN]) -> usize {
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
fn send_msg(mut serial_tx: Tx3_16, msg: &[u8]) -> Tx3_16 {
    // Send address.
    block!(serial_tx.write(SLAVE_ADDR as u16 | 0x100)).ok();

    // Switching from u16 to u8 data.
    let mut serial_tx = serial_tx.with_u8_data();

    // Send message len.
    assert!(msg.len() <= MSG_MAX_LEN);
    block!(serial_tx.write(msg.len() as u8)).ok();

    // Send message.
    for &b in msg {
        block!(serial_tx.write(b)).ok();
    }

    // Switching back from u8 to u16 data.
    serial_tx.with_u16_data()
}

#[entry]
fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate.
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs.
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`.
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers.
    let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheral.
    let mut gpiob = p.GPIOB.split();

    let tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let rx_pin = gpiob.pb11;

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let serial = Serial::usart3(
        p.USART3,
        (tx_pin, rx_pin),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()).wordlength_9(),
        clocks,
    )
    // Switching the 'Word' type parameter for the 'Read' and 'Write' traits from u8 to u16.
    .with_u16_data();

    // Split the serial struct into a transmitting and a receiving part.
    let (mut serial_tx, mut serial_rx) = serial.split();

    let mut buf = [0u8; MSG_MAX_LEN];

    // loopback
    loop {
        // Receive message from master device.
        let received_msg_len = receive_msg(&mut serial_rx, &mut buf);
        // Send the received message back.
        serial_tx = send_msg(serial_tx, &buf[..received_msg_len]);
    }
}
