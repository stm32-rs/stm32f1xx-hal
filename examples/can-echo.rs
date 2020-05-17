//! Simple CAN example. Requires a transceiver connected to PB5 and PB6.

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use nb::block;
use stm32f1xx_hal::{
    can::{Can, Filter, NUM_FILTER_BANKS},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // To meet CAN clock accuracy requirements an external crystal or ceramic
    // resonator must be used.
    rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

    let mut can2 = Can::new(dp.CAN2, &mut rcc.apb1);

    // Select pins for CAN2.
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let can_rx_pin = gpiob.pb5.into_floating_input(&mut gpiob.crl);
    let can_tx_pin = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    can2.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

    can2.configure(|config| {
        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        config.set_bit_timing(0x001c_0003);
    });

    // Filters are required to use the receiver part of CAN2.
    // Because the filter banks are part of CAN1 we first need to enable CAN1
    // and split the filters between the peripherals to use them for CAN2.
    let mut can1 = Can::new(dp.CAN1, &mut rcc.apb1);

    // Split the filters at index 0: No filters for CAN1 (unused), 28 filters
    // for CAN2.
    let (_filters1, mut filters2) = can1.split_filters(0).unwrap();
    assert_eq!(filters2.num_available(), NUM_FILTER_BANKS);
    filters2.add(&Filter::accept_all()).unwrap(); // Receive all messages.

    // Split the peripheral into transmitter and receiver parts.
    let mut rx = can2.take_rx(filters2).unwrap();
    let mut tx = can2.take_tx().unwrap();

    // Sync to the bus and start normal operation.
    block!(can2.enable()).ok();

    // Echo back received packages in sequence.
    // See the `can-rtfm` example for an echo implementation that adheres to
    // correct frame ordering based on the transfer id.
    loop {
        if let Ok(frame) = block!(rx.receive()) {
            block!(tx.transmit(&frame)).unwrap();
        }
    }
}
