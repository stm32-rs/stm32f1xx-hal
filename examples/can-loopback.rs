#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use nb::block;
use stm32f1xx_hal::{
    can::{Can, Filter, Frame},
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

    #[cfg(not(feature = "connectivity"))]
    let mut can = Can::new(dp.CAN1, &mut rcc.apb1, dp.USB);

    #[cfg(feature = "connectivity")]
    let mut can = Can::new(dp.CAN1, &mut rcc.apb1);

    // Use loopback mode: No pins need to be assigned to peripheral.
    can.configure(|config| {
        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        config.set_bit_timing(0x001c_0003);
        config.set_loopback(true);
        config.set_silent(true);
    });

    // Get the transmitter part.
    let mut tx = can.take_tx().unwrap();

    // Receive all messages.
    #[cfg(not(feature = "connectivity"))]
    let mut filters = can.split_filters().unwrap();
    #[cfg(feature = "connectivity")]
    let (mut filters, _) = can.split_filters().unwrap();
    filters.add(Filter::accept_all()).unwrap();
    let mut rx = can.take_rx(filters).unwrap();

    // Sync to the bus and start normal operation.
    block!(can.enable()).ok();

    // Send and receive messages with incrementing identifier.
    for id in (0..0x7FF_u32).cycle() {
        let frame_tx = Frame::new_standard(id, &[id as u8]);
        block!(tx.transmit(&frame_tx)).unwrap();

        let frame_rx = block!(rx.receive()).unwrap();

        assert_eq!(frame_tx, frame_rx); // Only compares the identifier.
        assert_eq!(frame_tx.data(), frame_rx.data());
    }

    loop {}
}
