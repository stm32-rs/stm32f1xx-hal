//! Showcases advanced CAN filter capabilities.
//! Does not require additional transceiver hardware.

#![no_main]
#![no_std]

use bxcan::{
    filter::{ListEntry16, ListEntry32, Mask16},
    ExtendedId, Frame, StandardId,
};
use panic_halt as _;

use cortex_m_rt::entry;
use nb::block;
use stm32f1xx_hal::{can::Can, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // To meet CAN clock accuracy requirements, an external crystal or ceramic
    // resonator must be used.
    rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

    #[cfg(not(feature = "connectivity"))]
    let can = Can::new(dp.CAN1, dp.USB);

    #[cfg(feature = "connectivity")]
    let can = Can::new(dp.CAN1);

    let mut can = bxcan::Can::new(can);

    // Use loopback mode: No pins need to be assigned to peripheral.
    // APB1 (PCLK1): 8MHz, Bit rate: 500Bit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    can.modify_config()
        .set_bit_timing(0x001c_0000)
        .set_loopback(true)
        .set_silent(true);

    let mut filters = can.modify_filters();
    assert!(filters.num_banks() > 3);

    // The order of the added filters is important: it must match configuration
    // of the `split_filters_advanced()` method.

    // 2x 11bit id + mask filter bank: Matches 0, 1, 2
    // TODO: Make this accept also ID 2
    filters.enable_bank(
        0,
        [
            // accepts 0 and 1
            Mask16::frames_with_std_id(StandardId::new(0).unwrap(), StandardId::new(1).unwrap()),
            // accepts 0 and 2
            Mask16::frames_with_std_id(StandardId::new(0).unwrap(), StandardId::new(2).unwrap()),
        ],
    );

    // 2x 29bit id filter bank: Matches 4, 5
    filters.enable_bank(
        1,
        [
            ListEntry32::data_frames_with_id(ExtendedId::new(4).unwrap()),
            ListEntry32::data_frames_with_id(ExtendedId::new(5).unwrap()),
        ],
    );

    // 4x 11bit id filter bank: Matches 8, 9, 10, 11
    filters.enable_bank(
        2,
        [
            ListEntry16::data_frames_with_id(StandardId::new(8).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(9).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(10).unwrap()),
            ListEntry16::data_frames_with_id(StandardId::new(11).unwrap()),
        ],
    );

    // Enable filters.
    drop(filters);

    // Sync to the bus and start normal operation.
    block!(can.enable()).ok();

    // Some messages shall pass the filters.
    for &id in &[0, 1, 2, 8, 9, 10, 11] {
        let frame_tx = Frame::new_data(StandardId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        let frame_rx = block!(can.receive()).unwrap();
        assert_eq!(frame_tx, frame_rx);
    }
    for &id in &[4, 5] {
        let frame_tx = Frame::new_data(ExtendedId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        let frame_rx = block!(can.receive()).unwrap();
        assert_eq!(frame_tx, frame_rx);
    }

    // Some messages shall not be received.
    for &id in &[3, 6, 7, 12] {
        let frame_tx = Frame::new_data(ExtendedId::new(id).unwrap(), [id as u8]);
        block!(can.transmit(&frame_tx)).unwrap();
        while !can.is_transmitter_idle() {}

        assert!(can.receive().is_err());
    }

    let mut gpiob = dp.GPIOB.split();
    let mut led = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
    led.set_high();

    loop {}
}
