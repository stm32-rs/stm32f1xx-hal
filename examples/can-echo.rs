//! Simple CAN example.
//! Requires a transceiver connected to PA11, PA12 (CAN1) or PB5 PB6 (CAN2).

#![no_main]
#![no_std]

use panic_halt as _;

use bxcan::filter::Mask32;
use cortex_m_rt::entry;
use nb::block;
use stm32f1xx_hal::{can::Can, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // To meet CAN clock accuracy requirements an external crystal or ceramic
    // resonator must be used. The blue pill has a 8MHz external crystal.
    // Other boards might have a crystal with another frequency or none at all.
    rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();

    let mut can1 = {
        #[cfg(not(feature = "connectivity"))]
        let can = Can::new(dp.CAN1, dp.USB);
        #[cfg(feature = "connectivity")]
        let can = Can::new(dp.CAN1);

        let mut gpioa = dp.GPIOA.split();
        let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        can.assign_pins((tx, rx), &mut afio.mapr);

        bxcan::Can::new(can)
    };

    // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    can1.modify_config().set_bit_timing(0x001c_0003);

    // Configure filters so that can frames can be received.
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());

    #[cfg(feature = "connectivity")]
    let _can2 = {
        let can = Can::new(dp.CAN2);

        let mut gpiob = dp.GPIOB.split();
        let rx = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        can.assign_pins((tx, rx), &mut afio.mapr);

        let mut can2 = bxcan::Can::new(can);

        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        can2.modify_config().set_bit_timing(0x001c_0003);

        // A total of 28 filters are shared between the two CAN instances.
        // Split them equally between CAN1 and CAN2.
        let mut slave_filters = filters.set_split(14).slave_filters();
        slave_filters.enable_bank(14, Mask32::accept_all());
        can2
    };

    // Drop filters to leave filter configuraiton mode.
    drop(filters);

    // Select the interface.
    let mut can = can1;
    //let mut can = _can2;

    // Split the peripheral into transmitter and receiver parts.
    block!(can.enable()).unwrap();

    // Echo back received packages in sequence.
    // See the `can-rtfm` example for an echo implementation that adheres to
    // correct frame ordering based on the transfer id.
    loop {
        if let Ok(frame) = block!(can.receive()) {
            block!(can.transmit(&frame)).unwrap();
        }
    }
}
