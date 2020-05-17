//! Interrupt driven CAN transmitter with RTFM.
//!
//! CAN frames are allocated from a static memory pool and stored in a priority
//! queue (min heap) for transmisison. To start transmission the CAN TX
//! interrupt has to be triggered manually once. With each successful
//! transmission the interrupt is reentered and more data is fetched from the
//! queue.
//! Received frames are simply echoed back. In contrast to the naive `can-echo`
//! example all messages are also correctly prioritized by the transmit queue.

#![no_main]
#![no_std]

use heapless::{
    binary_heap::{BinaryHeap, Min},
    consts::*,
    pool,
    pool::{
        singleton::{Box, Pool},
        Init,
    },
};
use panic_halt as _;
use rtfm::app;
use stm32f1xx_hal::{
    can::{Can, Filter, Frame, Id, Rx, Tx},
    pac::{Interrupt, CAN2},
    prelude::*,
};

pool!(
    #[allow(non_upper_case_globals)]
    CanFramePool: Frame
);

fn alloc_frame(id: Id, data: &[u8]) -> Box<CanFramePool, Init> {
    let frame_box = CanFramePool::alloc().unwrap();
    frame_box.init(Frame::new(id, data))
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        can_tx: Tx<CAN2>,
        can_tx_queue: BinaryHeap<Box<CanFramePool>, U8, Min>,
        tx_count: usize,
        can_rx: Rx<CAN2>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut CAN_POOL_MEMORY: [u8; 256] = [0; 256];

        unsafe { cx.core.SCB.vtor.write(0x0800_0000) };

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .hclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);

        let mut can2 = Can::new(cx.device.CAN2, &mut rcc.apb1);

        // Select pins for CAN2.
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let can_rx_pin = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        let can_tx_pin = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);
        can2.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        can2.configure(|config| {
            // APB1 (PCLK1): 36MHz, Bit rate: 125kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            config.set_bit_timing(0x001c_0011);
        });

        // Filters are required to use the receiver part of CAN2.
        // Because the filter banks are part of CAN1 we first need to enable CAN1
        // and split the filters between the peripherals to use them for CAN2.
        let mut can1 = Can::new(cx.device.CAN1, &mut rcc.apb1);
        let (_, mut filters) = can1.split_filters(0).unwrap();

        // To share load between FIFOs use one filter for standard messages and another
        // for extended messages. Accept all IDs by setting the mask to 0. Explicitly
        // allow to receive remote frames.
        filters
            .add(&Filter::new_standard(0).with_mask(0).allow_remote())
            .unwrap();
        filters
            .add(&Filter::new_extended(0).with_mask(0).allow_remote())
            .unwrap();

        let mut can_rx = can2.take_rx(filters).unwrap();
        can_rx.enable_interrupts();

        let mut can_tx = can2.take_tx().unwrap();
        can_tx.enable_interrupt();

        let can_tx_queue = BinaryHeap::new();
        CanFramePool::grow(CAN_POOL_MEMORY);

        // Sync to the bus and start normal operation.
        can2.enable().ok();

        init::LateResources {
            can_tx,
            can_tx_queue,
            tx_count: 0,
            can_rx,
        }
    }

    #[idle(resources = [can_tx_queue, tx_count])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut tx_queue = cx.resources.can_tx_queue;

        // Enqueue some messages. Higher ID means lower priority.
        tx_queue.lock(|tx_queue| {
            tx_queue
                .push(alloc_frame(Id::new_standard(9), &[0, 1, 2, 4]))
                .unwrap();
            tx_queue
                .push(alloc_frame(Id::new_standard(9), &[0, 1, 2, 4]))
                .unwrap();
            tx_queue
                .push(alloc_frame(Id::new_standard(8), &[0, 1, 2, 4]))
                .unwrap();

            // Extended frames have lower priority than standard frames.
            tx_queue
                .push(alloc_frame(Id::new_extended(8), &[0, 1, 2, 4]))
                .unwrap();
            tx_queue
                .push(alloc_frame(Id::new_extended(7), &[0, 1, 2, 4]))
                .unwrap();

            tx_queue
                .push(alloc_frame(Id::new_standard(7), &[0, 1, 2, 4]))
                .unwrap();
        });

        // Manually trigger the tx interrupt to start the transmission.
        rtfm::pend(Interrupt::CAN2_TX);

        // Add some higher priority messages when 3 messages have been sent.
        loop {
            let tx_count = cx.resources.tx_count.lock(|tx_count| *tx_count);

            if tx_count >= 3 {
                tx_queue.lock(|tx_queue| {
                    tx_queue
                        .push(alloc_frame(Id::new_standard(3), &[0, 1, 2, 4]))
                        .unwrap();
                    tx_queue
                        .push(alloc_frame(Id::new_standard(2), &[0, 1, 2, 4]))
                        .unwrap();
                    tx_queue
                        .push(alloc_frame(Id::new_standard(1), &[0, 1, 2, 4]))
                        .unwrap();
                });
                break;
            }
        }

        // Expected bus traffic:
        //
        // 1. ID:      007 DATA: 00 01 02 04 <- proper reordering happens
        // 2. ID:      008 DATA: 00 01 02 04
        // 3. ID:      009 DATA: 00 01 02 04
        // 4. ID:      001 DATA: 00 01 02 04 <- higher priority messages incoming
        // 5. ID:      002 DATA: 00 01 02 04
        // 6. ID:      003 DATA: 00 01 02 04
        // 7. ID:      009 DATA: 00 01 02 04
        // 8. ID: 00000007 DATA: 00 01 02 04 <- extended frames have the lowest priority
        // 9. ID: 00000008 DATA: 00 01 02 04    and reach the bus last
        //
        // The output can look different if there are other nodes on bus the sending messages.

        loop {
            cortex_m::asm::wfi();
        }
    }

    // This ISR is triggered by each finished frame transmission.
    #[task(binds = CAN2_TX, resources = [can_tx, can_tx_queue, tx_count])]
    fn can2_tx(cx: can2_tx::Context) {
        let tx = cx.resources.can_tx;
        let tx_queue = cx.resources.can_tx_queue;

        tx.clear_interrupt_flags();

        // There is now a free mailbox. Send the next frame if there is still someting
        // in the queue.
        while let Some(frame) = tx_queue.peek() {
            match tx.transmit(&frame) {
                Ok(None) => {
                    tx_queue.pop();
                    *cx.resources.tx_count += 1;
                }
                Ok(pending_frame) => {
                    // A lower priority frame was replaced with our high priority frame.
                    // Put the low priority frame back in the transmit queue.
                    tx_queue.pop();
                    if let Some(frame) = pending_frame {
                        tx_queue
                            .push(CanFramePool::alloc().unwrap().init(frame))
                            .unwrap();
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                _ => unreachable!(),
            }
        }
    }

    #[task(binds = CAN2_RX0, resources = [can_rx, can_tx_queue])]
    fn can2_rx0(cx: can2_rx0::Context) {
        // Echo back received packages with correct priority ordering.
        loop {
            match cx.resources.can_rx.receive() {
                Ok(frame) => {
                    cx.resources
                        .can_tx_queue
                        .push(CanFramePool::alloc().unwrap().init(frame))
                        .ok();
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }

        // Start transmission of the newly queue frames.
        rtfm::pend(Interrupt::CAN2_TX);
    }

    #[task(binds = CAN2_RX1)]
    fn can2_rx1(_: can2_rx1::Context) {
        // Jump to the other interrupt handler which handles both RX fifos.
        rtfm::pend(Interrupt::CAN2_RX0);
    }
};
