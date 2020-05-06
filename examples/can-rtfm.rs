//! Interrupt driven CAN transmitter with RTFM.
//!
//! CAN frames are allocated from a static memory pool and stored in a priority
//! queue (min heap) for transmisison. To start transmission the CAN TX
//! interrupt has to be triggered manually once. With each successful
//! transmission the interrupt is reentered and more data is fetched from the
//! queue.
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
    can::{Can, Frame, Tx},
    pac::{Interrupt, CAN2},
    prelude::*,
};

pool!(CanFramePool: Frame);

fn alloc_frame(id: u32, data: &[u8]) -> Box<CanFramePool, Init> {
    let frame_box = CanFramePool::alloc().unwrap();
    frame_box.init(Frame::new_standard(id, data))
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        can_tx: Tx<CAN2>,
        can_tx_queue: BinaryHeap<Box<CanFramePool>, U8, Min>,
        tx_count: usize,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut CAN_POOL_MEMORY: [u8; 256] = [0; 256];

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let _clocks = rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        let canrx = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        let cantx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let mut can = Can::new(
            cx.device.CAN2,
            (cantx, canrx),
            &mut afio.mapr,
            &mut rcc.apb1,
        );
        // APB1 (PCLK1): 8MHz, Bit rate: 125kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        unsafe { can.set_bit_timing(0x001c_0003) };
        can.enable_tx_interrupt();
        can.enable().ok();

        let can_tx = can.take_tx().unwrap();
        let can_tx_queue = BinaryHeap::new();
        CanFramePool::grow(CAN_POOL_MEMORY);

        init::LateResources {
            can_tx,
            can_tx_queue,
            tx_count: 0,
        }
    }

    #[idle(resources = [can_tx_queue, tx_count])]
    fn idle(mut cx: idle::Context) -> ! {
        static mut HIGH_PRIORITY_MESSAGES_PENDING: bool = true;

        // Enqueue some messages. Higher ID means lower priority.
        cx.resources.can_tx_queue.lock(|tx_queue| {
            tx_queue.push(alloc_frame(9, &[0, 1, 2, 4])).unwrap();
            tx_queue.push(alloc_frame(9, &[0, 1, 2, 4])).unwrap();
            tx_queue.push(alloc_frame(8, &[0, 1, 2, 4])).unwrap();
            tx_queue.push(alloc_frame(8, &[0, 1, 2, 4])).unwrap();
            tx_queue.push(alloc_frame(7, &[0, 1, 2, 4])).unwrap();
            tx_queue.push(alloc_frame(7, &[0, 1, 2, 4])).unwrap();
        });

        // Manually trigger the tx interrupt to start the transmission.
        rtfm::pend(Interrupt::CAN2_TX);

        loop {
            let tx_count = cx.resources.tx_count.lock(|tx_count| *tx_count);

            // Add some higher priority messages when 3 messages have been sent.
            if *HIGH_PRIORITY_MESSAGES_PENDING && tx_count >= 3 {
                *HIGH_PRIORITY_MESSAGES_PENDING = false;

                cx.resources.can_tx_queue.lock(|tx_queue| {
                    tx_queue.push(alloc_frame(3, &[0, 1, 2, 4])).unwrap();
                    tx_queue.push(alloc_frame(2, &[0, 1, 2, 4])).unwrap();
                    tx_queue.push(alloc_frame(1, &[0, 1, 2, 4])).unwrap();
                });
            }
            cortex_m::asm::wfi();
        }

        // Expected bus traffic:
        //
        // 1. ID: 7 DATA: 00 01 02 04
        // 2. ID: 7 DATA: 00 01 02 04
        // 3. ID: 8 DATA: 00 01 02 04
        // 4. ID: 1 DATA: 00 01 02 04
        // 5. ID: 2 DATA: 00 01 02 04
        // 6. ID: 3 DATA: 00 01 02 04
        // 7. ID: 8 DATA: 00 01 02 04
        // 8. ID: 9 DATA: 00 01 02 04
        // 9. ID: 9 DATA: 00 01 02 04
        //
        // The output can look different if there are other nodes on the bus that send
        // higher priority messages at the same time.
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
};
