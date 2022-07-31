//! Interrupt driven CAN transmitter with RTIC.
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

use panic_halt as _;

use bxcan::Frame;
use core::cmp::Ordering;

use heapless::binary_heap::{BinaryHeap, Max};

use stm32f1xx_hal::pac::Interrupt;

#[derive(Debug)]
pub struct PriorityFrame(Frame);

/// Ordering is based on the Identifier and frame type (data vs. remote) and can be used to sort
/// frames by priority.
impl Ord for PriorityFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for PriorityFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for PriorityFrame {}

fn enqueue_frame(queue: &mut BinaryHeap<PriorityFrame, Max, 16>, frame: Frame) {
    queue.push(PriorityFrame(frame)).unwrap();
    rtic::pend(Interrupt::USB_HP_CAN_TX);
}

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use super::{enqueue_frame, PriorityFrame};
    use bxcan::{filter::Mask32, ExtendedId, Fifo, Frame, Interrupts, Rx0, StandardId, Tx};
    use heapless::binary_heap::{BinaryHeap, Max};
    use stm32f1xx_hal::{can::Can, pac::CAN1, prelude::*};

    #[local]
    struct Local {
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx0<Can<CAN1>>,
    }
    #[shared]
    struct Shared {
        can_tx_queue: BinaryHeap<PriorityFrame, Max, 16>,
        tx_count: usize,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(64.MHz())
            .hclk(64.MHz())
            .pclk1(16.MHz())
            .pclk2(64.MHz())
            .freeze(&mut flash.acr);

        #[cfg(not(feature = "connectivity"))]
        let can = Can::new(cx.device.CAN1, cx.device.USB);

        #[cfg(feature = "connectivity")]
        let can = Can::new(cx.device.CAN1);

        // Select pins for CAN1.
        let mut gpioa = cx.device.GPIOA.split();
        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        let mut afio = cx.device.AFIO.constrain();
        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001c_0000)
            .leave_disabled();

        can.modify_filters()
            .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

        // Sync to the bus and start normal operation.
        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        nb::block!(can.enable_non_blocking()).unwrap();

        let (can_tx, can_rx, _) = can.split();

        let can_tx_queue = BinaryHeap::new();

        (
            Shared {
                can_tx_queue,
                tx_count: 0,
            },
            Local { can_tx, can_rx },
            init::Monotonics(),
        )
    }

    #[idle(shared = [can_tx_queue, tx_count])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut tx_queue = cx.shared.can_tx_queue;

        // Enqueue some messages. Higher ID means lower priority.
        tx_queue.lock(|mut tx_queue| {
            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(StandardId::new(9).unwrap(), []),
            );
            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(ExtendedId::new(9).unwrap(), []),
            );

            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(StandardId::new(8).unwrap(), []),
            );
            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(ExtendedId::new(8).unwrap(), []),
            );

            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(StandardId::new(0x7FF).unwrap(), []),
            );
            enqueue_frame(
                &mut tx_queue,
                Frame::new_data(ExtendedId::new(0x1FFF_FFFF).unwrap(), []),
            );
        });

        // Add some higher priority messages when 3 messages have been sent.
        loop {
            let tx_count = cx.shared.tx_count.lock(|tx_count| *tx_count);

            if tx_count >= 3 {
                tx_queue.lock(|mut tx_queue| {
                    enqueue_frame(
                        &mut tx_queue,
                        Frame::new_data(StandardId::new(3).unwrap(), []),
                    );
                    enqueue_frame(
                        &mut tx_queue,
                        Frame::new_data(StandardId::new(2).unwrap(), []),
                    );
                    enqueue_frame(
                        &mut tx_queue,
                        Frame::new_data(StandardId::new(1).unwrap(), []),
                    );
                });
                break;
            }
        }

        // Expected bus traffic:
        //
        // 1. ID: 0x00000008  <- proper reordering happens
        // 2. ID: 0x00000009
        // 3. ID: 0x008
        // 4. ID: 0x001       <- higher priority messages injected correctly
        // 5. ID: 0x002
        // 6. ID: 0x003
        // 7. ID: 0x009
        // 8. ID: 0x7FF
        // 9. ID: 0x1FFFFFFF
        //
        // The output can look different if there are other nodes on bus the sending messages.

        loop {
            cortex_m::asm::nop();
        }
    }

    // This ISR is triggered by each finished frame transmission.
    #[task(binds = USB_HP_CAN_TX, local = [can_tx], shared = [can_tx_queue, tx_count])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.local.can_tx;
        let mut tx_queue = cx.shared.can_tx_queue;
        let mut tx_count = cx.shared.tx_count;

        tx.clear_interrupt_flags();

        // There is now a free mailbox. Try to transmit pending frames until either
        // the queue is empty or transmission would block the execution of this ISR.
        (&mut tx_queue, &mut tx_count).lock(|tx_queue, tx_count| {
            while let Some(frame) = tx_queue.peek() {
                match tx.transmit(&frame.0) {
                    Ok(status) => match status.dequeued_frame() {
                        None => {
                            // Frame was successfully placed into a transmit buffer.
                            tx_queue.pop();
                            *tx_count += 1;
                        }
                        Some(pending_frame) => {
                            // A lower priority frame was replaced with our high priority frame.
                            // Put the low priority frame back in the transmit queue.
                            tx_queue.pop();
                            enqueue_frame(tx_queue, pending_frame.clone());
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    #[task(binds = USB_LP_CAN_RX0, local = [can_rx], shared = [can_tx_queue])]
    fn can_rx0(mut cx: can_rx0::Context) {
        // Echo back received packages with correct priority ordering.
        loop {
            match cx.local.can_rx.receive() {
                Ok(frame) => {
                    cx.shared.can_tx_queue.lock(|can_tx_queue| {
                        enqueue_frame(can_tx_queue, frame);
                    });
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }
}
