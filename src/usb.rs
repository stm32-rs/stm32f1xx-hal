//! USB peripheral
//!
//! Requires the `stm32-usbd` feature.
//! See https://github.com/stm32-rs/stm32f1xx-hal/tree/master/examples
//! for usage examples.

use crate::pac::USB;
use crate::rcc::{Enable, Reset};
use stm32_usbd::UsbPeripheral;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::{Floating, Input};
pub use stm32_usbd::UsbBus;

pub struct Peripheral {
    pub usb: USB,
    pub pin_dm: PA11<Input<Floating>>,
    pub pin_dp: PA12<Input<Floating>>,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();
    const DP_PULL_UP_FEATURE: bool = false;
    const EP_MEMORY: *const () = 0x4000_6000 as _;
    const EP_MEMORY_SIZE: usize = 512;
    const EP_MEMORY_ACCESS_2X16: bool = false;

    fn enable() {
        // TODO: use self.usb, after adding the &self parameter
        let usb = unsafe { crate::pac::Peripherals::steal().USB };
        // Enable USB peripheral
        usb.enable();
        // Reset USB peripheral
        usb.reset();
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        cortex_m::asm::delay(72);
    }
}

pub type UsbBusType = UsbBus<Peripheral>;
