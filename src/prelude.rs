pub use afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use dma::DmaChannel as _stm32_hal_dma_DmaChannel;
pub use dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use hal::prelude::*;
#[cfg(not(feature = "stm32f100"))]
pub use pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use rcc::RccExt as _stm32_hal_rcc_RccExt;
//pub use serial::ReadDma as _stm32_hal_serial_ReadDma;
//pub use serial::WriteDma as _stm32_hal_serial_WriteDma;
pub use time::U32Ext as _stm32_hal_time_U32Ext;
