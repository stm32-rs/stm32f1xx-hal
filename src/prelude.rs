pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use crate::dma::DmaChannel as _stm32_hal_dma_DmaChannel;
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use crate::hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;
#[cfg(not(feature = "stm32f100"))]
pub use crate::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
//pub use crate::serial::ReadDma as _stm32_hal_serial_ReadDma;
//pub use crate::serial::WriteDma as _stm32_hal_serial_WriteDma;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;
