#[cfg(feature = "afio")]
pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
#[cfg(feature = "dma")]
pub use crate::dma::DmaChannel as _stm32_hal_dma_DmaChannel;
#[cfg(feature = "dma")]
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
#[cfg(feature = "gpio")]
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
#[cfg(feature = "gpio")]
pub use crate::hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
#[cfg(feature = "gpio")]
pub use crate::hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;
#[cfg(any(feature = "gpioa", feature = "gpiob"))]
#[cfg(any(feature = "tim2", feature = "tim3", feature = "tim4"))]
#[cfg(feature = "pwm")]
pub use crate::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
//pub use crate::serial::ReadDma as _stm32_hal_serial_ReadDma;
//pub use crate::serial::WriteDma as _stm32_hal_serial_WriteDma;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;
