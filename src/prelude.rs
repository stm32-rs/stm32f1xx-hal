pub use crate::adc::ChannelTimeSequence as _stm32_hal_adc_ChannelTimeSequence;
pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
#[cfg(feature = "has-can")]
pub use crate::can::CanExt as _;
pub use crate::crc::CrcExt as _stm32_hal_crc_CrcExt;
pub use crate::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use crate::dma::ReadWriteDma as _stm32_hal_dma_ReadWriteDma;
pub use crate::dma::WriteDma as _stm32_hal_dma_WriteDma;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::hal_02::adc::OneShot as _embedded_hal_adc_OneShot;
pub use crate::hal_02::prelude::*;
pub use crate::i2c::I2cExt as _;
pub use crate::qei::QeiExt as _;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use crate::serial::SerialExt as _;
pub use crate::spi::SpiExt as _;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;
pub use crate::timer::pwm_input::PwmInputExt as _;
#[cfg(feature = "rtic")]
pub use crate::timer::MonoTimerExt as _stm32f4xx_hal_timer_MonoTimerExt;
pub use crate::timer::PwmExt as _stm32f4xx_hal_timer_PwmExt;
pub use crate::timer::SysTimerExt as _stm32f4xx_hal_timer_SysCounterExt;
pub use crate::timer::TimerExt as _stm32f4xx_hal_timer_TimerExt;
pub use fugit::ExtU32 as _fugit_ExtU32;
pub use fugit::RateExtU32 as _fugit_RateExtU32;
