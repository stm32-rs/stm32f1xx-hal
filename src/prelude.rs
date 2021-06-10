pub use crate::adc::ChannelTimeSequence as _stm32_hal_adc_ChannelTimeSequence;
pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use crate::crc::CrcExt as _stm32_hal_crc_CrcExt;
pub use crate::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use crate::dma::WriteDma as _stm32_hal_dma_WriteDma;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;

pub use crate::hal::adc::OneShot as _embedded_hal_adc_OneShot;
pub use crate::hal::blocking::delay::DelayMs as _embedded_hal_blocking_delay_DelayMs;
pub use crate::hal::blocking::delay::DelayUs as _embedded_hal_blocking_delay_DelayUs;
pub use crate::hal::blocking::i2c::{
    Read as _embedded_hal_blocking_i2c_Read, Write as _embedded_hal_blocking_i2c_Write,
    WriteRead as _embedded_hal_blocking_i2c_WriteRead,
};
pub use crate::hal::blocking::rng::Read as _embedded_hal_blocking_rng_Read;
pub use crate::hal::blocking::serial::Write as _embedded_hal_blocking_serial_Write;
pub use crate::hal::blocking::spi::{
    Transfer as _embedded_hal_blocking_spi_Transfer, Write as _embedded_hal_blocking_spi_Write,
};
pub use crate::hal::serial::Read as _embedded_hal_serial_Read;
pub use crate::hal::serial::Write as _embedded_hal_serial_Write;
pub use crate::hal::spi::FullDuplex as _embedded_hal_spi_FullDuplex;
pub use crate::hal::timer::CountDown as _embedded_hal_timer_CountDown;
pub use crate::hal::watchdog::Watchdog as _embedded_hal_watchdog_Watchdog;
pub use crate::hal::watchdog::WatchdogDisable as _embedded_hal_watchdog_WatchdogDisable;
pub use crate::hal::watchdog::WatchdogEnable as _embedded_hal_watchdog_WatchdogEnable;
pub use crate::hal::Capture as _embedded_hal_Capture;
pub use crate::hal::Pwm as _embedded_hal_Pwm;
pub use crate::hal::PwmPin as _embedded_hal_PwmPin;
pub use crate::hal::Qei as _embedded_hal_Qei;
