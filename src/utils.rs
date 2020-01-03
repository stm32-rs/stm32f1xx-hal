
pub trait RegisterBlock {
    type RB;
    fn rb() -> *const Self::RB;
}

macro_rules! register_block {
    ($($PER:ident => $per:ident,)*) => {
        $(
            impl RegisterBlock for crate::pac::$PER {
                type RB = crate::pac::$per::RegisterBlock;
                #[inline(always)]
                fn rb() -> *const Self::RB {
                    crate::pac::$PER::ptr()
                }
            }
        )+
    }
}

register_block!(
    I2C1 => i2c1,
    I2C2 => i2c1,

    SPI1 => spi1,
    SPI2 => spi1,
    SPI3 => spi1,

    USART1 => usart1,
    USART2 => usart1,
    USART3 => usart1,
);
