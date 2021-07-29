use super::*;
use crate::bb;

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $bit:literal),)+) => {
        $(
            impl crate::Sealed for crate::pac::$PER {}

            impl RccBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::enr(rcc), $bit);
                    }
                }
                #[inline(always)]
                fn disable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        bb::clear(Self::Bus::enr(rcc), $bit);
                    }
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::rstr(rcc), $bit);
                        bb::clear(Self::Bus::rstr(rcc), $bit);
                    }
                }
            }
        )+
    }
}

macro_rules! ahb_bus {
    ($($PER:ident => ($bit:literal),)+) => {
        $(
            impl crate::Sealed for crate::pac::$PER {}

            impl RccBus for crate::pac::$PER {
                type Bus = AHB;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        bb::set(Self::Bus::enr(rcc), $bit);
                    }
                }
                #[inline(always)]
                fn disable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        bb::clear(Self::Bus::enr(rcc), $bit);
                    }
                }
            }
        )+
    }
}

#[cfg(feature = "stm32f103")]
bus! {
    ADC2 => (APB2, 10),
    CAN1 => (APB1, 25),
}
#[cfg(feature = "connectivity")]
bus! {
    ADC2 => (APB2, 10),
    CAN1 => (APB1, 25),
    CAN2 => (APB1, 26),
}
#[cfg(all(feature = "stm32f103", feature = "high"))]
bus! {
    ADC3 => (APB2, 15),
    DAC => (APB1, 29),
    UART4 => (APB1, 19),
    UART5 => (APB1, 20),
}
bus! {
    ADC1 => (APB2, 9),
    AFIO => (APB2, 0),
    BKP => (APB1, 27),
    GPIOA => (APB2, 2),
    GPIOB => (APB2, 3),
    GPIOC => (APB2, 4),
    GPIOD => (APB2, 5),
    GPIOE => (APB2, 6),
    I2C1 => (APB1, 21),
    I2C2 => (APB1, 22),
    PWR => (APB1, 28),
    SPI1 => (APB2, 12),
    SPI2 => (APB1, 14),
    USART1 => (APB2, 14),
    USART2 => (APB1, 17),
    USART3 => (APB1, 18),
    WWDG => (APB1, 11),
}

#[cfg(any(feature = "xl", feature = "high"))]
bus! {
    GPIOF => (APB2, 7),
    GPIOG => (APB2, 8),
}

#[cfg(any(feature = "high", feature = "connectivity"))]
bus! {
    SPI3 => (APB1, 15),
}

ahb_bus! {
    CRC => (6),
    DMA1 => (0),
    DMA2 => (1),
}

#[cfg(feature = "high")]
ahb_bus! {
    FSMC => (8),
}

bus! {
    TIM2 => (APB1, 0),
    TIM3 => (APB1, 1),
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
bus! {
    TIM1 => (APB2, 11),
}

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
bus! {
    TIM6 => (APB1, 4),
}

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
bus! {
    TIM7 => (APB1, 5),
}

#[cfg(feature = "stm32f100")]
bus! {
    TIM15 => (APB2, 16),
    TIM16 => (APB2, 17),
    TIM17 => (APB2, 18),
}

#[cfg(feature = "medium")]
bus! {
    TIM4 => (APB1, 2),
}

#[cfg(any(feature = "high", feature = "connectivity"))]
bus! {
    TIM5 => (APB1, 3),
}

#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high",)))]
bus! {
    TIM12 => (APB1, 6),
    TIM13 => (APB1, 7),
    TIM14 => (APB1, 8),
}

#[cfg(all(feature = "stm32f103", feature = "high",))]
bus! {
    TIM8 => (APB2, 13),
}

#[cfg(feature = "xl")]
bus! {
    TIM9 => (APB2, 19),
    TIM10 => (APB2, 20),
    TIM11 => (APB2, 21),
}

#[cfg(any(feature = "stm32f102", feature = "stm32f103"))]
bus! {
    USB => (APB1, 23),
}
