use super::*;
use crate::bb;

macro_rules! bus_enable {
    ($PER:ident => $bit:literal) => {
        impl Enable for crate::pac::$PER {
            #[inline(always)]
            fn enable(rcc: &mut RCC) {
                unsafe {
                    bb::set(Self::Bus::enr(rcc), $bit);
                }
                // Stall the pipeline to work around erratum 2.1.13 (DM00037591)
                cortex_m::asm::dsb();
            }
            #[inline(always)]
            fn disable(rcc: &mut RCC) {
                unsafe {
                    bb::clear(Self::Bus::enr(rcc), $bit);
                }
            }
            #[inline(always)]
            fn is_enabled() -> bool {
                let rcc = RCC::ptr();
                (Self::Bus::enr(unsafe { &*rcc }).read().bits() >> $bit) & 0x1 != 0
            }
        }
    };
}

macro_rules! bus_reset {
    ($PER:ident => $bit:literal) => {
        impl Reset for crate::pac::$PER {
            #[inline(always)]
            fn reset(rcc: &mut RCC) {
                let rstr = Self::Bus::rstr(rcc);
                unsafe {
                    bb::set(rstr, $bit);
                    bb::clear(rstr, $bit);
                }
            }
        }
    };
}

macro_rules! bus {
    ($($PER:ident => ($busX:ty, $bit:literal),)+) => {
        $(
            impl crate::rcc::Instance for crate::pac::$PER {}
            impl RccBus for crate::pac::$PER {
                type Bus = $busX;
            }
            bus_enable!($PER => $bit);
            bus_reset!($PER => $bit);
        )+
    }
}

macro_rules! dbgstop {
    ($($PER:ident => ($bit:literal),)+) => {
        $(
            impl StopInDebug for crate::pac::$PER {
                #[inline(always)]
                fn stop_in_debug(&mut self, dbg: &mut DBGMCU, state: bool) {
                    unsafe {
                        bb::write(dbg.cr(), $bit, state);
                    }
                }
            }
        )+
    }
}

#[cfg(feature = "stm32f103")]
bus! {
    ADC2 => (APB2, 10),
    CAN => (APB1, 25),
}
#[cfg(feature = "stm32f103")]
dbgstop! {
    CAN => (14), // dbg_can1_stop
}

#[cfg(feature = "connectivity")]
bus! {
    ADC2 => (APB2, 10),
    CAN1 => (APB1, 25),
    CAN2 => (APB1, 26),
}
#[cfg(feature = "connectivity")]
dbgstop! {
    CAN1 => (14), // dbg_can1_stop
    CAN2 => (21), // dbg_can2_stop
}

#[cfg(feature = "has-dac")]
bus! {
    DAC => (APB1, 29),
}
#[cfg(any(all(feature = "stm32f103", feature = "high"), feature = "connectivity"))]
bus! {
    ADC3 => (APB2, 15),
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
dbgstop! {
    IWDG => (8), // dbg_iwdg_stop
    WWDG => (9), // dbg_wwdg_stop
    I2C1 => (15), // dbg_i2c1_smbus_timeout
    I2C2 => (16), // dbg_i2c2_smbus_timeout
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

impl RccBus for crate::pac::CRC {
    type Bus = AHB;
}
bus_enable! { CRC => 6 }
impl RccBus for crate::pac::DMA1 {
    type Bus = AHB;
}
bus_enable! { DMA1 => 0 }
impl RccBus for crate::pac::DMA2 {
    type Bus = AHB;
}
bus_enable! { DMA2 => 1 }

#[cfg(feature = "high")]
impl RccBus for crate::pac::FSMC {
    type Bus = AHB;
}
#[cfg(feature = "high")]
bus_enable! { FSMC => 8 }

bus! {
    TIM2 => (APB1, 0),
    TIM3 => (APB1, 1),
}
dbgstop! {
    TIM2 => (11), // dbg_tim2_stop
    TIM3 => (12), // dbg_tim3_stop
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
bus! {
    TIM1 => (APB2, 11),
}
#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
dbgstop! {
    TIM1 => (10), // dbg_tim1_stop
}

#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
bus! {
    TIM6 => (APB1, 4),
}
#[cfg(any(feature = "stm32f100", feature = "high", feature = "connectivity"))]
dbgstop! {
    TIM6 => (19), // dbg_tim6_stop
}

#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
bus! {
    TIM7 => (APB1, 5),
}
#[cfg(any(
    all(feature = "high", any(feature = "stm32f101", feature = "stm32f103")),
    any(feature = "stm32f100", feature = "connectivity")
))]
dbgstop! {
    TIM7 => (20), // dbg_tim7_stop
}

#[cfg(feature = "stm32f100")]
bus! {
    TIM15 => (APB2, 16),
    TIM16 => (APB2, 17),
    TIM17 => (APB2, 18),
}
#[cfg(feature = "stm32f100")]
dbgstop! {
    TIM15 => (22), // dbg_tim15_stop
    TIM16 => (23), // dbg_tim16_stop
    TIM17 => (24), // dbg_tim17_stop
}

#[cfg(feature = "medium")]
bus! {
    TIM4 => (APB1, 2),
}
#[cfg(feature = "medium")]
dbgstop! {
    TIM4 => (13), // dbg_tim4_stop
}

#[cfg(any(feature = "high", feature = "connectivity"))]
bus! {
    TIM5 => (APB1, 3),
}
#[cfg(any(feature = "high", feature = "connectivity"))]
dbgstop! {
    TIM5 => (18), // dbg_tim5_stop
}

#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
bus! {
    TIM12 => (APB1, 6),
    TIM13 => (APB1, 7),
    TIM14 => (APB1, 8),
}
#[cfg(any(feature = "xl", all(feature = "stm32f100", feature = "high")))]
dbgstop! {
    TIM12 => (25), // dbg_tim12_stop
    TIM13 => (26), // dbg_tim13_stop
    TIM14 => (27), // dbg_tim14_stop
}

#[cfg(all(feature = "stm32f103", feature = "high"))]
bus! {
    TIM8 => (APB2, 13),
}
#[cfg(all(feature = "stm32f103", feature = "high"))]
dbgstop! {
    TIM8 => (17), // dbg_tim8_stop
}

#[cfg(feature = "xl")]
bus! {
    TIM9 => (APB2, 19),
    TIM10 => (APB2, 20),
    TIM11 => (APB2, 21),
}
#[cfg(feature = "xl")]
dbgstop! {
    TIM9 => (28), // dbg_tim9_stop
    TIM10 => (29), // dbg_tim10_stop
    TIM11 => (30), // dbg_tim11_stop
}

#[cfg(feature = "stm32f103")] // feature = "stm32f102"
bus! {
    USB => (APB1, 23),
}
