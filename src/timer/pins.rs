use crate::pac;

pub trait CPin<REMAP, const C: u8> {}
pub struct Ch<const C: u8>;
pub const C1: u8 = 0;
pub const C2: u8 = 1;
pub const C3: u8 = 2;
pub const C4: u8 = 3;

pub(crate) mod sealed {
    pub trait Remap {
        type Periph;
        const REMAP: u8;

        fn remap(mapr: &mut crate::afio::MAPR);
    }
}

macro_rules! remap {
    ($($name:ident: ($TIMX:ty, $state:literal, $P1:ident, $P2:ident, $P3:ident, $P4:ident, { $remapex:expr }),)+) => {
        $(
            pub struct $name;
            impl sealed::Remap for $name {
                type Periph = $TIMX;
                const REMAP: u8 = $state;

                fn remap(mapr: &mut crate::afio::MAPR) {
                    mapr.modify_mapr($remapex);
                }
            }
            impl<MODE> CPin<$name, 0> for crate::gpio::$P1<MODE> {}
            impl<MODE> CPin<$name, 1> for crate::gpio::$P2<MODE> {}
            impl<MODE> CPin<$name, 2> for crate::gpio::$P3<MODE> {}
            impl<MODE> CPin<$name, 3> for crate::gpio::$P4<MODE> {}
        )+
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
remap!(
    Tim1NoRemap: (pac::TIM1, 0b00, PA8, PA9, PA10, PA11, {|_, w| unsafe { w.tim1_remap().bits(Self::REMAP)}}),
    //Tim1PartialRemap: (pac::TIM1, 0b01, PA8, PA9, PA10, PA11),
    Tim1FullRemap: (pac::TIM1, 0b11, PE9, PE11, PE13, PE14, {|_, w| unsafe { w.tim1_remap().bits(Self::REMAP)}}),
);

remap!(
    Tim2NoRemap: (pac::TIM2, 0b00, PA0, PA1, PA2, PA3, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2PartialRemap1: (pac::TIM2, 0b01, PA15, PB3, PA2, PA3, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2PartialRemap2: (pac::TIM2, 0b10, PA0, PA1, PB10, PB11, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),
    Tim2FullRemap: (pac::TIM2, 0b11, PA15, PB3, PB10, PB11, {|_, w| unsafe { w.tim2_remap().bits(Self::REMAP)}}),

    Tim3NoRemap: (pac::TIM3, 0b00, PA6, PA7, PB0, PB1, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
    Tim3PartialRemap: (pac::TIM3, 0b10, PB4, PB5, PB0, PB1, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
    Tim3FullRemap: (pac::TIM3, 0b11, PC6, PC7, PC8, PC9, {|_, w| unsafe { w.tim3_remap().bits(Self::REMAP)}}),
);

#[cfg(feature = "medium")]
remap!(
    Tim4NoRemap: (pac::TIM4, 0b00, PB6, PB7, PB8, PB9, {|_, w| w.tim4_remap().bit(Self::REMAP == 1)}),
    Tim4Remap: (pac::TIM4, 0b01, PD12, PD13, PD14, PD15, {|_, w| w.tim4_remap().bit(Self::REMAP == 1)}),
);
