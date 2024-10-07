use crate::afio::MAPR;
use crate::gpio::{self, Alternate, Cr};
use crate::pac;

pub const C1: u8 = 0;
pub const C2: u8 = 1;
pub const C3: u8 = 2;
pub const C4: u8 = 3;

pub struct InPins<CH1, CH2> {
    pub c1: CH1,
    pub c2: CH2,
}

impl<CH1, CH2> From<(CH1, CH2)> for InPins<CH1, CH2> {
    fn from(value: (CH1, CH2)) -> Self {
        Self {
            c1: value.0,
            c2: value.1,
        }
    }
}

pub struct Pins4<CH1, CH2, CH3, CH4> {
    pub c1: CH1,
    pub c2: CH2,
    pub c3: CH3,
    pub c4: CH4,
}

impl<CH1, CH2, CH3, CH4> From<(CH1, CH2, CH3, CH4)> for Pins4<CH1, CH2, CH3, CH4> {
    fn from(value: (CH1, CH2, CH3, CH4)) -> Self {
        Self {
            c1: value.0,
            c2: value.1,
            c3: value.2,
            c4: value.3,
        }
    }
}

pub struct Pins3<CH1, CH2, CH3> {
    pub c1: CH1,
    pub c2: CH2,
    pub c3: CH3,
}

impl<CH1, CH2, CH3> From<(CH1, CH2, CH3)> for Pins3<CH1, CH2, CH3> {
    fn from(value: (CH1, CH2, CH3)) -> Self {
        Self {
            c1: value.0,
            c2: value.1,
            c3: value.2,
        }
    }
}

pub struct Pins2<CH1, CH2> {
    pub c1: CH1,
    pub c2: CH2,
}

impl<CH1, CH2> From<(CH1, CH2)> for Pins2<CH1, CH2> {
    fn from(value: (CH1, CH2)) -> Self {
        Self {
            c1: value.0,
            c2: value.1,
        }
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
pub mod tim1 {
    use super::*;

    remap4! {
        pac::TIM1: [
            PA8, PA9, PA10, PA11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            PE9, PE11, PE13, PE14 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM1: [
            PA8, PA9 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            PE9, PE11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap_input! {
        pac::TIM1: [
            PA8, PA9 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            PE9, PE11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap34! {
        pac::TIM1: [
            PA10, PA11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            PE13, PE14 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }

    out_enums! {
        Ch1: PA8, PE9;
        Ch2: PA9, PE11;
        Ch3: PA10, PE13;
        Ch4: PA11, PE14;
    }
    in_enums! {
        InCh1: PA8, PE9;
        InCh2: PA9, PE11;
    }
}

pub mod tim2 {
    use super::*;

    remap4! {
        pac::TIM2: [
            PA0, PA1, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            PA15, PB3, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
            PA0, PA1, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b10)} };
            PA15, PB3, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM2: [
            PA0, PA1 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            PA15, PB3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
        ]
    }
    remap_input! {
        pac::TIM2: [
            PA0, PA1 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            PA15, PB3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
        ]
    }
    remap34! {
        pac::TIM2: [
            PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b10)} };
        ]
    }

    out_enums! {
        Ch1: PA0, PA15;
        Ch2: PA1, PB3;
        Ch3: PA2, PB10;
        Ch4: PA3, PB11;
    }
    in_enums! {
        InCh1: PA0, PA15;
        InCh2: PA1, PB3;
    }
}

pub mod tim3 {
    use super::*;

    remap4! {
        pac::TIM3: [
            PA6, PA7, PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            PB4, PB5, PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            PC6, PC7, PC8, PC9 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM3: [
            PA6, PA7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            PB4, PB5 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            PC6, PC7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap_input! {
        pac::TIM3: [
            PA6, PA7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            PB4, PB5 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            PC6, PC7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap34! {
        pac::TIM3: [
            PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            PC8, PC9 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }

    out_enums! {
        Ch1: PA6, PB4, PC6;
        Ch2: PA7, PB5, PC7;
        Ch3: PB0, PC8;
        Ch4: PB1, PC9;
    }
    in_enums! {
        InCh1: PA6, PB4, PC6;
        InCh2: PA7, PB5, PC7;
    }
}

#[cfg(feature = "medium")]
pub mod tim4 {
    use super::*;

    remap4! {
        pac::TIM4: [
            PB6, PB7, PB8, PB9 => MAPR { |_, w| w.tim4_remap().bit(false) };
            PD12, PD13, PD14, PD15 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap12! {
        pac::TIM4: [
            PB6, PB7 => MAPR { |_, w| w.tim4_remap().bit(false) };
            PD12, PD13 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap_input! {
        pac::TIM4: [
            PB6, PB7 => MAPR { |_, w| w.tim4_remap().bit(false) };
            PD12, PD13 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap34! {
        pac::TIM4: [
            PB8, PB9 => MAPR { |_, w| w.tim4_remap().bit(false) };
            PD14, PD15 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }

    out_enums! {
        Ch1: PB6, PD12;
        Ch2: PB7, PD13;
        Ch3: PB8, PD14;
        Ch4: PB9, PD15;
    }
    in_enums! {
        InCh1: PB6, PD12;
        InCh2: PB7, PD13;
    }
}

macro_rules! out_enums {
    ($($Ch: ident: $($P:ident),+;)+) => {
        $(
            pub enum $Ch {
                $(
                    $P(gpio::$P<Alternate>),
                )+
            }
        )+
    };
}
use out_enums;
macro_rules! in_enums {
    ($($Ch: ident: $($P:ident),+;)+) => {
        $(
            pub enum $Ch {
                $(
                    $P(gpio::$P),
                )+
            }
        )+
    };
}
use in_enums;

macro_rules! remap4 {
    ($TIM:ty: [
        $($P1:ident, $P2:ident, $P3:ident, $P4:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub type Channels1234 = Pins4<Ch1, Ch2, Ch3, Ch4>;
        remap_combo! {
            $TIM; C1, C2, C3, C4: [
                $($P1, $P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels123 = Pins3<Ch1, Ch2, Ch3>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2), (C3, Ch3): [
                $($P1, $P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels124 = Pins3<Ch1, Ch2, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2), (C4, Ch4): [
                $($P1, $P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels134 = Pins3<Ch1, Ch3, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C3, Ch3), (C4, Ch4): [
                $($P1, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels234 = Pins3<Ch2, Ch3, Ch4>;
        remap_combo! {
            $TIM; (C2, Ch2), (C3, Ch3), (C4, Ch4): [
                $($P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels13 = Pins2<Ch1, Ch3>;
        remap_combo! {
            $TIM; (C1, Ch1), (C3, Ch3): [
                $($P1, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels14 = Pins2<Ch1, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C4, Ch4): [
                $($P1, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels23 = Pins2<Ch2, Ch3>;
        remap_combo! {
            $TIM; (C2, Ch2), (C3, Ch3): [
                $($P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        pub type Channels24 = Pins2<Ch2, Ch4>;
        remap_combo! {
            $TIM; (C2, Ch2), (C4, Ch4): [
                $($P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap4;

macro_rules! remap12 {
    ($TIM:ty: [
        $($P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub type Channels12 = Pins2<Ch1, Ch2>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2): [
                $($P1, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM; (C1, Ch1): [
                $($P1 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM; (C2, Ch2): [
                $($P2 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap12;

macro_rules! remap34 {
    ($TIM:ty: [
        $($P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub type Channels34 = Pins2<Ch3, Ch4>;
        remap_combo! {
            $TIM; (C3, Ch3), (C4, Ch4): [
                $($P1, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM; (C3, Ch3): [
                $($P1 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM; (C4, Ch4): [
                $($P2 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap34;

macro_rules! remap_combo {
    ($TIM:ty; $C1:ident, $C2:ident, $C3:ident, $C4:ident: [
        $($P0:ident, $P1:ident, $P2:ident, $P3:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        impl Pins<$TIM> for Pins4<Ch1, Ch2, Ch3, Ch4> {
            const $C1: bool = true;
            const $C2: bool = true;
            const $C3: bool = true;
            const $C4: bool = true;
            type Channels = (PwmChannel<$TIM, $C1>, PwmChannel<$TIM, $C2>, PwmChannel<$TIM, $C3>, PwmChannel<$TIM, $C4>);
            fn split() -> Self::Channels {
                (PwmChannel::new(), PwmChannel::new(), PwmChannel::new(), PwmChannel::new())
            }
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)> for Pins4<Ch1, Ch2, Ch3, Ch4> {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.4.modify_mapr($remapex);)?
                    Self { c1: Ch1::$P0(p.0), c2: Ch2::$P1(p.1), c3: Ch3::$P2(p.2), c4: Ch4::$P3(p.3) }
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)> for Pins4<Ch1, Ch2, Ch3, Ch4> {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    let p2 = p.2.into_mode(&mut Cr);
                    let p3 = p.3.into_mode(&mut Cr);
                    $(p.4.modify_mapr($remapex);)?
                    Self { c1: Ch1::$P0(p0), c2: Ch2::$P1(p1), c3: Ch3::$P2(p2), c4: Ch4::$P3(p3) }
                }
            }
        )+
    };

    ($TIM:ty; ($C1:ident, $Ch1:ident), ($C2:ident, $Ch2:ident), ($C3:ident, $Ch3:ident): [
        $($P0:ident, $P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        impl Pins<$TIM> for Pins3<$Ch1, $Ch2, $Ch3> {
            const $C1: bool = true;
            const $C2: bool = true;
            const $C3: bool = true;
            type Channels = (PwmChannel<$TIM, $C1>, PwmChannel<$TIM, $C2>, PwmChannel<$TIM, $C3>);
            fn split() -> Self::Channels {
                (PwmChannel::new(), PwmChannel::new(), PwmChannel::new())
            }
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)> for Pins3<$Ch1, $Ch2, $Ch3> {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.3.modify_mapr($remapex);)?
                    Self { c1: $Ch1::$P0(p.0), c2: $Ch2::$P1(p.1), c3: $Ch3::$P2(p.2) }
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)> for Pins3<$Ch1, $Ch2, $Ch3> {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    let p2 = p.2.into_mode(&mut Cr);
                    $(p.3.modify_mapr($remapex);)?
                    Self { c1: $Ch1::$P0(p0), c2: $Ch2::$P1(p1), c3: $Ch3::$P2(p2) }
                }
            }
        )+
    };

    ($TIM:ty; ($C1:ident, $Ch1:ident), ($C2:ident, $Ch2:ident): [
        $($P0:ident, $P1:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        impl Pins<$TIM> for Pins2<$Ch1, $Ch2> {
            const $C1: bool = true;
            const $C2: bool = true;
            type Channels = (PwmChannel<$TIM, $C1>, PwmChannel<$TIM, $C2>);
            fn split() -> Self::Channels {
                (PwmChannel::new(), PwmChannel::new())
            }
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)> for Pins2<$Ch1, $Ch2> {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.2.modify_mapr($remapex);)?
                    Self { c1: $Ch1::$P0(p.0), c2: $Ch2::$P1(p.1) }
                }
            }

            impl From<(gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)> for Pins2<$Ch1, $Ch2> {
                fn from(p: (gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    $(p.2.modify_mapr($remapex);)?
                    Self { c1: $Ch1::$P0(p0), c2: $Ch2::$P1(p1) }
                }
            }
        )+
    };

    ($TIM:ty; ($C:ident, $Ch:ident): [
        $($P0:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        impl Pins<$TIM> for $Ch {
            const $C: bool = true;
            type Channels = PwmChannel<$TIM, $C>;
            fn split() -> Self::Channels {
                PwmChannel::new()
            }
        }

        $(
            impl From<(gpio::$P0<Alternate> $(, &mut $MAPR)?)> for $Ch {
                fn from(p: (gpio::$P0<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.1.modify_mapr($remapex);)?
                    $Ch::$P0(p.0)
                }
            }

            impl From<(gpio::$P0 $(, &mut $MAPR)?)> for $Ch {
                fn from(p: (gpio::$P0 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    $(p.1.modify_mapr($remapex);)?
                    $Ch::$P0(p0)
                }
            }
        )+
    };
}
use remap_combo;

use super::{Channel, PwmChannel};

pub trait Pins<TIM> {
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = false;
    const C4: bool = false;
    type Channels;

    fn check_used(c: Channel) -> Channel {
        if (c == Channel::C1 && Self::C1)
            || (c == Channel::C2 && Self::C2)
            || (c == Channel::C3 && Self::C3)
            || (c == Channel::C4 && Self::C4)
        {
            c
        } else {
            panic!("Unused channel")
        }
    }

    fn split() -> Self::Channels;
}

macro_rules! remap_input {
    ($TIM:ty: [
        $($P0:ident, $P1:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        impl InputPins for $TIM {
            type InCh1 = InCh1;
            type InCh2 = InCh2;
        }

        $(
            impl From<(gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)> for InPins<InCh1, InCh2> {
                fn from(p: (gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    $(p.2.modify_mapr($remapex);)?
                    Self { c1: InCh1::$P0(p.0), c2: InCh2::$P1(p.1) }
                }
            }
        )+
    };
}
use remap_input;

pub trait InputPins {
    type InCh1;
    type InCh2;
}
