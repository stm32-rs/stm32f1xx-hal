use crate::afio::{Remap, MAPR};
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

pinsx!(Pins2: (CH1, c1, C1), (CH2, c2, C2));
pinsx!(Pins3: (CH1, c1, C1), (CH2, c2, C2), (CH3, c3, C3));
pinsx!(Pins4: (CH1, c1, C1), (CH2, c2, C2), (CH3, c3, C3), (CH4, c4, C4));

macro_rules! pinsx {
    ($name:ident: $(($CH:ident, $c:ident, $C:ident)),+) => {
        pub struct $name<$($CH),+> {
            $(
                pub $c: $CH,
            )+
        }

        impl<$($CH),+> From<($($CH),+)> for $name<$($CH),+> {
            fn from(value: ($($CH),+)) -> Self {
                let ($($c),+) = value;
                Self {
                    $(
                        $c,
                    )+
                }
            }
        }

        impl<TIM, $($CH),+> Pins<TIM> for $name<$($CH),+>
        where
            $(
                $CH: Pins<TIM>,
            )+
        {
            const C1: bool = false $(|| $CH::C1)+;
            const C2: bool = false $(|| $CH::C2)+;
            const C3: bool = false $(|| $CH::C3)+;
            const C4: bool = false $(|| $CH::C4)+;
            type Channels = ($($CH::Channels),+);
            fn split() -> Self::Channels {
                ($($CH::split()),+)
            }
        }
    };
}
use pinsx;

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
pub mod tim1 {
    use super::*;

    remap4! {
        pac::TIM1: [
            C1: PA8, C2: PA9,  C3: PA10, C4: PA11 => MAPR: 0;
            C1: PE9, C2: PE11, C3: PE13, C4: PE14 => MAPR: 3;
        ]
    }
    remap12! {
        pac::TIM1: [
            C1: PA8, C2: PA9  => MAPR: 0;
            C1: PE9, C2: PE11 => MAPR: 3;
        ]
    }
    remap_input! {
        pac::TIM1: [
            C1: PA8, C2: PA9  => MAPR: 0;
            C1: PE9, C2: PE11 => MAPR: 3;
        ]
    }
    remap34! {
        pac::TIM1: [
            C3: PA10, C4: PA11 => MAPR: 0;
            C3: PE13, C4: PE14 => MAPR: 3;
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
            C1: PA0,  C2: PA1, C3: PA2,  C4: PA3  => MAPR: 0;
            C1: PA15, C2: PB3, C3: PA2,  C4: PA3  => MAPR: 1;
            C1: PA0,  C2: PA1, C3: PB10, C4: PB11 => MAPR: 2;
            C1: PA15, C2: PB3, C3: PB10, C4: PB11 => MAPR: 3;
        ]
    }
    remap12! {
        pac::TIM2: [
            C1: PA0,  C2: PA1 => MAPR: 0;
            C1: PA15, C2: PB3 => MAPR: 1;
        ]
    }
    remap_input! {
        pac::TIM2: [
            C1: PA0,  C2: PA1 => MAPR: 0;
            C1: PA15, C2: PB3 => MAPR: 1;
        ]
    }
    remap34! {
        pac::TIM2: [
            C3: PA2,  C4: PA3  => MAPR: 0;
            C3: PB10, C4: PB11 => MAPR: 2;
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
            C1: PA6, C2: PA7, C3: PB0, C4: PB1 => MAPR: 0;
            C1: PB4, C2: PB5, C3: PB0, C4: PB1 => MAPR: 2;
            C1: PC6, C2: PC7, C3: PC8, C4: PC9 => MAPR: 3;
        ]
    }
    remap12! {
        pac::TIM3: [
            C1: PA6, C2: PA7 => MAPR: 0;
            C1: PB4, C2: PB5 => MAPR: 2;
            C1: PC6, C2: PC7 => MAPR: 3;
        ]
    }
    remap_input! {
        pac::TIM3: [
            C1: PA6, C2: PA7 => MAPR: 0;
            C1: PB4, C2: PB5 => MAPR: 2;
            C1: PC6, C2: PC7 => MAPR: 3;
        ]
    }
    remap34! {
        pac::TIM3: [
            C3: PB0, C4: PB1 => MAPR: 0;
            C3: PC8, C4: PC9 => MAPR: 3;
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
            C1: PB6,  C2: PB7,  C3: PB8,  C4: PB9  => MAPR: 0;
            C1: PD12, C2: PD13, C3: PD14, C4: PD15 => MAPR: 1;
        ]
    }
    remap12! {
        pac::TIM4: [
            C1: PB6,  C2: PB7  => MAPR: 0;
            C1: PD12, C2: PD13 => MAPR: 1;
        ]
    }
    remap_input! {
        pac::TIM4: [
            C1: PB6,  C2: PB7  => MAPR: 0;
            C1: PD12, C2: PD13 => MAPR: 1;
        ]
    }
    remap34! {
        pac::TIM4: [
            C3: PB8,  C4: PB9  => MAPR: 0;
            C3: PD14, C4: PD15 => MAPR: 1;
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
        $(C1: $P1:ident, C2: $P2:ident, C3: $P3:ident, C4: $P4:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        pub type Channels1234 = Pins4<Ch1, Ch2, Ch3, Ch4>;
        remap_combo! {
            $TIM; C1, C2, C3, C4: [
                $($P1, $P2, $P3, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels123 = Pins3<Ch1, Ch2, Ch3>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2), (C3, Ch3): [
                $($P1, $P2, $P3 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels124 = Pins3<Ch1, Ch2, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2), (C4, Ch4): [
                $($P1, $P2, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels134 = Pins3<Ch1, Ch3, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C3, Ch3), (C4, Ch4): [
                $($P1, $P3, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels234 = Pins3<Ch2, Ch3, Ch4>;
        remap_combo! {
            $TIM; (C2, Ch2), (C3, Ch3), (C4, Ch4): [
                $($P2, $P3, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels13 = Pins2<Ch1, Ch3>;
        remap_combo! {
            $TIM; (C1, Ch1), (C3, Ch3): [
                $($P1, $P3 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels14 = Pins2<Ch1, Ch4>;
        remap_combo! {
            $TIM; (C1, Ch1), (C4, Ch4): [
                $($P1, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels23 = Pins2<Ch2, Ch3>;
        remap_combo! {
            $TIM; (C2, Ch2), (C3, Ch3): [
                $($P2, $P3 $( => $MAPR: $remap)?;)+
            ]
        }
        pub type Channels24 = Pins2<Ch2, Ch4>;
        remap_combo! {
            $TIM; (C2, Ch2), (C4, Ch4): [
                $($P2, $P4 $( => $MAPR: $remap)?;)+
            ]
        }
    }
}
use remap4;

macro_rules! remap12 {
    ($TIM:ty: [
        $(C1: $P1:ident, C2: $P2:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        pub type Channels12 = Pins2<Ch1, Ch2>;
        remap_combo! {
            $TIM; (C1, Ch1), (C2, Ch2): [
                $($P1, $P2 $( => $MAPR: $remap)?;)+
            ]
        }
        remap_combo! {
            $TIM; (C1, Ch1): [
                $($P1 $( => $MAPR: $remap)?;)+
            ]
        }
        remap_combo! {
            $TIM; (C2, Ch2): [
                $($P2 $( => $MAPR: $remap)?;)+
            ]
        }
    }
}
use remap12;

macro_rules! remap34 {
    ($TIM:ty: [
        $(C3: $P1:ident, C4: $P2:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        pub type Channels34 = Pins2<Ch3, Ch4>;
        remap_combo! {
            $TIM; (C3, Ch3), (C4, Ch4): [
                $($P1, $P2 $( => $MAPR: $remap)?;)+
            ]
        }
        remap_combo! {
            $TIM; (C3, Ch3): [
                $($P1 $( => $MAPR: $remap)?;)+
            ]
        }
        remap_combo! {
            $TIM; (C4, Ch4): [
                $($P2 $( => $MAPR: $remap)?;)+
            ]
        }
    }
}
use remap34;


macro_rules! remap_combo_x {
    ($TIM:ty, $Pins:ident: [
        $($(($P:ident, $CH:ident, $c:ident, $C:ident),)+ $( => $MAPR:ident: $mapr:ident, $remap:literal)?;)+
    ]) => {
        impl From<($(gpio::$P<Alternate>)+, $(, &mut $MAPR)?)> for Pins4<$($CH),+> {
            fn from(p: ($(gpio::$P<Alternate>)+, $(, &mut $MAPR)?)) -> Self {
                let ($($c),+ $(,$mapr)?) = p;
                $(<$TIM>::remap($($mapr)?, $remap);)?
                Self { $($c: $CH::$P($c)),+ }
            }
        }

        impl From<($(gpio::$P<Alternate>)+, $(, &mut $MAPR)?)> for Pins4<$($CH),+> {
            fn from(p: ($(gpio::$P<Alternate>)+, $(, &mut $MAPR)?)) -> Self {
                let ($($c),+ $(,$mapr)?) = p;
                $(
                    #[allow(confusable_idents)]
                    let $с = $c.into_mode(&mut Cr);
                )+
                $(<$TIM>::remap($($mapr)?, $remap);)?
                Self { $($c: $CH::$P($c)),+ }
            }
        }
    }
}

macro_rules! remap_combo {
    ($TIM:ty; $C1:ident, $C2:ident, $C3:ident, $C4:ident: [
        $($P1:ident, $P2:ident, $P3:ident, $P4:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        $(
            remap_combo_x!()
            /*impl From<(gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate>, gpio::$P4<Alternate> $(, &mut $MAPR)?)> for Pins4<Ch1, Ch2, Ch3, Ch4> {
                fn from(p: (gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate>, gpio::$P4<Alternate> $(, &mut $MAPR)?)) -> Self {
                    let (c1, c2, c3, c4, mapr) = p;
                    $(<$TIM>::remap(mapr, $remap);)?
                    Self { c1: Ch1::$P1(c1), c2: Ch2::$P2(c2), c3: Ch3::$P3(c3), c4: Ch4::$P4(c4) }
                }
            }

            impl From<(gpio::$P1, gpio::$P2, gpio::$P3, gpio::$P4 $(, &mut $MAPR)?)> for Pins4<Ch1, Ch2, Ch3, Ch4> {
                fn from(p: (gpio::$P1, gpio::$P2, gpio::$P3, gpio::$P4 $(, &mut $MAPR)?)) -> Self {
                    let (c1, c2, c3, c4, mapr) = p;
                    let с1 = c1.into_mode(&mut Cr);
                    let с2 = c2.into_mode(&mut Cr);
                    let с3 = c3.into_mode(&mut Cr);
                    let с4 = c4.into_mode(&mut Cr);
                    $(<$TIM>::remap(mapr, $remap);)?
                    Self { c1: Ch1::$P1(с1), c2: Ch2::$P2(с2), c3: Ch3::$P3(с3), c4: Ch4::$P4(с4) }
                }
            }*/
        )+
    };

    ($TIM:ty; ($C1:ident, $Ch1:ident), ($C2:ident, $Ch2:ident), ($C3:ident, $Ch3:ident): [
        $($P1:ident, $P2:ident, $P3:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        $(
            impl From<(gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)> for Pins3<$Ch1, $Ch2, $Ch3> {
                fn from(p: (gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(<$TIM>::remap(p.3, $remap);)?
                    Self { c1: $Ch1::$P1(p.0), c2: $Ch2::$P2(p.1), c3: $Ch3::$P3(p.2) }
                }
            }

            impl From<(gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)> for Pins3<$Ch1, $Ch2, $Ch3> {
                fn from(p: (gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)) -> Self {
                    let p1 = p.0.into_mode(&mut Cr);
                    let p2 = p.1.into_mode(&mut Cr);
                    let p3 = p.2.into_mode(&mut Cr);
                    $(<$TIM>::remap(p.3, $remap);)?
                    Self { c1: $Ch1::$P1(p1), c2: $Ch2::$P2(p2), c3: $Ch3::$P3(p3) }
                }
            }
        )+
    };

    ($TIM:ty; ($C1:ident, $Ch1:ident), ($C2:ident, $Ch2:ident): [
        $($P1:ident, $P2:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        $(
            impl From<(gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)> for Pins2<$Ch1, $Ch2> {
                fn from(p: (gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(<$TIM>::remap(p.2, $remap);)?
                    Self { c1: $Ch1::$P1(p.0), c2: $Ch2::$P2(p.1) }
                }
            }

            impl From<(gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)> for Pins2<$Ch1, $Ch2> {
                fn from(p: (gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)) -> Self {
                    let p1 = p.0.into_mode(&mut Cr);
                    let p2 = p.1.into_mode(&mut Cr);
                    $(<$TIM>::remap(p.2, $remap);)?
                    Self { c1: $Ch1::$P1(p1), c2: $Ch2::$P2(p2) }
                }
            }
        )+
    };

    ($TIM:ty; ($C:ident, $Ch:ident): [
        $($P1:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        impl Pins<$TIM> for $Ch {
            const $C: bool = true;
            type Channels = PwmChannel<$TIM, $C>;
            fn split() -> Self::Channels {
                PwmChannel::new()
            }
        }

        $(
            impl From<(gpio::$P1<Alternate> $(, &mut $MAPR)?)> for $Ch {
                fn from(p: (gpio::$P1<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(<$TIM>::remap(p.1, $remap);)?
                    $Ch::$P1(p.0)
                }
            }

            impl From<(gpio::$P1 $(, &mut $MAPR)?)> for $Ch {
                fn from(p: (gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    let p1 = p.0.into_mode(&mut Cr);
                    $(<$TIM>::remap(p.1, $remap);)?
                    $Ch::$P1(p1)
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
        $(C1: $P1:ident, C2: $P2:ident $( => $MAPR:ident: $remap:literal)?;)+
    ]) => {
        impl InputPins for $TIM {
            type InCh1 = InCh1;
            type InCh2 = InCh2;
        }

        $(
            impl From<(gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)> for InPins<InCh1, InCh2> {
                fn from(p: (gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)) -> Self {
                    $(<$TIM>::remap(p.2, $remap);)?
                    Self { c1: InCh1::$P1(p.0), c2: InCh2::$P2(p.1) }
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
