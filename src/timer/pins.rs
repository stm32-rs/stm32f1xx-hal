use crate::afio::MAPR;
use crate::gpio::{self, Alternate, Cr};
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

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
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

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
pub mod tim1 {
    use super::*;

    remap4! {
        pac::TIM1: [
            No, PA8, PA9, PA10, PA11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            Full, PE9, PE11, PE13, PE14 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM1: [
            No, PA8, PA9 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            Full, PE9, PE11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap_input! {
        pac::TIM1, InputChannels12: [
            No, PA8, PA9 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            Full, PE9, PE11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
    remap34! {
        pac::TIM1: [
            No, PA10, PA11 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0)} };
            Full, PE13, PE14 => MAPR { |_, w| unsafe { w.tim1_remap().bits(0b11)} };
        ]
    }
}

pub mod tim2 {
    use super::*;

    remap4! {
        pac::TIM2: [
            No, PA0, PA1, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            Partial1, PA15, PB3, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
            Partial2, PA0, PA1, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b10)} };
            Full, PA15, PB3, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM2: [
            No, PA0, PA1 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            Partial1, PA15, PB3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
        ]
    }
    remap_input! {
        pac::TIM2, InputChannels12: [
            No, PA0, PA1 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            Partial1, PA15, PB3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b01)} };
        ]
    }
    remap34! {
        pac::TIM2: [
            No, PA2, PA3 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0)} };
            Partial2, PB10, PB11 => MAPR { |_, w| unsafe { w.tim2_remap().bits(0b10)} };
        ]
    }
}

pub mod tim3 {
    use super::*;

    remap4! {
        pac::TIM3: [
            No, PA6, PA7, PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            Partial, PB4, PB5, PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            Full, PC6, PC7, PC8, PC9 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap12! {
        pac::TIM3: [
            No, PA6, PA7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            Partial, PB4, PB5 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            Full, PC6, PC7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap_input! {
        pac::TIM3, InputChannels12: [
            No, PA6, PA7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            Partial, PB4, PB5 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b10)} };
            Full, PC6, PC7 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
    remap34! {
        pac::TIM3: [
            No, PB0, PB1 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0)} };
            Full, PC8, PC9 => MAPR { |_, w| unsafe { w.tim3_remap().bits(0b11)} };
        ]
    }
}

#[cfg(feature = "medium")]
pub mod tim4 {
    use super::*;

    remap4! {
        pac::TIM4: [
            No, PB6, PB7, PB8, PB9 => MAPR { |_, w| w.tim4_remap().bit(false) };
            Full, PD12, PD13, PD14, PD15 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap12! {
        pac::TIM4: [
            No, PB6, PB7 => MAPR { |_, w| w.tim4_remap().bit(false) };
            Full, PD12, PD13 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap_input! {
        pac::TIM4, InputChannels12: [
            No, PB6, PB7 => MAPR { |_, w| w.tim4_remap().bit(false) };
            Full, PD12, PD13 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
    remap34! {
        pac::TIM4: [
            No, PB8, PB9 => MAPR { |_, w| w.tim4_remap().bit(false) };
            Full, PD14, PD15 => MAPR { |_, w| w.tim4_remap().bit(true) };
        ]
    }
}

macro_rules! remap4 {
    ($TIM:ty: [
        $($rname:ident, $P1:ident, $P2:ident, $P3:ident, $P4:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        remap_combo! {
            $TIM, Channels1234; C1, C2, C3, C4: [
                $($rname, $P1, $P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels123; C1, C2, C3: [
                $($rname, $P1, $P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels124; C1, C2, C4: [
                $($rname, $P1, $P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels134; C1, C3, C4: [
                $($rname, $P1, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels234; C2, C3, C4: [
                $($rname, $P2, $P3, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels13; C1, C3: [
                $($rname, $P1, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels14; C1, C4: [
                $($rname, $P1, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels23; C2, C3: [
                $($rname, $P2, $P3 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channels24; C2, C4: [
                $($rname, $P2, $P4 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap4;

macro_rules! remap12 {
    ($TIM:ty: [
        $($rname:ident, $P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        remap_combo! {
            $TIM, Channels12; C1, C2: [
                $($rname, $P1, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channel1; C1: [
                $($rname, $P1 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channel2; C2: [
                $($rname, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap12;

macro_rules! remap34 {
    ($TIM:ty: [
        $($rname:ident, $P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        remap_combo! {
            $TIM, Channels34; C3, C4: [
                $($rname, $P1, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channel3; C3: [
                $($rname, $P1 $( => $MAPR { $remapex })?;)+
            ]
        }
        remap_combo! {
            $TIM, Channel4; C4: [
                $($rname, $P2 $( => $MAPR { $remapex })?;)+
            ]
        }
    }
}
use remap34;

macro_rules! remap_combo {
    ($TIM:ty, $name:ident; $C1:ident, $C2:ident, $C3:ident, $C4:ident: [
        $($rname:ident, $P0:ident, $P1:ident, $P2:ident, $P3:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate>),
            )+
        }

        impl Pins<$TIM> for $name {
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
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>, gpio::$P3<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.4.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1, p.2, p.3)
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2, gpio::$P3 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    let p2 = p.2.into_mode(&mut Cr);
                    let p3 = p.3.into_mode(&mut Cr);
                    $(p.4.modify_mapr($remapex);)?
                    Self::$rname(p0, p1, p2, p3)
                }
            }
        )+
    };

    ($TIM:ty, $name:ident; $C1:ident, $C2:ident, $C3:ident: [
        $($rname:ident, $P0:ident, $P1:ident, $P2:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate>),
            )+
        }

        impl Pins<$TIM> for $name {
            const $C1: bool = true;
            const $C2: bool = true;
            const $C3: bool = true;
            type Channels = (PwmChannel<$TIM, $C1>, PwmChannel<$TIM, $C2>, PwmChannel<$TIM, $C3>);
            fn split() -> Self::Channels {
                (PwmChannel::new(), PwmChannel::new(), PwmChannel::new())
            }
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate>, gpio::$P2<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.3.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1, p.2)
                }
            }

            impl From<(gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1, gpio::$P2 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    let p2 = p.2.into_mode(&mut Cr);
                    $(p.3.modify_mapr($remapex);)?
                    Self::$rname(p0, p1, p2)
                }
            }
        )+
    };

    ($TIM:ty, $name:ident; $C1:ident, $C2:ident: [
        $($rname:ident, $P0:ident, $P1:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>, gpio::$P1<Alternate>),
            )+
        }

        impl Pins<$TIM> for $name {
            const $C1: bool = true;
            const $C2: bool = true;
            type Channels = (PwmChannel<$TIM, $C1>, PwmChannel<$TIM, $C2>);
            fn split() -> Self::Channels {
                (PwmChannel::new(), PwmChannel::new())
            }
        }

        $(
            impl From<(gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate>, gpio::$P1<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.2.modify_mapr($remapex);)?
                    Self::$rname(p.0, p.1)
                }
            }

            impl From<(gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    $(p.2.modify_mapr($remapex);)?
                    Self::$rname(p0, p1)
                }
            }
        )+
    };

    ($TIM:ty, $name:ident; $C:ident: [
        $($rname:ident, $P0:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0<Alternate>),
            )+
        }

        impl Pins<$TIM> for $name {
            const $C: bool = true;
            type Channels = PwmChannel<$TIM, $C>;
            fn split() -> Self::Channels {
                PwmChannel::new()
            }
        }

        $(
            impl From<(gpio::$P0<Alternate> $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0<Alternate> $(, &mut $MAPR)?)) -> Self {
                    $(p.1.modify_mapr($remapex);)?
                    Self::$rname(p.0)
                }
            }

            impl From<(gpio::$P0 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    $(p.1.modify_mapr($remapex);)?
                    Self::$rname(p0)
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
    ($TIM:ty, $name:ident: [
        $($rname:ident, $P0:ident, $P1:ident $( => $MAPR:ident { $remapex:expr })?;)+
    ]) => {
        pub enum $name {
            $(
                $rname(gpio::$P0, gpio::$P1),
            )+
        }

        impl InputPins for $TIM {
            type Channels12 = $name;
        }

        $(
            impl From<(gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)> for $name {
                fn from(p: (gpio::$P0, gpio::$P1 $(, &mut $MAPR)?)) -> Self {
                    let p0 = p.0.into_mode(&mut Cr);
                    let p1 = p.1.into_mode(&mut Cr);
                    $(p.2.modify_mapr($remapex);)?
                    Self::$rname(p0, p1)
                }
            }
        )+
    };
}
use remap_input;

pub trait InputPins {
    type Channels12;
}
