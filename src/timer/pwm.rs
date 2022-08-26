/*!
  # Pulse width modulation

  The general purpose timers (`TIM2`, `TIM3`, and `TIM4`) can be used to output
  pulse width modulated signals on some pins. The timers support up to 4
  simultaneous pwm outputs in separate `Channels`

  ## Usage for pre-defined channel combinations

  This crate only defines basic channel combinations for default AFIO remappings,
  where all the channels are enabled. Start by setting all the pins for the
  timer you want to use to alternate push pull pins:

  ```rust
  let gpioa = ..; // Set up and split GPIOA
  // Select the pins you want to use
  let pins = (
      gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
      gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
  );

  // Set up the timer as a PWM output. If selected pins may correspond to different remap options,
  // then you must specify the remap generic parameter. Otherwise, if there is no such ambiguity,
  // the remap generic parameter can be omitted without complains from the compiler.
  let (c1, c2, c3, c4) = Timer::tim2(p.TIM2, &clocks)
      .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz())
      .3;

  // Start using the channels
  c1.set_duty(c1.get_max_duty());
  // ...
  ```

  Then call the `pwm` function on the corresponding timer.

  NOTE: In some cases you need to specify remap you need, especially for TIM2
  (see [Alternate function remapping](super)):

  ```
    let device: pac::Peripherals = ..;

    // Put the timer in PWM mode using the specified pins
    // with a frequency of 100 Hz.
    let (c0, c1, c2, c3) = Timer::tim2(device.TIM2, &clocks)
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 100.Hz());

    // Set the duty cycle of channel 0 to 50%
    c0.set_duty(c0.get_max_duty() / 2);
    // PWM outputs are disabled by default
    c0.enable()
  ```
*/

use crate::afio::MAPR;
use crate::gpio::{self, Alternate};

use super::{compute_arr_presc, Channel, FTimer, Instance, Ocm, Timer, WithPwm};
use crate::rcc::Clocks;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use fugit::{HertzU32 as Hertz, TimerDurationU32};

pub trait Pins<REMAP, P> {
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

pub use super::{pins::sealed::Remap, CPin, Ch, C1, C2, C3, C4};

pub struct PwmChannel<TIM, const C: u8> {
    pub(super) _tim: PhantomData<TIM>,
}

macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($ENCHX:ident),+ ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<TIM, REMAP, OUTMODE, $($PINX,)+> Pins<REMAP, ($(Ch<$ENCHX>),+)> for ($($PINX),+)
            where
                TIM: Instance + WithPwm,
                REMAP: Remap<Periph = TIM>,
                $($PINX: CPin<REMAP, $ENCHX> + gpio::PinExt<Mode=Alternate<OUTMODE>>,)+
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(PwmChannel<TIM, $ENCHX>),+);
                fn split() -> Self::Channels {
                    ($(PwmChannel::<TIM, $ENCHX>::new()),+)
                }
            }
        )+
    };
}

pins_impl!(
    (P1, P2, P3, P4), (C1, C2, C3, C4);
    (P2, P3, P4), (C2, C3, C4);
    (P1, P3, P4), (C1, C3, C4);
    (P1, P2, P4), (C1, C2, C4);
    (P1, P2, P3), (C1, C2, C3);
    (P3, P4), (C3, C4);
    (P2, P4), (C2, C4);
    (P2, P3), (C2, C3);
    (P1, P4), (C1, C4);
    (P1, P3), (C1, C3);
    (P1, P2), (C1, C2);
    (P1), (C1);
    (P2), (C2);
    (P3), (C3);
    (P4), (C4);
);

pub trait PwmExt
where
    Self: Sized + Instance + WithPwm,
{
    fn pwm<REMAP, P, PINS, const FREQ: u32>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
        clocks: &Clocks,
    ) -> Pwm<Self, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>;

    fn pwm_hz<REMAP, P, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        freq: Hertz,
        clocks: &Clocks,
    ) -> PwmHz<Self, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>;

    fn pwm_us<REMAP, P, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<1_000_000>,
        clocks: &Clocks,
    ) -> Pwm<Self, REMAP, P, PINS, 1_000_000>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>,
    {
        self.pwm::<_, _, _, 1_000_000>(pins, mapr, time, clocks)
    }
}

impl<TIM> PwmExt for TIM
where
    Self: Sized + Instance + WithPwm,
{
    fn pwm<REMAP, P, PINS, const FREQ: u32>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
        clocks: &Clocks,
    ) -> Pwm<TIM, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>,
    {
        FTimer::<Self, FREQ>::new(self, clocks).pwm(pins, mapr, time)
    }

    fn pwm_hz<REMAP, P, PINS>(
        self,
        pins: PINS,
        mapr: &mut MAPR,
        time: Hertz,
        clocks: &Clocks,
    ) -> PwmHz<TIM, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>,
    {
        Timer::new(self, clocks).pwm_hz(pins, mapr, time)
    }
}

impl<TIM: Instance + WithPwm, const C: u8> PwmChannel<TIM, C> {
    pub(crate) fn new() -> Self {
        Self {
            _tim: core::marker::PhantomData,
        }
    }
}

impl<TIM: Instance + WithPwm, const C: u8> PwmChannel<TIM, C> {
    #[inline]
    pub fn disable(&mut self) {
        TIM::enable_channel(C, false);
    }

    #[inline]
    pub fn enable(&mut self) {
        TIM::enable_channel(C, true);
    }

    #[inline]
    pub fn get_duty(&self) -> u16 {
        TIM::read_cc_value(C) as u16
    }

    /// If `0` returned means max_duty is 2^16
    #[inline]
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    #[inline]
    pub fn set_duty(&mut self, duty: u16) {
        TIM::set_cc_value(C, duty as u32)
    }
}

pub struct PwmHz<TIM, REMAP, P, PINS>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    timer: Timer<TIM>,
    _pins: PhantomData<(REMAP, P, PINS)>,
}

impl<TIM, REMAP, P, PINS> PwmHz<TIM, REMAP, P, PINS>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn release(mut self) -> Timer<TIM> {
        // stop timer
        self.tim.cr1_reset();
        self.timer
    }

    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }
}

impl<TIM, REMAP, P, PINS> Deref for PwmHz<TIM, REMAP, P, PINS>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    type Target = Timer<TIM>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, REMAP, P, PINS> DerefMut for PwmHz<TIM, REMAP, P, PINS>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

impl<TIM: Instance + WithPwm> Timer<TIM> {
    pub fn pwm_hz<REMAP, P, PINS>(
        mut self,
        _pins: PINS,
        mapr: &mut MAPR,
        freq: Hertz,
    ) -> PwmHz<TIM, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM>,
        PINS: Pins<REMAP, P>,
    {
        REMAP::remap(mapr);

        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1);
        }
        if PINS::C2 && TIM::CH_NUMBER > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1);
        }
        if PINS::C3 && TIM::CH_NUMBER > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1);
        }
        if PINS::C4 && TIM::CH_NUMBER > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1);
        }

        // The reference manual is a bit ambiguous about when enabling this bit is really
        // necessary, but since we MUST enable the preload for the output channels then we
        // might as well enable for the auto-reload too
        self.tim.enable_preload(true);

        let (psc, arr) = compute_arr_presc(freq.raw(), self.clk.raw());
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();

        // Trigger update event to load the registers
        self.tim.trigger_update();

        self.tim.start_pwm();

        PwmHz {
            timer: self,
            _pins: PhantomData,
        }
    }
}

impl<TIM, REMAP, P, PINS> PwmHz<TIM, REMAP, P, PINS>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty as u32)
    }

    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    pub fn get_period(&self) -> Hertz {
        let clk = self.clk;
        let psc = self.tim.read_prescaler() as u32;
        let arr = TIM::read_auto_reload();

        // Length in ms of an internal clock pulse
        clk / ((psc + 1) * (arr + 1))
    }

    pub fn set_period(&mut self, period: Hertz) {
        let clk = self.clk;

        let (psc, arr) = compute_arr_presc(period.raw(), clk.raw());
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();
    }
}

pub struct Pwm<TIM, REMAP, P, PINS, const FREQ: u32>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    timer: FTimer<TIM, FREQ>,
    _pins: PhantomData<(REMAP, P, PINS)>,
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> Pwm<TIM, REMAP, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }

    pub fn release(mut self) -> FTimer<TIM, FREQ> {
        // stop counter
        self.tim.cr1_reset();
        self.timer
    }
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> Deref for Pwm<TIM, REMAP, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    type Target = FTimer<TIM, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> DerefMut for Pwm<TIM, REMAP, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

impl<TIM: Instance + WithPwm, const FREQ: u32> FTimer<TIM, FREQ> {
    pub fn pwm<REMAP, P, PINS>(
        mut self,
        _pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = TIM>,
        PINS: Pins<REMAP, P>,
    {
        REMAP::remap(mapr);

        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1);
        }
        if PINS::C2 && TIM::CH_NUMBER > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1);
        }
        if PINS::C3 && TIM::CH_NUMBER > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1);
        }
        if PINS::C4 && TIM::CH_NUMBER > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1);
        }

        // The reference manual is a bit ambiguous about when enabling this bit is really
        // necessary, but since we MUST enable the preload for the output channels then we
        // might as well enable for the auto-reload too
        self.tim.enable_preload(true);

        self.tim.set_auto_reload(time.ticks() - 1).unwrap();

        // Trigger update event to load the registers
        self.tim.trigger_update();

        self.tim.start_pwm();

        Pwm {
            timer: self,
            _pins: PhantomData,
        }
    }
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> Pwm<TIM, REMAP, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty.into())
    }

    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    pub fn get_period(&self) -> TimerDurationU32<FREQ> {
        TimerDurationU32::from_ticks(TIM::read_auto_reload() + 1)
    }

    pub fn set_period(&mut self, period: TimerDurationU32<FREQ>) {
        self.tim.set_auto_reload(period.ticks() - 1).unwrap();
    }
}
