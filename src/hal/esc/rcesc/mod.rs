mod builder;
use core::ops::Neg;

pub use builder::Builder;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::PwmPin;
use num_traits::{Float, Num, NumCast, One, ToPrimitive};

use super::ESC;

// TODO scale final pwm output
pub struct RCESC<T: PwmPin> {
    arm: T::Duty,
    min: T::Duty,
    max: T::Duty,
    pin: T,
}

impl<T> RCESC<T>
where
    T: PwmPin,
    T::Duty: Num + Copy,
{
    pub fn new(arm: T::Duty, min: T::Duty, max: T::Duty, pin: T) -> Self {
        Self { arm, min, max, pin }
    }

    pub fn builder() -> Builder<T::Duty>
    where
        T::Duty: Default,
    {
        Builder::default()
    }

    pub fn calibrate<D>(&mut self, delay: &mut D)
    where
        D: DelayMs<u16>,
    {
        self.pin.set_duty(self.max);
        delay.delay_ms(2000);

        self.pin.set_duty(self.min);
        delay.delay_ms(2000);

        self.arm_inner();
    }

    fn arm_inner(&mut self) {
        self.pin.set_duty(self.arm)
    }
}

impl<T, U> ESC<U> for RCESC<T>
where
    T: PwmPin,
    T::Duty: Num + NumCast + ToPrimitive + Copy,
    U: Float + NumCast,
{
    fn arm(&mut self) {
        self.arm_inner()
    }

    fn output(&mut self, output: U) {
        let min = U::from(self.min).unwrap();
        let max = U::from(self.max).unwrap();

        let duty: U = (output + U::one()) * (max - min) / (U::one() + U::one()) + min;
        self.pin.set_duty(<T::Duty as NumCast>::from(duty).unwrap());
    }
}