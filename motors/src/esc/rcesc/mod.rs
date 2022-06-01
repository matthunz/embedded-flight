mod builder;
use core::ops::Neg;

pub use builder::Builder;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::PwmPin;
use num_traits::{Float, One, Num};

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

        self.arm();
    }
}

impl<T> ESC for RCESC<T>
where
    T: PwmPin,
    T::Duty:  Num + Copy,
{
    type Output = T::Duty;

    fn arm(&mut self) {
        self.output(self.arm)
    }

    fn output(&mut self, output: Self::Output) {
        self.pin.set_duty(output);
    }
}
