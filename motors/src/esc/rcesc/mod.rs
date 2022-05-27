mod builder;
pub use builder::Builder;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::PwmPin;
use num_traits::{Float, One};

use super::ESC;

pub struct RCESC<T: PwmPin> {
    arm: T::Duty,
    min: T::Duty,
    max: T::Duty,
    pin: T,
}

impl<T> RCESC<T>
where
    T: PwmPin,
    T::Duty: Float,
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
    T::Duty: Float + Clone,
{
    type Output = T::Duty;

    fn arm(&mut self) {
        self.output(self.arm.clone())
    }

    fn output(&mut self, output: Self::Output) {
        let duty = (output - -T::Duty::one()) * (self.max - self.min)
            / (T::Duty::one() - -T::Duty::one())
            + T::Duty::one();
        self.pin.set_duty(duty);
    }
}
