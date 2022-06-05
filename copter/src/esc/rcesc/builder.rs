use embedded_hal::PwmPin;

use super::RCESC;

pub struct Builder<T> {
    arm: T,
    min: T,
    max: Option<T>,
    delay_ms: u16,
}

impl<T: Default> Default for Builder<T> {
    fn default() -> Self {
        Self {
            arm: T::default(),
            min: T::default(),
            max: None,
            delay_ms: 0,
        }
    }
}

impl<T> Builder<T> {
    pub fn arm(mut self, arm: T) -> Self {
        self.arm = arm;
        self
    }

    pub fn min(mut self, min: T) -> Self {
        self.min = min;
        self
    }

    pub fn max(mut self, max: T) -> Self {
        self.max = Some(max);
        self
    }

    pub fn delay(mut self, ms: u16) -> Self {
        self.delay_ms = ms;
        self
    }

    pub fn build<P>(self, pin: P) -> RCESC<P>
    where
        P: PwmPin<Duty = T>,
    {
        RCESC {
            arm: self.arm,
            min: self.min,
            max: self.max.unwrap_or(pin.get_max_duty()),
            pin,
        }
    }
}
