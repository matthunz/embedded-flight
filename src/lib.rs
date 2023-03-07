//! # embedded-flight
//! A `#![no_std]` flight software library for embedded rust
//!
//! # Generic components
//! [`scheduler`] contains the real-time scheduler to run tasks at desired frequencies.
//!
//! [`hal`] contains the hardware abstraction layer.
//!
//! # Multi-copter components
//! [`Copter`] is a high level position controller for a multi-copter
//! (see [`QuadCopter`] for a quad-motor implementation)
//!
//! [`control`](copter::control) contains the low level flight controllers.
//!
//! [`MotorControl`](copter::control::MotorControl) is a trait for the low level motor control.
//! (see [`QuadMotorController`](copter::control::QuadMotorController) for a quad-motor implementation)

#![no_std]

pub mod copter;
pub use copter::{Copter, QuadCopter};

pub mod filter;

pub mod hal;
use filter::{DerivativeFilter, LowPassFilter};
pub use hal::{Actuator, Sensors, ESC};

pub mod plane;
pub use plane::Plane;

pub mod scheduler;
pub use scheduler::Scheduler;

pub struct MultiCopterMotors {
    /// Pilot throttle input filter
    throttle_filter: LowPassFilter,

    /// Last throttle input from set_throttle caller
    throttle_in: f32,

    /// Time difference (in seconds) since the last loop time
    dt: f32,

    /// Throttle output slew detector
    throttle_slew: DerivativeFilter<7>,

    /// Throttle slew rate from input
    throttle_slew_rate: f32,

    /// Filter for the output of the throttle slew
    throttle_slew_filter: LowPassFilter,
    
    /// Armed state of the motors
    is_armed: bool,
}

impl MultiCopterMotors {
    pub fn output(&mut self, dt: u32) {
        self.update_throttle_filter(dt);
    }

    pub fn update_throttle_filter(&mut self, dt: u32) {
        let last_throttle = self.throttle_filter.output();

        if self.is_armed {
            self.throttle_filter.apply(self.throttle_in, self.dt);

            // Constrain filtered throttle
            if self.throttle_filter.output() < 0.0 {
                self.throttle_filter.reset(0.0);
            } else if self.throttle_filter.output() > 1.0 {
                self.throttle_filter.reset(1.0);
            }
        } else {
            self.throttle_filter.reset(0.0);
        }

        let new_throttle = self.throttle_filter.output();
        if last_throttle != new_throttle {
            self.throttle_slew.update(new_throttle, dt);
        }

        // Calculate slope normalized from per-micro
        let rate = (self.throttle_slew.slope() * 1e6).abs();
        self.throttle_slew_rate = self.throttle_slew_filter.apply(rate, self.dt);
    }
}
