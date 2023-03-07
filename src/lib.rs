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

pub struct Motor<E> {
    esc: E,
    is_enabled: bool,
    thrust_rpyt_out: f32,
    actuator: f32,
}

pub struct MotorMatrix<E, const N: usize> {
    motors: [Motor<E>; N],
    controller: MultiCopterMotors,
}

impl<E, const N: usize> MotorMatrix<E, N> {
    pub fn output(&mut self, dt: u32) {
        self.controller.update_throttle_filter(dt);

        self.output_to_motors();
    }

    pub fn output_to_motors(&mut self) {
        // Set motor output based on thrust requests
        for motor in &mut self.motors {
            if motor.is_enabled {
                motor.actuator = self.controller.actuator_with_slew(
                    motor.actuator,
                    self.controller.thrust_to_actuator(motor.thrust_rpyt_out),
                );
            }
        }
    }
}

pub struct MultiCopterMotors {
    /// Time difference (in seconds) since the last loop time
    dt: f32,

    speed_hz: u16,    // speed in hz to send updates to motors
    roll_in: f32,     // desired roll control from attitude controllers, -1 ~ +1
    roll_in_ff: f32,  // desired roll feed forward control from attitude controllers, -1 ~ +1
    pitch_in: f32,    // desired pitch control from attitude controller, -1 ~ +1
    pitch_in_ff: f32, // desired pitch feed forward control from attitude controller, -1 ~ +1
    yaw_in: f32,      // desired yaw control from attitude controller, -1 ~ +1
    yaw_in_ff: f32,   // desired yaw feed forward control from attitude controller, -1 ~ +1

    forward_in: f32,       // last forward input from set_forward caller
    lateral_in: f32,       // last lateral input from set_lateral caller
    throttle_avg_max: f32, // last throttle input from set_throttle_avg_max

    /// Pilot throttle input filter
    throttle_filter: LowPassFilter,

    /// Last throttle input from set_throttle caller
    throttle_in: f32,

    /// Throttle after mixing is complete
    throttle_out: f32,

    /// Throttle output slew detector
    throttle_slew: DerivativeFilter<7>,

    /// Throttle slew rate from input
    throttle_slew_rate: f32,

    /// Filter for the output of the throttle slew
    throttle_slew_filter: LowPassFilter,

    /// throttle increase slew limitting
    slew_up_time: f32,

    /// throttle increase slew limitting
    slew_dn_time: f32,

    ///
    batter_voltage_filter: LowPassFilter,

    /// Curve used to linearize pwm to thrust conversion. set to 0 for linear and 1 for second order approximation
    thrust_curve_exp: f32,

    /// Throttle out ratio which produces the minimum thrust. (i.e. 0 ~ 1 ) of the full throttle range
    spin_min: f32,

    /// Throttle out ratio which produces the maximum thrust. (i.e. 0 ~ 1 ) of the full throttle range
    spin_max: f32,

    /// Maximum lift ratio from battery voltage
    max_lift: f32,

    /// Armed state of the motors
    is_armed: bool,
}

impl MultiCopterMotors {
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

    /// Add slew rate limiting to actuator output
    pub fn actuator_with_slew(&self, actuator_output: f32, input: f32) -> f32 {
        /*
        If MOT_SLEW_UP_TIME is 0 (default), no slew limit is applied to increasing output.
        If MOT_SLEW_DN_TIME is 0 (default), no slew limit is applied to decreasing output.
        MOT_SLEW_UP_TIME and MOT_SLEW_DN_TIME are constrained to 0.0~0.5 for sanity.
        If spool mode is shutdown, no slew limit is applied to allow immediate disarming of motors.
        */

        // Output limits with no slew time applied
        let mut output_slew_limit_up = 1.0;
        let mut output_slew_limit_dn = 0.0;

        // If MOT_SLEW_UP_TIME is set, calculate the highest allowed new output value, constrained 0.0~1.0
        if self.slew_up_time > 0.0 {
            let output_delta_up_max = self.dt / constrain_float(self.slew_up_time, 0., 0.5);
            output_slew_limit_up = constrain_float(actuator_output + output_delta_up_max, 0., 1.);
        }

        // If MOT_SLEW_DN_TIME is set, calculate the lowest allowed new output value, constrained 0.0~1.0
        if self.slew_dn_time > 0.0 {
            let output_delta_dn_max = self.dt / constrain_float(self.slew_dn_time, 0., 0.5);
            output_slew_limit_dn = constrain_float(actuator_output - output_delta_dn_max, 0., 1.);
        }

        // Constrain change in output to within the above limits
        input.max(output_slew_limit_dn).min(output_slew_limit_up)
    }

    /// Apply_thrust_curve_and_volt_scaling. Returns throttle in the range 0 ~ 1
    pub fn apply_thrust_curve_and_volt_scaling(&self, thrust: f32) -> f32 {
        let mut battery_scale = 1.;
        if self.batter_voltage_filter.output() > 0. {
            battery_scale = 1.0 / self.batter_voltage_filter.output();
        }
        // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
        let thrust_curve_expo = constrain_float(self.thrust_curve_exp, -1., 1.);
        if thrust_curve_expo == 0. {
            // zero expo means linear, avoid floating point exception for small values
            return self.max_lift * thrust * battery_scale;
        }
        let throttle_ratio = ((thrust_curve_expo - 1.)
            + safe_sqrt(
                (1. - thrust_curve_expo) * (1. - thrust_curve_expo)
                    + 4. * thrust_curve_expo * self.max_lift * thrust,
            ))
            / (2. * thrust_curve_expo);
        constrain_float(throttle_ratio * battery_scale, 0., 1.)
    }

    pub fn thrust_to_actuator(&self, thrust: f32) -> f32 {
        let thrust_constrained = constrain_float(thrust, 0., 1.);
        return self.spin_min
            + (self.spin_max - self.spin_min)
                * self.apply_thrust_curve_and_volt_scaling(thrust_constrained);
    }
}

fn constrain_float(amt: f32, low: f32, high: f32) -> f32 {
    constrain_value_line(amt, low, high, 17)
}

fn constrain_value_line(amt: f32, low: f32, high: f32, line: u32) -> f32 {
    if amt.is_nan() {
        //AP::internalerror().error(AP_InternalError::error_t::constraining_nan, line);
        return (low + high) / 2.0;
    }

    if amt < low {
        return low;
    }

    if amt > high {
        return high;
    }

    amt
}

fn safe_sqrt(v: f32) -> f32 {
    let ret = v.sqrt();
    if ret.is_nan() {
        return 0.0;
    }
    ret
}
