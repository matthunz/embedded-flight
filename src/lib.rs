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

//#![no_std]

pub mod copter;

pub use copter::{Copter, QuadCopter};

pub mod filter;

pub mod hal;

pub use hal::{Actuator, Sensors, ESC};

pub mod plane;
pub use plane::Plane;

pub mod scheduler;
pub use scheduler::Scheduler;

pub mod motor;

/*
pub async fn motor_task<const N: usize>(motors: &Mutex<MotorMatrix<impl ESC<i16>, N>>) {
    let mut interval = interval(Duration::from_millis(1));
    loop {
        interval.tick().await;

        let mut motors = motors.lock().unwrap();
        motors.output(1);
    }
}
 */

fn constrain_float(amt: f32, low: f32, high: f32) -> f32 {
    constrain_value_line(amt, low, high, 17)
}

fn constrain_value_line(amt: f32, low: f32, high: f32, _line: u32) -> f32 {
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
