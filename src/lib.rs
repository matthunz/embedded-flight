//! # embedded-flight
//! A `#![no_std]` flight software library for embedded rust
//!
//! # Examples
//!
//! For more check out the
//! [basic](https://github.com/matthunz/embedded-flight/blob/main/examples/basic.rs) or
//! [quad](https://github.com/matthunz/embedded-flight/tree/main/examples/quad)
//! examples on GitHub.
//!
//! Create and run a [`MultiCopter`] from a motor matrix, inertial sensor, clock, and frequency (in hz).
//! ```ignore
//! use embedded_flight::copter::{MultiCopter, multi_copter_tasks};
//! use embedded_flight::motors::MotorMatrix;
//!
//! // Create a quad-copter motor matrix from 4 ESCs
//! let motor_matrix = MotorMatrix::quad(ESC(0), ESC(1), ESC(2), ESC(3));
//!
//! // Initialize the tasks to run from the scheduler
//! // By default this will stabilize the attitude
//! let mut tasks = multi_copter_tasks();
//!
//! // Create the quad-copter
//! let mut copter = MultiCopter::new(motors, imu, &mut tasks, clock, 400);
//!
//! // Run the tasks in the scheduler and output to the motors in a loop
//! drone.run()?
//! ```
//!
//! Use the lower level [`MultiCopterAttitudeController`](control::attitude::MultiCopterAttitudeController) to get the motor output for a desired attitude:
//! ```
//! use embedded_flight::control::attitude::MultiCopterAttitudeController;
//! use nalgebra::{Vector3, Quaternion};
//!
//! let mut controller = MultiCopterAttitudeController::default();
//!
//! // Input the desired attitude and angular velocity with the current attitude
//! controller.attitude_controller.input(
//!     Quaternion::default(),
//!     Vector3::default(),
//!     Quaternion::default(),
//! );
//!
//! // Output the control to the motors with the current gyroscope data.
//! let output = controller.motor_output(Vector3::default(), 1);
//! dbg!(output);
//! ```

#![cfg_attr(not(test), no_std)]

use control::{attitude::sqrt_controller, PositionController};
pub use embedded_flight_control as control;
pub use embedded_flight_core as core;
pub use embedded_flight_motors as motors;
pub use embedded_flight_scheduler as scheduler;

pub mod copter;
pub use copter::MultiCopter;

pub fn land_run_vertical_control(
    pos_control: &mut PositionController,
    land_alt_low: i16,
    land_speed: i16,
    land_speed_high: i16,
    alt_above_ground_cm: f32,
    land_complete_maybe: bool,
    pause_descent: bool,
    dt: f32,
) {
    let mut cmb_rate = 0.;
    let mut ignore_descent_limit = false;
    let land_alt_low = land_alt_low.max(100) as f32;

    if !pause_descent {
        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = land_alt_low > alt_above_ground_cm || land_complete_maybe;

        let mut max_land_descent_velocity = if land_speed_high > 0 {
            -land_speed_high as _
        } else {
            pos_control.vel_max_down_cms
        };

        // Don't speed up for landing.
        max_land_descent_velocity = max_land_descent_velocity.min(-(land_speed.abs()) as _);

        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = sqrt_controller(
            land_alt_low - alt_above_ground_cm,
            pos_control.p_pos_z.kp,
            pos_control.accel_max_z_cmss,
            dt,
        );

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = cmb_rate
            .max(max_land_descent_velocity)
            .min(-(land_speed.abs()) as _);
    }

    pos_control.land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control.update_z_control();
}
