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

pub use embedded_flight_control as control;
pub use embedded_flight_core as core;
pub use embedded_flight_motors as motors;
pub use embedded_flight_scheduler as scheduler;

pub mod copter;
pub use copter::MultiCopter;
use nalgebra::{UnitQuaternion, Vector3};

pub struct IMU {
    delta_angle_dt: f32,
    delta_velocity_dt: f32,
    attitude: UnitQuaternion<f32>,
    delta_velocity: Vector3<f32>,
}

impl IMU {
    // Acceleration in m/s/s
    pub fn down_sample(
        &mut self,
        delta_angle: Vector3<f32>,
        acceleration: Vector3<f32>,
        delta_angle_dt: f32,
        delta_velocity_dt: f32,
    ) {
        let delta_velocity = acceleration * delta_velocity_dt;
        self.down_sample_with_velocity(
            delta_angle,
            delta_velocity,
            delta_angle_dt,
            delta_velocity_dt,
        )
    }

    pub fn down_sample_with_velocity(
        &mut self,
        delta_angle: Vector3<f32>,
        delta_velocity: Vector3<f32>,
        delta_angle_dt: f32,
        delta_velocity_dt: f32,
    ) {
        // Accumulate the measurement time interval for the delta velocity and angle data
        self.delta_angle_dt += delta_angle_dt;
        self.delta_velocity_dt += delta_velocity_dt;

        // Rotate quaternion attitude from previous to new and normalize.
        // Accumulation using quaternions prevents introduction of coning errors due to down-sampling
        self.attitude = self.attitude * UnitQuaternion::new(delta_angle);

        // Rotate the latest delta velocity into body frame at the start of accumulation
        let delta_rotation = self.attitude.to_rotation_matrix();

        // Apply the delta velocity to the delta velocity accumulator
        self.delta_velocity += delta_rotation * delta_velocity;
    }
}
