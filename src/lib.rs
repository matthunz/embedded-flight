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
