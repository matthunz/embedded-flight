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

#![no_std]

use super::Error;
use embedded_time::duration::Microseconds;

/// An event containing the current time, available time, and state for a task.
pub struct Event<'a, T> {
    /// The state of the system running the scheduler.
    pub state: &'a mut T,

    /// The current time in microseconds.
    pub now: Microseconds<u32>,

    /// The available to run this task time (in microseconds).
    pub available: Microseconds<u32>,
}

type TaskFn<T, E> = fn(Event<'_, T>) -> Result<(), E>;

/// A task to run at specific frequency
pub struct Task<T, E = Error> {
    /// The function to run.
    pub f: TaskFn<T, E>,

    /// The desired frequency (in hz) to run the task.
    pub hz: f32,

    /// The max time for this task (in microseconds).
    pub max_time_micros: u16,

    /// Determines if this task should be run every time the scheduler loops.
    pub is_high_priority: bool,

    /// The last tick this task was ran.
    pub last_run: u16,
}

impl<T, E> Task<T, E> {
    /// Create a new task from the function to run.
    pub fn new(f: TaskFn<T, E>) -> Self {
        Self {
            f,
            hz: 0.,
            max_time_micros: 0,
            is_high_priority: false,
            last_run: 0,
        }
    }

    /// Create a new high priority task from the function to run.
    pub fn high_priority(f: TaskFn<T, E>) -> Self {
        Self::new(f).with_high_priority(true)
    }

    /// Builder method to set `hz` and return `self`
    pub fn with_hz(mut self, hz: f32) -> Self {
        self.hz = hz;
        self
    }

    /// Builder method to set `max_time_micros` and return `self`
    pub fn with_max_time(mut self, micros: u16) -> Self {
        self.max_time_micros = micros;
        self
    }

    /// Builder method to set `is_high_priority` and return `self`
    pub fn with_high_priority(mut self, is_high_priority: bool) -> Self {
        self.is_high_priority = is_high_priority;
        self
    }

    /// Calculate the desired ticks between each run of the task
    pub fn ticks(&self, loop_rate_hz: i16) -> i16 {
        // A 0hz task should be ran at the rate of the scheduler loop
        loop_rate_hz.checked_div(self.hz as i16).unwrap_or(1)
    }

    /// If this task is ready returns the ticks elapsed since the last run.
    /// Otherwise this returns `None`.
    pub fn ready(&self, current_tick: u16, ticks: i16) -> Option<u16> {
        let dt = current_tick - self.last_run;

        if (dt as i16) >= ticks {
            Some(dt)
        } else {
            None
        }
    }

    /// Run this task at the current tick.
    pub fn run(&mut self, state: Event<'_, T>, tick: u16) -> Result<(), E> {
        (self.f)(state)?;

        // Record the tick counter when we ran
        // This determines when we next run the event
        self.last_run = tick;

        Ok(())
    }
}
