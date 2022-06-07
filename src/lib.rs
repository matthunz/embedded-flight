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

pub mod hal;
pub use hal::{Sensors, ESC};

pub mod plane;

pub mod scheduler;
pub use scheduler::Scheduler;
