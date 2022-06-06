//! # embedded-flight
//! A `#![no_std]` flight software library for embedded rust
//!
//! # Generic components
//! [`Scheduler`] real-time scheduler to run tasks at desired frequencies. 
//! 
//! [`hal`] contains the hardware abstraction layer.
//! 
//! 
//! 
//! # Multi-copter components
//! [`Copter`] a high level position controller for a multi-copter
//! (see [`QuadCopter`] for a quad-motor implementation)
//! 
//! [`control`](copter::control) contains the low level flight controllers.
//! 
//! [`motor`](copter::control::motor) contains the low level motor control.
pub mod copter;
pub use copter::{Copter, QuadCopter};

pub mod hal;
pub use hal::{Sensors, ESC};

pub mod scheduler;
pub use scheduler::Scheduler;
