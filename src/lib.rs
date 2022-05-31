//! ```
//! use embedded_flight::control::PositionController;
//! use nalgebra::Vector3;
//!
//! // Current state of the craft
//! let position = Vector3::zeros();
//! let velocity = Vector3::new(1., 1., 1.);
//! let attitude = Vector3::zeros();
//! let gyro = Vector3::zeros();
//!
//! // Jerk moments of position and velocity to apply
//! let position_cmd = Vector3::new(1., 1., 1.);
//! let velocity_cmd = Vector3::new(0., 0., 1.);
//!
//! let controller = PositionController::default();
//!
//! let moment = controller.position_control(
//!     position_cmd,
//!     velocity_cmd,
//!     position,
//!     velocity,
//!     attitude,
//!     gyro
//! );
//! dbg!(moment);
//! ```

#![cfg_attr(not(test), no_std)]

pub use embedded_flight_control as control;
pub use embedded_flight_core as core;
pub use embedded_flight_motors as motors;

pub mod copter;
pub use copter::MultiCopter;

use nalgebra::Vector3;
