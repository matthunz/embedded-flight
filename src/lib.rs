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

#![no_std]

use nalgebra::Vector3;

use control::{Moment, PositionController};
pub use embedded_flight_control as control;

pub use embedded_flight_motors as motors;
use motors::{esc::ESC, MotorMatrix};

pub struct Copter<E, const N: usize> {
    pub controller: PositionController,
    pub motors: MotorMatrix<E, f32, N>,
    pub max_thrust: f32,
    pub max_radian_rate: f32,
}

impl<E, const N: usize> Copter<E, N>
where
    E: ESC<Output = f32>,
{
    pub fn output_moment(&mut self, moment: Moment) {
        self.motors.output(
            moment.attitude / self.max_radian_rate,
            Vector3::new(0., 0., moment.thrust / self.max_thrust),
        );
    }
}
