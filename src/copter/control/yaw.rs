use core::f32::consts::PI;
use pid_controller::{error, P};

/// Proportional yaw controller
#[derive(Clone, Copy, Debug, Default)]
pub struct YawController {
    /// The proportional control for yaw-rate
    pub p: P<f32>,
}

impl YawController {
    /// Calculate the target yaw-rate in radians/second to reach the commanded yaw from the current yaw (in radians).
    pub fn yaw_control(&self, yaw_cmd: f32, yaw: f32) -> f32 {
        // 1. Calculate the error in yaw between the commanded and current yaw.
        let mut yaw_error = error(yaw_cmd, yaw);

        // 2. Wrap the yaw error between -PI and PI
        if yaw_error > PI {
            yaw_error = yaw_error - 2. * PI;
        } else if yaw_error < -PI {
            yaw_error = yaw_error + 2. * PI;
        };

        // 3. Output the proportionally controlled yaw rate from the error
        self.p.control_with_error(yaw_error)
    }
}
