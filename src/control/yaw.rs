use core::f32::consts::PI;

use pid_controller::{error, P};

#[derive(Clone, Copy, Debug, Default)]
pub struct YawController {
    p: P,
}

impl YawController {
    /// Calculate the target yaw-rate in radians/second.
    pub fn yaw_control(&self, yaw_cmd: f32, yaw: f32) -> f32 {
        let mut yaw_error = error(yaw_cmd, yaw);
        if yaw_error > PI {
            yaw_error = yaw_error - 2. * PI;
        } else if yaw_error < -PI {
            yaw_error = yaw_error + 2. * PI;
        };

        self.p.control_with_error(yaw_error)
    }
}
