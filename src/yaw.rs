use std::f32::consts::PI;

pub struct YawController {
    yaw_k_p: f32,
}

impl Default for YawController {
    fn default() -> Self {
        Self { yaw_k_p: 4.5 }
    }
}

impl YawController {
    /// Calculate the target yaw-rate in radians/second.
    pub fn yaw_control(&self, yaw_cmd: f32, yaw: f32) -> f32 {
        let mut yaw_error = yaw_cmd - yaw;
        if yaw_error > PI {
            yaw_error = yaw_error - 2. * PI;
        } else if yaw_error < -PI {
            yaw_error = yaw_error + 2. * PI;
        };

        self.yaw_k_p * yaw_error
    }
}
