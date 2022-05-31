use super::AttitudeController;
use crate::PID;
use embedded_flight_core::MotorOutput;
use nalgebra::Vector3;

/// ```
/// use embedded_flight_control::MultiCopterAttitudeController;
///
/// let mut controller = MultiCopterAttitudeController::default();
///
/// // Input the desired attitude and angular velocity with the current attitude
/// controller.attitude_controller.input(
///     Quaternion::default(),
///     Vector3::default(),
///     Quaternion::default(),
/// );
///
/// // Output the control to the motors with the current gyroscope data.
/// let output = controller.motor_output(Vector3::default(), 1);
/// dbg!(output);
/// ```

pub struct MultiCopterAttitudeController {
    // The angular velocity (in radians per second) in the body frame.
    pub roll_rate: PID,
    pub pitch_rate: PID,
    pub yaw_rate: PID,
    pub attitude_controller: AttitudeController,
}

impl Default for MultiCopterAttitudeController {
    fn default() -> Self {
        const AC_ATC_MULTI_RATE_RP_P: f32 = 0.135;
        const AC_ATC_MULTI_RATE_RP_I: f32 = 0.135;
        const AC_ATC_MULTI_RATE_RP_D: f32 = 0.0036;
        const AC_ATC_MULTI_RATE_RP_IMAX: f32 = 0.5;
        const AC_ATC_MULTI_RATE_RP_FILT_HZ: f32 = 20.;
        const AC_ATC_MULTI_RATE_YAW_P: f32 = 0.180;
        const AC_ATC_MULTI_RATE_YAW_I: f32 = 0.018;
        const AC_ATC_MULTI_RATE_YAW_D: f32 = 0.;
        const AC_ATC_MULTI_RATE_YAW_IMAX: f32 = 0.5;
        const AC_ATC_MULTI_RATE_YAW_FILT_HZ: f32 = 2.5;

        // 400 hz
        let dt = 0.0025;

        Self {
            roll_rate: PID::new(
                AC_ATC_MULTI_RATE_RP_P,
                AC_ATC_MULTI_RATE_RP_I,
                AC_ATC_MULTI_RATE_RP_D,
                0.,
                AC_ATC_MULTI_RATE_RP_IMAX,
                AC_ATC_MULTI_RATE_RP_FILT_HZ,
                0.,
                AC_ATC_MULTI_RATE_RP_FILT_HZ,
                dt,
            ),
            pitch_rate: PID::new(
                AC_ATC_MULTI_RATE_RP_P,
                AC_ATC_MULTI_RATE_RP_I,
                AC_ATC_MULTI_RATE_RP_D,
                0.,
                AC_ATC_MULTI_RATE_RP_IMAX,
                AC_ATC_MULTI_RATE_RP_FILT_HZ,
                0.,
                AC_ATC_MULTI_RATE_RP_FILT_HZ,
                dt,
            ),
            yaw_rate: PID::new(
                AC_ATC_MULTI_RATE_YAW_P,
                AC_ATC_MULTI_RATE_YAW_I,
                AC_ATC_MULTI_RATE_YAW_D,
                0.,
                AC_ATC_MULTI_RATE_YAW_IMAX,
                AC_ATC_MULTI_RATE_RP_FILT_HZ,
                AC_ATC_MULTI_RATE_YAW_FILT_HZ,
                0.,
                dt,
            ),
            attitude_controller: Default::default(),
        }
    }
}

impl MultiCopterAttitudeController {
    /// Calculate the motor output of the controller (in -1 ~ +1).
    pub fn motor_output(&mut self, gyro: Vector3<f32>, now_ms: u32) -> MotorOutput<f32> {
        self.motor_output_with_limit(gyro, now_ms, [false; 3])
    }

    /// Calculate the motor output of the controller (in -1 ~ +1) with an optional limit for roll, pitch, and yaw.
    pub fn motor_output_with_limit(
        &mut self,
        gyro: Vector3<f32>,
        now_ms: u32,
        limit: [bool; 3],
    ) -> MotorOutput<f32> {
        // Move throttle vs attitude mixing towards desired.
        // Called from here because this is conveniently called on every iteration
        self.update_throttle_rpy_mix();

        self.attitude_controller.ang_vel_body += self.attitude_controller.sysid_ang_vel_body;

        let roll = self.roll_rate.update(
            self.attitude_controller.ang_vel_body[0],
            gyro[0],
            limit[0],
            now_ms,
        ) + self.attitude_controller.actuator_sysid[0];
        let pitch = self.roll_rate.update(
            self.attitude_controller.ang_vel_body[1],
            gyro[1],
            limit[1],
            now_ms,
        ) + self.attitude_controller.actuator_sysid[1];
        let yaw = self.roll_rate.update(
            self.attitude_controller.ang_vel_body[2],
            gyro[2],
            limit[2],
            now_ms,
        ) + self.attitude_controller.actuator_sysid[2];

        let roll_ff = self.roll_rate.feed_forward();
        let pitch_ff = self.pitch_rate.feed_forward();
        let yaw_ff = self.yaw_rate.feed_forward() * self.attitude_controller.feed_forward_scalar;

        self.attitude_controller.sysid_ang_vel_body = Vector3::zeros();
        self.attitude_controller.actuator_sysid = Vector3::zeros();

        // TODO control_monitor_update();

        MotorOutput::new(
            Vector3::new(roll, pitch, yaw),
            Vector3::new(roll_ff, pitch_ff, yaw_ff),
        )
    }

    // Slew set_throttle_rpy_mix to requested value
    pub fn update_throttle_rpy_mix(&mut self) {
        // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
        if self.attitude_controller.throttle_rpy_mix
            < self.attitude_controller.throttle_rpy_mix_desired
        {
            // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
            self.attitude_controller.throttle_rpy_mix += (2. * self.attitude_controller.dt).min(
                self.attitude_controller.throttle_rpy_mix_desired
                    - self.attitude_controller.throttle_rpy_mix,
            );
        } else if self.attitude_controller.throttle_rpy_mix
            > self.attitude_controller.throttle_rpy_mix_desired
        {
            // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
            self.attitude_controller.throttle_rpy_mix -= (0.5 * self.attitude_controller.dt).min(
                self.attitude_controller.throttle_rpy_mix
                    - self.attitude_controller.throttle_rpy_mix_desired,
            );
        }

        self.attitude_controller.throttle_rpy_mix = self
            .attitude_controller
            .throttle_rpy_mix
            .max(0.1)
            .min(self.attitude_controller.attitude_control_max);
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, Vector3};

    use super::MultiCopterAttitudeController;

    #[test]
    fn f() {}
}
