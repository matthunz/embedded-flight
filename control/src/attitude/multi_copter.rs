use super::AttitudeController;
use crate::PID;
use embedded_flight_core::MotorOutput;
use nalgebra::Vector3;

pub struct MultiCopterAttitudeController {
    // The angular velocity (in radians per second) in the body frame.
    pub roll_rate: PID,
    pub pitch_rate: PID,
    pub yaw_rate: PID,
    pub attitude_controller: AttitudeController,
}

impl MultiCopterAttitudeController {
    /// Returns a tuple containing the desired pitch, roll, and yaw control and feed forward in -1 ~ +1.
    pub fn rate_control(
        &mut self,
        gyro: Vector3<f32>,
        limit: [bool; 3],
        now_ms: u32,
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
