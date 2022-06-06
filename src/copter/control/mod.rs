mod altitude;
pub use altitude::AltitudeController;

mod attitude;
pub use attitude::AttitudeController;

mod body_rate;
pub use body_rate::BodyRateController;

mod lateral_pos;
pub use lateral_pos::LateralPositionController;

mod motor;
pub use motor::{MotorControl, QuadMotorControl};

mod yaw;
pub use yaw::YawController;

use nalgebra::{Vector2, Vector3};

#[derive(Clone, Debug, Default)]
pub struct Controller {
    pub altitude: AltitudeController,
    pub attitude: AttitudeController,
    pub body_rate: BodyRateController,
    pub lateral_position: LateralPositionController,
    pub yaw: YawController,
}

impl Controller {
    /// Calculate the desired roll, pitch and yaw torque (in Nm) with the thrust acceleration (in m/s^2).
    pub fn position_control(
        &self,
        local_position_cmd: Vector3<f32>,
        local_velocity_cmd: Vector3<f32>,
        local_position: Vector3<f32>,
        local_velocity: Vector3<f32>,
        attitude: Vector3<f32>,
        gyro: Vector3<f32>,
        moment_of_inertia: Vector3<f32>,
    ) -> (Vector3<f32>, f32) {
        self.position_control_with_feed_forward(
            local_position_cmd,
            local_velocity_cmd,
            local_position,
            local_velocity,
            attitude,
            gyro,
            moment_of_inertia,
            Vector2::zeros(),
        )
    }

    /// Calculate the desired roll, pitch and yaw torque (in Nm) with the thrust acceleration (in m/s^2).
    pub fn position_control_with_feed_forward(
        &self,
        local_position_cmd: Vector3<f32>,
        local_velocity_cmd: Vector3<f32>,
        local_position: Vector3<f32>,
        local_velocity: Vector3<f32>,
        attitude: Vector3<f32>,
        gyro: Vector3<f32>,
        moment_of_inertia: Vector3<f32>,
        acceleration_ff: Vector2<f32>,
    ) -> (Vector3<f32>, f32) {
        let acceleration_cmd = self.lateral_position.lateral_position_control(
            Vector2::new(local_position_cmd[0], local_position_cmd[1]),
            Vector2::new(local_velocity_cmd[0], local_velocity_cmd[1]),
            Vector2::new(local_position[0], local_position[1]),
            Vector2::new(local_velocity[0], local_velocity[1]),
            acceleration_ff,
        );
        let thrust_acceleration = self.altitude.acceleration(
            -local_position_cmd[2],
            -local_velocity_cmd[2],
            -local_position[2],
            -local_velocity[2],
            attitude,
            9.81,
        );

        let roll_pitch_rate_cmd =
            self.attitude
                .roll_pitch_control(acceleration_cmd, attitude, thrust_acceleration);
        let yaw_rate_cmd = self.yaw.yaw_control(local_position_cmd[2], attitude[2]);

        let body_rate_cmd =
            Vector3::new(roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yaw_rate_cmd);
        let moment = self
            .body_rate
            .body_rate_control(body_rate_cmd, gyro, moment_of_inertia);

        (moment, thrust_acceleration)
    }
}
