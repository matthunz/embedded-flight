#![no_std]

use nalgebra::{Vector2, Vector3};

mod attitude;
pub use attitude::AttitudeController;

mod body_rate;
pub use body_rate::BodyRateController;

mod lateral_pos;
pub use lateral_pos::LateralPositionController;

mod yaw;
pub use yaw::YawController;

pub struct Moment {
    pub attitude: Vector3<f32>,
    pub thrust: f32,
}

impl Moment {
    pub fn new(moment: Vector3<f32>, thrust: f32) -> Self {
        Self {
            attitude: moment,
            thrust,
        }
    }
}

pub struct Trajectory {
    pub position_cmd: Vector3<f32>,
    pub velocity_cmd: Vector3<f32>,
}

impl Trajectory {
    pub fn new(position_cmd: Vector3<f32>, velocity_cmd: Vector3<f32>) -> Self {
        Self {
            position_cmd,
            velocity_cmd,
        }
    }

    /// Calculate a commanded position and velocity based on the trajectory.
    pub fn calculate(
        from_position: Vector3<f32>,
        from_time: f32,
        to_position: Vector3<f32>,
        to_time: f32,
        current_time: f32,
    ) -> Self {
        let position_cmd = (to_position - from_position) * (current_time - from_time)
            / (to_time - from_time)
            + from_position;
        let velocity_cmd = (to_position - from_position) / (to_time - from_time);

        Self::new(position_cmd, velocity_cmd)
    }
}

pub struct PositionController {
    pub attitude_controller: AttitudeController,
    pub body_rate_controller: BodyRateController,
    pub yaw_controller: YawController,
    pub lateral_position: LateralPositionController,
    pub acceleration_ff: Vector2<f32>,
}

impl Default for PositionController {
    fn default() -> Self {
        Self {
            attitude_controller: AttitudeController::default(),
            yaw_controller: YawController::default(),
            body_rate_controller: BodyRateController::default(),
            lateral_position: LateralPositionController::default(),
            acceleration_ff: Vector2::zeros(),
        }
    }
}

impl PositionController {
    /// Calculate the desired roll, pitch, yaw, and thrust moment commands from a trajectory in Newtons*meters.
    pub fn trajectory_control(
        &self,
        trajectory: Trajectory,
        local_position: Vector3<f32>,
        local_velocity: Vector3<f32>,
        attitude: Vector3<f32>,
        gyro: Vector3<f32>,
    ) -> Moment {
        self.position_control(
            trajectory.position_cmd,
            trajectory.velocity_cmd,
            local_position,
            local_velocity,
            attitude,
            gyro,
        )
    }

    /// Calculate the desired roll, pitch, yaw, and thrust moment commands in Newtons*meters.
    pub fn position_control(
        &self,
        local_position_cmd: Vector3<f32>,
        local_velocity_cmd: Vector3<f32>,
        local_position: Vector3<f32>,
        local_velocity: Vector3<f32>,
        attitude: Vector3<f32>,
        gyro: Vector3<f32>,
    ) -> Moment {
        let acceleration_cmd = self.lateral_position.lateral_position_control(
            Vector2::new(local_position_cmd[0], local_position_cmd[1]),
            Vector2::new(local_velocity_cmd[0], local_velocity_cmd[1]),
            Vector2::new(local_position[0], local_position[1]),
            Vector2::new(local_velocity[0], local_velocity[1]),
            self.acceleration_ff,
        );
        let thrust_cmd = self.attitude_controller.altitude_control(
            -local_position_cmd[2],
            -local_velocity_cmd[2],
            -local_position[2],
            -local_velocity[2],
            attitude,
            9.81,
        );

        let roll_pitch_rate_cmd =
            self.attitude_controller
                .roll_pitch_control(acceleration_cmd, attitude, thrust_cmd);
        let yaw_rate_cmd = self
            .yaw_controller
            .yaw_control(local_position_cmd[2], attitude[2]);

        let body_rate_cmd =
            Vector3::new(roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yaw_rate_cmd);
        let moment = self
            .body_rate_controller
            .body_rate_control(body_rate_cmd, gyro);

        Moment::new(moment, thrust_cmd)
    }
}

// From `t_rise` and `delta` returns kp and kd
fn pid_config(t_rise: f32, delta: f32) -> (f32, f32) {
    let w = 1. / (1.57 * t_rise);
    (w * w, 2. * delta * w)
}
