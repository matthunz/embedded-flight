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

mod pid;
pub use pid::PID;

mod slew_limiter;
pub use slew_limiter::SlewLimiter;

pub struct MultiCopterAttitudeController {
    ang_vel_body: Vector3<f32>,
    actuator_sysid: Vector3<f32>,
    sysid_ang_vel_body: Vector3<f32>,
    roll_rate: PID,
    pitch_rate: PID,
    yaw_rate: PID,
    feed_forward_scalar: f32,
    throttle_rpy_mix: f32,
    throttle_rpy_mix_desired: f32,
    attitude_control_max: f32,
    dt: f32,
}

impl MultiCopterAttitudeController {
    /// Returns a tuple containing the desired pitch, roll, and yaw control and feed forward in -1 ~ +1.
    pub fn rate_control(
        &mut self,
        gyro: Vector3<f32>,
        limit: [bool; 3],
        now_ms: u32,
    ) -> (Vector3<f32>, Vector3<f32>) {
        // Move throttle vs attitude mixing towards desired.
        // Called from here because this is conveniently called on every iteration
        self.update_throttle_rpy_mix();

        self.ang_vel_body += self.sysid_ang_vel_body;

        let roll = self
            .roll_rate
            .update_all(self.ang_vel_body[0], gyro[0], limit[0], now_ms)
            + self.actuator_sysid[0];
        let pitch = self
            .roll_rate
            .update_all(self.ang_vel_body[1], gyro[1], limit[1], now_ms)
            + self.actuator_sysid[1];
        let yaw = self
            .roll_rate
            .update_all(self.ang_vel_body[2], gyro[2], limit[2], now_ms)
            + self.actuator_sysid[2];

        let roll_ff = self.roll_rate.feed_forward();
        let pitch_ff = self.pitch_rate.feed_forward();
        let yaw_ff = self.yaw_rate.feed_forward() * self.feed_forward_scalar;

        self.sysid_ang_vel_body = Vector3::zeros();
        self.actuator_sysid = Vector3::zeros();

        // TODO control_monitor_update();

        (
            Vector3::new(roll, pitch, yaw),
            Vector3::new(roll_ff, pitch_ff, yaw_ff),
        )
    }

    // update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
    pub fn update_throttle_rpy_mix(&mut self) {
        // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
        if (self.throttle_rpy_mix < self.throttle_rpy_mix_desired) {
            // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
            self.throttle_rpy_mix +=
                (2. * self.dt).min(self.throttle_rpy_mix_desired - self.throttle_rpy_mix);
        } else if (self.throttle_rpy_mix > self.throttle_rpy_mix_desired) {
            // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
            self.throttle_rpy_mix -=
                (0.5 * self.dt).min(self.throttle_rpy_mix - self.throttle_rpy_mix_desired);
        }
        self.throttle_rpy_mix = self
            .throttle_rpy_mix
            .max(0.1)
            .min(self.attitude_control_max);
    }
}

fn control_monitor_filter_pid(value: f32, rms: f32) -> f32 {
    let filter_constant = 0.99;
    filter_constant * rms + (1. - filter_constant) * (value * value)
}

#[derive(Clone, Debug)]
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

/// The trajectory for a craft with a position and velocity commanded moment.
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
    pub fn from_position(
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

/// Cascaded PID flight position controller
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
