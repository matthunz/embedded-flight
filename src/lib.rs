use std::f32::consts::PI;

use nalgebra::{Matrix2, Rotation3, Vector2, Vector3};

pub struct Controller {
    gravity: f32,
    moi: Vector3<f32>,
    max_torque: f32,
    drone_mass_kg: f32,
    body_rate_k_p: Vector3<f32>,
    altitude_k_p: f32,
    altitude_k_d: f32,
    roll_pitch_k_p_roll: f32,
    roll_pitch_k_p_pitch: f32,
    yaw_k_p: f32,
    lateral_k_p: f32,
    lateral_k_d: f32,
}

impl Default for Controller {
    fn default() -> Self {
        let (altitude_k_p, altitude_k_d) = pid_config(1., 0.85);
        let (lateral_k_p, lateral_k_d) = pid_config(0.28, 0.95);

        Self {
            gravity: 9.81,
            moi: Vector3::new(0.005, 0.005, 0.01),
            max_torque: 1.,
            body_rate_k_p: Vector3::new(20., 20., 5.),
            drone_mass_kg: 0.5,
            altitude_k_p,
            altitude_k_d,
            roll_pitch_k_p_roll: 7.,
            roll_pitch_k_p_pitch: 7.,
            yaw_k_p: 4.5,
            lateral_k_p,
            lateral_k_d,
        }
    }
}

impl Controller {
    pub fn trajectory_control(
        &self,
        from_position: Vector3<f32>,
        from_time: f32,
        to_position: Vector3<f32>,
        to_time: f32,
        current_time: f32,
    ) -> (Vector3<f32>, Vector3<f32>) {
        let position_cmd = (to_position - from_position) * (current_time - from_time)
            / (to_time - from_time)
            + from_position;
        let velocity_cmd = (to_position - from_position) / (to_time - from_time);

        (position_cmd, velocity_cmd)
    }

    pub fn lateral_position_control(
        &self,
        local_position_cmd: Vector2<f32>,
        local_velocity_cmd: Vector2<f32>,
        local_position: Vector2<f32>,
        local_velocity: Vector2<f32>,
        acceleration_ff: Vector2<f32>,
    ) -> Vector2<f32> {
        let err_p = local_position_cmd - local_position;
        let err_dot = local_velocity_cmd - local_velocity;

        self.lateral_k_p * err_p + self.lateral_k_d * err_dot + acceleration_ff
    }

    /// Calculate a tuple containing the desired roll, pitch, and yaw moment commands and thrust in Newtons*meters.
    pub fn body_rate_target(
        &self,
        altitude_cmd: f32,
        vertical_velocity_cmd: f32,
        altitude: f32,
        vertical_velocity: f32,
        attitude: Vector3<f32>,
        acceleration_ff: f32,
        acceleration_cmd: Vector2<f32>,
        yaw_cmd: f32,
    ) -> (Vector3<f32>, f32) {
        let thrust_cmd = self.attitude_control(
            altitude_cmd,
            vertical_velocity_cmd,
            altitude,
            vertical_velocity,
            attitude,
            acceleration_ff,
        );

        let roll_pitch_rate_cmd = self.roll_pitch_control(acceleration_cmd, attitude, thrust_cmd);
        let yaw_rate_cmd = self.yaw_control(yaw_cmd, attitude[2]);

        (
            Vector3::new(roll_pitch_rate_cmd[0], roll_pitch_rate_cmd[1], yaw_rate_cmd),
            thrust_cmd,
        )
    }

    /// Calculate the vertical acceleration (thrust) command.
    pub fn attitude_control(
        &self,
        altitude_cmd: f32,
        vertical_velocity_cmd: f32,
        altitude: f32,
        vertical_velocity: f32,
        attitude: Vector3<f32>,
        acceleration_ff: f32,
    ) -> f32 {
        let z_err = altitude_cmd - altitude;
        let z_err_dot = vertical_velocity_cmd - vertical_velocity;
        let b_z = Rotation3::from_euler_angles(attitude[0], attitude[1], attitude[2])[(2, 2)];

        let u_1 = self.altitude_k_p * z_err + self.altitude_k_d * z_err_dot + acceleration_ff;
        let acc = (u_1 - self.gravity) / b_z;

        self.drone_mass_kg * acc
    }

    /// Calculate the roll-rate and pitch-rate commands in the body frame in radians/second.
    pub fn roll_pitch_control(
        &self,
        acceleration_cmd: Vector2<f32>,
        attitude: Vector3<f32>,
        thrust_cmd: f32,
    ) -> Vector2<f32> {
        if thrust_cmd > 0. {
            let c = -thrust_cmd / self.drone_mass_kg;

            let (b_x_c, b_y_c) = {
                let b_c = (acceleration_cmd / c).map(|n| n.min(1f32.max(-1.)));
                (b_c[0], b_c[1])
            };

            let rot_mat = Rotation3::from_euler_angles(attitude[0], attitude[1], attitude[2]);

            let b_x = rot_mat[(0, 2)];
            let b_x_err = b_x_c - b_x;
            let b_x_p_term = self.roll_pitch_k_p_roll * b_x_err;

            let b_y = rot_mat[(1, 2)];
            let b_y_err = b_y_c - b_y;
            let b_y_p_term = self.roll_pitch_k_p_pitch * b_y_err;

            let b_x_commanded_dot = b_x_p_term;
            let b_y_commanded_dot = b_y_p_term;

            let rot_mat1 = Matrix2::new(
                rot_mat[(1, 0)],
                -rot_mat[(0, 0)],
                rot_mat[(1, 1)],
                -rot_mat[(0, 1)],
            ) / rot_mat[(2, 2)];

            let rot_rate = rot_mat1 * Vector2::new(b_x_commanded_dot, b_y_commanded_dot);
            let p_c = rot_rate[0];
            let q_c = rot_rate[1];
            Vector2::new(p_c, q_c)
        } else {
            Vector2::zeros()
        }
    }

    /// Generate the roll, pitch, yaw moment commands in the body frame in Newtons*meters
    pub fn body_rate_control(
        &self,
        body_rate_cmd: Vector3<f32>,
        body_rate: Vector3<f32>,
    ) -> Vector3<f32> {
        let taus = mul_array(
            self.moi,
            mul_array(self.body_rate_k_p, body_rate_cmd - body_rate),
        );
        let taus_mod = taus.norm();

        if taus_mod > self.max_torque {
            taus * self.max_torque / taus_mod
        } else {
            taus
        }
    }

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

/// Calculate the vertical acceleration (thrust) command.
pub fn attitude_control(
    altitude_cmd: f32,
    vertical_velocity_cmd: f32,
    altitude: f32,
    vertical_velocity: f32,
    attitude: f32,
    acceleration_ff: f32,
    gravity: f32,
    drone_mass_kg: f32,
    altitude_k_p: f32,
    altitude_k_d: f32,
) -> f32 {
    let z_err = altitude_cmd - altitude;
    let z_err_dot = vertical_velocity_cmd - vertical_velocity;
    let b_z = Rotation3::from_euler_angles(attitude, attitude, attitude)[(2, 2)];

    let u_1 = altitude_k_p * z_err + altitude_k_d * z_err_dot + acceleration_ff;
    let acc = (u_1 - gravity) / b_z;

    drone_mass_kg * acc
}

/// Calculate the roll-rate and pitch-rate commands in the body frame in radians/second.
pub fn roll_pitch_control(
    acceleration_cmd: Vector2<f32>,
    attitude: f32,
    thrust_cmd: f32,
    drone_mass_kg: f32,
    roll_pitch_k_p_roll: f32,
    roll_pitch_k_p_pitch: f32,
) -> Vector2<f32> {
    if thrust_cmd > 0. {
        let c = -thrust_cmd / drone_mass_kg;

        let (b_x_c, b_y_c) = {
            let b_c = (acceleration_cmd / c).map(|n| n.min(1f32.max(-1.)));
            (b_c[0], b_c[1])
        };

        let rot_mat = Rotation3::from_euler_angles(attitude, attitude, attitude);

        let b_x = rot_mat[(0, 2)];
        let b_x_err = b_x_c - b_x;
        let b_x_p_term = roll_pitch_k_p_roll * b_x_err;

        let b_y = rot_mat[(1, 2)];
        let b_y_err = b_y_c - b_y;
        let b_y_p_term = roll_pitch_k_p_pitch * b_y_err;

        let b_x_commanded_dot = b_x_p_term;
        let b_y_commanded_dot = b_y_p_term;

        let rot_mat1 = Matrix2::new(
            rot_mat[(1, 0)],
            -rot_mat[(0, 0)],
            rot_mat[(1, 1)],
            -rot_mat[(0, 1)],
        ) / rot_mat[(2, 2)];

        let rot_rate = rot_mat1 * Vector2::new(b_x_commanded_dot, b_y_commanded_dot);
        let p_c = rot_rate[0];
        let q_c = rot_rate[1];
        Vector2::new(p_c, q_c)
    } else {
        Vector2::zeros()
    }
}

// From `t_rise` and `delta` returns kp and kd
fn pid_config(t_rise: f32, delta: f32) -> (f32, f32) {
    let w = 1. / (1.57 * t_rise);
    (w * w, 2. * delta * w)
}

fn mul_array(lhs: Vector3<f32>, rhs: Vector3<f32>) -> Vector3<f32> {
    lhs.zip_map(&rhs, |l, r| l * r)
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
