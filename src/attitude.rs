use nalgebra::{Matrix2, Rotation3, Vector2, Vector3};
use crate::pid_config;

pub struct AttitudeController {
    gravity: f32,
    drone_mass_kg: f32,
    altitude_k_p: f32,
    altitude_k_d: f32,
    roll_pitch_k_p_roll: f32,
    roll_pitch_k_p_pitch: f32,
}

impl Default for AttitudeController {
    fn default() -> Self {
        let (altitude_k_p, altitude_k_d) = pid_config(1., 0.85);

        Self {
            gravity: 9.81,
            drone_mass_kg: 0.5,
            altitude_k_p,
            altitude_k_d,
            roll_pitch_k_p_roll: 7.,
            roll_pitch_k_p_pitch: 7.,
        }
    }
}

impl AttitudeController {
    /// Calculate the vertical acceleration (thrust) command.
    pub fn altitude_control(
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
}