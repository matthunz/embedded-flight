use nalgebra::{Matrix2, Rotation3, Vector2};

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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
