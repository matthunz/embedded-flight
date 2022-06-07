use nalgebra::{Matrix2, Rotation3, Vector2, Vector3};
use pid_controller::P;

#[derive(Clone, Debug)]
pub struct AttitudeController {
    drone_mass_kg: f32,
    roll: P<f32>,
    pitch: P<f32>,
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self {
            drone_mass_kg: 0.5,
            roll: P::default(),
            pitch: P::default(),
        }
    }
}

impl AttitudeController {
    /// Calculate the roll-rate and pitch-rate commands in the body frame in radians/second.
    pub fn roll_pitch_control(
        &self,
        acceleration_cmd: Vector2<f32>,
        attitude: Vector3<f32>,
        thrust_cmd: f32,
    ) -> Vector2<f32> {
        if thrust_cmd > 0. {
            // Calculate the collective acceleration.
            let c = -thrust_cmd / self.drone_mass_kg;

            let (b_x_c, b_y_c) = {
                let b_c = (acceleration_cmd / c).map(|n| n.min(1f32.max(-1.)));
                (b_c[0], b_c[1])
            };

            // Calculate the rotation matrix of the current attitude.
            let rot_mat = Rotation3::from_euler_angles(attitude[0], attitude[1], attitude[2]);

            let b_x = rot_mat[(0, 2)];
            let b_x_cmd_dot = self.roll.control(b_x_c, b_x);

            let b_y = rot_mat[(1, 2)];
            let b_y_cmd_dot = self.pitch.control(b_y_c, b_y);

            let rot_mat1 = Matrix2::new(
                rot_mat[(1, 0)],
                -rot_mat[(0, 0)],
                rot_mat[(1, 1)],
                -rot_mat[(0, 1)],
            ) / rot_mat[(2, 2)];

            rot_mat1 * Vector2::new(b_x_cmd_dot, b_y_cmd_dot)
        } else {
            Vector2::zeros()
        }
    }
}
