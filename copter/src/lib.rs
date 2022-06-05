mod body_rate;
pub use body_rate::BodyRateController;
use nalgebra::{Rotation3, Vector3};
use pid_controller::PD;

pub mod motor;

pub struct AltitudeController {
    pub pd: PD,
    pub gravity: f32,
}

impl AltitudeController {
    pub fn acceleration(
        &self,
        altitude_cmd: f32,
        vertical_velocity_cmd: f32,
        altitude: f32,
        vertical_velocity: f32,
        attitude: Vector3<f32>,
        acceleration_ff: f32,
    ) -> f32 {
        let b_z = Rotation3::from_euler_angles(attitude.x, attitude.y, attitude.z)[(2, 2)];
        let u_1 = self.pd.control(
            altitude_cmd,
            altitude,
            vertical_velocity_cmd,
            vertical_velocity,
        );

        (u_1 + acceleration_ff - self.gravity) / b_z
    }
}
