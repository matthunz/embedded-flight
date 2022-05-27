use crate::pid_config;
use nalgebra::Vector2;

pub struct LateralPositionController {
    lateral_k_p: f32,
    lateral_k_d: f32,
}

impl Default for LateralPositionController {
    fn default() -> Self {
        let (lateral_k_p, lateral_k_d) = pid_config(0.28, 0.95);
        Self {
            lateral_k_p,
            lateral_k_d,
        }
    }
}

impl LateralPositionController {
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
}
