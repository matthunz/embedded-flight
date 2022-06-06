use nalgebra::Vector2;
use pid_controller::PD;

#[derive(Clone, Debug)]
pub struct LateralPositionController {
    pd: PD<f32>,
}

impl Default for LateralPositionController {
    fn default() -> Self {
        Self {
            pd: PD {
                p: Default::default(),
                kd: 1.,
            },
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
    ) -> Vector2<f32> {
        self.pd.control(
            local_position_cmd,
            local_velocity_cmd,
            local_position,
            local_velocity,
        )
    }

    pub fn lateral_position_control_with_feed_forward(
        &self,
        local_position_cmd: Vector2<f32>,
        local_velocity_cmd: Vector2<f32>,
        local_position: Vector2<f32>,
        local_velocity: Vector2<f32>,
        acceleration_ff: Vector2<f32>,
    ) -> Vector2<f32> {
        self.lateral_position_control(
            local_position_cmd,
            local_velocity_cmd,
            local_position,
            local_velocity,
        ) + acceleration_ff
    }
}
