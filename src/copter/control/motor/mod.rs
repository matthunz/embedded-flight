use nalgebra::Vector3;

mod quad;
pub use quad::QuadMotorControl;

pub trait MotorControl {
    fn motor_control(&mut self, moment_cmd: Vector3<f32>, collective_thrust: f32);
}