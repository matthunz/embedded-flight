use nalgebra::Vector3;

pub mod esc;
pub use esc::{ESC, RCESC};

pub trait Actuator {
    /// Output a percentage in [-1, 1].
    fn output(&mut self, output: f32);
}

pub trait Sensors<Attitude> {
    fn attitude(&mut self) -> Attitude;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}
