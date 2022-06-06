use nalgebra::Vector3;

pub mod esc;
pub use esc::{ESC, RCESC};

pub trait Sensors {
    fn attitude(&mut self) -> Vector3<f32>;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}
