mod copter;
pub use copter::{Copter, QuadCopter};

use nalgebra::Vector3;

pub mod esc;

pub mod scheduler;

pub trait Sensors {
    fn attitude(&mut self) -> Vector3<f32>;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}

