use nalgebra::Vector3;

pub mod esc;
pub use esc::{ESC, RCESC};

pub trait Actuator<T> {
    /// Output a percentage in [-1, 1].
    fn output(&mut self, output: T);
}

impl<T, U> Actuator<U> for &'_ mut T
where
    T: Actuator<U> + ?Sized,
{
    fn output(&mut self, output: U) {
        (&mut **self).output(output);
    }
}

pub trait Sensors<Attitude> {
    fn attitude(&mut self) -> Attitude;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}
