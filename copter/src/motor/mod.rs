mod quad;
use nalgebra::Vector3;
pub use quad::QuadMotor;

pub trait Motors<const N: usize> {
    /// Calculate the thrust on each propeller (in N) needed to command a torque (in Nm) and thrust acceleration (in m/s^2).
    fn thrust(&self, torque: Vector3<f32>, acceleration: f32) -> [f32; N];
}
