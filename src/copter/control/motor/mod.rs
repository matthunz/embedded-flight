use nalgebra::Vector3;

mod quad;
pub use quad::QuadMotorControl;

pub trait MotorControl {
    /// Calculate the thrust on each propeller (in N) needed to command a torque (in Nm) and thrust acceleration (in m/s^2).
    fn motor_control(
        &mut self,
        torque: Vector3<f32>,
        acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    );
}
