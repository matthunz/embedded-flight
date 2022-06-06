pub mod control;
pub use control::Controller;
use control::MotorControl;
use nalgebra::Vector3;

pub mod esc;

pub mod scheduler;

pub trait Sensors {
    fn attitude(&mut self) -> Vector3<f32>;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}

pub struct Copter<S, M: MotorControl> {
    pub controller: Controller,
    pub motor_control: M,
    pub moment_of_inertia: Vector3<f32>,
    pub sensors: S,
}

impl<S, M> Copter<S, M>
where
    S: Sensors,
    M: MotorControl,
{
    pub fn control(&mut self, local_position_cmd: Vector3<f32>, local_velocity_cmd: Vector3<f32>) {
        let (torque, acceleration) = self.controller.position_control(
            local_position_cmd,
            local_velocity_cmd,
            self.sensors.position(),
            self.sensors.velocity(),
            self.sensors.attitude(),
            self.sensors.gyro(),
            self.moment_of_inertia,
        );

        self.motor_control
            .motor_control(torque, acceleration, &self.moment_of_inertia);
    }
}
