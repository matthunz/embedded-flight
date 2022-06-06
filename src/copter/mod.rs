pub mod control;
pub use control::{Controller, MotorControl, QuadMotorControl};

use crate::Sensors;
use nalgebra::Vector3;

pub type QuadCopter<S, E> = Copter<S, QuadMotorControl<E>>;

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
    /// Arm the copter's motors.
    pub fn arm(&mut self) {
        self.motor_control.arm();
    }

    /// Control the copter to move to a local position with a desired velocity.
    pub fn control(&mut self, position: Vector3<f32>, velocity: Vector3<f32>) {
        let (torque, acceleration) = self.controller.position_control(
            position,
            velocity,
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
