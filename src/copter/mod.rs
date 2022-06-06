pub mod control;
pub use control::{Controller, MotorControl, QuadMotorControl};

use crate::Sensors;
use nalgebra::Vector3;

pub type QuadCopter<S, E> = Copter<S, QuadMotorControl<E>>;

#[derive(Clone, Debug, Default)]
pub struct Copter<S, M: MotorControl> {
    /// The copter's moment of inertia (in kg*m^2)
    pub moment_of_inertia: Vector3<f32>,

    /// The position controller
    pub controller: Controller,

    /// The motor control to output commands.
    pub motor_control: M,

    /// The sensor measurement source.
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

    /// Control the copter to move to a position in the local frame [north, east] (in m)
    /// with a desired velocity in the local frame [north_velocity, east_velocity] (in m/s).
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
