use crate::Sensors;
use nalgebra::{UnitQuaternion, Vector2, Vector3};

pub mod control;
pub use control::Control;
use control::{MotorControl, QuadMotorControl};

pub type QuadCopter<E, S> = Copter<QuadMotorControl<E>, S>;



pub struct Copter<M, S> {
    control: Control,
    motors: M,
    sensors: S,
    home: Vector2<f32>,
}

impl<M, S> Copter<M, S>
where
    M: MotorControl,
    S: Sensors<UnitQuaternion<f32>>,
{
    pub fn altitude_control(
        &mut self,
        altitude_cmd: f32,
        ascent_rate_cmd: f32,
        body_rate_cmd: Vector3<f32>,
        dt: f32,
    ) {
        let collective_thrust_cmd = self.control.altitude_control(
            self.sensors.attitude(),
            self.sensors.position().z,
            self.sensors.velocity().z,
            altitude_cmd,
            ascent_rate_cmd,
            0.,
            dt,
        );

        self.body_rate_control(body_rate_cmd, collective_thrust_cmd)
    }

    pub fn body_rate_control(&mut self, body_rate_cmd: Vector3<f32>, collective_thrust_cmd: f32) {
        let torque_cmd = self
            .control
            .body_rate_control(self.sensors.gyro(), body_rate_cmd);
        self.motors.motor_control(torque_cmd, collective_thrust_cmd);
    }
}
