use nalgebra::Vector3;
use crate::ESC;
use super::MotorControl;

pub struct QuadMotorControl<E> {
    /// Perpendicular distance to axes [m].
    pub length: f32,

    /// Drag/thrust ratio
    pub kappa: f32,

    pub motors: [E; 4],

    pub min_thrust: f32,
    pub max_thrust: f32,
}

impl<E: ESC<f32>> QuadMotorControl<E> {
    pub fn output_motor_thrust(&mut self, motor: usize, thrust: f32) {
        let output = (thrust - self.min_thrust) * 2. / (self.max_thrust - self.min_thrust) + -1.;
        self.motors[motor].output(output);
    }
}

impl<E: ESC<f32>> MotorControl for QuadMotorControl<E> {
    fn motor_control(&mut self, moment_cmd: Vector3<f32>, collective_thrust: f32) {
        // 1. Calculate the commanded angular acceleration [m/s^2]
        let t1 = moment_cmd.x / self.length;
        let t2 = moment_cmd.y / self.length;
        let t3 = -moment_cmd.z / self.kappa;

        // 3. Output the commanded thrust on each rotor [N*m]
        self.output_motor_thrust(0, (t1 + t2 + t3 + collective_thrust) / 4.);
        self.output_motor_thrust(0, (-t1 + t2 - t3 + collective_thrust) / 4.);
        self.output_motor_thrust(0, (t1 - t2 - t3 + collective_thrust) / 4.);
        self.output_motor_thrust(0, (-t1 - t2 + t3 + collective_thrust) / 4.);
    }
}

