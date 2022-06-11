pub mod control;
pub use control::{Controller, MotorControl, QuadMotorController};

use crate::Sensors;
use nalgebra::{UnitQuaternion, Vector3};

pub type QuadCopter<S, E> = Copter<S, QuadMotorController<E>>;

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

pub struct Control {
    gravity: f32,
    mass: f32,
    integrated_altitude_error: f32,
    ki_pos_z: f32,
    kp_pos_z: f32,
    kp_vel_z: f32,
    max_descent_rate: f32,
    max_ascent_rate: f32,
}

impl Control {
    pub fn altitude_control(
        &mut self,
        attitude: UnitQuaternion<f32>,
        altitude: f32,
        ascent_rate: f32,
        altitude_cmd: f32,
        ascent_rate_cmd: f32,
        ascent_acceleration_cmd: f32,
        dt: f32,
    ) -> f32 {
        // 1. Convert the attitude to a rotation matrix and take the vertical component
        let attitude_rotation = attitude.to_rotation_matrix();
        let bz = attitude_rotation[(2, 2)];

        // 2. Calculate the error in position and add it to the integration
        let altitude_error = altitude_cmd - altitude;
        self.integrated_altitude_error += altitude_error * dt;

        // 3. Calculate the error in velocity
        let ascent_rate_error = ascent_rate_cmd - ascent_rate;

        // 4. Calculate output from the PID control
        let ul_bar = self.kp_pos_z * altitude_error
            + self.kp_vel_z * ascent_rate_error
            + ascent_rate
            + self.ki_pos_z * self.integrated_altitude_error
            + ascent_acceleration_cmd;

        // 5. Calculate the commanded vertical acceleration [m/s^2]
        //    by subtracting the acceleration of gravity and dividing by the current altitude component
        let accel = (ul_bar - self.gravity) / bz;

        // 6. Constrain the acceleration to our max ascent/descent rates
        // 7. Multiply by the mass to calculate thrust
        -self.mass
            * accel
                .max(-self.max_descent_rate / dt)
                .min(self.max_ascent_rate / dt)
    }
}
