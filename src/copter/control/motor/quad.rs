use super::MotorControl;
use crate::ESC;
use core::f32::consts::SQRT_2;
use nalgebra::Vector3;

/// Motor control for a quad-copter.
pub struct QuadMotorController<E> {
    /// Mass of the quad-copter (in kg).
    pub m: f32,

    /// Mass proportional gain.
    pub k_m: f32,

    /// Force proportional gain.
    pub k_f: f32,

    /// Perpendicular distance to axes (in meters).
    pub l: f32,

    /// Minimum thrust on each rotor (in N).
    pub min_thrust: f32,

    /// Maximum thrust on each rotor (in N).
    pub max_thrust: f32,

    /// Motor array to output linear actuation.
    pub motors: [E; 4],
}

impl<E: ESC<f32>> QuadMotorController<E> {
    // Create a new `QuadMotor` from the quad's mass (in grams) and rotor to rotor length (in meters).
    pub fn new(m: f32, length: f32, max_thrust: f32, min_thrust: f32, motors: [E; 4]) -> Self {
        Self {
            k_f: 1.,
            k_m: 1.,
            m,
            l: length / (2. * SQRT_2),
            motors,
            min_thrust,
            max_thrust,
        }
    }

    /// Calculate the thrust on each propeller (in N)
    /// needed to command an angular acceleration (in m/s^2)
    /// and thrust acceleration (in m/s^2)
    /// with the moment of inertia (kgm^2).
    pub fn thrust_from_acceleration(
        &self,
        acceleration: Vector3<f32>,
        thrust_acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) -> [f32; 4] {
        let angular_velocity = self.angular_velocity_from_acceleration(
            acceleration,
            thrust_acceleration,
            moment_of_inertia,
        );
        self.thrust_from_angular_velocity(angular_velocity)
    }

    /// Calculate the angular velocity on each propeller (in m/s)
    /// needed to command a torque (in Nm) and thrust acceleration (in m/s^2)
    /// with the moment of inertia (kgm^2).
    pub fn angular_velocity(
        &self,
        torque: Vector3<f32>,
        thrust_acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) -> [f32; 4] {
        let acceleration = torque.zip_map(moment_of_inertia, |t, moi| acceleration(t, moi));
        self.angular_velocity_from_acceleration(
            acceleration,
            thrust_acceleration,
            moment_of_inertia,
        )
    }

    /// Calculate the angular velocity on each propeller (in m/s) needed to command an angular acceleration (in m/s^2) and thrust acceleration (in m/s^2).
    pub fn angular_velocity_from_acceleration(
        &self,
        acceleration: Vector3<f32>,
        thrust_acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) -> [f32; 4] {
        // Calculate the torque on each rotor (Nm)
        let c_bar = -thrust_acceleration * self.m / self.k_f;
        let p_bar = acceleration.x * moment_of_inertia.x / (self.k_f * self.l);
        let q_bar = acceleration.y * moment_of_inertia.y / (self.k_f * self.l);
        let r_bar = acceleration.z * moment_of_inertia.z / self.k_m;

        // Calculate the angular velocities (squared)
        let omega_4 = (c_bar + p_bar - r_bar - q_bar) / 4.;
        let omega_3 = (r_bar - p_bar) / 2. + omega_4;
        let omega_2 = (c_bar - p_bar) / 2. - omega_3;
        let omega_1 = c_bar - omega_2 - omega_3 - omega_4;

        // Take the square-root of each squared angular velocity
        // Then negate output on the counter-clockwise motors
        [
            -(omega_1.sqrt()),
            omega_2.sqrt(),
            -(omega_3.sqrt()),
            omega_4.sqrt(),
        ]
    }

    /// Calculate the thrust on each rotor from the angular velocity of each propeller.
    pub fn thrust_from_angular_velocity(&self, angular_velocity: [f32; 4]) -> [f32; 4] {
        angular_velocity.map(|v| self.k_f * v.powi(2))
    }

    /// Output a thrust on each rotor to the motors.
    pub fn output_thrust(&mut self, thrust: [f32; 4]) {
        for (esc, thrust) in self.motors.iter_mut().zip(thrust) {
            // Convert the thrust on this rotor to the linear actuation of the motor.
            let actuation =
                (thrust - self.min_thrust) * 2. / (self.max_thrust - self.min_thrust) + 1.;

            // Output the converted linear actuation
            esc.output(actuation);
        }
    }
}

impl<E> MotorControl for QuadMotorController<E>
where
    E: ESC<f32>,
{
    fn arm(&mut self) {
        for motor in &mut self.motors {
            motor.arm();
        }
    }

    fn motor_control(
        &mut self,
        torque: Vector3<f32>,
        acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) {
        // Calculate the angular velocity of each propeller
        let angular_velocity = self.angular_velocity(torque, acceleration, moment_of_inertia);

        // Convert to thrust
        let thrust = self.thrust_from_angular_velocity(angular_velocity);

        // Output the thrust
        self.output_thrust(thrust);
    }
}

// Calculate the acceleration from net torque and moment of inertia
fn acceleration(net_torque: f32, moment_of_inertia: f32) -> f32 {
    net_torque / moment_of_inertia
}
