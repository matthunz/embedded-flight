use std::f32::consts::SQRT_2;

use nalgebra::Vector3;

use super::Motors;

pub struct QuadMotor {
    pub m: f32,
    pub k_m: f32,
    pub k_f: f32,
    /// Perpendicular distance to axes (in meters)
    pub l: f32,
}

impl QuadMotor {
    // Create a new `QuadMotor` from the quad's mass (in grams) and rotor to rotor length (in meters).
    pub fn new(m: f32, length: f32) -> Self {
        Self {
            k_f: 1.,
            k_m: 1.,
            m,
            l: length / (2. * SQRT_2),
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
    /// needed to command a torque (in Nm)and thrust acceleration (in m/s^2)
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
    /// ```
    /// use embedded_flight_copter::motor::Quad;
    /// use nalgebra::Vector3;
    /// use approx::abs_diff_eq;
    ///
    /// let quad = Quad::new(0.5, 0.566);
    /// let props = quad.propeller_angular_velocities(Vector3::new(1.0, -2.0, 0.3), -5.0);
    ///
    /// abs_diff_eq!(props[0], -0.71768341);
    /// abs_diff_eq!(props[1],  0.48498291);
    /// abs_diff_eq!(props[2], -0.87460307);
    /// abs_diff_eq!(props[3], -0.99236666);
    /// ```
    pub fn angular_velocity_from_acceleration(
        &self,
        acceleration: Vector3<f32>,
        thrust_acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) -> [f32; 4] {
        // Torque (Nm)
        let c_bar = -thrust_acceleration * self.m / self.k_f;
        let p_bar = acceleration.x * moment_of_inertia.x / (self.k_f * self.l);
        let q_bar = acceleration.y * moment_of_inertia.y / (self.k_f * self.l);
        let r_bar = acceleration.z * moment_of_inertia.z / self.k_m;

        let omega_4 = (c_bar + p_bar - r_bar - q_bar) / 4.;
        let omega_3 = (r_bar - p_bar) / 2. + omega_4;
        let omega_2 = (c_bar - p_bar) / 2. - omega_3;
        let omega_1 = c_bar - omega_2 - omega_3 - omega_4;

        [
            -(omega_1.sqrt()),
            omega_2.sqrt(),
            -(omega_3.sqrt()),
            omega_4.sqrt(),
        ]
    }

    pub fn thrust_from_angular_velocity(&self, angular_velocity: [f32; 4]) -> [f32; 4] {
        angular_velocity.map(|v| self.k_f * v.powi(2))
    }
}

fn acceleration(net_torque: f32, moment_of_inertia: f32) -> f32 {
    net_torque / moment_of_inertia
}

impl Motors<4> for QuadMotor {
    fn thrust(
        &self,
        torque: Vector3<f32>,
        acceleration: f32,
        moment_of_inertia: &Vector3<f32>,
    ) -> [f32; 4] {
        let angular_velocity = self.angular_velocity(torque, acceleration, moment_of_inertia);
        self.thrust_from_angular_velocity(angular_velocity)
    }
}
