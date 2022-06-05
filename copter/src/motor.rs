use std::f32::consts::SQRT_2;

use nalgebra::Vector3;

pub struct Quad {
    pub m: f32,
    pub k_m: f32,
    pub k_f: f32,
    pub i_x: f32,
    pub i_y: f32,
    pub i_z: f32,
    // Full rotor to rotor distance
    pub l: f32,
}

pub fn acceleration(net_torque: f32, moment_of_inertia: f32) -> f32 {
    net_torque / moment_of_inertia
}

impl Quad {
    pub fn new(m: f32, l: f32) -> Self {
        Self {
            k_f: 1.,
            k_m: 1.,
            m,
            l: l / (2. * SQRT_2),
            i_x: 0.1,
            i_y: 0.1,
            i_z: 0.2,
        }
    }

    pub fn torque(
        &self,
        torque: Vector3<f32>,
        moment_of_inertia: f32,
        thrust_acceleration: f32,
    ) -> [f32; 4] {
        let acceleration = torque.map(|t| acceleration(t, moment_of_inertia));
        self.propeller_angular_velocity(acceleration, thrust_acceleration)
    }

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
    pub fn propeller_angular_velocity(
        &self,
        acceleration: Vector3<f32>,
        thrust_acceleration: f32,
    ) -> [f32; 4] {
        // Torque (Nm)
        let c_bar = -thrust_acceleration * self.m / self.k_f;
        let p_bar = acceleration.x * self.i_x / (self.k_f * self.l);
        let q_bar = acceleration.y * self.i_y / (self.k_f * self.l);
        let r_bar = acceleration.z * self.i_z / self.k_m;

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

    pub fn propeller_force(
        &self,
        acceleration: Vector3<f32>,
        thrust_acceleration: f32,
    ) -> [f32; 4] {
        self.propeller_angular_velocity(acceleration, thrust_acceleration)
            .map(|v| self.k_f * v.powi(2))
    }
}

// TODO thrust factor
pub struct Motor {
    pub factor: Vector3<f32>,
}

impl Motor {
    pub fn new(factor: Vector3<f32>) -> Self {
        Self { factor }
    }

    pub fn from_angle(angle: f32, yaw_factor: f32) -> Self {
        Self::from_degrees(angle, angle, yaw_factor)
    }

    pub fn from_degrees(
        roll_factor_degrees: f32,
        pitch_factor_degrees: f32,
        yaw_factor: f32,
    ) -> Self {
        Self::new(Vector3::new(
            (roll_factor_degrees + 90.).to_radians().cos(),
            pitch_factor_degrees.to_radians().cos(),
            yaw_factor,
        ))
    }

    pub fn thrust(&self, moment: Vector3<f32>, thrust: f32) -> f32 {
        self.factor
            .zip_fold(&moment, thrust, |acc, factor, moment| factor * moment + acc)
    }
}

pub fn quad_motors() -> [Motor; 4] {
    [
        Motor::from_angle(90., 1.),
        Motor::from_angle(-90., 1.),
        Motor::from_angle(0., -1.),
        Motor::from_angle(180., -1.),
    ]
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{quad_motors, Quad};

    #[test]
    fn it_works() {
        let quad = Quad::new(0.5, 0.566);
        let ang_vel = quad.propeller_angular_velocity(Vector3::new(1.0, -2.0, 0.3), -5.0);
        let s = [-0.71768341, 0.48498291, -0.87460307, 0.99236666];
        dbg!(ang_vel, s);
    }
}
