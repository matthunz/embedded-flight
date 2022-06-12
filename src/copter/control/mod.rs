use core::f32::consts::PI;
use nalgebra::{UnitQuaternion, Vector2, Vector3};

mod motor;
pub use motor::{MotorControl, QuadMotorControl};

pub struct Control {
    pub mass: f32,
    pub gravity: f32,
    pub ki_pos_z: f32,
    integrated_altitude_error: f32,
    pub kp_pos_z: f32,
    pub kp_vel_z: f32,
    pub max_descent_rate: f32,
    pub max_ascent_rate: f32,
    pub kp_yaw: f32,
    pub moment_of_inertia: Vector3<f32>,
    pub kp_body_rate: f32,
    pub min_thrust: f32,
    pub max_thrust: f32,
    pub kp_bank: f32,
    pub max_tilt_angle: f32,
}

impl Control {
    pub fn position_control(
        &mut self,
        attitude: UnitQuaternion<f32>,
        altitude: f32,
        ascent_rate: f32,
        altitude_cmd: f32,
        ascent_rate_cmd: f32,
        ascent_acceleration_cmd: f32,
        dt: f32,
    ) -> (Vector3<f32>, f32) {
        let collective_thrust_cmd = self.altitude_control(
            attitude,
            altitude,
            ascent_rate,
            altitude_cmd,
            ascent_rate_cmd,
            ascent_acceleration_cmd,
            dt,
        );

        let thrust_margin = 0.1 * (self.max_thrust - self.min_thrust);
        let collective_thrust_cmd = collective_thrust_cmd
            .max((self.min_thrust + thrust_margin) * 4.)
            .min((self.max_thrust - thrust_margin) * 4.);

        let torque_cmd = self.body_rate_control(Vector3::zeros(), Vector3::zeros());

        (torque_cmd, collective_thrust_cmd)
    }

    pub fn roll_pitch_control(
        &mut self,
        attitude: UnitQuaternion<f32>,
        accel_cmd: Vector2<f32>,
        collective_thrust_cmd: f32,
    ) -> Vector2<f32> {
        let attitude_rotation = attitude.to_rotation_matrix();
        if collective_thrust_cmd > 0. {
            let c = -collective_thrust_cmd / self.mass;
            let b_x_cmd = (accel_cmd.x / c)
                .max(-self.max_tilt_angle)
                .min(self.max_tilt_angle);
            let b_x_err = b_x_cmd - attitude_rotation[(0, 2)];
            let b_x_p_term = self.kp_bank * b_x_err;

            let b_y_cmd = (accel_cmd.y / c)
                .max(-self.max_tilt_angle)
                .min(self.max_tilt_angle);
            let b_y_err = b_y_cmd - attitude_rotation[(1, 2)];
            let b_y_p_term = self.kp_bank * b_y_err;

            Vector2::new(
                (attitude_rotation[(1, 0)] * b_x_p_term - attitude_rotation[(0, 0)] * b_y_p_term)
                    / attitude_rotation[(2, 2)],
                (attitude_rotation[(1, 1)] * b_x_p_term - attitude_rotation[(0, 1)] * b_y_p_term)
                    / attitude_rotation[(2, 2)],
            )
        } else {
            Vector2::zeros()
        }
    }

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

    /// Calculate a desired yaw rate to control yaw to yaw_cmd [rad/s]
    pub fn yaw_control(&self, yaw_cmd: f32, yaw: f32) -> f32 {
        // 1. Calculate the error in yaw [rad]
        let yaw_error = yaw_cmd - yaw;

        // 2. Wrap the error between -2PI and 2PI
        let yaw_error_2_pi = if yaw_error > 0. {
            yaw_error % (2. * PI)
        } else {
            -(-yaw_error % 2. * PI)
        };

        // 3. Add the proportionally controlled yaw [rad/s]
        self.kp_yaw * yaw_error_2_pi
    }

    /// Calculate the desired torque to control body_rate to body_rate_cmd [rad/s]
    pub fn body_rate_control(
        &self,
        body_rate: Vector3<f32>,
        body_rate_cmd: Vector3<f32>,
    ) -> Vector3<f32> {
        let body_rate_output = self.kp_body_rate * (body_rate_cmd - body_rate);
        self.moment_of_inertia.component_mul(&body_rate_output)
    }
}
