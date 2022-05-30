use nalgebra::Vector3;

mod multi_copter;
pub use multi_copter::MultiCopterAttitudeController;

pub struct AttitudeController {
    /// The angular velocity (in radians per second) in the body frame.
    pub ang_vel_body: Vector3<f32>,
    pub actuator_sysid: Vector3<f32>,
    pub sysid_ang_vel_body: Vector3<f32>,
    pub feed_forward_scalar: f32,
    pub throttle_rpy_mix: f32,
    pub throttle_rpy_mix_desired: f32,
    pub attitude_control_max: f32,
    pub dt: f32,
    /// The acceleration limit in radians/s
    // TODO arducopter uses centidegrees/s
    pub accel_max: Vector3<f32>,
    use_sqrt_controller: bool,
    pub p_angle_roll: P,
    pub p_angle_pitch: P,
    pub p_angle_yaw: P,
}

impl AttitudeController {
    // Calculate the rate target angular velocity using the attitude error rotation vector (in radians)
    pub fn angular_velocity_target_from_attitude_error(
        &self,
        attitude_error_rot_vec_rad: Vector3<f32>,
    ) -> Vector3<f32> {
        let ac_attitude_accel_rp_controller_min_radss = 40f32.to_radians();
        let ac_attitude_accel_rp_controller_max_radss = 720f32.to_radians();

        let ac_attitude_accel_y_rp_controller_min_radss = 10f32.to_radians();
        let ac_attitude_accel_y_rp_controller_max_radss = 120f32.to_radians();

        let mut rate_target_ang_vel = Vector3::zeros();
        // Compute the roll angular velocity demand from the roll angle error
        if self.use_sqrt_controller && self.accel_max[0] != 0. {
            rate_target_ang_vel[0] = sqrt_controller(
                attitude_error_rot_vec_rad.x,
                self.p_angle_roll.kp,
                (self.accel_max[0] / 2.)
                    .max(ac_attitude_accel_rp_controller_min_radss)
                    .min(ac_attitude_accel_rp_controller_max_radss),
                self.dt,
            );
        } else {
            rate_target_ang_vel.x = self.p_angle_roll.kp * attitude_error_rot_vec_rad.x;
        }

        // Compute the pitch angular velocity demand from the pitch angle error
        if self.use_sqrt_controller && self.accel_max[1] != 0. {
            rate_target_ang_vel[1] = sqrt_controller(
                attitude_error_rot_vec_rad.y,
                self.p_angle_pitch.kp,
                (self.accel_max[1] / 2.)
                    .max(ac_attitude_accel_rp_controller_min_radss)
                    .min(ac_attitude_accel_rp_controller_max_radss),
                self.dt,
            );
        } else {
            rate_target_ang_vel.y = self.p_angle_pitch.kp * attitude_error_rot_vec_rad.y;
        }

        // Compute the yaw angular velocity demand from the yaw angle error
        if self.use_sqrt_controller && self.accel_max[2] != 0. {
            rate_target_ang_vel.z = sqrt_controller(
                attitude_error_rot_vec_rad.z,
                self.p_angle_yaw.kp,
                (self.accel_max[2] / 2.)
                    .max(ac_attitude_accel_y_rp_controller_min_radss)
                    .min(ac_attitude_accel_y_rp_controller_max_radss),
                self.dt,
            );
        } else {
            rate_target_ang_vel.z = self.p_angle_yaw.kp * attitude_error_rot_vec_rad.z;
        }

        rate_target_ang_vel
    }
}

pub struct P {
    kp: f32,
}

/// Limit the acceleration and deceleration of a velocity request.
pub fn input_shaping_ang_vel(
    target_ang_vel: f32,
    desired_ang_vel: f32,
    accel_max: f32,
    dt: f32,
) -> f32 {
    // Acceleration is limited directly to smooth the beginning of the curve.
    if accel_max >= 0. {
        let delta_ang_vel = accel_max * dt;
        desired_ang_vel
            .max(target_ang_vel - delta_ang_vel)
            .min(target_ang_vel + delta_ang_vel)
    } else {
        desired_ang_vel
    }
}

/// Calculate the velocity correction from an angle error.
/// The angular velocity has acceleration and deceleration limits
/// including basic jerk limiting using _input_tc
pub fn input_shaping_angle(
    error_angle: f32,
    input_tc: f32,
    accel_max: f32,
    target_ang_vel: f32,
    mut desired_ang_vel: f32,
    max_ang_vel: f32,
    dt: f32,
) -> f32 {
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_rads
    desired_ang_vel += sqrt_controller(error_angle, 1.0 / input_tc.max(0.01), accel_max, dt);
    if max_ang_vel > 0. {
        desired_ang_vel = desired_ang_vel.max(-max_ang_vel).min(max_ang_vel);
    }

    // Acceleration is limited directly to smooth the beginning of the curve.
    input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt)
}

/// Calculate the correction based on a proportional controller (with piecewise sqrt sections to constrain second derivative).
pub fn sqrt_controller(error: f32, p: f32, second_ord_lim: f32, dt: f32) -> f32 {
    let correction_rate;
    if second_ord_lim <= 0. {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if p == 0. {
        // P term is zero but we have a second order limit.
        if error > 0. {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error));
        } else if error < 0. {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.;
        }
    } else {
        // Both the P and second order limit have been defined.
        let linear_dist = second_ord_lim / (p * p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2. * second_ord_lim * (error - (linear_dist / 2.)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2. * second_ord_lim * (-error - (linear_dist / 2.)));
        } else {
            correction_rate = error * p;
        }
    }
    if dt != 0. {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        correction_rate
            .max(-(error.abs()) / dt)
            .min(error.abs() / dt)
    } else {
        correction_rate
    }
}

fn safe_sqrt(v: f32) -> f32 {
    let ret = v.sqrt();
    if ret.is_nan() {
        0.
    } else {
        ret
    }
}
