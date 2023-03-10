use crate::{constrain_float, safe_sqrt, wrap_PI};
use nalgebra::{UnitQuaternion, Vector2, Vector3};

pub struct AttitudeController {
    attitude_target: UnitQuaternion<f32>,
    ang_vel_roll_max: f32,
    ang_vel_pitch_max: f32,
    ang_vel_yaw_max: f32,
    ang_vel_target: Vector3<f32>,
    dt: f32,

    /// rate controller input smoothing time constant
    pub input_tc: f32,

    accel_roll_max: f32,
    accel_pitch_max: f32,
    accel_yaw_max: f32,

    pub euler_angle_target: Vector3<f32>,
    pub euler_rate_target: Vector3<f32>,
}

impl AttitudeController {
    pub fn accel_roll_max_radss(&self) -> f32 {
        (self.accel_roll_max * 0.01).to_radians()
    }

    pub fn accel_pitch_max_radss(&self) -> f32 {
        (self.accel_pitch_max * 0.01).to_radians()
    }

    pub fn accel_yaw_max_radss(&self) -> f32 {
        (self.accel_yaw_max * 0.01).to_radians()
    }

    // The attitude controller works around the concept of the desired attitude, target attitude
    // and measured attitude. The desired attitude is the attitude input into the attitude controller
    // that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
    // to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
    // directly into the rate controllers. The angular error between the measured attitude and the target attitude is
    // fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
    // By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
    // remain very close together.
    //
    // All input functions below follow the same procedure
    // 1. define the desired attitude the aircraft should attempt to achieve using the input variables
    // 2. using the desired attitude and input variables, define the target angular velocity so that it should
    //    move the target attitude towards the desired attitude
    // 3. if _rate_bf_ff_enabled is not being used then make the target attitude
    //    and target angular velocities equal to the desired attitude and desired angular velocities.
    // 4. ensure _attitude_target, _euler_angle_target, _euler_rate_target and
    //    _ang_vel_target have been defined. This ensures input modes can be changed without discontinuity.
    // 5. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
    //    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
    //    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
    //    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.

    /// Command a Quaternion attitude with feedforward and smoothing
    /// `attitude_desired_quat`: is updated on each time_step by the integral of the angular velocity
    pub fn input_quaternion(
        &mut self,
        attitude_desired_quat: UnitQuaternion<f32>,
        ang_vel_target: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let attitude_error_quat = self.attitude_target.inverse() * attitude_desired_quat;
        let attitude_error_angle = attitude_error_quat.scaled_axis();

        let ang_vel_target = ang_vel_limit(
            ang_vel_target,
            self.ang_vel_roll_max.to_radians(),
            self.ang_vel_pitch_max.to_radians(),
            self.ang_vel_yaw_max.to_radians(),
        );

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        self.ang_vel_target.x = input_shaping_angle(
            wrap_PI(attitude_error_angle.x),
            self.input_tc,
            self.accel_roll_max_radss(),
            self.ang_vel_target.x,
            ang_vel_target.x,
            self.ang_vel_roll_max.to_radians(),
            self.dt,
        );
        self.ang_vel_target.y = input_shaping_angle(
            wrap_PI(attitude_error_angle.y),
            self.input_tc,
            self.accel_pitch_max_radss(),
            self.ang_vel_target.y,
            ang_vel_target.y,
            self.ang_vel_pitch_max.to_radians(),
            self.dt,
        );
        self.ang_vel_target.z = input_shaping_angle(
            wrap_PI(attitude_error_angle.z),
            self.input_tc,
            self.accel_yaw_max_radss(),
            self.ang_vel_target.z,
            ang_vel_target.z,
            self.ang_vel_yaw_max.to_radians(),
            self.dt,
        );

        // calculate the attitude target euler angles
        self.attitude_target = UnitQuaternion::from_euler_angles(
            self.euler_angle_target.x,
            self.euler_angle_target.y,
            self.euler_angle_target.z,
        );

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        if let Some(euler_rate_target) = ang_vel_to_euler_rate(
            self.euler_angle_target,
            self.ang_vel_target,
            self.euler_rate_target,
        ) {
            self.euler_rate_target = euler_rate_target;
        }

        // rotate target and normalize
        let attitude_desired_update = UnitQuaternion::from_scaled_axis(ang_vel_target * self.dt);
        let attitude_desired_quat = attitude_desired_quat * attitude_desired_update;

        // Call quaternion attitude controller
        //attitude_controller_run_quat();

        attitude_desired_quat
    }
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
// Returns false if the vehicle is pitched 90 degrees up or down
fn ang_vel_to_euler_rate(
    euler_rad: Vector3<f32>,
    ang_vel_rads: Vector3<f32>,
    mut euler_rate_rads: Vector3<f32>,
) -> Option<Vector3<f32>> {
    let sin_theta = euler_rad.y.sin();
    let cos_theta = euler_rad.y.cos();
    let sin_phi = euler_rad.x.sin();
    let cos_phi = euler_rad.x.cos();

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if cos_theta == 0. {
        return None;
    }

    euler_rate_rads.x = ang_vel_rads.x
        + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y
        + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z =
        (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    Some(euler_rate_rads)
}

// limits angular velocity
fn ang_vel_limit(
    mut euler_rad: Vector3<f32>,
    ang_vel_roll_max: f32,
    ang_vel_pitch_max: f32,
    ang_vel_yaw_max: f32,
) -> Vector3<f32> {
    if ang_vel_roll_max == 0. || ang_vel_pitch_max == 0. {
        if ang_vel_roll_max != 0. {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
        }
        if ang_vel_pitch_max != 0. {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
        }
    } else {
        let thrust_vector_ang_vel = Vector2::new(
            euler_rad.x / ang_vel_roll_max,
            euler_rad.y / ang_vel_pitch_max,
        );
        let thrust_vector_length = thrust_vector_ang_vel.norm();
        if thrust_vector_length > 1. {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
        }
    }

    if ang_vel_yaw_max != 0. {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
    }

    euler_rad
}

// calculates the velocity correction from an angle error. The angular velocity has acceleration and
// deceleration limits including basic jerk limiting using _input_tc
fn input_shaping_angle(
    error_angle: f32,
    input_tc: f32,
    accel_max: f32,
    target_ang_vel: f32,
    mut desired_ang_vel: f32,
    max_ang_vel: f32,
    dt: f32,
) -> f32 {
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
    desired_ang_vel += sqrt_controller(error_angle, 1. / input_tc.max(0.01), accel_max, dt);
    if max_ang_vel > 0. {
        desired_ang_vel = constrain_float(desired_ang_vel, -max_ang_vel, max_ang_vel);
    }

    // Acceleration is limited directly to smooth the beginning of the curve.
    input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt, 0.)
}

// Shapes the velocity request based on a rate time constant. The angular acceleration and deceleration is limited.
fn input_shaping_ang_vel(
    target_ang_vel: f32,
    mut desired_ang_vel: f32,
    accel_max: f32,
    dt: f32,
    input_tc: f32,
) -> f32 {
    if input_tc > 0. {
        // Calculate the acceleration to smoothly achieve rate. Jerk is not limited.
        let error_rate = desired_ang_vel - target_ang_vel;
        let desired_ang_accel = sqrt_controller(error_rate, 1. / input_tc.max(0.01), 0., dt);
        desired_ang_vel = target_ang_vel + desired_ang_accel * dt;
    }
    // Acceleration is limited directly to smooth the beginning of the curve.
    if accel_max > 0. {
        let delta_ang_vel = accel_max * dt;
        constrain_float(
            desired_ang_vel,
            target_ang_vel - delta_ang_vel,
            target_ang_vel + delta_ang_vel,
        )
    } else {
        desired_ang_vel
    }
}

// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
fn sqrt_controller(error: f32, p: f32, second_ord_lim: f32, dt: f32) -> f32 {
    let correction_rate = if second_ord_lim < 0. || second_ord_lim == 0. {
        // second order limit is zero or negative.
        error * p
    } else if p == 0. {
        // P term is zero but we have a second order limit.
        if error > 0. {
            safe_sqrt(2. * second_ord_lim * (error))
        } else if error < 0. {
            -safe_sqrt(2. * second_ord_lim * (-error))
        } else {
            0.
        }
    } else {
        // Both the P and second order limit have been defined.
        let linear_dist = second_ord_lim / p.powi(2);
        if error > linear_dist {
            safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)))
        } else if error < -linear_dist {
            -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)))
        } else {
            error * p
        }
    };

    if dt > 0. {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        constrain_float(correction_rate, -error.abs() / dt, error.abs() / dt)
    } else {
        correction_rate
    }
}
