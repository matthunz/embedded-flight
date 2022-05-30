use core::f32::consts::PI;

use nalgebra::{Quaternion, Vector3};

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
    pub thrust_error_angle: f32,
}

impl AttitudeController {
    /// Calculate the body frame angular velocities to follow the target attitude.
    /// `attitude_body` represents a quaternion rotation in NED frame to the body
    pub fn target(&mut self, attitude_body: Quaternion<f32>, attitude_target: Quaternion<f32>) {
        // Vector representing the angular error to rotate the thrust vector using x and y and heading using z
        let (_attitude_target, attitude_error) = self.thrust_heading_rotation_angles(
            attitude_target,
            attitude_body,
            self.thrust_error_angle,
        );

        // Compute the angular velocity corrections in the body frame from the attitude error
        self.ang_vel_body = self.angular_velocity_target_from_attitude_error(attitude_error);
    }

    // Calculates two ordered rotations to move `attitude_body` to `attitude_target`.
    // The maximum error in the yaw axis is limited based on the angle yaw P value and acceleration.
    pub fn thrust_heading_rotation_angles(
        &self,
        mut attitude_target: Quaternion<f32>,
        attitude_body: Quaternion<f32>,
        thrust_error_angle: f32,
    ) -> (Quaternion<f32>, Vector3<f32>) {
        let ac_attitude_accel_y_controller_max_radss = 120f32.to_radians();

        let (thrust_angle, thrust_vector_correction, mut attitude_error, thrust_error_angle) =
            thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_error_angle);

        // Todo: Limit roll an pitch error based on output saturation and maximum error.

        // Limit Yaw Error based on maximum acceleration - Update to include output saturation and maximum error.
        // Currently the limit is based on the maximum acceleration using the linear part of the SQRT controller.
        // This should be updated to be based on an angle limit, saturation, or unlimited based on user defined parameters.

        if (self.p_angle_yaw.kp != 0.
            && attitude_error.z.abs()
                > ac_attitude_accel_y_controller_max_radss / self.p_angle_yaw.kp)
        {
            attitude_error.z = wrap_PI(attitude_error.z)
                .max(-ac_attitude_accel_y_controller_max_radss / self.p_angle_yaw.kp)
                .min(ac_attitude_accel_y_controller_max_radss / self.p_angle_yaw.kp);
            let yaw_vec_correction_quat = from_axis_angle(Vector3::new(0., 0., attitude_error.z));
            attitude_target = attitude_body * thrust_vector_correction * yaw_vec_correction_quat;
        }

        (attitude_target, attitude_error)
    }

    /// Calculate the rate target angular velocity using the attitude error rotation vector (in radians)
    pub fn angular_velocity_target_from_attitude_error(
        &self,
        attitude_error_rot_vec_rad: Vector3<f32>,
    ) -> Vector3<f32> {
        let ac_attitude_accel_rp_controller_min_radss = 40f32.to_radians();
        let ac_attitude_accel_rp_controller_max_radss = 720f32.to_radians();

        let ac_attitude_accel_y_rp_controller_min_radss = 10f32.to_radians();
        let ac_attitude_accel_y_rp_controller_max_radss = 120f32.to_radians();

        // Compute the roll angular velocity demand from the roll angle error
        let target_roll_angular_velocity = if self.use_sqrt_controller && self.accel_max[0] != 0. {
            sqrt_controller(
                attitude_error_rot_vec_rad.x,
                self.p_angle_roll.kp,
                (self.accel_max[0] / 2.)
                    .max(ac_attitude_accel_rp_controller_min_radss)
                    .min(ac_attitude_accel_rp_controller_max_radss),
                self.dt,
            )
        } else {
            self.p_angle_roll.kp * attitude_error_rot_vec_rad.x
        };

        // Compute the pitch angular velocity demand from the pitch angle error
        let target_pitch_angular_velocity = if self.use_sqrt_controller && self.accel_max[1] != 0. {
            sqrt_controller(
                attitude_error_rot_vec_rad.y,
                self.p_angle_pitch.kp,
                (self.accel_max[1] / 2.)
                    .max(ac_attitude_accel_rp_controller_min_radss)
                    .min(ac_attitude_accel_rp_controller_max_radss),
                self.dt,
            )
        } else {
            self.p_angle_pitch.kp * attitude_error_rot_vec_rad.y
        };

        // Compute the yaw angular velocity demand from the yaw angle error
        let target_yaw_angular_velocity = if self.use_sqrt_controller && self.accel_max[2] != 0. {
            sqrt_controller(
                attitude_error_rot_vec_rad.z,
                self.p_angle_yaw.kp,
                (self.accel_max[2] / 2.)
                    .max(ac_attitude_accel_y_rp_controller_min_radss)
                    .min(ac_attitude_accel_y_rp_controller_max_radss),
                self.dt,
            )
        } else {
            self.p_angle_yaw.kp * attitude_error_rot_vec_rad.z
        };

        Vector3::new(
            target_roll_angular_velocity,
            target_pitch_angular_velocity,
            target_yaw_angular_velocity,
        )
    }
}

pub struct P {
    kp: f32,
}

/// thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
/// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
pub fn thrust_vector_rotation_angles(
    attitude_target: Quaternion<f32>,
    attitude_body: Quaternion<f32>,

    mut thrust_error_angle: f32,
) -> (f32, Quaternion<f32>, Vector3<f32>, f32) {
    // The direction of thrust is [0,0,-1] is any body-fixed frame, inc. body frame and target frame.
    let thrust_vector_up = Vector3::new(0., 0., -1.);

    // attitude_target and attitute_body are passive rotations from target / body frames to the NED frame

    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the target thrust vector in the inertial frame
    let att_target_thrust_vec = mul(attitude_target, thrust_vector_up); // target thrust vector

    // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the current thrust vector in the inertial frame
    let att_body_thrust_vec = mul(attitude_body, thrust_vector_up); // current thrust vector

    // the dot product is used to calculate the current lean angle for use of external functions
    let thrust_angle = ((thrust_vector_up.dot(&att_body_thrust_vec))
        .max(-1.)
        .min(1.))
    .acos();

    // the cross product of the desired and target thrust vector defines the rotation vector
    let mut thrust_vec_cross = att_body_thrust_vec.cross(&att_target_thrust_vec);

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_error_angle = ((att_body_thrust_vec.dot(&att_target_thrust_vec))
        .max(-1.)
        .min(1.))
    .acos();

    // Normalize the thrust rotation vector
    let thrust_vector_length = thrust_vec_cross.norm();
    if thrust_vector_length == 0. || thrust_error_angle == 0. {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction is defined relative to the body frame but its axis `thrust_vec_cross` was computed in
    // the inertial frame. First rotate it by the inverse of attitude_body to express it back in the body frame
    thrust_vec_cross = mul(attitude_body.try_inverse().unwrap(), thrust_vec_cross);
    let thrust_vector_correction = from_axis_angle_with_theta(thrust_vec_cross, thrust_error_angle);

    // calculate the angle error in x and y.
    let mut rotation = to_axis_angle(thrust_vector_correction);
    let attitude_error_x = rotation.x;
    let attitude_error_y = rotation.y;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    // heading_vector_correction
    let heading_vec_correction_quat = thrust_vector_correction.try_inverse().unwrap()
        * attitude_body.try_inverse().unwrap()
        * attitude_target;

    // calculate the angle error in z (x and y should be zero here).
    rotation = to_axis_angle(heading_vec_correction_quat);
    let attitude_error_z = rotation.z;

    (
        thrust_angle,
        thrust_vector_correction,
        Vector3::new(attitude_error_x, attitude_error_y, attitude_error_z),
        thrust_error_angle,
    )
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

// Optimized quaternion rotation operator, equivalent to converting
// (*this) to a rotation matrix then multiplying it to the argument `v`.
//
// 15 multiplies and 15 add / subtracts. Caches 3 floats
fn mul(quat: Quaternion<f32>, v: Vector3<f32>) -> Vector3<f32> {
    // This uses the formula
    //
    //    v2 = v1 + 2 q1 * qv x v1 + 2 qv x qv x v1
    //
    // where "x" is the cross product (explicitly inlined for performance below),
    // "q1" is the scalar part and "qv" is the vector part of this quaternion

    let mut ret = v;

    // Compute and cache "qv x v1"
    let mut uv = [
        quat[2] * v.z - quat[3] * v.y,
        quat[3] * v.x - quat[1] * v.z,
        quat[3] * v.y - quat[4] * v.x,
    ];

    uv[0] += uv[0];
    uv[1] += uv[1];
    uv[2] += uv[2];
    ret.x += quat[0] * uv[0] + quat[2] * uv[2] - quat[3] * uv[1];
    ret.y += quat[0] * uv[1] + quat[3] * uv[0] - quat[1] * uv[2];
    ret.z += quat[0] * uv[2] + quat[1] * uv[1] - quat[2] * uv[0];
    ret
}

// create a quaternion from its axis-angle representation
fn from_axis_angle(v: Vector3<f32>) -> Quaternion<f32> {
    let theta = v.norm();
    if theta == 0. {
        Quaternion::new(1., 0., 0., 0.)
    } else {
        from_axis_angle_with_theta(v / theta, theta)
    }
}

// create a quaternion from its axis-angle representation
// the axis vector must be length 1, theta is in radians
fn from_axis_angle_with_theta(axis: Vector3<f32>, theta: f32) -> Quaternion<f32> {
    // axis must be a unit vector as there is no check for length
    if theta == 0. {
        Quaternion::new(1., 0., 0., 0.)
    } else {
        let st2 = (0.5 * theta).sin();

        Quaternion::new(
            (0.5 * theta).cos(),
            axis.x * st2,
            axis.y * st2,
            axis.z * st2,
        )
    }
}

fn to_axis_angle(quat: Quaternion<f32>) -> Vector3<f32> {
    let l = (quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]).sqrt();
    let v = Vector3::new(quat[1], quat[2], quat[3]);
    if l == 0. {
        (v / l) * wrap_PI(2. * l.atan2(quat[0]))
    } else {
        v
    }
}

fn wrap_PI(radian: f32) -> f32 {
    let res = wrap_2PI(radian);
    if res > PI {
        res - (PI * 2.)
    } else {
        res
    }
}

fn wrap_2PI(radian: f32) -> f32 {
    let res = radian % (PI * 2.);
    if res < 0. {
        res + (PI * 2.)
    } else {
        res
    }
}
