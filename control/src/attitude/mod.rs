use core::f32::consts::PI;
use nalgebra::{Quaternion, Vector2, Vector3};
use num_traits::Float;

mod multi_copter;
pub use multi_copter::MultiCopterAttitudeController;

/// The attitude controller works around the concept of the desired attitude, target attitude
/// and measured attitude. The desired attitude is the attitude input into the attitude controller
/// that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
/// to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
/// directly into the rate controllers. The angular error between the measured attitude and the target attitude is
/// fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
/// By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
/// remain very close together.
///
/// All input functions below follow the same procedure
/// 1. define the desired attitude the aircraft should attempt to achieve using the input variables
/// 2. using the desired attitude and input variables, define the target angular velocity so that it should
///    move the target attitude towards the desired attitude
/// 3. if _rate_bf_ff_enabled is not being used then make the target attitude
///    and target angular velocities equal to the desired attitude and desired angular velocities.
/// 4. ensure _attitude_target, _euler_angle_target, _euler_rate_target and
///    _ang_vel_target have been defined. This ensures input modes can be changed without discontinuity.
/// 5. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
///    integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
///    corrected by first correcting the thrust vector until the angle between the target thrust vector measured
///    trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.
pub struct AttitudeController {
    /// The angular velocity (in radians per second) in the body frame.
    pub ang_vel_body: Vector3<f32>,
    pub actuator_sysid: Vector3<f32>,
    pub sysid_ang_vel_body: Vector3<f32>,
    pub feed_forward_scalar: f32,
    pub throttle_rpy_mix: f32,
    pub throttle_rpy_mix_desired: f32,

    // Maximum throttle mix
    pub attitude_control_max: f32,
    pub dt: f32,

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max
    // @Description: Maximum acceleration
    // @Units: radian/s/s
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 30000:VerySlow, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    pub accel_max: Vector3<f32>,
    pub use_sqrt_controller: bool,

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    pub p_angle_roll: P,

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    pub p_angle_pitch: P,

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @User: Standard
    pub p_angle_yaw: P,
    pub thrust_error_angle: f32,
    pub attitude_target: Quaternion<f32>,

    // @Param: RATE_P_MAX
    // @DisplayName: Angular Velocity Max for Pitch
    // @Description: Maximum angular velocity in pitch axis
    // @Units: radians/s
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:Disabled, 60:Slow, 180:Medium, 360:Fast
    // @User: Advanced
    pub ang_vel_max: Vector3<f32>,
    pub ang_vel_target: Vector3<f32>,

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    pub rate_bf_ff_enabled: bool,

    // @Param: INPUT_TC
    // @DisplayName: Attitude control input time constant
    // @Description: Attitude control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    pub input_tc: f32,

    pub euler_angle_target: Vector3<f32>,
    pub euler_rate_target: Vector3<f32>,
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self {
            ang_vel_body: Vector3::zeros(),
            actuator_sysid: Vector3::zeros(),
            sysid_ang_vel_body: Vector3::zeros(),
            feed_forward_scalar: 1.,
            throttle_rpy_mix: 0.5,
            throttle_rpy_mix_desired: 0.5,
            attitude_control_max: 5.,
            dt: Default::default(),
            accel_max: Vector3::new(110000., 110000., 27000.),
            use_sqrt_controller: true,
            p_angle_roll: P::new(4.5),
            p_angle_pitch: P::new(4.5),
            p_angle_yaw: P::new(4.5),
            thrust_error_angle: 0.,
            attitude_target: Quaternion::default(),
            ang_vel_max: Vector3::zeros(),
            ang_vel_target: Vector3::zeros(),
            rate_bf_ff_enabled: true,
            input_tc: 0.15,
            euler_angle_target: Vector3::zeros(),
            euler_rate_target: Vector3::zeros(),
        }
    }
}

impl AttitudeController {
    /// Command a Quaternion attitude with feed-forward and smoothing.
    /// Returns `attitude_desired` updated by the integral of the angular velocity
    pub fn input(
        &mut self,
        mut attitude_desired: Quaternion<f32>,
        ang_vel_target: Vector3<f32>,
        attitude_body: Quaternion<f32>,
    ) -> Quaternion<f32> {
        let attitude_error_quat =
            self.attitude_target.try_inverse().unwrap_or_default() * attitude_desired;
        let attitude_error_angle = to_axis_angle(attitude_error_quat);

        // Limit the angular velocity
        let ang_vel_target = ang_vel_limit(ang_vel_target, self.ang_vel_max);

        if self.rate_bf_ff_enabled {
            // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
            // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
            // and an exponential decay specified by _input_tc at the end.
            self.ang_vel_target.x = input_shaping_angle(
                wrap_PI(attitude_error_angle.x),
                self.input_tc,
                self.accel_max.x,
                self.ang_vel_target.x,
                ang_vel_target.x,
                self.ang_vel_max.x,
                self.dt,
            );
            self.ang_vel_target.y = input_shaping_angle(
                wrap_PI(attitude_error_angle.y),
                self.input_tc,
                self.accel_max.y,
                self.ang_vel_target.y,
                ang_vel_target.y,
                self.ang_vel_max.x,
                self.dt,
            );
            self.ang_vel_target.z = input_shaping_angle(
                wrap_PI(attitude_error_angle.z),
                self.input_tc,
                self.accel_max.z,
                self.ang_vel_target.z,
                ang_vel_target.z,
                self.ang_vel_max.x,
                self.dt,
            );
        } else {
            self.attitude_target = attitude_desired;
            self.ang_vel_target = ang_vel_target;
        }

        // calculate the attitude target euler angles
        self.euler_angle_target = to_euler(self.attitude_target);

        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        if let Some(euler_rate) =
            ang_vel_to_euler_rate(self.euler_angle_target, self.ang_vel_target)
        {
            self.euler_rate_target = euler_rate;
        }

        let attitude_desired_update = from_axis_angle(ang_vel_target * self.dt);
        attitude_desired = (attitude_desired * attitude_desired_update).normalize();

        // Call quaternion attitude controller
        self.attitude_control(attitude_body);

        attitude_desired
    }

    /// Calculate the body frame angular velocities to follow the target attitude.
    /// `attitude_body` represents a quaternion rotation in NED frame to the body
    pub fn attitude_control(&mut self, attitude_body: Quaternion<f32>) {
        // Vector representing the angular error to rotate the thrust vector using x and y and heading using z
        let (_attitude_target, attitude_error) =
            self.thrust_heading_rotation_angles(self.attitude_target, attitude_body);

        // Compute the angular velocity corrections in the body frame from the attitude error
        self.ang_vel_body = self.angular_velocity_target_from_attitude_error(attitude_error);
    }

    // Calculates two ordered rotations to move `attitude_body` to `attitude_target`.
    // The maximum error in the yaw axis is limited based on the angle yaw P value and acceleration.
    pub fn thrust_heading_rotation_angles(
        &self,
        mut attitude_target: Quaternion<f32>,
        attitude_body: Quaternion<f32>,
    ) -> (Quaternion<f32>, Vector3<f32>) {
        let ac_attitude_accel_y_controller_max_radss = 120f32.to_radians();

        let (thrust_angle, thrust_vector_correction, mut attitude_error, thrust_error_angle) =
            thrust_vector_rotation_angles(attitude_target, attitude_body);

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

// get euler roll angle
pub fn euler_roll(q: Quaternion<f32>) -> f32 {
    (2. * (q[0] * q[1] + q[2] * q[3])).atan2(1. - 2. * (q[1] * q[1] + q[2] * q[2]))
}

// get euler pitch angle
pub fn euler_pitch(q: Quaternion<f32>) -> f32 {
    safe_asin(2. * (q[0] * q[2] - q[3] * q[1]))
}

// get euler yaw angle
pub fn euler_yaw(q: Quaternion<f32>) -> f32 {
    (2. * (q[0] * q[3] + q[1] * q[2])).atan2(1. - 2. * (q[2] * q[2] + q[3] * q[3]))
}

// create eulers from a quaternion
pub fn to_euler(q: Quaternion<f32>) -> Vector3<f32> {
    Vector3::new(euler_roll(q), euler_pitch(q), euler_yaw(q))
}

fn safe_asin(f: f32) -> f32 {
    if f.is_nan() {
        0.
    } else if f >= 1. {
        PI * 2.
    } else if f <= -1. {
        -PI * 2.
    } else {
        f.asin()
    }
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative (in radians/second).
// Returns None if the vehicle is pitched 90 degrees up or down
pub fn ang_vel_to_euler_rate(
    euler_rad: Vector3<f32>,
    ang_vel_rads: Vector3<f32>,
) -> Option<Vector3<f32>> {
    let sin_theta = euler_rad.y.sin();
    let cos_theta = euler_rad.y.cos();
    let sin_phi = euler_rad.x.sin();
    let cos_phi = euler_rad.x.cos();

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous.
    // In this case, we return None
    if cos_theta != 0. {
        let euler_rate_rads = Vector3::new(
            ang_vel_rads.x
                + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y
                + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z,
            cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z,
            (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z,
        );
        Some(euler_rate_rads)
    } else {
        None
    }
}

pub struct P {
    pub kp: f32,
}

impl Default for P {
    fn default() -> Self {
        Self::new(0.)
    }
}

impl P {
    pub fn new(kp: f32) -> Self {
        Self { kp }
    }
}

pub fn ang_vel_limit(mut euler_rad: Vector3<f32>, ang_vel_max: Vector3<f32>) -> Vector3<f32> {
    if ang_vel_max[0] == 0. || ang_vel_max[1] == 0. {
        if ang_vel_max[0] != 0. {
            euler_rad.x = euler_rad.x.max(-ang_vel_max[0]).min(ang_vel_max[0]);
        }
        if ang_vel_max[1] != 0. {
            euler_rad.y = euler_rad.y.max(-ang_vel_max[1]).min(ang_vel_max[1]);
        }
    } else {
        let thrust_vector_ang_vel =
            Vector2::new(euler_rad.x / ang_vel_max[0], euler_rad.y / ang_vel_max[1]);
        let thrust_vector_length = thrust_vector_ang_vel.norm();
        if thrust_vector_length > 1. {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_max[0] / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_max[1] / thrust_vector_length;
        }
    }
    if ang_vel_max[2] != 0. {
        euler_rad.z = euler_rad.z.max(-ang_vel_max[2]).min(ang_vel_max[2]);
    }

    euler_rad
}

/// thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body quaternion to the attitude_target quaternion.
/// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
pub fn thrust_vector_rotation_angles(
    attitude_target: Quaternion<f32>,
    attitude_body: Quaternion<f32>,
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
    let thrust_error_angle = ((att_body_thrust_vec.dot(&att_target_thrust_vec))
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
    thrust_vec_cross = mul(
        attitude_body.try_inverse().unwrap_or_default(),
        thrust_vec_cross,
    );
    let thrust_vector_correction = from_axis_angle_with_theta(thrust_vec_cross, thrust_error_angle);

    // calculate the angle error in x and y.
    let mut rotation = to_axis_angle(thrust_vector_correction);
    let attitude_error_x = rotation.x;
    let attitude_error_y = rotation.y;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    // heading_vector_correction
    let heading_vec_correction_quat = thrust_vector_correction.try_inverse().unwrap_or_default()
        * attitude_body.try_inverse().unwrap_or_default()
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
    //    v2 = v1 + 2 q[0] * qv x v1 + 2 qv x qv x v1
    //
    // where "x" is the cross product (explicitly inlined for performance below),
    // "q[0]" is the scalar part and "qv" is the vector part of this quaternion

    let mut ret = v;

    // Compute and cache "qv x v1"
    let mut uv = [
        quat[2] * v.z - quat[3] * v.y,
        quat[3] * v.x - quat[1] * v.z,
        quat[1] * v.y - quat[2] * v.x,
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
