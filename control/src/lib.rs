#![no_std]

mod body_rate;
use attitude::sqrt_controller;
pub use body_rate::BodyRateController;

mod lateral_pos;
pub use lateral_pos::LateralPositionController;

mod yaw;
use nalgebra::{UnitQuaternion, Vector3};
pub use yaw::YawController;

pub mod pid;
pub use pid::PID;

pub mod attitude;

pub struct IMU {
    delta_angle_dt: f32,
    delta_velocity_dt: f32,
    attitude: UnitQuaternion<f32>,
    delta_velocity: Vector3<f32>,
}

impl IMU {
    // Acceleration in m/s/s
    pub fn down_sample(
        &mut self,
        delta_angle: Vector3<f32>,
        acceleration: Vector3<f32>,
        delta_angle_dt: f32,
        delta_velocity_dt: f32,
    ) {
        let delta_velocity = acceleration * delta_velocity_dt;
        self.down_sample_with_velocity(
            delta_angle,
            delta_velocity,
            delta_angle_dt,
            delta_velocity_dt,
        )
    }

    pub fn down_sample_with_velocity(
        &mut self,
        delta_angle: Vector3<f32>,
        delta_velocity: Vector3<f32>,
        delta_angle_dt: f32,
        delta_velocity_dt: f32,
    ) {
        // Accumulate the measurement time interval for the delta velocity and angle data
        self.delta_angle_dt += delta_angle_dt;
        self.delta_velocity_dt += delta_velocity_dt;

        // Rotate quaternion attitude from previous to new and normalize.
        // Accumulation using quaternions prevents introduction of coning errors due to down-sampling
        self.attitude = self.attitude * UnitQuaternion::new(delta_angle);

        // Rotate the latest delta velocity into body frame at the start of accumulation
        let delta_rotation = self.attitude.to_rotation_matrix();

        // Apply the delta velocity to the delta velocity accumulator
        self.delta_velocity += delta_rotation * delta_velocity;
    }
}

pub struct P1 {
    pub error: f32,
    pub kp: f32,
}

pub struct BasicPID {
    pub error: f32,
}

pub struct PositionController {
    pub pos_target: Vector3<f32>,
    pub vel_desired: Vector3<f32>,
    pub accel_desired: Vector3<f32>,
    pub over_speed_gain: f32,
    pub vel_max_down_cms: f32,
    pub vel_max_up_cms: f32,
    pub accel_max_z_cmss: f32,
    pub jerk_max_z_cmsss: f32,
    pub dt: f32,
    pub limit_vector: Vector3<f32>,
    pub p_pos_z: P1,
    pub pid_vel_z: BasicPID,
}

impl PositionController {
    pub fn update_z_control(&mut self) {}

    pub fn land_at_climb_rate_cm(&mut self, vel: f32, ignore_descent_limit: bool) {
        self.input_vel_accel_z(vel, 0., ignore_descent_limit);
    }

    pub fn input_vel_accel_z(&mut self, vel: f32, accel: f32, ignore_descent_limit: bool) -> f32 {
        self.input_vel_accel_z_with_limit(vel, accel, ignore_descent_limit, true)
    }

    pub fn input_vel_accel_z_with_limit(
        &mut self,
        vel: f32,
        accel: f32,
        ignore_descent_limit: bool,
        limit_output: bool,
    ) -> f32 {
        if ignore_descent_limit {
            // turn off limits in the negative z direction
            self.limit_vector.z = self.limit_vector.z.max(0.);
        }

        let accel_max_z_cmss = self.accel_max_z_cmss * self.over_speed_gain();
        let jerk_max_z_cmsss = self.jerk_max_z_cmsss * self.over_speed_gain();
        {
            let (pos, vel) = update_pos_vel_accel(
                self.pos_target.z,
                self.vel_desired.z,
                self.accel_desired.z,
                self.dt,
                self.limit_vector.z,
                self.p_pos_z.error,
                self.pid_vel_z.error,
            );
            self.pos_target.z = pos;
            self.vel_desired.z = vel;
        }

        let accel = shape_vel_accel(
            vel,
            accel,
            self.vel_desired.z,
            self.accel_desired.z,
            -(accel_max_z_cmss.max(0.0).min(750.0)),
            accel_max_z_cmss,
            jerk_max_z_cmsss,
            self.dt,
            limit_output,
        );

        update_vel_accel(vel, accel, self.dt, 0., 0.)
    }

    // calculate_overspeed_gain - calculated increased maximum acceleration and jerk if over speed condition is detected
    pub fn over_speed_gain(&self) -> f32 {
        if self.vel_desired.z < self.vel_max_down_cms && self.vel_max_down_cms != 0. {
            self.over_speed_gain * self.vel_desired.z / self.vel_max_down_cms
        } else if self.vel_desired.z > self.vel_max_up_cms && self.vel_max_up_cms != 0. {
            self.over_speed_gain * self.vel_desired.z / self.vel_max_up_cms
        } else {
            1.
        }
    }
}

// update_pos_vel_accel - single axis projection of position and velocity forward in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
pub fn update_pos_vel_accel(
    mut pos: f32,
    vel: f32,
    accel: f32,
    dt: f32,
    limit: f32,
    pos_error: f32,
    vel_error: f32,
) -> (f32, f32) {
    // move position and velocity forward by dt if it does not increase error when limited.
    let delta_pos = vel * dt + accel * 0.5 * dt * dt;
    // do not add delta_pos if it will increase the velocity error in the direction of limit
    if !(delta_pos * limit > 0. && pos_error * limit > 0.) {
        pos += delta_pos;
    }

    (pos, update_vel_accel(vel, accel, dt, limit, vel_error))
}

// update_vel_accel - single axis projection of velocity, vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// vel_error - specifies the direction of the velocity error used in limit handling.
pub fn update_vel_accel(vel: f32, accel: f32, dt: f32, limit: f32, vel_error: f32) -> f32 {
    let delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the direction of limit
    if !(delta_vel * limit > 0. && vel_error * limit > 0.) {
        vel + delta_vel
    } else {
        vel
    }
}

/* shape_vel_accel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    maximum velocity - vel_max,
    maximum acceleration - accel_max,
    time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to vel_input and accel_input.
 The accel_max limit can be removed by setting it to zero.
*/
pub fn shape_vel_accel(
    vel_input: f32,
    accel_input: f32,
    vel: f32,
    accel: f32,
    accel_min: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    limit_total_accel: bool,
) -> f32 {
    // sanity check accel_max
    if !(accel_min < 0. && accel_max < 0.) {
        // INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        todo!()
    }

    // velocity error to be corrected
    let vel_error = vel_input - vel;

    // Calculate time constants and limits to ensure stable operation
    // The direction of acceleration limit is the same as the velocity error.
    // This is because the velocity error is negative when slowing down while
    // closing a positive position error.
    let kpa = if vel_error > 0. {
        jerk_max / accel_max
    } else {
        jerk_max / (-accel_min)
    };

    // acceleration to correct velocity
    // constrain correction acceleration from accel_min to accel_max
    let mut accel_target = sqrt_controller(vel_error, kpa, jerk_max, dt)
        .max(accel_min)
        .min(accel_max)
        + accel_input;

    // constrain total acceleration from accel_min to accel_max
    if limit_total_accel {
        accel_target = accel_target.max(accel_min).min(accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt)
}

/* shape_accel calculates a jerk limited path from the current acceleration to an input acceleration.
The function takes the current acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
The kinematic path is constrained by :
   acceleration limits - accel_min, accel_max,
   time constant - tc.
The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
The time constant also defines the time taken to achieve the maximum acceleration.
The time constant must be positive.
The function alters the variable accel to follow a jerk limited kinematic path to accel_input.
*/
pub fn shape_accel(accel_input: f32, accel: f32, jerk_max: f32, dt: f32) -> f32 {
    // jerk limit acceleration change
    let mut accel_delta = accel_input - accel;
    if jerk_max > 0. {
        accel_delta = (accel_delta).max(-jerk_max * dt).min(jerk_max * dt);
    }
    accel + accel_delta
}

// From `t_rise` and `delta` returns kp and kd
fn pid_config(t_rise: f32, delta: f32) -> (f32, f32) {
    let w = 1. / (1.57 * t_rise);
    (w * w, 2. * delta * w)
}
