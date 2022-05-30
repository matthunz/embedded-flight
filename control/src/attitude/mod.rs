use nalgebra::Vector3;

mod multi_copter;
pub use multi_copter::MultiCopterAttitudeController;

pub struct AttitudeController {
    // The angular velocity (in radians per second) in the body frame.
    pub ang_vel_body: Vector3<f32>,
    pub actuator_sysid: Vector3<f32>,
    pub sysid_ang_vel_body: Vector3<f32>,
    pub feed_forward_scalar: f32,
    pub throttle_rpy_mix: f32,
    pub throttle_rpy_mix_desired: f32,
    pub attitude_control_max: f32,
    pub dt: f32,
}
