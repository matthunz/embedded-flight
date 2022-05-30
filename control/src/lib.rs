#![no_std]

mod body_rate;
pub use body_rate::BodyRateController;

mod lateral_pos;
pub use lateral_pos::LateralPositionController;

mod yaw;
pub use yaw::YawController;

mod pid;
pub use pid::PID;

mod slew_limiter;
pub use slew_limiter::SlewLimiter;

pub mod attitude;

// From `t_rise` and `delta` returns kp and kd
fn pid_config(t_rise: f32, delta: f32) -> (f32, f32) {
    let w = 1. / (1.57 * t_rise);
    (w * w, 2. * delta * w)
}
