use crate::{hal::Actuator, Sensors};
use core::f32::consts::{FRAC_PI_2, PI};
use nalgebra::{Vector2, Vector3};
use pid_controller::P;

pub mod control;
use control::{LateralControl, LongitudinalControl};

pub trait Mode<T> {
    fn run(&mut self, vehicle: &mut T, dt: f32);
}

/// Output the commands needed to orbit a radius (in meters) around a center location in the NED frame (in meters).
/// Maintains the commanded airspeed (in m/s) with time-step `dt` in seconds.
/// If `clockwise` is set to `true`, the course will be set in the opposite direction.
pub struct Orbit {
    p: P<f32>,
    airspeed_cmd: f32,
    center: Vector3<f32>,
    orbit_radius: f32,
    clockwise: bool,
}

impl<S, A, E, T> Mode<Plane<S, A, E, T>> for Orbit
where
    S: Sensors<Vector3<f32>>,
    A: Actuator<f32>,
    E: Actuator<f32>,
    T: Actuator<f32>,
{
    fn run(&mut self, plane: &mut Plane<S, A, E, T>, dt: f32) {
        let position = plane.sensors.position();

        let attitude = plane.sensors.attitude();
        let yaw = attitude.z;

        let radius = (Vector2::new(self.center.x, self.center.y)
            - Vector2::new(position.x, position.y))
        .norm();

        let course_cmd =
            FRAC_PI_2 + (self.p.control(radius, self.orbit_radius) / self.orbit_radius).atan();
        let course_cmd = if self.clockwise {
            course_cmd
        } else {
            -course_cmd
        };

        let mut addon = (position[1] - self.center[1]).atan2(position[0] - self.center[0]);
        if addon - yaw < -PI {
            while addon - yaw < -PI {
                addon += PI * 2.;
            }
        } else if addon - yaw > PI {
            while addon - yaw > PI {
                addon -= PI * 2.;
            }
        }

        let yaw_cmd = course_cmd + addon;
        let roll_cmd = plane.lateral_control.yaw_control(yaw_cmd, yaw, dt, 0.);

        let aileron_cmd =
            plane
                .lateral_control
                .roll_control(roll_cmd, attitude.x, plane.sensors.gyro().x);
        let (throttle_cmd, elevator_cmd) = plane.longitudinal_control.airspeed_control(
            plane.sensors.velocity().x,
            self.airspeed_cmd,
            attitude.y,
            plane.sensors.gyro().y,
            dt,
            0.,
        );

        plane.aileron.output(aileron_cmd);
        plane.elevator.output(elevator_cmd);
        plane.throttle.output(throttle_cmd);
    }
}

pub struct Plane<S, A, E, T> {
    pub sensors: S,
    pub aileron: A,
    pub elevator: E,
    pub throttle: T,
    pub longitudinal_control: LongitudinalControl,
    pub lateral_control: LateralControl,
}
