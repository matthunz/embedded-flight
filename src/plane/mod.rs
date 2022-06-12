use crate::{hal::Actuator, Sensors};
use core::f32::consts::{FRAC_PI_2, PI};
use nalgebra::{Vector2, Vector3};
use pid_controller::P;

pub mod control;
use control::{LateralControl, LongitudinalControl};

pub struct Plane<S, A, E, T> {
    pub sensors: S,
    pub aileron: A,
    pub elevator: E,
    pub throttle: T,
    pub longitudinal_control: LongitudinalControl,
    pub lateral_control: LateralControl,
    pub orbit_control: P<f32>,
}

impl<S, A, E, T> Plane<S, A, E, T>
where
    S: Sensors<Vector3<f32>>,
    A: Actuator,
    E: Actuator,
    T: Actuator,
{
    /// Output the commands needed to orbit a radius (in meters) around a center location in the NED frame (in meters).
    /// Maintains the commanded airspeed (in m/s) with time-step `dt` in seconds.
    /// If `clockwise` is set to `true`, the course will be set in the opposite direction.
    pub fn orbit(
        &mut self,
        airspeed_cmd: f32,
        center: Vector3<f32>,
        orbit_radius: f32,
        clockwise: bool,
        dt: f32,
    ) {
        // Local position (in NED frame)
        let position = self.sensors.position();

        let attitude = self.sensors.attitude();
        let yaw = attitude.z;

        let radius =
            (Vector2::new(center.x, center.y) - Vector2::new(position.x, position.y)).norm();

        let course_cmd =
            FRAC_PI_2 + (self.orbit_control.control(radius, orbit_radius) / orbit_radius).atan();
        let course_cmd = if clockwise { course_cmd } else { -course_cmd };

        let mut addon = (position[1] - center[1]).atan2(position[0] - center[0]);
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
        let roll_cmd = self.lateral_control.yaw_control(yaw_cmd, yaw, dt, 0.);

        let aileron_cmd =
            self.lateral_control
                .roll_control(roll_cmd, attitude.x, self.sensors.gyro().x);
        let (throttle_cmd, elevator_cmd) = self.longitudinal_control.airspeed_control(
            self.sensors.velocity().x,
            airspeed_cmd,
            attitude.y,
            self.sensors.gyro().y,
            dt,
            0.,
        );

        self.aileron.output(aileron_cmd);
        self.elevator.output(elevator_cmd);
        self.throttle.output(throttle_cmd);
    }
}
