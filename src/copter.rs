use crate::control::{Moment, PositionController, Trajectory};
use crate::motors::{esc::ESC, MotorMatrix};
use crate::AHRS;
use embedded_time::{clock, duration::Seconds, Clock, Instant};
use nalgebra::Vector3;

pub struct Copter<C: Clock, A, E, const N: usize> {
    pub controller: PositionController,
    pub motors: MotorMatrix<E, f32, N>,
    pub max_thrust: f32,
    pub max_radian_rate: f32,
    pub last_position: Vector3<f32>,
    pub last_time: f32,
    pub takeoff_time: Instant<C>,
    pub clock: C,
    pub ahrs: A,
}

impl<C, A, E, const N: usize> Copter<C, A, E, N>
where
    C: Clock<T = u32>,
    A: AHRS,
    E: ESC<Output = f32>,
{
    pub fn take_off(&mut self, height: f32, time: f32) -> Result<(), clock::Error> {
        self.motors.arm();
        self.takeoff_time = self.clock.try_now()?;
        
        self.fly(Vector3::new(0., 0., height), time)
    }

    pub fn fly(&mut self, to: Vector3<f32>, time: f32) -> Result<(), clock::Error> {
        let now: u32 = Seconds::try_from(self.clock.try_now()? - self.takeoff_time)
            .unwrap()
            .0;
        let trajectory =
            Trajectory::calculate(self.last_position, self.last_time, to, time, now as f32);

        let motor_cmd = self.controller.trajectory_control(
            trajectory,
            self.ahrs.local_position(),
            self.ahrs.local_velocity(),
            self.ahrs.attitude(),
            self.ahrs.angle_rates(),
        );
        self.output_moment(motor_cmd);

        Ok(())
    }

    pub fn output_moment(&mut self, moment: Moment) {
        self.motors.output(
            moment.attitude / self.max_radian_rate,
            Vector3::new(0., 0., moment.thrust / self.max_thrust),
        );
    }
}
