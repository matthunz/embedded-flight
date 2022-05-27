use crate::Copter;
use embedded_flight_control::PositionController;
use embedded_flight_motors::MotorMatrix;
use embedded_time::{Clock, Instant};
use nalgebra::Vector3;

pub struct Builder<C, A, E, const N: usize> {
    controller: Option<PositionController>,
    motors: Option<MotorMatrix<E, f32, N>>,
    ahrs: Option<A>,
    clock: Option<C>,
    max_thrust: f32,
    max_radian_rate: f32,
}

impl<C, A, E, const N: usize> Default for Builder<C, A, E, N> {
    fn default() -> Self {
        Self {
            controller: None,
            motors: None,
            ahrs: None,
            clock: None,
            max_thrust: 10.,
            max_radian_rate: 10.,
        }
    }
}

impl<C, A, E, const N: usize> Builder<C, A, E, N>
where
    C: Clock<T = u32>,
{
    pub fn controller(mut self, controller: PositionController) -> Self {
        self.controller = Some(controller);
        self
    }

    pub fn motors(mut self, motors: MotorMatrix<E, f32, N>) -> Self {
        self.motors = Some(motors);
        self
    }

    pub fn ahrs(mut self, ahrs: A) -> Self {
        self.ahrs = Some(ahrs);
        self
    }

    pub fn clock(mut self, clock: C) -> Self {
        self.clock = Some(clock);
        self
    }

    pub fn max_thrust(mut self, thrust: f32) -> Self {
        self.max_thrust = thrust;
        self
    }

    pub fn max_radian_rate(mut self, rate: f32) -> Self {
        self.max_radian_rate = rate;
        self
    }

    pub fn build(self) -> Copter<C, A, E, N> {
        Copter {
            controller: self.controller.unwrap(),
            motors: self.motors.unwrap(),
            clock: self.clock.unwrap(),
            ahrs: self.ahrs.unwrap(),
            max_thrust: self.max_thrust,
            max_radian_rate: self.max_radian_rate,
            last_position: Vector3::zeros(),
            last_time: 0.,
            takeoff_time: Instant::new(0),
        }
    }
}
