pub mod control;
pub use control::Controller;
use esc::ESC;
use motor::Motors;
use nalgebra::Vector3;

pub mod esc;

pub mod motor;

pub trait Sensors {
    fn attitude(&mut self) -> Vector3<f32>;

    fn gyro(&mut self) -> Vector3<f32>;

    fn velocity(&mut self) -> Vector3<f32>;

    fn position(&mut self) -> Vector3<f32>;
}

pub struct Copter<S, E, M: Motors<N>, const N: usize> {
    pub controller: Controller,
    pub moment_of_inertia: Vector3<f32>,
    pub sensors: S,
    pub motors: M,
    pub escs: [E; N],
}

impl<S, E, M, const N: usize> Copter<S, E, M, N>
where
    S: Sensors,
    E: ESC<Output = f32>,
    M: Motors<N>,
{
    pub fn control(&mut self, local_position_cmd: Vector3<f32>, local_velocity_cmd: Vector3<f32>) {
        let (torque, acceleration) = self.controller.position_control(
            local_position_cmd,
            local_velocity_cmd,
            self.sensors.position(),
            self.sensors.velocity(),
            self.sensors.attitude(),
            self.sensors.gyro(),
            self.moment_of_inertia,
        );

        let thrust = self
            .motors
            .thrust(torque, acceleration, &self.moment_of_inertia);

        for (esc, thrust) in self.escs.iter_mut().zip(thrust) {
            esc.output(thrust);
        }
    }
}
