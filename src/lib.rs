use control::PositionController;
pub use embedded_flight_control as control;

pub use embedded_flight_motors as motors;
use motors::MotorMatrix;

pub struct Copter<E, T, const N: usize> {
    controller: PositionController,
    motors: MotorMatrix<E, T, N>,
}

impl<E, T, const N: usize> Copter<E, T, N> {
    pub fn output(&self) {}
}
