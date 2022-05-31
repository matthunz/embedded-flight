use embedded_flight_motors::{esc::ESC, MotorMatrix};
use embedded_time::{rate::Fraction, Clock, Instant};
use nalgebra::{Quaternion, Vector3};

use embedded_flight::core::IntertialSensor;
use embedded_flight::MultiCopter;

struct ExampleESC;

impl ESC for ExampleESC {
    type Output = f32;

    fn arm(&mut self) {}

    fn output(&mut self, output: Self::Output) {}
}

struct ExampleInertialSensor;

impl IntertialSensor for ExampleInertialSensor {
    fn attitude(&mut self) -> Quaternion<f32> {
        Quaternion::default()
    }

    fn gyro(&mut self) -> Vector3<f32> {
        Vector3::zeros()
    }
}

struct ExampleClock;

impl Clock for ExampleClock {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1);

    fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(0))
    }
}

fn main() {
    let motor_matrix = MotorMatrix::quad(ExampleESC, ExampleESC, ExampleESC, ExampleESC);

    let mut copter = MultiCopter::new(motor_matrix, ExampleInertialSensor, ExampleClock, 400);
    copter.run();
}
