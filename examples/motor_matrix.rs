use embedded_flight::{Actuator, Motor, MotorMatrix, ESC};

struct ExampleESC;

impl Actuator<i16> for ExampleESC {
    fn output(&mut self, output: i16) {
        dbg!(output);
    }
}

impl ESC<i16> for ExampleESC {
    fn arm(&mut self) {
        dbg!("arming");
    }
}

fn main() {
    let mut motor_matrix = MotorMatrix::new([Motor::new(ExampleESC)]);

    motor_matrix.output(1);
}
