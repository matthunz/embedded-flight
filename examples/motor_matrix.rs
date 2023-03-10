use embedded_flight::{
    motor::{motor_matrix::Motor, Context, MotorMatrix, MultiCopterMotors, SpoolState},
    Actuator, ESC,
};

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
    let mut motor_matrix =
        MultiCopterMotors::new(MotorMatrix::new([Motor::new(ExampleESC)]), Context::new(1));

    motor_matrix.cx.spool_desired = SpoolState::ThrottleUnlimited;

    motor_matrix.output(1);
}
