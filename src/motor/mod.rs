mod context;
pub use self::context::{Context, SpoolState};

pub mod motor_matrix;
pub use motor_matrix::MotorMatrix;

mod tail_sitter;
pub use self::tail_sitter::TailSitterMotors;

pub struct MultiCopterMotors<M> {
    pub motors: M,
    pub cx: Context,
}

impl<M> MultiCopterMotors<M> {
    pub fn new(motors: M, cx: Context) -> Self {
        Self { motors, cx }
    }

    pub fn output(&mut self, dt: u32)
    where
        M: Motors,
    {
        // update throttle filter
        self.cx.update_throttle_filter(dt);

        // calc filtered battery voltage and lift_max
        // update_lift_max_from_batt_voltage();

        // run spool logic
        self.cx.output_logic();

        // calculate thrust
        self.motors.output_armed_stabilizing(&mut self.cx);

        // apply any thrust compensation for the frame
        // thrust_compensation();

        // convert rpy_thrust values to pwm
        self.motors.output_to_motors(&mut self.cx);

        // output any booster throttle
        //output_boost_throttle();

        // output raw roll/pitch/yaw/thrust
        // output_rpyt();

        // check for any external limit flags
        // update_external_limits();
    }
}

pub trait Motors {
    fn output_armed_stabilizing(&mut self, cx: &mut Context);

    fn output_to_motors(&mut self, cx: &mut Context);
}
