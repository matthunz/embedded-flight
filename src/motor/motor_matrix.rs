use super::{Context, Motors, SpoolState};
use crate::ESC;

pub struct Motor<E> {
    esc: E,
    is_enabled: bool,
    thrust_rpyt_out: f32,
    actuator: f32,
}

impl<E> Motor<E> {
    pub fn new(esc: E) -> Self {
        Self {
            esc,
            is_enabled: true,
            thrust_rpyt_out: 0.,
            actuator: 0.,
        }
    }
}

pub struct MotorMatrix<E, const N: usize> {
    pub motors: [Motor<E>; N],
}

impl<E, const N: usize> MotorMatrix<E, N>
where
    E: ESC<i16>,
{
    pub fn new(motors: [Motor<E>; N]) -> Self {
        Self { motors }
    }
}

impl<E: ESC<i16>, const N: usize> Motors for MotorMatrix<E, N> {
    fn output_armed_stabilizing(&mut self, cx: &mut Context) {
        // apply voltage and air pressure compensation
        let compensation_gain = cx.compensation_gain(); // compensation for battery voltage and altitude
        let _roll_thrust = (cx.roll_in + cx.roll_in_ff) * compensation_gain;
        let _pitch_thrust = (cx.pitch_in + cx.pitch_in_ff) * compensation_gain;
        let _yaw_thrust = (cx.yaw_in + cx.yaw_in_ff) * compensation_gain;
        let _throttle_thrust = cx.throttle() * compensation_gain;
        let _throttle_avg_max = cx.throttle_avg_max * compensation_gain;
    }

    fn output_to_motors(&mut self, cx: &mut Context) {
        match cx.spool_state {
            SpoolState::ShutDown => {
                for motor in &mut self.motors {
                    if motor.is_enabled {
                        motor.actuator = 0.;
                    }
                }
            }
            SpoolState::GroundIdle => {
                // sends output to motors when armed but not flying
                for motor in &mut self.motors {
                    if motor.is_enabled {
                        motor.actuator = cx.actuator_with_slew(
                            motor.actuator,
                            cx.actuator_spin_up_to_ground_idle(),
                        );
                    }
                }
            }
            SpoolState::SpoolingDown | SpoolState::SpoolingUp | SpoolState::ThrottleUnlimited => {
                // Set motor output based on thrust requests
                for motor in &mut self.motors {
                    if motor.is_enabled {
                        motor.actuator = cx.actuator_with_slew(
                            motor.actuator,
                            cx.thrust_to_actuator(motor.thrust_rpyt_out),
                        );
                    }
                }
            }
        };

        for motor in &mut self.motors {
            if motor.is_enabled {
                let pwm = cx.output_to_pwm(motor.actuator);
                motor.esc.output(pwm);
            }
        }
    }
}
