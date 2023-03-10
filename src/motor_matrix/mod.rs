mod multi_copter;
use crate::ESC;

pub use self::multi_copter::{MultiCopterMotors, SpoolState};

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
    pub controller: MultiCopterMotors,
}

impl<E, const N: usize> MotorMatrix<E, N>
where
    E: ESC<i16>,
{
    pub fn new(motors: [Motor<E>; N]) -> Self {
        Self {
            motors,
            controller: MultiCopterMotors::default(),
        }
    }

    pub fn output(&mut self, dt: u32) {
        self.controller.update_throttle_filter(dt);

        self.output_to_motors();
    }

    pub fn output_to_motors(&mut self) {
        match self.controller.spool_state {
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
                        motor.actuator = self.controller.actuator_with_slew(
                            motor.actuator,
                            self.controller.actuator_spin_up_to_ground_idle(),
                        );
                    }
                }
            }
            SpoolState::SpoolingDown | SpoolState::SpoolingUp | SpoolState::ThrottleUnlimited => {
                // Set motor output based on thrust requests
                for motor in &mut self.motors {
                    if motor.is_enabled {
                        motor.actuator = self.controller.actuator_with_slew(
                            motor.actuator,
                            self.controller.thrust_to_actuator(motor.thrust_rpyt_out),
                        );
                    }
                }
            }
        };

        for motor in &mut self.motors {
            if motor.is_enabled {
                let pwm = self.controller.output_to_pwm(motor.actuator);
                motor.esc.output(pwm);
            }
        }
    }
}
