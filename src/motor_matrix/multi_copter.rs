use crate::{
    constrain_float,
    filter::{DerivativeFilter, LowPassFilter},
    safe_sqrt,
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SpoolState {
    GroundIdle,
    ShutDown,
    SpoolingUp,
    SpoolingDown,
    ThrottleUnlimited,
}

pub struct Limit {
    pub roll: bool,
    pub pitch: bool,
    pub yaw: bool,
    pub throttle_lower: bool,
    pub throttle_upper: bool,
}

pub struct MultiCopterMotors {
    /// Time difference (in seconds) since the last loop time
    dt: f32,

    speed_hz: u16,        // speed in hz to send updates to motors
    pub roll_in: f32,     // desired roll control from attitude controllers, -1 ~ +1
    pub roll_in_ff: f32,  // desired roll feed forward control from attitude controllers, -1 ~ +1
    pub pitch_in: f32,    // desired pitch control from attitude controller, -1 ~ +1
    pub pitch_in_ff: f32, // desired pitch feed forward control from attitude controller, -1 ~ +1
    pub yaw_in: f32,      // desired yaw control from attitude controller, -1 ~ +1
    pub yaw_in_ff: f32,   // desired yaw feed forward control from attitude controller, -1 ~ +1

    pub forward_in: f32,       // last forward input from set_forward caller
    pub lateral_in: f32,       // last lateral input from set_lateral caller
    pub throttle_avg_max: f32, // last throttle input from set_throttle_avg_max

    /// Pilot throttle input filter
    throttle_filter: LowPassFilter,

    /// Last throttle input from set_throttle caller
    throttle_in: f32,

    /// Throttle after mixing is complete
    pub throttle_out: f32,

    /// Throttle output slew detector
    throttle_slew: DerivativeFilter<7>,

    /// Throttle slew rate from input
    throttle_slew_rate: f32,

    /// Filter for the output of the throttle slew
    throttle_slew_filter: LowPassFilter,

    /// throttle increase slew limitting
    slew_up_time: f32,

    /// throttle increase slew limitting
    slew_dn_time: f32,

    ///
    batter_voltage_filter: LowPassFilter,

    /// Curve used to linearize pwm to thrust conversion. set to 0 for linear and 1 for second order approximation
    thrust_curve_exp: f32,

    /// Throttle out ratio which produces the minimum thrust. (i.e. 0 ~ 1 ) of the full throttle range
    spin_min: f32,

    /// Throttle out ratio which produces the maximum thrust. (i.e. 0 ~ 1 ) of the full throttle range
    spin_max: f32,

    /// Maximum lift ratio from battery voltage
    max_lift: f32,

    /// throttle percentage (0 ~ 1) between zero and throttle_min
    pub spin_up_ratio: f32,

    /// Armed state of the motors
    pub is_armed: bool,

    /// Minimum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's min pwm used)
    pub min_pwm: i16,

    /// Maximum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's max pwm used)
    pub max_pwm: i16,

    pub spool_state: SpoolState,

    pub spool_desired: SpoolState,

    pub spool_up_time: f32,

    pub limit: Limit,

    pub throttle_thrust_max: f32,

    /// yaw control is given at least this pwm range
    pub yaw_headroom: f32,
}

impl Default for MultiCopterMotors {
    fn default() -> Self {
        Self::new(490)
    }
}

impl MultiCopterMotors {
    pub fn new(speed_hz: u16) -> Self {
        Self {
            dt: 0.,
            speed_hz,
            roll_in: 0.,
            roll_in_ff: 0.,
            pitch_in: 0.,
            pitch_in_ff: 0.,
            yaw_in: 0.,
            yaw_in_ff: 0.,
            forward_in: 0.,
            lateral_in: 0.,
            throttle_avg_max: 0.,
            throttle_filter: LowPassFilter::with_cutoff(0.),
            throttle_in: 0.,
            throttle_out: 0.,
            throttle_slew: DerivativeFilter::new(),
            throttle_slew_rate: 0.,
            throttle_slew_filter: LowPassFilter::with_cutoff(50.),
            slew_up_time: 0.,
            slew_dn_time: 0.,
            batter_voltage_filter: LowPassFilter::with_cutoff(0.5),
            thrust_curve_exp: 0.65,
            spin_min: 0.15,
            spin_max: 0.95,
            max_lift: 1.,
            spin_up_ratio: 0.,
            is_armed: false,
            min_pwm: 1000,
            max_pwm: 2000,
            spool_state: SpoolState::GroundIdle,
            spool_desired: SpoolState::GroundIdle,
            spool_up_time: 0.5,
            throttle_thrust_max: 1.,
            yaw_headroom: 0.,
            limit: Limit {
                roll: false,
                pitch: false,
                yaw: false,
                throttle_lower: false,
                throttle_upper: false,
            },
        }
    }
    pub fn update_throttle_filter(&mut self, dt: u32) {
        let last_throttle = self.throttle_filter.output();

        if self.is_armed {
            self.throttle_filter.apply(self.throttle_in, self.dt);

            // Constrain filtered throttle
            if self.throttle_filter.output() < 0.0 {
                self.throttle_filter.reset(0.0);
            } else if self.throttle_filter.output() > 1.0 {
                self.throttle_filter.reset(1.0);
            }
        } else {
            self.throttle_filter.reset(0.0);
        }

        let new_throttle = self.throttle_filter.output();
        if last_throttle != new_throttle {
            self.throttle_slew.update(new_throttle, dt);
        }

        // Calculate slope normalized from per-micro
        let rate = (self.throttle_slew.slope() * 1e6).abs();
        self.throttle_slew_rate = self.throttle_slew_filter.apply(rate, self.dt);
    }

    /// Add slew rate limiting to actuator output
    pub fn actuator_with_slew(&self, actuator_output: f32, input: f32) -> f32 {
        /*
        If MOT_SLEW_UP_TIME is 0 (default), no slew limit is applied to increasing output.
        If MOT_SLEW_DN_TIME is 0 (default), no slew limit is applied to decreasing output.
        MOT_SLEW_UP_TIME and MOT_SLEW_DN_TIME are constrained to 0.0~0.5 for sanity.
        If spool mode is shutdown, no slew limit is applied to allow immediate disarming of motors.
        */

        // Output limits with no slew time applied
        let mut output_slew_limit_up = 1.0;
        let mut output_slew_limit_dn = 0.0;

        // If MOT_SLEW_UP_TIME is set, calculate the highest allowed new output value, constrained 0.0~1.0
        if self.slew_up_time > 0.0 {
            let output_delta_up_max = self.dt / constrain_float(self.slew_up_time, 0., 0.5);
            output_slew_limit_up = constrain_float(actuator_output + output_delta_up_max, 0., 1.);
        }

        // If MOT_SLEW_DN_TIME is set, calculate the lowest allowed new output value, constrained 0.0~1.0
        if self.slew_dn_time > 0.0 {
            let output_delta_dn_max = self.dt / constrain_float(self.slew_dn_time, 0., 0.5);
            output_slew_limit_dn = constrain_float(actuator_output - output_delta_dn_max, 0., 1.);
        }

        // Constrain change in output to within the above limits
        input.max(output_slew_limit_dn).min(output_slew_limit_up)
    }

    /// Apply_thrust_curve_and_volt_scaling. Returns throttle in the range 0 ~ 1
    pub fn apply_thrust_curve_and_volt_scaling(&self, thrust: f32) -> f32 {
        let mut battery_scale = 1.;
        if self.batter_voltage_filter.output() > 0. {
            battery_scale = 1.0 / self.batter_voltage_filter.output();
        }
        // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
        let thrust_curve_expo = constrain_float(self.thrust_curve_exp, -1., 1.);
        if thrust_curve_expo == 0. {
            // zero expo means linear, avoid floating point exception for small values
            return self.max_lift * thrust * battery_scale;
        }
        let throttle_ratio = ((thrust_curve_expo - 1.)
            + safe_sqrt(
                (1. - thrust_curve_expo) * (1. - thrust_curve_expo)
                    + 4. * thrust_curve_expo * self.max_lift * thrust,
            ))
            / (2. * thrust_curve_expo);
        constrain_float(throttle_ratio * battery_scale, 0., 1.)
    }

    pub fn thrust_to_actuator(&self, thrust: f32) -> f32 {
        let thrust_constrained = constrain_float(thrust, 0., 1.);
        return self.spin_min
            + (self.spin_max - self.spin_min)
                * self.apply_thrust_curve_and_volt_scaling(thrust_constrained);
    }

    // gradually increase actuator output to spin_min
    pub fn actuator_spin_up_to_ground_idle(&self) -> f32 {
        constrain_float(self.spin_up_ratio, 0., 1.) * self.spin_min
    }

    pub fn compensation_gain(&self) -> f32 {
        // avoid divide by zero
        if self.max_lift <= 0. {
            return 1.;
        }

        1. / self.max_lift
    }

    pub fn output_armed_stabilizing(&self) {
        // apply voltage and air pressure compensation
        let compensation_gain = self.compensation_gain(); // compensation for battery voltage and altitude
        let _roll_thrust = (self.roll_in + self.roll_in_ff) * compensation_gain;
        let _pitch_thrust = (self.pitch_in + self.pitch_in_ff) * compensation_gain;
        let _yaw_thrust = (self.yaw_in + self.yaw_in_ff) * compensation_gain;
        let _throttle_thrust = self.throttle() * compensation_gain;
        let _throttle_avg_max = self.throttle_avg_max * compensation_gain;
    }

    pub fn throttle(&self) -> f32 {
        constrain_float(self.throttle_filter.output(), 0., 1.)
    }

    // convert actuator output (0~1) range to pwm range
    pub fn output_to_pwm(&self, actuator: f32) -> i16 {
        if self.spool_state == SpoolState::ShutDown {
            // in shutdown mode, use PWM 0 or minimum PWM
            if self.is_armed {
                self.min_pwm
            } else {
                0
            }
        } else {
            // in all other spool modes, covert to desired PWM
            ((self.min_pwm + (self.max_pwm - self.min_pwm)) as f32 * actuator) as _
        }
    }

    pub fn output_logic(&mut self) {
        let spool_step = self.dt / self.spool_up_time;
        match self.spool_state {
            SpoolState::GroundIdle => {
                self.limit = Limit {
                    roll: true,
                    pitch: true,
                    yaw: true,
                    throttle_lower: true,
                    throttle_upper: true,
                };

                match self.spool_desired {
                    SpoolState::ThrottleUnlimited => {
                        self.spin_up_ratio += spool_step;
                        // constrain ramp value and update mode
                        if self.spin_up_ratio >= 1. {
                            self.spin_up_ratio = 1.;

                            // Only advance from ground idle if spoolup checks have passed
                            self.spool_state = SpoolState::SpoolingUp;
                        }
                    }
                    _ => todo!(),
                }
            }

            _ => todo!(),
        }
    }
}
