use crate::ESC;
use embedded_flight_core::{filter::LowPassFilter, MotorOutput};
use num_traits::{Float, FloatConst, FromPrimitive};

mod motor;
pub use motor::Motor;

pub struct Limit {
    pub roll: bool,
    pub pitch: bool,
    pub yaw: bool,
    pub throttle_lower: bool,
    pub throttle_upper: bool,
}

pub struct MotorMatrix<E, T, const N: usize> {
    pub motors: [Motor<E, T>; N],
    // In -1..1
    throttle_avg_max: T,
    throttle_filter: LowPassFilter<T>,
    rpy_scale: T,
    throttle_thrust_max: T,
    loop_rate: T,
    // choice between highest and second highest motor output for output mixing (0 ~ 1). Zero is normal operation
    pub thrust_boost_ratio: T,
    pub compensation_gain: T,
}

impl<E, T> MotorMatrix<E, T, 4>
where
    E: ESC<Output = T>,
    T: Float + FloatConst + FromPrimitive,
{
    /// Create a new `MotorMatrix` for a quad-copter with 4 ESC motors.
    pub fn quad(a: E, b: E, c: E, d: E) -> Self {
        Self::from_motors([
            Motor::from_angle(a, T::from_i8(90).unwrap(), T::from_i8(1).unwrap()),
            Motor::from_angle(b, T::from_i8(-90).unwrap(), T::from_i8(1).unwrap()),
            Motor::from_angle(c, T::from_i8(0).unwrap(), T::from_i8(-1).unwrap()),
            Motor::from_angle(d, T::from_i16(180).unwrap(), T::from_i8(-1).unwrap()),
        ])
    }
}

impl<E, T, const N: usize> MotorMatrix<E, T, N>
where
    E: ESC<Output = T>,
    T: Float + FloatConst + FromPrimitive,
{
    pub fn from_motors(motors: [Motor<E, T>; N]) -> Self {
        Self {
            motors,
            throttle_avg_max: T::zero(),
            throttle_filter: LowPassFilter::default(),
            rpy_scale: T::one(),
            throttle_thrust_max: T::one(),
            loop_rate: T::zero(),
            thrust_boost_ratio: T::zero(),
            compensation_gain: T::one(),
        }
    }

    pub fn arm(&mut self) {
        for motor in &mut self.motors {
            motor.esc.arm();
        }
    }

    pub fn output(&mut self, output: MotorOutput<T>) -> Limit {
        self.output_with_throttle(output, T::zero())
    }

    pub fn output_with_throttle(&mut self, output: MotorOutput<T>, throttle: T) -> Limit {
        let roll_thrust = (output.control[0] + output.feed_forward[0]) * self.compensation_gain;
        let pitch_thrust = (output.control[1] + output.feed_forward[1]) * self.compensation_gain;
        let yaw_thrust = (output.control[2] + output.feed_forward[2]) * self.compensation_gain;
        let mut throttle_thrust =
            self.throttle_filter
                .filter(throttle, T::zero(), T::one() / self.loop_rate);

        let throttle_thrust_max = self.thrust_boost_ratio
            + (T::one() - self.thrust_boost_ratio)
                * self.throttle_thrust_max
                * self.compensation_gain;

        // Throttle providing maximum roll, pitch and yaw range without climbing
        let mut throttle_thrust_best_rpy = T::from_f32(0.5).unwrap().min(self.throttle_avg_max);

        let mut limit = Limit {
            roll: false,
            pitch: false,
            yaw: false,
            throttle_lower: false,
            throttle_upper: false,
        };

        // Sanity check throttle is above zero and below current limited throttle
        if throttle_thrust <= T::zero() {
            throttle_thrust = T::zero();
            limit.throttle_lower = true;
        }
        if throttle_thrust >= throttle_thrust_max {
            throttle_thrust = throttle_thrust_max;
            limit.throttle_upper = true;
        }

        let mut yaw_allowed = T::one();

        let mut rp_low = T::one(); // lowest thrust value
        let mut rp_high = -T::one(); // highest thrust value
        for motor in &mut self.motors {
            // calculate the thrust outputs for roll and pitch
            motor.thrust_rpyt_out = roll_thrust * motor.factor[0] + pitch_thrust * motor.factor[1];
            // record lowest roll + pitch command
            if motor.thrust_rpyt_out < rp_low {
                rp_low = motor.thrust_rpyt_out;
            }
            // record highest roll + pitch command
            if motor.thrust_rpyt_out > rp_high {
                rp_high = motor.thrust_rpyt_out;
            }

            // Check the maximum yaw control that can be used on this channel
            // Exclude any lost motors if thrust boost is enabled
            if motor.factor[2] == T::zero() {
                if (yaw_thrust * motor.factor[2]) >= T::zero() {
                    yaw_allowed = yaw_allowed.min(
                        ((T::one()
                            - (throttle_thrust_best_rpy + motor.thrust_rpyt_out).max(T::zero()))
                            / motor.factor[2])
                            .abs(),
                    );
                } else {
                    yaw_allowed = yaw_allowed.min(
                        ((throttle_thrust_best_rpy + motor.thrust_rpyt_out.min(T::zero()))
                            / motor.factor[2])
                            .abs(),
                    );
                }
            }
        }

        let mut rpy_low = T::one(); // lowest thrust value
        let mut rpy_high = -T::one(); // highest thrust value
        for motor in &mut self.motors {
            motor.thrust_rpyt_out = motor.thrust_rpyt_out + yaw_thrust * motor.factor[2];

            // record lowest roll + pitch + yaw command
            if motor.thrust_rpyt_out < rpy_low {
                rpy_low = motor.thrust_rpyt_out;
            }

            // record highest roll + pitch + yaw command
            // Exclude any lost motors if thrust boost is enabled
            if motor.thrust_rpyt_out > rpy_high {
                rpy_high = motor.thrust_rpyt_out;
            }
        }

        self.rpy_scale = if rpy_high - rpy_low > T::one() {
            T::one() / (rpy_high - rpy_low)
        } else {
            self.rpy_scale.min(-self.throttle_avg_max / rpy_low)
        };
        self.throttle_avg_max = self
            .throttle_avg_max
            .max(throttle_thrust)
            .min(self.throttle_thrust_max);

        // calculate how close the motors can come to the desired throttle
        rpy_high = rpy_high * self.rpy_scale;
        rpy_low = rp_low * self.rpy_scale;
        throttle_thrust_best_rpy = -rpy_low;

        // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
        let mut thr_adj = throttle_thrust - throttle_thrust_best_rpy;

        if self.rpy_scale < T::one() {
            // Full range is being used by roll, pitch, and yaw.
            limit.roll = true;
            limit.pitch = true;
            limit.yaw = true;
            if thr_adj > T::zero() {
                limit.throttle_upper = true;
            }
            thr_adj = T::zero();
        } else {
            if thr_adj < T::zero() {
                // Throttle can't be reduced to desired value
                // todo: add lower limit flag and ensure it is handled correctly in altitude controller
                thr_adj = T::zero();
            } else if thr_adj > T::one() - (throttle_thrust_best_rpy + rpy_high) {
                // Throttle can't be increased to desired value
                thr_adj = T::one() - (throttle_thrust_best_rpy + rpy_high);
                limit.throttle_upper = true;
            }
        }

        // add scaled roll, pitch, constrained yaw and throttle for each motor
        let throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
        for motor in &mut self.motors {
            motor.thrust_rpyt_out = (throttle_thrust_best_plus_adj * motor.throttle_factor)
                + (self.rpy_scale * motor.thrust_rpyt_out);

            motor.esc.output(motor.thrust_rpyt_out)
        }

        limit
    }
}
