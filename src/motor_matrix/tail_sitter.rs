use super::MultiCopterMotors;
use crate::constrain_float;

pub struct TailSitter {
    pub controller: MultiCopterMotors,
    pub thrust_left: f32,
    pub thrust_right: f32,
    pub tilt_left: f32,
    pub tilt_right: f32,
    pub external_min_throttle: f32,
    pub throttle: f32,
    pub has_diff_thrust: bool,
}

impl TailSitter {
    pub fn f(&mut self) {
        let mut roll_thrust; // roll thrust input value, +/- 1.0
        let pitch_thrust; // pitch thrust input value, +/- 1.0
        let yaw_thrust; // yaw thrust input value, +/- 1.0
        let mut throttle_thrust; // throttle thrust input value, 0.0 - 1.0
        let thrust_max; // highest motor value
        let thrust_min; // lowest motor value
        let mut thr_adj = 0.; // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

        // apply voltage and air pressure compensation
        let compensation_gain = self.controller.compensation_gain();
        roll_thrust = (self.controller.roll_in + self.controller.roll_in_ff) * compensation_gain;
        pitch_thrust = self.controller.pitch_in + self.controller.pitch_in_ff;
        yaw_thrust = self.controller.yaw_in + self.controller.yaw_in_ff;
        throttle_thrust = self.controller.throttle() * compensation_gain;
        let max_boost_throttle = self.controller.throttle_avg_max * compensation_gain;

        // never boost above max, derived from throttle mix params
        let min_throttle_out = self.external_min_throttle.min(max_boost_throttle);
        let max_throttle_out = self.controller.throttle_thrust_max * compensation_gain;

        // sanity check throttle is above min and below current limited throttle
        if throttle_thrust <= min_throttle_out {
            throttle_thrust = min_throttle_out;
            self.controller.limit.throttle_lower = true;
        }
        if throttle_thrust >= max_throttle_out {
            throttle_thrust = max_throttle_out;
            self.controller.limit.throttle_upper = true;
        }

        if roll_thrust >= 1.0 {
            // cannot split motor outputs by more than 1
            roll_thrust = 1.;
            self.controller.limit.roll = true;
        }

        // calculate left and right throttle outputs
        self.thrust_left = throttle_thrust + roll_thrust * 0.5;
        self.thrust_right = throttle_thrust - roll_thrust * 0.5;

        thrust_max = self.thrust_right.max(self.thrust_left);
        thrust_min = self.thrust_right.min(self.thrust_left);
        if thrust_max > 1.0 {
            // if max thrust is more than one reduce average throttle
            thr_adj = 1.0 - thrust_max;
            self.controller.limit.throttle_upper = true;
        } else if thrust_min < 0.0 {
            // if min thrust is less than 0 increase average throttle
            // but never above max boost
            thr_adj = -thrust_min;
            if (throttle_thrust + thr_adj) > max_boost_throttle {
                thr_adj = (max_boost_throttle - throttle_thrust).max(0.0);
                // in this case we throw away some roll output, it will be uneven
                // constraining the lower motor more than the upper
                // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
                // so its worth keeping the higher roll control power at a minor cost to yaw
                self.controller.limit.roll = true;
            }
            self.controller.limit.throttle_lower = true;
        }

        // Add adjustment to reduce average throttle
        self.thrust_left = constrain_float(self.thrust_left + thr_adj, 0.0, 1.0);
        self.thrust_right = constrain_float(self.thrust_right + thr_adj, 0.0, 1.0);

        self.throttle = throttle_thrust;

        // compensation_gain can never be zero
        // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
        if self.has_diff_thrust {
            self.controller.throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
        } else {
            self.controller.throttle_out = throttle_thrust / compensation_gain;
        }

        // thrust vectoring
        self.tilt_left = pitch_thrust - yaw_thrust;
        self.tilt_right = pitch_thrust + yaw_thrust;
    }
}
