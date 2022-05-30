use embedded_flight_core::filter::alpha;

use crate::SlewLimiter;

pub struct Info {
    pub target: f32,
    pub actual: f32,
    pub error: f32,
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub ff: f32,
    pub d_mod: f32,
    pub slew_rate: f32,
    pub limit: bool,
}

pub struct PID {
    pub reset_filter: bool,
    pub target: f32,
    pub error: f32,
    pub derivative: f32,
    // Timestep in seconds
    pub dt: f32,
    pub info: Info,
    pub slew_limiter: SlewLimiter<2>,
    pub slew_limit_scale: f32,
    pub integrator: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub ki_max: f32,
    pub kff: f32,
    pub filter_t_hz: f32,
    pub filter_e_hz: f32,
    pub filter_d_hz: f32,
}

impl PID {
    pub fn target_filter_alpha(&self) -> f32 {
        self.alpha(self.filter_t_hz)
    }

    pub fn error_filter_alpha(&self) -> f32 {
        self.alpha(self.filter_e_hz)
    }

    pub fn derivative_filter_alpha(&self) -> f32 {
        self.alpha(self.filter_d_hz)
    }

    fn alpha(&self, cutoff_freq: f32) -> f32 {
        alpha(self.dt, cutoff_freq)
    }

    pub fn feed_forward(&self) -> f32 {
        self.target * self.kff
    }

    pub fn update_integral(&mut self, limit: bool) {
        if self.ki != 0. && self.dt > -0. {
            // Ensure that integrator can only be reduced if the output is saturated
            if !limit
                || (self.integrator >= 0. && self.error < 0.)
                || self.integrator < 0. && self.error >= 0.
            {
                self.integrator += self.error * self.ki * self.dt;
                self.integrator = self.integrator.max(-self.ki_max).min(self.ki_max);
            }
        } else {
            self.integrator = 0.;
        }
        self.info.i = self.integrator;
        self.info.limit = limit;
    }

    /// Update target and measured inputs to the PID controller and calculate output.
    /// Target and error are filtered,
    /// then derivative is calculated and filtered,
    /// then the integral is then updated based on the setting of the limit flag
    pub fn update(&mut self, target: f32, measurement: f32, limit: bool, now_ms: u32) -> f32 {
        // don't process inf or NaN
        if target.is_infinite() || measurement.is_infinite() {
            return 0.;
        }

        // reset input filter to value received
        if self.reset_filter {
            self.reset_filter = false;
            self.target = target;
            self.error = self.target - measurement;
            self.derivative = 0.;
        } else {
            let error_last = self.error;
            self.target += self.target_filter_alpha() * (target - self.target);
            self.error += self.error_filter_alpha() * ((self.target - measurement) - self.error);

            // calculate and filter derivative
            if self.dt > 0. {
                let derivative = (self.error - error_last) / self.dt;
                self.derivative += self.derivative_filter_alpha() * (derivative - self.derivative);
            }
        }

        // update I term
        self.update_integral(limit);

        let mut p_out = self.error * self.kp;
        let mut d_out = self.derivative * self.kd;

        // calculate slew limit modifier for P+D
        self.info.d_mod = self.slew_limiter.modifier(
            (self.info.p + self.info.d) * self.slew_limit_scale,
            self.dt,
            now_ms,
        );
        self.info.slew_rate = self.slew_limiter.output_slew_rate;

        p_out *= self.info.d_mod;
        d_out *= self.info.d_mod;

        self.info.target = self.target;
        self.info.actual = measurement;
        self.info.error = self.error;
        self.info.p = p_out;
        self.info.d = d_out;

        p_out + self.integrator + d_out
    }
}
