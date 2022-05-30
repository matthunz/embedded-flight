use embedded_flight_core::filter::alpha;

use crate::SlewLimiter;

struct Flags {
    reset_filter: bool,
}

struct Info {
    target: f32,
    actual: f32,
    error: f32,
    P: f32,
    I: f32,
    D: f32,
    FF: f32,
    Dmod: f32,
    slew_rate: f32,
    limit: bool,
}

pub struct PID {
    flags: Flags,
    target: f32,
    error: f32,
    derivative: f32,
    dt: f32,
    info: Info,
    slew_limiter: SlewLimiter<2>,
    slew_limit_scale: f32,
    integrator: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    ki_max: f32,
    kff: f32,
    filter_t_hz: f32,
    filter_e_hz: f32,
    filter_d_hz: f32,
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

    pub fn update_i(&mut self, limit: bool) {
        if (self.ki != 0. && self.dt > -0.) {
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
        self.info.I = self.integrator;
        self.info.limit = limit;
    }

    pub fn update_all(&mut self, target: f32, measurement: f32, limit: bool, now_ms: u32) -> f32 {
        // don't process inf or NaN
        if target.is_infinite() || measurement.is_infinite() {
            return 0.;
        }

        // reset input filter to value received
        if self.flags.reset_filter {
            self.flags.reset_filter = false;
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
        self.update_i(limit);

        let mut P_out = (self.error * self.kp);
        let mut D_out = (self.derivative * self.kd);

        // calculate slew limit modifier for P+D
        self.info.Dmod = self.slew_limiter.modifier(
            (self.info.P + self.info.D) * self.slew_limit_scale,
            self.dt,
            now_ms,
        );
        self.info.slew_rate = self.slew_limiter.output_slew_rate;

        P_out *= self.info.Dmod;
        D_out *= self.info.Dmod;

        self.info.target = self.target;
        self.info.actual = measurement;
        self.info.error = self.error;
        self.info.P = P_out;
        self.info.D = D_out;

        P_out + self.integrator + D_out
    }
}
