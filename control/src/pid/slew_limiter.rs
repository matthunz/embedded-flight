use embedded_flight_core::filter::LowPassFilter;

/// Slew rate limiting filter.
/// Used to prevent oscillation of a controller
/// by modifying the controllers output based on a maximum slew rate

pub struct SlewLimiter<const N: usize> {
    slew_rate_max: f32,
    slew_rate_tau: f32,
    slew_filter: LowPassFilter<f32>,
    pub output_slew_rate: f32,
    _modifier_slew_rate: f32,
    last_sample: f32,
    _max_pos_slew_rate: f32,
    _max_neg_slew_rate: f32,
    _max_pos_slew_event_ms: u32,
    _max_neg_slew_event_ms: u32,
    _pos_event_index: u8,
    _neg_event_index: u8,
    _pos_event_ms: [u32; N],
    _neg_event_ms: [u32; N],
    _pos_event_stored: bool,
    _neg_event_stored: bool,
    window_ms: u32,
    modifier_gain: f32,
    cutoff_freq: f32,
}

impl<const N: usize> SlewLimiter<N> {
    pub fn new(slew_rate_max: f32, slew_rate_tau: f32) -> Self {
        Self {
            slew_rate_max,
            slew_rate_tau,
            slew_filter: LowPassFilter::default(),
            output_slew_rate: 0.,
            _modifier_slew_rate: 0.,
            last_sample: 0.,
            _max_pos_slew_rate: 0.,
            _max_neg_slew_rate: 0.,
            _max_pos_slew_event_ms: 0,
            _max_neg_slew_event_ms: 0,
            _pos_event_index: 0,
            _neg_event_index: 0,
            _pos_event_ms: [0; N],
            _neg_event_ms: [0; N],
            _pos_event_stored: false,
            _neg_event_stored: false,
            window_ms: 300,
            modifier_gain: 1.5,
            cutoff_freq: 25.,
        }
    }
    /// Apply the filter to a sample, returning multiplier between 0 and 1 to keep output within slew rate
    pub fn modifier(&mut self, sample: f32, dt: f32, now_ms: u32) -> f32 {
        if self.slew_rate_max <= 0. {
            self.output_slew_rate = 0.;
            return 1.;
        }

        // Calculate a low pass filtered slew rate
        let slew_rate =
            self.slew_filter
                .filter((sample - self.last_sample) / dt, self.cutoff_freq, dt);
        self.last_sample = sample;

        let decay_alpha = dt.min(self.slew_rate_tau) / self.slew_rate_tau;

        // Store a series of positive slew rate exceedance events
        if !self._pos_event_stored && slew_rate > self.slew_rate_max {
            if self._pos_event_index as usize >= N {
                self._pos_event_index = 0;
            }
            self._pos_event_ms[self._pos_event_index as usize] = now_ms;
            self._pos_event_index += 1;
            self._pos_event_stored = true;
            self._neg_event_stored = false;
        }

        // Store a series of negative slew rate exceedance events
        if !self._neg_event_stored && slew_rate < -self.slew_rate_max {
            if self._neg_event_index as usize >= N {
                self._neg_event_index = 0;
            }
            self._neg_event_ms[self._neg_event_index as usize] = now_ms;
            self._neg_event_index += 1;
            self._neg_event_stored = true;
            self._pos_event_stored = false;
        }

        // Find the oldest event time
        let mut oldest_ms = now_ms;
        for index in 0..N {
            if self._pos_event_ms[index] < oldest_ms {
                oldest_ms = self._pos_event_ms[index];
            }
            if self._neg_event_ms[index] < oldest_ms {
                oldest_ms = self._neg_event_ms[index];
            }
        }

        // Decay the peak positive and negative slew rate if they are outside the window
        // Never drop PID gains below 10% of configured value
        if slew_rate > self._max_pos_slew_rate {
            self._max_pos_slew_rate = slew_rate.min(10. * self.slew_rate_max);
            self._max_pos_slew_event_ms = now_ms;
        } else if now_ms - self._max_pos_slew_event_ms > self.window_ms {
            self._max_pos_slew_rate *= 1. - decay_alpha;
        }

        if slew_rate < -self._max_neg_slew_rate {
            self._max_neg_slew_rate = -slew_rate.min(10. * self.slew_rate_max);
            self._max_neg_slew_event_ms = now_ms;
        } else if now_ms - self._max_neg_slew_event_ms > self.window_ms {
            self._max_neg_slew_rate *= 1. - decay_alpha;
        }

        let raw_slew_rate = 0.5 * (self._max_pos_slew_rate + self._max_neg_slew_rate);

        // Apply a further reduction when the oldest exceedance event falls outside the window rewuired for the
        // specified number of exceedance events. This prevents spikes due to control mode changed, etc causing
        // unwanted gain reduction and is only applied to the slew rate used for gain reduction
        let mut modifier_input = raw_slew_rate;
        if (now_ms - oldest_ms) as usize > (N + 1) * self.window_ms as usize {
            let oldest_time_from_window = 0.001
                * ((now_ms - oldest_ms) as f32 - ((N + 1) * self.window_ms as usize) as f32) as f32;
            modifier_input *= (-oldest_time_from_window / self.slew_rate_tau).exp();
        }

        // Apply a filter to increases in slew rate only to reduce the effect of gusts and large controller
        // setpoint changeschanges
        let attack_alpha = (2. * decay_alpha).min(1.);

        self._modifier_slew_rate =
            (1. - attack_alpha) * self._modifier_slew_rate + attack_alpha * modifier_input;
        self._modifier_slew_rate = self._modifier_slew_rate.min(modifier_input);

        self.output_slew_rate =
            (1. - attack_alpha) * self.output_slew_rate + attack_alpha * raw_slew_rate;
        self.output_slew_rate = self.output_slew_rate.min(raw_slew_rate);

        // Calculate the gain adjustment
        if self._modifier_slew_rate > self.slew_rate_max {
            self.slew_rate_max
                / (self.slew_rate_max
                    + self.modifier_gain * (self._modifier_slew_rate - self.slew_rate_max))
        } else {
            1.
        }
    }
}
