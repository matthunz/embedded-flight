pub struct DigitalLowPassFilter {
    output: f32,
    alpha: f32,
    is_initialised: bool,
}

impl Default for DigitalLowPassFilter {
    fn default() -> Self {
        DigitalLowPassFilter {
            output: 0.,
            alpha: 1.,
            is_initialised: false,
        }
    }
}

impl DigitalLowPassFilter {
    pub fn apply(&mut self, sample: f32, cutoff_freq: f32, dt: f32) -> f32 {
        self.compute_alpha(dt, cutoff_freq);
        self.output = if self.is_initialised {
            sample * self.alpha + self.output * (1. - self.alpha)
        } else {
            self.is_initialised = true;
            sample
        };
        self.output
    }

    pub fn apply_without_cutoff(&mut self, sample: f32) -> f32 {
        if !self.is_initialised {
            self.is_initialised = true;
            self.output = sample;
        }
        self.output
    }

    fn compute_alpha(&mut self, sample_freq: f32, cutoff_freq: f32) {
        let dt = 1.0 / sample_freq;
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_freq);
        self.alpha = dt / (dt + rc);
    }

    pub fn reset(&mut self, value: f32) {
        self.is_initialised = true;
        self.output = value;
    }
}

#[derive(Default)]
pub struct LowPassFilter {
    cutoff_freq: f32,
    filter: DigitalLowPassFilter,
}

impl LowPassFilter {
    pub fn with_cutoff(cutoff_freq: f32) -> Self {
        LowPassFilter {
            cutoff_freq,
            filter: DigitalLowPassFilter::default(),
        }
    }

    pub fn with_sample_rate_and_cutoff(sample_freq: f32, cutoff_freq: f32) -> Self {
        let mut filter = DigitalLowPassFilter::default();
        filter.compute_alpha(sample_freq, cutoff_freq);
        LowPassFilter {
            cutoff_freq,
            filter,
        }
    }

    pub fn set_cutoff_freq(&mut self, cutoff_freq: f32) {
        self.cutoff_freq = cutoff_freq;
        self.filter
            .compute_alpha(self.cutoff_freq, self.cutoff_freq);
    }

    pub fn get_cutoff_freq(&self) -> f32 {
        self.cutoff_freq
    }

    pub fn apply(&mut self, sample: f32, dt: f32) -> f32 {
        self.filter.apply(sample, self.cutoff_freq, dt)
    }

    pub fn apply_without_cutoff(&mut self, sample: f32) -> f32 {
        self.filter.apply_without_cutoff(sample)
    }

    pub fn reset(&mut self, value: f32) {
        self.filter.reset(value)
    }
}
