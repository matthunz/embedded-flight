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

    pub fn output(&self) -> f32 {
        self.filter.output
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

pub struct DerivativeFilter<const FILTER_SIZE: usize> {
    buffer: [f32; FILTER_SIZE],
    timestamps: [u32; FILTER_SIZE],
    idx: usize,
    new_data: bool,
    last_slope: f32,
}

impl<const FILTER_SIZE: usize> DerivativeFilter<FILTER_SIZE> {
    pub fn new() -> Self {
        Self {
            buffer: [0.0; FILTER_SIZE],
            timestamps: [0; FILTER_SIZE],
            idx: 0,
            new_data: false,
            last_slope: 0.0,
        }
    }

    pub fn update(&mut self, sample: f32, timestamp: u32) {
        self.buffer[self.idx] = sample;
        self.timestamps[self.idx] = timestamp;
        self.idx = (self.idx + 1) % FILTER_SIZE;
        self.new_data = true;
    }

    pub fn slope(&mut self) -> f32 {
        if !self.new_data {
            return self.last_slope;
        }

        let n = FILTER_SIZE;
        let dt = self.timestamps[self.idx] as i32 - self.timestamps[(self.idx + n - 1) as usize % n] as i32;
        let idx_prev = (self.idx + n - 2) as usize % n;
        let idx_next = (self.idx + 1) as usize % n;

        let y_prev = self.buffer[idx_prev];
        let y_next = self.buffer[idx_next];
        let dy = y_next - y_prev;

        if dt == 0 {
            self.last_slope = 0.0;
        } else {
            self.last_slope = dy / (dt as f32 * 1e-6);
        }

        self.new_data = false;
        self.last_slope
    }

    pub fn reset(&mut self) {
        self.buffer = [0.0; FILTER_SIZE];
        self.timestamps = [0; FILTER_SIZE];
        self.idx = 0;
        self.new_data = false;
        self.last_slope = 0.0;
    }
}


