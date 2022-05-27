use num_traits::{Float, FloatConst, Zero};

pub struct LowPassFilter<T> {
    output: T,
    alpha: T,
}

impl<T: Zero> Default for LowPassFilter<T> {
    fn default() -> Self {
        Self {
            output: T::zero(),
            alpha: T::zero(),
        }
    }
}

impl<T> LowPassFilter<T>
where
    T: Float + FloatConst,
{
    pub fn filter(&mut self, sample: T, cutoff_freq: T, dt: T) -> T {
        if cutoff_freq <= T::zero() || dt <= T::zero() {
            self.output = sample;
            return self.output;
        }

        let rc = T::one() / (T::FRAC_2_PI() * cutoff_freq);
        let alpha = dt / (dt + rc).min(T::one()).max(T::zero());
        self.output = self.output + (sample - self.output) * alpha;
        self.output
    }

    pub fn set_freq(&mut self, sample_freq: T, cutoff_freq: T) {
        self.alpha = if sample_freq <= T::zero() {
            T::one()
        } else {
            alpha(T::one() / sample_freq, cutoff_freq)
        };
    }
}

pub fn alpha<T: Float + FloatConst>(dt: T, cutoff_freq: T) -> T {
    if cutoff_freq <= T::zero() || dt <= T::zero() {
        return T::one();
    }

    let rc = T::one() / (T::FRAC_2_PI() * cutoff_freq);
    dt / (dt + rc).min(T::one()).max(T::zero())
}

pub fn alpha_unchecked<T: Float + FloatConst>(dt: T, cutoff_freq: T) -> T {
    let rc = T::one() / (T::FRAC_2_PI() * cutoff_freq);
    dt / (dt + rc).min(T::one()).max(T::zero())
}
