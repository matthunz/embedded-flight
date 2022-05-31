use embedded_time::duration::Microseconds;

pub struct State<'a, T> {
    pub controller: &'a mut T,
    pub now: Microseconds<u32>,
}

pub struct Task<T> {
    pub f: fn(State<'_, T>),
    pub hz: f32,
    pub max_time_micros: u16,
    pub is_high_priority: bool,
    pub last_run: u16,
}

impl<T> Task<T> {
    pub fn new(f: fn(State<'_, T>)) -> Self {
        Self {
            f,
            hz: 0.,
            max_time_micros: 0,
            is_high_priority: false,
            last_run: 0,
        }
    }

    pub fn with_hz(mut self, hz: f32) -> Self {
        self.hz = hz;
        self
    }

    pub fn with_max_time(mut self, micros: u16) -> Self {
        self.max_time_micros = micros;
        self
    }

    pub fn with_high_priority(mut self, is_high_priority: bool) -> Self {
        self.is_high_priority = is_high_priority;
        self
    }
}
