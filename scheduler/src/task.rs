use embedded_time::duration::Microseconds;

pub struct State<'a, T> {
    pub controller: &'a mut T,
    pub now: Microseconds<u32>,
}

pub struct Task<T> {
    pub f: fn(State<'_, T>),
    pub hz: f32,
    pub max_time_micros: u16,
    pub priority: u8,
    pub last_run: u16,
}

impl<T> Task<T> {
    pub fn new(f: fn(State<'_, T>), hz: f32, max_time_micros: u16, priority: u8) -> Self {
        Self {
            f,
            hz,
            max_time_micros,
            priority,
            last_run: 0,
        }
    }

    pub fn fast(f: fn(State<'_, T>)) -> Self {
        Self::new(f, 0., 0, 0)
    }
}