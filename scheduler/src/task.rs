use embedded_time::duration::Microseconds;

use crate::Error;

pub struct State<'a, T> {
    pub system: &'a mut T,
    pub now: Microseconds<u32>,
    pub available: Microseconds<u32>,
}

type TaskFn<T, E> = fn(State<'_, T>) -> Result<(), E>;

pub struct Task<T, E = Error> {
    pub f: TaskFn<T, E>,
    pub hz: f32,
    pub max_time_micros: u16,
    pub is_high_priority: bool,
    pub last_run: u16,
}

impl<T, E> Task<T, E> {
    pub fn new(f: TaskFn<T, E>) -> Self {
        Self {
            f,
            hz: 0.,
            max_time_micros: 0,
            is_high_priority: false,
            last_run: 0,
        }
    }

    pub fn high_priority(f: TaskFn<T, E>) -> Self {
        Self::new(f).with_high_priority(true)
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

    pub fn ticks(&self, loop_rate_hz: i16) -> i16 {
        // A 0hz task should be ran at the rate of the scheduler loop
        if self.hz == 0. {
            1
        } else {
            (loop_rate_hz / self.hz as i16).max(1)
        }
    }
    
    pub fn ready(&self, tick: u16, ticks: i16) -> Option<u16> {
        let dt = tick - self.last_run;

        if (dt as i16) >= ticks {
            Some(dt)
        } else {
            None
        }
    }
}
