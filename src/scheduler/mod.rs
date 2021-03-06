//! Real time task scheduler
//!
//! For more check out the [scheduler](https://github.com/matthunz/embedded-flight/tree/main/examples/scheduler.rs) example on GitHub.
//! ```ignore
//! let clock = StandardClock::default();
//!
//! let a: Task<(), Error> = Task::new(|_| {
//!    dbg!("A");
//!    Ok(())
//! });
//!
//! let b: Task<(), Error> = Task::new(|_| {
//!    dbg!("B");
//!    Ok(())
//! });
//!
//! let mut tasks = [a.with_hz(2.), b.with_hz(1.)];
//!
//! let mut scheduler = Scheduler::new(&mut tasks, clock, 400);
//!
//! loop {
//!    scheduler.run(&mut ())?;
//! }
//! ```

use embedded_time::{duration::Microseconds, Clock};
use num_traits::ToPrimitive;

mod error;
pub use error::Error;

mod task;
pub use task::{Event, Task};

/// Task scheduler for flight controllers
pub struct Scheduler<'a, C, T, E = Error> {
    /// The tasks to run in order of highest to lowest priority.
    pub tasks: &'a mut [Task<T, E>],

    /// The clock used for timing.
    pub clock: C,

    /// The current tick, incremented each run.
    pub tick: u16,

    /// The maximum amount of ticks a task can miss before slowing down the scheduler.
    pub max_task_slowdown: u8,

    /// The desired loop rate to run (in hz).
    pub loop_rate_hz: i16,

    // The period of the loop rate (in microseconds).
    loop_period_us: u16,

    /// The amount of ticks with tasks that exceed the `max_task_slowdown`.
    pub task_not_achieved: u32,

    /// The amount of ticks with tasks that remain in the `max_task_slowdown`.
    pub task_all_achieved: u32,

    /// The start time of the run loop (in microseconds).
    pub loop_timer_start_us: u32,

    /// The start time of the run loop (in seconds).
    pub last_loop_time_s: f32,

    /// Extra time to spend in the loop to catch up on tasks not achieved (in microseconds).
    pub extra_loop_us: u32,
}

impl<'a, C, T, E> Scheduler<'a, C, T, E>
where
    C: Clock,
    C::T: ToPrimitive,
    E: From<Error>,
{
    /// Create a new scheduler from a slice of tasks, a clock, and the loop rate (in hz)
    pub fn new(tasks: &'a mut [Task<T, E>], clock: C, loop_rate_hz: i16) -> Self {
        let loop_period_us = (1000000 / loop_rate_hz as i32) as _;
        Self {
            tasks,
            clock,
            tick: 0,
            loop_rate_hz,
            loop_period_us,
            max_task_slowdown: 4,
            task_not_achieved: 0,
            task_all_achieved: 0,
            loop_timer_start_us: 0,
            last_loop_time_s: 0.,
            extra_loop_us: 0,
        }
    }

    /// Calculate the time available and run as many tasks as possible.
    pub fn run(&mut self, state: &mut T) -> Result<(), E> {
        let sample_time_us = self.micros_since_epoch()?.0;

        // Set initial loop_timer_start if not set
        if self.loop_timer_start_us == 0 {
            self.loop_timer_start_us = sample_time_us;
            self.last_loop_time_s = 1. / self.loop_rate_hz as f32;
        } else {
            self.last_loop_time_s = (sample_time_us - self.loop_timer_start_us) as f32 * 1.0e-6;
        }

        // Reset the tick counter if we reach the limit
        if self.tick == u16::MAX {
            self.tick = 0;

            // Todo maybe don't?
            for task in self.tasks.iter_mut() {
                task.last_run = 0;
            }
        } else {
            self.tick += 1;
        }

        // run all the tasks that are due to run. Note that we only
        // have to call this once per loop, as the tasks are scheduled
        // in multiples of the main loop tick. So if they don't run on
        // the first call to the scheduler they won't run on a later
        // call until scheduler.tick() is called again
        let loop_us = self.loop_period_us;
        let now = self.micros_since_epoch()?;

        let mut time_available = 0;
        let loop_tick_us = now.0 - sample_time_us;
        if loop_tick_us < loop_us as _ {
            // get remaining time available for this loop
            time_available = loop_us as u32 - loop_tick_us;
        }

        // add in extra loop time determined by not achieving scheduler tasks
        time_available += self.extra_loop_us;

        self.run_with_time_available_inner(state, now, time_available)?;

        if self.task_not_achieved > 0 {
            // add some extra time to the budget
            self.extra_loop_us = (self.extra_loop_us + 100).min(5000);
            self.task_not_achieved = 0;
            self.task_all_achieved = 0;
        } else if self.extra_loop_us > 0 {
            self.task_all_achieved += 1;
            if self.task_all_achieved > 50 {
                // we have gone through 50 loops without a task taking too
                // long. CPU pressure has eased, so drop the extra time we're
                // giving each loop
                self.task_all_achieved = 0;
                // we are achieving all tasks, slowly lower the extra loop time
                self.extra_loop_us = 0.max(self.extra_loop_us - 50);
            }
        }

        self.loop_timer_start_us = sample_time_us;

        Ok(())
    }

    /// Run as many tasks as possible in the given `time_available`.
    pub fn run_with_time_available(
        &mut self,
        system: &mut T,
        time_available: u32,
    ) -> Result<(), E> {
        let now = self.micros_since_epoch()?;
        self.run_with_time_available_inner(system, now, time_available)
    }

    fn run_with_time_available_inner(
        &mut self,
        system: &mut T,
        now: Microseconds<u32>,
        time_available: u32,
    ) -> Result<(), E> {
        for task in self.tasks.iter_mut() {
            if !task.is_high_priority {
                let ticks = task.ticks(self.loop_rate_hz);

                if let Some(dt) = task.ready(self.tick, ticks) {
                    // Check if the scheduler is going beyond the maximum slowdown factor
                    if dt as i16 >= ticks * self.max_task_slowdown as i16 {
                        // This will trigger increasing the time budget
                        self.task_not_achieved += 1;
                    }

                    if task.max_time_micros as u32 > time_available {
                        // Not enough time to run this task
                        // Try to fit another task into the time remaining
                        continue;
                    }
                } else {
                    // This task is not ready
                    continue;
                }
            }

            let state = Event {
                state: system,
                now,
                available: Microseconds::new(time_available),
            };
            task.run(state, self.tick)?;
        }

        Ok(())
    }

    fn micros_since_epoch(&mut self) -> Result<Microseconds<u32>, Error> {
        let instant = self.clock.try_now()?;
        let duration = instant.duration_since_epoch();
        let ms: Microseconds<C::T> = Microseconds::try_from(duration)?;
        Ok(Microseconds::new(ms.0.to_u32().unwrap()))
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
