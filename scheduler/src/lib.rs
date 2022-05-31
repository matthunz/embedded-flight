use embedded_time::{clock::Error, duration::Microseconds, Clock};

mod task;
pub use task::{State, Task};

pub struct Scheduler<'a, C, T> {
    tasks: &'a mut [Task<T>],
    clock: C,
    tick_counter: u16,
    loop_rate_hz: i16,
    loop_period_us: u16,
    task_time_allowed: u32,
    max_task_slowdown: u8,
    // counters to handle dynamically adjusting extra loop time to
    // cope with low CPU conditions
    task_not_achieved: u32,
    task_all_achieved: u32,
    loop_timer_start_us: u32,
    last_loop_time_s: f32,
    extra_loop_us: u32,
}

impl<'a, C, T> Scheduler<'a, C, T>
where
    C: Clock<T = u32>,
{
    pub fn new(tasks: &'a mut [Task<T>], clock: C, loop_rate_hz: i16) -> Self {
        let loop_period_us = (1000000 / loop_rate_hz as i32) as _;
        Self {
            tasks,
            clock,
            tick_counter: 0,
            loop_rate_hz,
            loop_period_us,
            task_time_allowed: loop_period_us as _,
            max_task_slowdown: 4,
            task_not_achieved: 0,
            task_all_achieved: 0,
            loop_timer_start_us: 0,
            last_loop_time_s: 0.,
            extra_loop_us: 0,
        }
    }

    pub fn run(&mut self, controller: &mut T) -> Result<(), Error> {
        let sample_time_us = self.micros_since_epoch()?.0;

        if self.loop_timer_start_us == 0 {
            self.loop_timer_start_us = sample_time_us;
            self.last_loop_time_s = 1. / self.loop_rate_hz as f32;
        } else {
            self.last_loop_time_s = (sample_time_us - self.loop_timer_start_us) as f32 * 1.0e-6;
        }

        if self.tick_counter == u16::MAX {
            self.tick_counter = 0;
        } else {
            self.tick_counter += 1;
        }

        // run all the tasks that are due to run. Note that we only
        // have to call this once per loop, as the tasks are scheduled
        // in multiples of the main loop tick. So if they don't run on
        // the first call to the scheduler they won't run on a later
        // call until scheduler.tick() is called again
        let loop_us = self.loop_period_us;
        let now = Microseconds::try_from(self.clock.try_now()?.duration_since_epoch()).unwrap();

        let mut time_available = 0;
        let loop_tick_us = now.0 - sample_time_us;
        if loop_tick_us < loop_us as _ {
            // get remaining time available for this loop
            time_available = loop_us as u32 - loop_tick_us;
        }

        // add in extra loop time determined by not achieving scheduler tasks
        time_available += self.extra_loop_us;

        self.run_with_time_available_inner(controller, now, time_available)?;

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

    pub fn run_with_time_available(
        &mut self,
        controller: &mut T,
        time_available: u32,
    ) -> Result<(), Error> {
        let now = self.micros_since_epoch()?;
        self.run_with_time_available_inner(controller, now, time_available)
    }

    fn run_with_time_available_inner(
        &mut self,
        controller: &mut T,
        now: Microseconds<u32>,
        time_available: u32,
    ) -> Result<(), Error> {
        for task in self.tasks.iter_mut() {
            if task.priority > 3 {
                let dt = self.tick_counter - task.last_run;
                // A 0hz task should be ran at the rate of the scheduler loop
                let interval_ticks = if task.hz == 0. {
                    1
                } else {
                    (self.loop_rate_hz / task.hz as i16).max(1)
                };

                if (dt as i16) < interval_ticks {
                    // this task is not yet scheduled to run again
                    continue;
                }

                // this task is due to run. Do we have enough time to run it?
                self.task_time_allowed = task.max_time_micros as _;

                if dt as i16 >= interval_ticks * self.max_task_slowdown as i16 {
                    // we are going beyond the maximum slowdown factor for a
                    // task. This will trigger increasing the time budget
                    self.task_not_achieved += 1;
                }

                if self.task_time_allowed > time_available {
                    // not enough time to run this task.  Continue loop -
                    // maybe another task will fit into time remaining
                    continue;
                }
            } else {
                self.task_time_allowed = self.loop_period_us as _;
            }

            (task.f)(State { controller, now });
        }

        Ok(())
    }

    fn micros_since_epoch(&mut self) -> Result<Microseconds<u32>, Error> {
        self.clock
            .try_now()
            .map(|instant| Microseconds::try_from(instant.duration_since_epoch()).unwrap())
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
