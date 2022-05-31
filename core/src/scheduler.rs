use embedded_time::{clock::Error, duration::Microseconds, Clock};
use crate::IntertialSensor;

pub struct Task {
    f: fn(),
    hz: f32,
    max_time_micros: u16,
    priority: u8,
    last_run: u16,
}

impl Task {
    pub fn new(f: fn(), hz: f32, max_time_micros: u16, priority: u8) -> Self {
        Self {
            f,
            hz,
            max_time_micros,
            priority,
            last_run: 0,
        }
    }
}

pub struct Scheduler<'a, C> {
    common_tasks: &'a [Task],
    vehicle_tasks: &'a [Task],
    clock: C,
    num_tasks: u8,
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

impl<'a, C> Scheduler<'a, C>
where
    C: Clock<T = u32>,
{
    pub fn new(
        common_tasks: &'a [Task],
        vehicle_tasks: &'a [Task],
        clock: C,
        loop_rate_hz: i16,
    ) -> Self {
        let loop_period_us = (1000000 / loop_rate_hz as i32) as _;
        Self {
            common_tasks,
            vehicle_tasks,
            clock,
            num_tasks: (common_tasks.len() + vehicle_tasks.len()) as _,
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

    pub fn run<I: IntertialSensor>(&mut self, inertial_sensor: &mut I) -> Result<(), Error> {
        inertial_sensor.sample();

        let sample_time_us = Microseconds::try_from(self.clock.try_now()?.duration_since_epoch())
            .unwrap()
            .0;

        if self.loop_timer_start_us == 0 {
            self.loop_timer_start_us = sample_time_us;
            self.last_loop_time_s = 1. / self.loop_rate_hz as f32;
        } else {
            self.last_loop_time_s = (sample_time_us - self.loop_timer_start_us) as f32 * 1.0e-6;
        }

        self.tick_counter += 1;

        // run all the tasks that are due to run. Note that we only
        // have to call this once per loop, as the tasks are scheduled
        // in multiples of the main loop tick. So if they don't run on
        // the first call to the scheduler they won't run on a later
        // call until scheduler.tick() is called again
        let loop_us = self.loop_period_us;
        let now: u32 = Microseconds::try_from(self.clock.try_now()?.duration_since_epoch())
            .unwrap()
            .0;

        let mut time_available = 0;
        let loop_tick_us = now - sample_time_us;
        if loop_tick_us < loop_us as _ {
            // get remaining time available for this loop
            time_available = loop_us as u32 - loop_tick_us;
        }

        // add in extra loop time determined by not achieving scheduler tasks
        time_available += self.extra_loop_us;

        self.run_with_time_available(time_available)?;

        if self.task_not_achieved > 0 {
            // add some extra time to the budget
            self.extra_loop_us = (self.extra_loop_us + 100).min(5000);
            self.task_not_achieved = 0;
            self.task_all_achieved = 0;
        } else if (self.extra_loop_us > 0) {
            self.task_all_achieved += 1;
            if (self.task_all_achieved > 50) {
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

    pub fn run_with_time_available(&mut self, time_available: u32) -> Result<(), Error> {
        let run_started_usec = self.clock.try_now()?;
        let now = run_started_usec;

        let mut vehicle_tasks_offset = 0;
        let mut common_tasks_offset = 0;

        for _ in 0..self.num_tasks {
            let mut run_vehicle_task = false;
            if vehicle_tasks_offset < self.vehicle_tasks.len()
                && common_tasks_offset < self.common_tasks.len()
            {
                // still have entries on both lists; compare the
                // priorities.  In case of a tie the vehicle-specific
                // entry wins.
                let vehicle_task = &self.vehicle_tasks[vehicle_tasks_offset as usize];
                let common_task = &self.common_tasks[common_tasks_offset as usize];
                if vehicle_task.priority <= common_task.priority {
                    run_vehicle_task = true;
                }
            } else if vehicle_tasks_offset < self.vehicle_tasks.len() {
                // out of common tasks to run
                run_vehicle_task = true;
            } else if common_tasks_offset < self.common_tasks.len() {
                // out of vehicle tasks to run
                run_vehicle_task = false;
            } else {
                // this is an error; the outside loop should have terminated
                //      INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                break;
            }

            let task = if run_vehicle_task {
                &self.vehicle_tasks[vehicle_tasks_offset as usize]
            } else {
                &self.common_tasks[common_tasks_offset as usize]
            };

            if (run_vehicle_task) {
                vehicle_tasks_offset += 1;
            } else {
                common_tasks_offset += 1;
            }

            if (task.priority > 3) {
                let dt = self.tick_counter - task.last_run;
                // we allow 0 to mean loop rate
                let mut interval_ticks = if task.hz == 0. {
                    1
                } else {
                    self.loop_rate_hz / task.hz as i16
                };
                if (interval_ticks < 1) {
                    interval_ticks = 1;
                }
                if ((dt as i16) < interval_ticks) {
                    // this task is not yet scheduled to run again
                    continue;
                }
                // this task is due to run. Do we have enough time to run it?
                self.task_time_allowed = task.max_time_micros as _;

                if (dt as i16 >= interval_ticks * 2) {
                    //perf_info.task_slipped(i);
                }

                if (dt as i16 >= interval_ticks * self.max_task_slowdown as i16) {
                    // we are going beyond the maximum slowdown factor for a
                    // task. This will trigger increasing the time budget
                    self.task_not_achieved += 1;
                }

                if (self.task_time_allowed > time_available) {
                    // not enough time to run this task.  Continue loop -
                    // maybe another task will fit into time remaining
                    continue;
                }
            } else {
                self.task_time_allowed = self.loop_period_us as _;
            }

            (task.f)();
        }

        Ok(())
    }
}