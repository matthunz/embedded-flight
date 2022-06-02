use embedded_flight_scheduler::{Error, Scheduler, Task};
use std_embedded_time::StandardClock;

fn main() {
    let clock = StandardClock::default();

    let a: Task<(), Error> = Task::new(|_| {
        dbg!("A");
        Ok(())
    });

    let b: Task<(), Error> = Task::new(|_| {
        dbg!("B");
        Ok(())
    });

    let mut tasks = [a.with_hz(2.), b.with_hz(1.)];

    let mut scheduler = Scheduler::new(&mut tasks, clock, 400);

    loop {
        scheduler.run(&mut ()).unwrap();
    }
}
