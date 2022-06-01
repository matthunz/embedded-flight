use embedded_flight_control::attitude::MultiCopterAttitudeController;
use embedded_flight_core::InertialSensor;
use embedded_flight_motors::{esc::ESC, MotorMatrix};
use embedded_flight_scheduler::{Scheduler, State, Task};
use embedded_time::Clock;
use nalgebra::{Quaternion, Vector3};

pub struct MultiCopterState<E, const N: usize> {
    pub attitude: Quaternion<f32>,
    pub gyro: Vector3<f32>,
    pub attitude_controller: MultiCopterAttitudeController,
    pub motor_matrix: MotorMatrix<E, f32, N>,
}

pub struct MultiCopter<'t, C, I, E, const N: usize> {
    pub scheduler: Scheduler<'t, C, MultiCopterState<E, N>>,
    pub inertial_sensor: I,
    pub state: MultiCopterState<E, N>,
}

impl<'t, C, I, E, const N: usize> MultiCopter<'t, C, I, E, N>
where
    C: Clock<T = u32>,
    I: InertialSensor,
    E: ESC<Output = f32>,
{
    pub fn new(
        motor_matrix: MotorMatrix<E, f32, N>,
        inertial_sensor: I,
        tasks: &'t mut [Task<MultiCopterState<E, N>>],
        clock: C,
        loop_rate_hz: i16,
    ) -> Self {
        Self {
            scheduler: Scheduler::new(tasks, clock, loop_rate_hz),
            inertial_sensor,
            state: MultiCopterState {
                attitude: Quaternion::default(),
                gyro: Vector3::default(),
                attitude_controller: MultiCopterAttitudeController::default(),
                motor_matrix,
            },
        }
    }

    pub fn run(&mut self) {
        loop {
            self.state.attitude = self.inertial_sensor.attitude();
            self.state.gyro = self.inertial_sensor.gyro();

            self.scheduler.run(&mut self.state).unwrap();
        }
    }
}

/// Create an array of multi copter tasks to control attitude and output to the motors.
pub fn multi_copter_tasks<E, const N: usize>() -> [Task<MultiCopterState<E, N>>; 1]
where
    E: ESC<Output = f32>,
{
    [motor_output_task()]
}

/// Create the high priority task to run body rate control and output to the motors.
pub fn motor_output_task<E, const N: usize>() -> Task<MultiCopterState<E, N>>
where
    E: ESC<Output = f32>,
{
    Task::high_priority(|state: State<'_, MultiCopterState<E, N>>| {
        let motor_output = state
            .system
            .attitude_controller
            .motor_output(state.system.gyro, state.now.0);

        state.system.motor_matrix.output(motor_output);
    })
}
