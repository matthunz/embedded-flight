use embedded_flight_control::attitude::MultiCopterAttitudeController;
use embedded_flight_core::{
    scheduler::{Scheduler, State, Task},
    IntertialSensor,
};
use embedded_flight_motors::{esc::ESC, MotorMatrix};
use embedded_time::Clock;
use nalgebra::{Quaternion, Vector3};

pub struct MultiCopterState<E, const N: usize> {
    pub attitude: Quaternion<f32>,
    pub gyro: Vector3<f32>,
    pub attitude_controller: MultiCopterAttitudeController,
    pub motor_matrix: MotorMatrix<E, f32, N>,
}

pub struct MultiCopter<C, I, E, const N: usize> {
    pub scheduler: Scheduler<C, MultiCopterState<E, N>, 0, 1>,
    pub inertial_sensor: I,
    pub state: MultiCopterState<E, N>,
}

impl<C, I, E, const N: usize> MultiCopter<C, I, E, N>
where
    C: Clock<T = u32>,
    I: IntertialSensor,
    E: ESC<Output = f32>,
{
    pub fn new(
        motor_matrix: MotorMatrix<E, f32, N>,
        inertial_sensor: I,
        clock: C,
        loop_rate_hz: i16,
    ) -> Self {
        let motor_output_task = Task::fast(|state: State<'_, MultiCopterState<E, N>>| {
            let motor_output = state
                .controller
                .attitude_controller
                .motor_output(state.controller.gyro, state.now);

            state.controller.motor_matrix.output(motor_output);
        });

        let vehicle_tasks = [motor_output_task];
        let scheduler = Scheduler::new([], vehicle_tasks, clock, loop_rate_hz);

        Self {
            scheduler,
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
