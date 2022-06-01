#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use embedded_flight::{
    copter::multi_copter_tasks,
    core::InertialSensor,
    motors::{esc::Builder, MotorMatrix, ESC},
    MultiCopter,
};
use embedded_time::{rate::Fraction, Clock, Instant};
use nalgebra::{Quaternion, Vector3};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{SysCounterUs, Tim2NoRemap, Timer},
};

struct SysClock {
    counter: SysCounterUs,
}

impl Clock for SysClock {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1);

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        let instant = self.counter.now();
        Ok(Instant::new(instant.ticks()))
    }
}

struct IMU {}

impl InertialSensor for IMU {
    fn attitude(&mut self) -> Quaternion<f32> {
        todo!()
    }

    fn gyro(&mut self) -> Vector3<f32> {
        todo!()
    }
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain();

    let mut gpioa = p.GPIOA.split();

    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let pins = (c1, c2, c3, c4);

    let (mut p1, mut p2, mut p3, mut p4) = p
        .TIM2
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz(), &clocks)
        .split();

    // Enable clock on each of the channels
    p1.enable();
    p2.enable();
    p3.enable();
    p4.enable();

    // Configure the syst timer to trigger an update every second
    let counter = Timer::syst(cp.SYST, &clocks).counter_us();
    let clock = SysClock { counter };

    let a = &mut Builder::default().build(p1) as &mut dyn ESC<Output = u16>;
    let b = &mut Builder::default().build(p2) as &mut dyn ESC<Output = u16>;
    let c = &mut Builder::default().build(p3) as &mut dyn ESC<Output = u16>;
    let d = &mut Builder::default().build(p4) as &mut dyn ESC<Output = u16>;

    let motors = MotorMatrix::quad(a, b, c, d);
    let imu = IMU {};
    let mut tasks = multi_copter_tasks();

    let mut drone = MultiCopter::new(motors, imu, &mut tasks, clock, 400);
    drone.run().unwrap();

    loop {}
}
