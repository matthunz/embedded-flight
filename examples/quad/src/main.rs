//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use embedded_flight::{
    motors::{esc::Builder, MotorMatrix, ESC, RCESC},
    MultiCopter,
};
use embedded_hal::PwmPin;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    time::ms,
    timer::{Channel, Tim2NoRemap},
};

#[entry]
fn main() -> ! {
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

    let a = &mut Builder::default().build(p1) as &mut dyn ESC<Output = u16>;
    let b = &mut Builder::default().build(p2) as &mut dyn ESC<Output = u16>;
    let c = &mut Builder::default().build(p3) as &mut dyn ESC<Output = u16>;
    let d = &mut Builder::default().build(p4) as &mut dyn ESC<Output = u16>;

    let mut motors = MotorMatrix::quad(a, b, c, d);
    motors.thrust_boost_ratio = 0f32;

    loop {}
}
