#[derive(Clone, Copy, Debug, Default)]
pub struct P {
    kp: f32,
}

impl P {
    pub fn control(self, target: f32, actual: f32) -> f32 {
        let error = target - actual;
        error * self.kp
    }
}

pub fn thrust_control(p: P, mass: f32, gravity: f32, z_target: f32, z_actual: f32) -> f32 {
    let acceleration = p.control(z_target, z_actual);
    mass * (gravity - acceleration)
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
