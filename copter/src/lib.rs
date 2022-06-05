use nalgebra::Vector3;

// TODO thrust factor
pub struct Motor {
    pub factor: Vector3<f32>,
}

impl Motor {
    pub fn new(factor: Vector3<f32>) -> Self {
        Self { factor }
    }

    pub fn from_angle(angle: f32, yaw_factor: f32) -> Self {
        Self::from_degrees(angle, angle, yaw_factor)
    }

    pub fn from_degrees(
        roll_factor_degrees: f32,
        pitch_factor_degrees: f32,
        yaw_factor: f32,
    ) -> Self {
        Self::new(Vector3::new(
            (roll_factor_degrees + 90.).to_radians().cos(),
            pitch_factor_degrees.to_radians().cos(),
            yaw_factor,
        ))
    }

    pub fn output(&self, moment: Vector3<f32>, thrust: f32) -> f32 {
        self.factor
            .zip_fold(&moment, 0., |acc, factor, moment| factor * moment + acc)
            + thrust
    }
}

pub fn quad_motors() -> [Motor; 4] {
    [
        Motor::from_angle(90., 1.),
        Motor::from_angle(-90., 1.),
        Motor::from_angle(0., -1.),
        Motor::from_angle(180., -1.),
    ]
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
