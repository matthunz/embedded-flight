use nalgebra::Vector3;


#[derive(Clone, Debug)]
pub struct BodyRateController {
    k_p: Vector3<f32>,
    max_torque: f32,
}

impl Default for BodyRateController {
    fn default() -> Self {
        Self {
            k_p: Vector3::new(20., 20., 5.),
            max_torque: 1.,
        }
    }
}

impl BodyRateController {
    /// Generate the roll, pitch, yaw moment commands in the body frame in Newtons*meters
    pub fn body_rate_control(
        &self,
        body_rate_cmd: Vector3<f32>,
        body_rate: Vector3<f32>,
        moi: Vector3<f32>,
    ) -> Vector3<f32> {
        let taus = mul_array(moi, mul_array(self.k_p, body_rate_cmd - body_rate));
        let taus_mod = taus.norm();

        if taus_mod > self.max_torque {
            taus * self.max_torque / taus_mod
        } else {
            taus
        }
    }
}

fn mul_array(lhs: Vector3<f32>, rhs: Vector3<f32>) -> Vector3<f32> {
    lhs.zip_map(&rhs, |l, r| l * r)
}
