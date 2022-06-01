use nalgebra::Vector3;
use num_traits::{Float, FromPrimitive};

pub struct Motor<E, T> {
    pub esc: E,
    pub factor: Vector3<T>,
    pub throttle_factor: T,
    pub thrust_rpyt_out: T,
}

impl<E, T> Motor<E, T>
where
    T: Float,
{
    pub fn new(esc: E, factor: Vector3<T>, throttle_factor: T) -> Self {
        Self {
            esc,
            factor,
            throttle_factor,
            thrust_rpyt_out: T::zero(),
        }
    }

    pub fn from_degrees(
        esc: E,
        roll_factor_degrees: T,
        pitch_factor_degrees: T,
        yaw_factor: T,
    ) -> Self
    where
        T: FromPrimitive,
    {
        Self::new(
            esc,
            Vector3::new(
                (roll_factor_degrees + T::from_u8(90).unwrap())
                    .to_radians()
                    .cos(),
                pitch_factor_degrees.to_radians().cos(),
                yaw_factor,
            ),
            T::one(),
        )
    }

    pub fn from_angle(esc: E, angle: T, yaw_factor: T) -> Self
    where
        T: FromPrimitive,
    {
        Self::from_degrees(esc, angle, angle, yaw_factor)
    }
}
