mod rcesc;
pub use rcesc::{Builder, RCESC};

use super::Actuator;

/// Electronic speed controller
pub trait ESC<T>: Actuator<T> {
    /// Arm this ESC.
    fn arm(&mut self);
}

impl<T, U> ESC<U> for &mut T
where
    T: ESC<U> + Actuator<U> + ?Sized,
{
    fn arm(&mut self) {
        (&mut **self).arm()
    }
}
