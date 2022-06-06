mod rcesc;
pub use rcesc::{Builder, RCESC};

/// Electronic speed controller
pub trait ESC<T> {
    /// Arm this ESC.
    fn arm(&mut self);

    /// Output linear actuated motion between -1 and 1
    fn output(&mut self, output: T);
}

impl<T, U> ESC<U> for &mut T
where
    T: ESC<U> + ?Sized,
{
    fn arm(&mut self) {
        (&mut **self).arm()
    }

    fn output(&mut self, output: U) {
        (&mut **self).output(output);
    }
}
