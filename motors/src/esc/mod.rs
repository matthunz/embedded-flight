mod rcesc;
pub use rcesc::{Builder, RCESC};

pub trait ESC {
    type Output;

    fn arm(&mut self);

    /// Output linear actuated motion between -1 and 1
    fn output(&mut self, output: Self::Output);
}

impl<T: ESC> ESC for &mut T {
    type Output = T::Output;

    fn arm(&mut self) {
        (&mut **self).arm()
    }

    fn output(&mut self, output: Self::Output) {
        (&mut **self).output(output);
    }
}
