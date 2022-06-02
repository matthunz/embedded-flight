use embedded_time::{clock, ConversionError};

/// A scheduler error caused by clock timing
#[derive(Debug)]
pub enum Error {
    Clock(clock::Error),
    Time(ConversionError),
}

impl From<clock::Error> for Error {
    fn from(clock_error: clock::Error) -> Self {
        Error::Clock(clock_error)
    }
}

impl From<ConversionError> for Error {
    fn from(time_error: ConversionError) -> Self {
        Error::Time(time_error)
    }
}
