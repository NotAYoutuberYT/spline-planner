use std::fmt;

#[derive(Clone, Copy, Debug)]
pub enum SplineErrorType {
    InvalidPreviousSegment,
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct SplineError {
    message: String,
    error_type: SplineErrorType,
}

impl SplineError {
    pub fn new(message: &str, error_type: &SplineErrorType) -> Self {
        Self {
            message: message.to_string(),
            error_type: *error_type,
        }
    }
}

impl fmt::Display for SplineError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}
