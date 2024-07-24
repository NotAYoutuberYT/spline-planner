use nalgebra::Vector2;

use crate::splines::{
    spline_error::SplineError, spline_segment::SplineSegment,
    spline_segment_factory::SplineSegmentFactory,
};

use super::linear_segment::LinearSegment;

/// A factory that creates LinearSegments.
pub struct LinearSegmentFactory {
    p0: Vector2<f64>,
    p1: Vector2<f64>,
}

impl LinearSegmentFactory {
    /// Directly creates a linear segment from the given points.
    pub fn new(p0: Vector2<f64>, p1: Vector2<f64>) -> Self {
        Self { p0, p1 }
    }
}

impl SplineSegmentFactory for LinearSegmentFactory {
    fn build(
        &self,
        _previous_segment: Option<&Box<dyn SplineSegment>>,
    ) -> Result<Box<dyn SplineSegment>, SplineError> {
        Ok(Box::new(LinearSegment::new(self.p0, self.p1)))
    }
}
