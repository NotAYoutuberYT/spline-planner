use nalgebra::Vector2;

use crate::splines::{
    cubic_segments::bezier_segment::BezierSegment, spline_error::SplineError,
    spline_segment::SplineSegment, spline_segment_factory::SplineSegmentFactory,
};

/// A factory that creates C0 continuous spline segments.
pub struct C0CubicBezierSegmentFactory {
    p0: Vector2<f64>,
    p1: Vector2<f64>,
    p2: Vector2<f64>,
    p3: Vector2<f64>,
}

impl C0CubicBezierSegmentFactory {
    /// Creates a bezier segment from four control points.
    pub fn new(p0: Vector2<f64>, p1: Vector2<f64>, p2: Vector2<f64>, p3: Vector2<f64>) -> Self {
        Self { p0, p1, p2, p3 }
    }
}

impl SplineSegmentFactory for C0CubicBezierSegmentFactory {
    fn build(
        &self,
        _previous_segment: Option<&Box<dyn SplineSegment>>,
    ) -> Result<Box<dyn SplineSegment>, SplineError> {
        Ok(Box::new(BezierSegment::new(
            self.p0, self.p1, self.p2, self.p3,
        )))
    }
}
