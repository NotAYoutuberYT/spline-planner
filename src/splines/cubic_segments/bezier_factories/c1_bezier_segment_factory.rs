use std::any;

use nalgebra::Vector2;

use crate::splines::{
    cubic_segments::bezier_segment::BezierSegment,
    spline_error::{SplineError, SplineErrorType},
    spline_segment::SplineSegment,
    spline_segment_factory::SplineSegmentFactory,
};

/// A factory that creates C1 continuous spline segments.
pub struct C1CubicBezierSegmentFactory {
    p1: Vector2<f64>,
    p2: Vector2<f64>,
    p3: Vector2<f64>,
}

impl C1CubicBezierSegmentFactory {
    /// Takes in the final three control points. The first control point
    /// is extrapolated from the previous segment.
    pub fn new(p1: Vector2<f64>, p2: Vector2<f64>, p3: Vector2<f64>) -> Self {
        Self { p1, p2, p3 }
    }
}

impl SplineSegmentFactory for C1CubicBezierSegmentFactory {
    fn build(
        &self,
        previous_segment: Option<&Box<dyn SplineSegment>>,
    ) -> Result<Box<dyn SplineSegment>, SplineError> {
        match previous_segment {
            Some(previous_segment) => {
                let p0 = previous_segment.sample(1.0);
                Ok(Box::new(BezierSegment::new(p0, self.p1, self.p2, self.p3)))
            }
            None => Err(SplineError::new(
                &format!(
                    "{} requires a previous segment for building",
                    any::type_name::<Self>()
                ),
                &SplineErrorType::InvalidPreviousSegment,
            )),
        }
    }
}
