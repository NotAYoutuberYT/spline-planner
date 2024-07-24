use std::any;

use nalgebra::Vector2;

use crate::splines::{
    cubic_segments::bezier_segment::BezierSegment,
    spline_error::{SplineError, SplineErrorType},
    spline_segment::SplineSegment,
    spline_segment_factory::SplineSegmentFactory,
};

/// A factory that creates C2 continuous spline segments.
pub struct C2HermiteSegmentFactory {
    p1: Vector2<f64>,
    v1: Vector2<f64>,
}

impl C2HermiteSegmentFactory {
    /// Takes in the final velocity and control point. The first velocity and control point
    /// are extrapolated from the previous segment.
    pub fn new(p1: Vector2<f64>, v1: Vector2<f64>) -> Self {
        Self { p1, v1 }
    }
}

impl SplineSegmentFactory for C2HermiteSegmentFactory {
    fn build(
        &self,
        previous_segment: Option<&Box<dyn SplineSegment>>,
    ) -> Result<Box<dyn SplineSegment>, SplineError> {
        match previous_segment {
            Some(previous_segment) => {
                let p0 = previous_segment.sample(1.0);
                let v0 = previous_segment.derivative(1.0);
                Ok(Box::new(BezierSegment::new(
                    p0,
                    p0 + (v0 / 3.0),
                    self.p1 - (self.v1 / 3.0),
                    self.p1,
                )))
            }
            None => Err(SplineError::new(
                &format!(
                    "{} requires a previous segment for building",
                    any::type_name::<C2HermiteSegmentFactory>()
                ),
                &SplineErrorType::InvalidPreviousSegment,
            )),
        }
    }
}
