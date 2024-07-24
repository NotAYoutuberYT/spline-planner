use nalgebra::Vector2;

use super::cubic_segment::CubicSegment;

/// A specific type of cubic segment.
pub struct BezierSegment {}

impl BezierSegment {
    pub fn new(
        p0: Vector2<f64>,
        p1: Vector2<f64>,
        p2: Vector2<f64>,
        p3: Vector2<f64>,
    ) -> CubicSegment {
        CubicSegment::new(
            p0,
            (-3.0 * p0) + (3.0 * p1),
            (3.0 * p0) + (-6.0 * p1) + (3.0 * p2),
            -p0 + (3.0 * p1) + (-3.0 * p2) + p3,
        )
    }
}
