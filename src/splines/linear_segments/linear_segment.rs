use nalgebra::Vector2;

use crate::splines::spline_segment::SplineSegment;

/// Linear interpolation between two points to form a SplineSegment.
pub struct LinearSegment {
    p0: Vector2<f64>,
    p1: Vector2<f64>,
    total_length: f64,
}

impl LinearSegment {
    /// Creates a linear segment from two points.
    pub fn new(p0: Vector2<f64>, p1: Vector2<f64>) -> Self {
        Self {
            p0,
            p1,
            total_length: (p1 - p0).magnitude(),
        }
    }
}

impl SplineSegment for LinearSegment {
    fn sample(&self, t: f64) -> Vector2<f64> {
        self.p0.lerp(&self.p1, t)
    }

    fn derivative(&self, _t: f64) -> Vector2<f64> {
        return self.p1 - self.p0;
    }

    fn arc_length(&self, t: f64) -> f64 {
        return self.total_length * t;
    }

    fn time_at_arc_length(&self, arc_length: f64) -> f64 {
        return arc_length / self.total_length;
    }

    fn time_at_arc_length_from_guess(&self, arc_length: f64, _initial_guess: f64) -> f64 {
        // since there's a simple expression for the arc length, the guess is irrelevant
        return self.time_at_arc_length(arc_length);
    }
}
