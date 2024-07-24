use nalgebra::Vector2;

use crate::splines::spline_segment::SplineSegment;

/// SplineSegments that can be expressed as third-degree polynomial.
/// [this video](https://www.youtube.com/watch?v=jvPPXbo87ds)
/// is a particularly good first introduction to cubic splines.
pub struct CubicSegment {
    degree_zero_term: Vector2<f64>,
    degree_one_term: Vector2<f64>,
    degree_two_term: Vector2<f64>,
    degree_three_term: Vector2<f64>,
    total_arc_length: f64,
}

impl CubicSegment {
    /// Creates a cubic segment given the coefficients of all 4 terms.
    pub fn new(
        degree_zero_term: Vector2<f64>,
        degree_one_term: Vector2<f64>,
        degree_two_term: Vector2<f64>,
        degree_three_term: Vector2<f64>,
    ) -> Self {
        let mut segment = Self {
            degree_zero_term,
            degree_one_term,
            degree_two_term,
            degree_three_term,
            total_arc_length: 0.0,
        };

        segment.total_arc_length = segment.arc_length(1.0);
        segment
    }
}

impl SplineSegment for CubicSegment {
    fn sample(&self, t: f64) -> Vector2<f64> {
        self.degree_zero_term
            + (self.degree_one_term * t)
            + (self.degree_two_term * t * t)
            + (self.degree_three_term * t * t * t)
    }

    fn derivative(&self, t: f64) -> Vector2<f64> {
        self.degree_one_term
            + (2.0 * self.degree_two_term * t)
            + (3.0 * self.degree_three_term * t * t)
    }

    fn arc_length(&self, t: f64) -> f64 {
        // Uses gaussian quadrature with n = 5. (This is computationally
        // cheap enough and *extremely* accurate. That's important, as this eventually
        // gets fed into newton-raphson, adding yet another source of error).
        //
        // See https://en.wikipedia.org/wiki/Gaussian_quadrature for more.
        let term_one: f64 = 0.568889 * self.derivative(t / 2.0).magnitude();
        let term_two: f64 = 0.478629 * self.derivative(t * 0.538469 / 2.0 + t / 2.0).magnitude();
        let term_three: f64 = 0.478629 * self.derivative(-t * 0.538469 / 2.0 + t / 2.0).magnitude();
        let term_four: f64 = 0.236927 * self.derivative(t * 0.90618 / 2.0 + t / 2.0).magnitude();
        let term_five: f64 = 0.236927 * self.derivative(-t * 0.90618 / 2.0 + t / 2.0).magnitude();

        (t / 2.0) * (term_one + term_two + term_three + term_four + term_five)
    }

    fn total_arc_length(&self) -> f64 {
        return self.total_arc_length;
    }
}
