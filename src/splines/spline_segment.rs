use nalgebra::Vector2;

use crate::constants::NEWTON_ITERATIONS;

/// A single segment of a Spline. Functions that take in
/// a parameterization have undefined behavior for t disobeying
/// 0 <= t <= 1.
pub trait SplineSegment {
    /// Samples the segment at a specific parameterization.
    fn sample(&self, t: f64) -> Vector2<f64>;

    /// Samples the segment's derivative at a specific parameterization.
    /// The derivative is not normalized.
    fn derivative(&self, t: f64) -> Vector2<f64>;

    /// Samples the segments's total arc length at a specific parameterization.
    /// Namely, it finds the integral of the derivative's magnitude from 0 to t.
    fn arc_length(&self, t: f64) -> f64;

    /// Finds the segment's total arc length.
    /// Is more efficient than arcLength(1) and/or cached for some types of segments.
    fn total_arc_length(&self) -> f64 {
        self.arc_length(1.0)
    }

    /// Given some arc length L, finds the parameterization t at which arcLength(t) = L.
    fn time_at_arc_length(&self, arc_length: f64) -> f64 {
        // set the initial guess as if the function is linear
        self.time_at_arc_length_from_guess(arc_length, arc_length / self.total_arc_length())
    }

    /// Works the same as time_at_arc_length, but uses an explicit
    /// first guess for potentially more accurate results.
    fn time_at_arc_length_from_guess(&self, arc_length: f64, initial_guess: f64) -> f64 {
        // apply newton's method to approximate the zero of
        // f(t) = arcLength(t) - length, where f'(t) is of course
        // the magnitude of the curve's derivative
        let mut guess = initial_guess;
        for _ in 0..NEWTON_ITERATIONS {
            guess =
                guess - (self.arc_length(guess) - arc_length) / self.derivative(guess).magnitude();
        }

        guess
    }
}
