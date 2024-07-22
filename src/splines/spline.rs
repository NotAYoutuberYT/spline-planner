use nalgebra::Vector2;

use super::{spline_segment::SplineSegment, spline_segment_factory::SplineSegmentFactory};

/// A series of SplineSegments forming a
/// spline in 2D space.
///
/// The recommended way to construct splines is by using the
/// add_segment method, as it allows the spline to guarantee
/// it maintains the level of continuity provided by factories.
pub struct Spline {
    segments: Vec<Box<dyn SplineSegment>>,
}

impl Spline {
    /// Creates a spline from an initial array of SplineSegments.
    /// It's not recommended to provide more than perhaps one initial segment;
    /// see add_segment for more information on adding segments.
    pub fn from_segments(segments: Vec<Box<dyn SplineSegment>>) -> Self {
        Self { segments }
    }

    /// Creates a new, empty spline.
    pub fn new() -> Self {
        Self::from_segments(Vec::new())
    }

    /// Extends the spline by taking SplineSegmentFactories,
    /// providing the previous segment to factory.build(),
    /// and appending the new SplineSegment to the spline.
    pub fn add_segment(&mut self, factory: Box<dyn SplineSegmentFactory>) {
        let segment = factory.build(self.segments.last());
        self.segments.push(segment);
    }

    /// Samples the spline at a specific parameterization.
    pub fn sample(&self, t: f64) -> Vector2<f64> {
        let index = t as usize;

        let segment = self.segments.get(index);
        segment.map_or(Vector2::default(), |segment| segment.sample(t - t.floor()))
    }

    /// Samples the spline's derivative at a specific parameterization.
    /// The derivative is not normalized.
    pub fn derivative(&self, t: f64) -> Vector2<f64> {
        let index = t as usize;

        let segment = self.segments.get(index);
        segment.map_or(Vector2::default(), |segment| {
            segment.derivative(t - t.floor())
        })
    }

    /// Samples the spline's total arc length at a specific parameterization.
    /// Namely, it finds the integral of the derivative's magnitude from 0 to t.
    pub fn arc_length(&self, t: f64) -> f64 {
        let mut length: f64 = 0.0;
        let mut iterator = self.segments.iter().enumerate();

        if t > iterator.len() as f64 {
            return f64::default();
        }

        while let Some((index, segment)) = iterator.next() {
            if t as usize > index {
                length += segment.total_arc_length();
            } else {
                length += segment.arc_length(t - t.floor());
                break;
            }
        }

        length
    }

    /// Finds the total arc length of the spline.
    pub fn total_arc_length(&self) -> f64 {
        let mut total: f64 = 0.0;
        for segment in self.segments.iter() {
            total += segment.total_arc_length();
        }

        total
    }

    /// Given some arc length L, finds the parameterization t at which arcLength(t) = L.
    pub fn time_at_arc_length(&self, arc_length: f64) -> f64 {
        // while it would be possible to do this as a call
        // to the initial guess version, it's best to just leave
        // the initial guess to the spline segment
        let mut total_length: f64 = 0.0;
        let mut iterator = self.segments.iter().enumerate();

        if arc_length > self.total_arc_length() as f64 {
            return self.segments.len() as f64;
        }

        while let Some((index, segment)) = iterator.next() {
            let segment_length = segment.total_arc_length();
            if arc_length >= total_length + segment_length {
                total_length += segment_length;
            } else {
                return index as f64 + segment.time_at_arc_length(arc_length - total_length);
            }
        }

        f64::default()
    }

    /// Works the same as time_at_arc_length, but uses an explicit
    /// first guess for potentially more accurate results.
    pub fn time_at_arc_length_from_guess(&self, arc_length: f64, initial_guess: f64) -> f64 {
        let mut total_length: f64 = 0.0;
        let mut iterator = self.segments.iter().enumerate();

        if arc_length > self.total_arc_length() as f64 {
            return self.segments.len() as f64;
        }

        while let Some((index, segment)) = iterator.next() {
            let segment_length = segment.total_arc_length();
            if arc_length >= total_length + segment_length {
                total_length += segment_length;
            } else {
                return index as f64
                    + segment.time_at_arc_length_from_guess(
                        arc_length - total_length,
                        initial_guess - initial_guess.floor(),
                    );
            }
        }

        f64::default()
    }
}
