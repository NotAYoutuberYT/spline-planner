use super::spline_segment::SplineSegment;

/// Used in the construction of SplineSegments and
/// Splines. Takes in some information, and is able to build
/// a new spline given a previous spline. See specific implementations
/// for more information.
pub trait SplineSegmentFactory {
    /// Builds a new SplineSegment, given a previous segment.
    fn build(&self, previous_segment: Option<&Box<dyn SplineSegment>>) -> Box<dyn SplineSegment>;
}
