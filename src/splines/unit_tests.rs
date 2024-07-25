#[cfg(test)]
pub mod tests {
    use core::f64;

    use nalgebra::Vector2;

    use crate::splines::{
        linear_segments::{
            linear_segment::LinearSegment, linear_segment_factory::LinearSegmentFactory,
        },
        spline_segment::SplineSegment,
        spline_segment_factory::SplineSegmentFactory,
    };

    macro_rules! assert_approx_equal {
        ($vector1:expr, $vector2:expr, $d:expr) => {
            if !($vector1.relative_eq(&$vector2, $d, $d)) {
                panic!("expected: {}, got: {}", $vector1, $vector2);
            }
        };
    }

    #[test]
    fn linear_segment() {
        let segment = LinearSegment::new(Vector2::new(1.0, 1.0), Vector2::new(2.0, 2.0));

        assert_approx_equal!(segment.sample(0.0), Vector2::new(1.0, 1.0), f64::EPSILON);
        assert_approx_equal!(segment.sample(0.2), Vector2::new(1.2, 1.2), f64::EPSILON);
        assert_approx_equal!(segment.sample(1.0), Vector2::new(2.0, 2.0), f64::EPSILON);

        assert_approx_equal!(
            segment.derivative(0.0),
            Vector2::new(1.0, 1.0),
            f64::EPSILON
        );
        assert_approx_equal!(
            segment.derivative(0.2),
            Vector2::new(1.0, 1.0),
            f64::EPSILON
        );
        assert_approx_equal!(
            segment.derivative(1.0),
            Vector2::new(1.0, 1.0),
            f64::EPSILON
        );
    }

    #[test]
    fn linear_factory() {
        let factory = LinearSegmentFactory::new(Vector2::new(0.0, 1.0), Vector2::new(1.0, 0.0));
        let segment = factory.build(None).unwrap();

        assert_approx_equal!(segment.sample(0.5), Vector2::new(0.5, 0.5), f64::EPSILON);
        assert_approx_equal!(
            segment.derivative(0.5),
            Vector2::new(1.0, -1.0),
            f64::EPSILON
        );

        let segment = factory.build(Some(&segment)).unwrap();

        assert_approx_equal!(segment.sample(0.5), Vector2::new(0.5, 0.5), f64::EPSILON);
        assert_approx_equal!(
            segment.derivative(0.5),
            Vector2::new(1.0, -1.0),
            f64::EPSILON
        );
    }
}
