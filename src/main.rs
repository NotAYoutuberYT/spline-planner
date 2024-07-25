use std::io::{stdin, stdout, Write};

use nalgebra::Vector2;
use splines::{
    cubic_segments::{
        bezier_factories::{
            c0_bezier_segment_factory::C0CubicBezierSegmentFactory,
            c1_bezier_segment_factory::C1CubicBezierSegmentFactory,
        },
        hermite_factories::{
            c1_hermite_segment_factory::C1HermiteSegmentFactory,
            c2_hermite_segment_factory::C2HermiteSegmentFactory,
        },
    },
    linear_segments::linear_segment_factory::LinearSegmentFactory,
    spline::Spline,
};

pub mod constants;
pub mod splines;

fn main() {
    let segment1 = LinearSegmentFactory::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0));
    let segment2 = C1HermiteSegmentFactory::new(
        Vector2::new(0.0, 1.0),
        Vector2::new(0.0, 1.0),
        Vector2::new(-1.0, 1.0),
    );
    let segment3 = C2HermiteSegmentFactory::new(Vector2::new(1.0, 1.0), Vector2::new(1.0, 0.0));
    let segment4 = C0CubicBezierSegmentFactory::new(
        Vector2::new(1.0, 1.0),
        Vector2::new(1.0, 2.0),
        Vector2::new(2.0, 1.0),
        Vector2::new(2.0, 2.0),
    );
    let segment5 = C1CubicBezierSegmentFactory::new(
        Vector2::new(-1.0, 3.0),
        Vector2::new(0.0, 1.0),
        Vector2::new(0.0, 0.0),
    );

    let mut spline = Spline::new();
    spline.add_segment(&segment1).unwrap();
    spline.add_segment(&segment2).unwrap();
    spline.add_segment(&segment3).unwrap();
    spline.add_segment(&segment4).unwrap();
    spline.add_segment(&segment5).unwrap();

    print!("test number: ");
    stdout().flush().unwrap();
    let mut input = String::new();
    stdin().read_line(&mut input).unwrap();

    let t = input.trim().parse().unwrap();
    let position = spline.sample(t);
    let derivative = spline.derivative(t);
    let arc_length = spline.arc_length(t);
    let t_guess = spline.time_at_arc_length(arc_length);

    println!("t: {t}\nposition: {position}\nderivative: {derivative}\narc_length: {arc_length}\nt_guess: {t_guess}");
    assert!((t - t_guess).abs() <= 1e-8);
}
