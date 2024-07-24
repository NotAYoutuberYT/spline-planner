use std::io::{stdin, stdout, Write};

use nalgebra::Vector2;
use splines::{
    cubic_segments::hermite_factories::c2_hermite_segment_factory::C2HermiteSegmentFactory,
    linear_segments::linear_segment_factory::LinearSegmentFactory, spline::Spline,
};

pub mod constants;
pub mod splines;

fn main() {
    let segment1 = LinearSegmentFactory::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0));
    let segment2 = C2HermiteSegmentFactory::new(Vector2::new(5.0, 0.0), Vector2::new(1.0, 0.0));

    let mut spline = Spline::new();
    spline.add_segment(&segment1).unwrap();
    spline.add_segment(&segment2).unwrap();

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
    assert!((t - t_guess).abs() <= f64::EPSILON);
}
