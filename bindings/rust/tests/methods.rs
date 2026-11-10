//! The methods of the standard types, from rant_math::prelude, and of math's own types.

use rant_math::prelude::*;
use rant_math::rant_types::*;
use rant_math::{Axes, AxesPreset};

fn d3(x: f64, y: f64, z: f64) -> Double3 {
    Double3 { x, y, z }
}

#[test]
fn methods() {
    let q = Quaternion::from_axis_angle(d3(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
    let p = Pose { position: d3(1.0, 0.0, 0.0), orientation: q };
    assert!(p.apply(d3(1.0, 0.0, 0.0)).is_near(d3(1.0, 1.0, 0.0), 1e-12));
    assert!(p.inverse().apply(p.apply(d3(2.0, 3.0, 4.0))).is_near(d3(2.0, 3.0, 4.0), 1e-12));
    assert_eq!(d3(3.0, 4.0, 0.0).length(), 5.0);
    assert!(q.is_near(Quaternion { x: -q.x, y: -q.y, z: -q.z, w: -q.w }, 1e-12), "q is near -q");
    let twist = Twist { linear: d3(1.0, 0.0, 0.0), angular: Double3::default() };
    assert!(twist.transform(p).linear.is_near(d3(0.0, 1.0, 0.0), 1e-12), "the value need not come first in C");
    let ball = Sphere { center: Double3::default(), radius: 2.0 };
    assert_eq!(ball.to_shape().distance(d3(5.0, 0.0, 0.0)), 3.0);
    let square = Polygon2D { points: vec![Double2 { x: 0.0, y: 0.0 }, Double2 { x: 2.0, y: 0.0 }, Double2 { x: 2.0, y: 2.0 }, Double2 { x: 0.0, y: 2.0 }] };
    assert_eq!(square.area(), 4.0);
    assert_eq!(Axes::of(AxesPreset::Unity).point_to(d3(1.0, 2.0, 3.0)), rant_math::axes_point_to(rant_math::axes_of(AxesPreset::Unity), d3(1.0, 2.0, 3.0)));
}
