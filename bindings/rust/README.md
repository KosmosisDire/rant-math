# Rant Math for Rust

Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.

```rust
use rant_math::prelude::*;
use rant_math::rant_types::{Double3, Pose, Quaternion};

let pose = Pose { position: Double3 { x: 1.0, y: 2.0, z: 3.0 }, orientation: Quaternion::from_axis_angle(Double3 { x: 0.0, y: 0.0, z: 1.0 }, 0.5) };
let origin = pose.inverse().apply(Double3::default());
```
