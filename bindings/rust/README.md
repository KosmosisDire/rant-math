# Rant Math for Rust

Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.

```rust
use rant_math::rant_types::{Double3, Pose};

let pose = Pose { position: Double3 { x: 1.0, y: 2.0, z: 3.0 }, orientation: rant_math::quaternion_from_axis_angle(Double3 { x: 0.0, y: 0.0, z: 1.0 }, 0.5) };
let origin = rant_math::pose_apply(rant_math::pose_inverse(pose), Double3::default());
```
