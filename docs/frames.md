# Frames

Where things are and how they move.

| Header | For |
|---|---|
| `vector.h` | Double2, Double3 and Double4 math, and Float and Int vectors to and from them |
| `rotation.h` | quaternions, Euler angles, axis and angle, rotation vectors, 2D angles |
| `matrix.h` | Matrix3x3, Matrix4x4 and any dense matrix: solve, SVD, eigen, Cholesky |
| `pose.h` | poses in 3D and 2D: compose, invert, apply, interpolate, teach from points |
| `motion.h` | twists, wrenches, covariances and inertia moved between frames |
| `axes.h` | other tools' axes: Unity, Unreal, Godot, glTF, Blender, OpenCV, NED |
| `geo.h` | latitude, longitude and altitude to and from ECEF and local east north up |

## Poses

```c
RantPose camera_in_world = rant_pose_compose(robot_in_world, camera_on_robot);
RantDouble3 seen = rant_pose_apply_inverse(camera_in_world, target);
```

- A pose maps a point from its own frame into its parent's.
- `compose(a, b)` is b given in a's frame, so frames chain left to right.
- A Transform's math is its `.pose`.

## Rotations

- Quaternions are x, y, z, w, and `rant_quaternion_compose` is the Hamilton product a b.
- `RANT_EULER_FIXED_*` turns about the parent's axes, `RANT_EULER_MOVING_*` about the axes as they turn.
- `RANT_EULER_FIXED_XYZ` is roll, pitch, yaw, the same turn as `RANT_EULER_MOVING_ZYX` with the angles reversed.
- The angles come in the order the name lists the axes.
- A rotation vector is the axis times the angle, as robot controllers and gyros give it.

## Matrices

- A matrix takes points as columns, `p' = M p`, so a Matrix4x4 holds its translation in `m[0][3]`, `m[1][3]` and `m[2][3]`.
- Matrix3x3 and Matrix4x4 are the f32 wire types, computed in double.
- `RantMatrixView` is any matrix of doubles, rows one after another: `{ &c.m[0][0], 6, 6 }` for a Covariance6.

## Motion

- A twist is the velocity of the point at its frame's origin and the angular velocity, both in that frame's axes.
- `rant_pose_integrate` follows the exact arc a constant twist traces, as Odometry gives it.

## Other tools' axes

```c
RantAxes unity = rant_axes_of(RANT_AXES_UNITY);
RantPose in_unity = rant_axes_pose_to(unity, pose);
```

- `_to` turns a Rant value into the tool's axes, `_from` turns it back.
- A `RantAxes` names the Rant axis each tool axis is, and how many tool units make a meter.
- A turn, as an angular velocity, a torque or a quaternion's x y z, also flips for a left handed tool.
- `rant_axes_make` makes any other.

## Places on the Earth

```c
RantDouble3 local = rant_geo_point_to_enu(home, fix);   /* meters east, north, up of home */
```

- Latitude and longitude are degrees on WGS84, altitude is meters above the ellipsoid.
