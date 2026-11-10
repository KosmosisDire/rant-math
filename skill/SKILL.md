---
name: math
description: Writing code with Rant Math in C, C++, Python, C#, Unity, Rust, TypeScript or HMI code: poses, axes, shapes, fits, cameras, images and point clouds on Rant's standard types
---

# Rant Math

Math on Rant's standard types, in every language Rant has. Reach for it before writing vector, pose or shape
math by hand. Full docs: https://github.com/KosmosisDire/rant-math/tree/main/docs (`languages.md` first).

- Add it with `rant add math`, never by hand.
- Bodies are x forward, y left, z up. Meters, radians, seconds.
- It needs no Rant, and goes beside it in one program.

## One name in every language

C `rant_pose_inverse` is:

| C++ | Python and Rust | C# | TypeScript and HMI |
|---|---|---|---|
| `rant::math::pose_inverse` | `rant_math.pose_inverse` | `RantMath.PoseInverse` | `poseInverse` (`math.poseInverse` in HMI code) |

`rant_pose2d_apply` is `Pose2DApply` in C# and `pose2dApply` in TypeScript.

## Methods

Prefer a call as a method of its first value: `pose.inverse()`, `q.rotate(v)`, `box.contains(p)`.

- C++ and Python also have operators: `+ -` on vectors, `* k`, `/ k`, and `*` to compose and apply.
- C# has extension methods (`p.Inverse()`, under `using Rant.Math;`) and no operators.
- Rust needs `use rant_math::prelude::*;` and has no operators.
- TypeScript groups each type's calls by its name: `Pose.inverse(p)`, `Quaternion.fromAxisAngle(a, t)`.
- A maker is a static: `Quaternion::from_axis_angle(a, t)` in C++ and Rust, `Quaternion.from_axis_angle` in Python.
- `<other>_from_<type>` is `to_<other>`: `sphere.to_shape()`. A list type reads its points: `polygon.area()`.
- C++: linking `rant_math` defines `RANT_MATH`, which every file of the program needs.

## What a call gives back

- A call that can fail throws `MathError` (`MathException` in C#), or in Rust returns `Result`. In C it returns
  `RantMathResult` and writes through its last pointer.
- A call that can miss, such as a raycast, gives an optional, `None` or `null`.
- A list comes back as a list. No binding needs a count or room.
- A view of a list in C is the list: `polyline_length(points)`. A mesh is the standard Mesh: `mesh_area(mesh)`,
  and `shape_mesh(shape, segments, crease)` returns one, its corners x, y, z and a normal as f32, plus rgba when
  the mesh has colors in rgba or rgb.

```python
import rant_math as m
pose = m.Pose(m.Double3(1, 2, 3), m.Quaternion.from_axis_angle(m.Double3(0, 0, 1), 0.5))
pose.inverse() * m.Double3()
```

```cpp
#include "rant_math.hpp"
using namespace rant::math;
Pose pose{ { 1, 2, 3 }, Quaternion::from_axis_angle({ 0, 0, 1 }, 0.5) };
Double3 back = pose.inverse() * Double3{ 1, 2, 3 };
```

## Find the right call

| Goal | Calls |
|---|---|
| Move between frames | `pose_compose`, `pose_inverse`, `pose_between`, `pose_apply`, `pose_apply_points` |
| Another tool's axes | `axes_of(Unity)`, `axes_point_to`, `axes_pose_from`; Unity's `v.ToUnity()` |
| Distance and contact | `<shape>_distance`, `_closest`, `_contains`, `_raycast`, `shape_gap`, `shape_overlaps` |
| Mixed kinds of shape | `shape_from_sphere` and its kin, then `shape_distance` |
| Fit to points | `points_fit_plane`, `points_fit_line`, `points_fit_pose`; `FitOptions.tolerance` for RANSAC |
| Point clouds | `point_cloud_read_points`, `points_downsample`, `points_normals`, `points_clusters`, `points_align` |
| Grids and maps | `grid_read_values`, `grid_read_colors`, `grid_cell`, `grid_center` |
| Near points | `kdtree_create`, `kdtree_nearest`, `kdtree_within` |
| Cameras | `camera_project`, `camera_ray`, `camera_unproject`, `image_depth_points` |
| Images | `image_convert`, `image_resize`, `image_rotate`, `image_colorize`, `image_undistort`, `image_draw` |
| Colour a depth image | `colormap_ramp(Turbo, 256)` or `color_ramp(stops, 256)`, then `image_colorize` |
| Dense matrices | `matrix_solve`, `matrix_svd`, `matrix_eigen`, `matrix_inverse` |

## Keep in mind

- A default `Quaternion` is zeros, not a rotation. Use `identity_rotation` and `identity_pose`.
- `a * b` on poses is a's frame then b's.
- A distance is signed for a solid, negative inside.
- A mask is one `uint8` per point, 1 to keep.
- numpy in gives numpy out, and a `Float64Array` in gives one out in TypeScript.
- In C++ a `cv::Mat` and a `pcl::PointCloud` go straight in as an image and a cloud.
