# Every language

Rant Math is one set of calls in C, C++, Python, C#, Unity, Rust, TypeScript and HMI code. Each takes the
standard types of its language, so a value from the mesh goes straight in. The pages here use C names, and
this page says how a name reads in each.

## Add it

`rant add math` adds it to a package in C, C++, Python, C# or Rust. Or by hand:

| Language | Add | Use |
|---|---|---|
| C | `rant_math.h` from a release | `#include "rant_math.h"` |
| C++ 17 | CPM, or `rant_math.hpp` from a release | `#include "rant_math.hpp"` |
| Python 3.9+ | the wheel for your platform from a release | `import rant_math` |
| C# | `Rant.Math.<version>.nupkg` from a release | `using Rant.Math;` |
| Unity | `https://github.com/KosmosisDire/rant-math.git#upm` | `using Rant.Math;` |
| Rust | `rant-math = { git = "https://github.com/KosmosisDire/rant-math", tag = "crate-v<version>" }` | `use rant_math::*;` |
| TypeScript | `rant-math-<version>.tgz` from a release | `import * as math from "rant-math";` |
| HMI code | nothing | `math.poseInverse(p)` |

- Each brings the standard types with it and goes beside Rant in one program. In C++ include both headers in
  either order. In Python, C# and TypeScript nothing more is needed.
- Unity allows one assembly of a name, so add the standard types too: Rant itself from
  `https://github.com/KosmosisDire/Rant.git#upm`, or with no Rant `https://github.com/KosmosisDire/Rant.git#upm-types`.
- Rust takes `rant-types` from Rant at one tag. A program using both needs the Rant that tag names.

## Names

| C | C++ | Python and Rust | C# | TypeScript and HMI |
|---|---|---|---|---|
| `rant_pose_inverse` | `rant::math::pose_inverse` | `rant_math.pose_inverse` | `RantMath.PoseInverse` | `poseInverse` |
| `rant_pose2d_apply` | `rant::math::pose2d_apply` | `rant_math.pose2d_apply` | `RantMath.Pose2DApply` | `pose2dApply` |

## Answers

| C | C++ | Python | C# | Rust | TypeScript |
|---|---|---|---|---|---|
| `RantMathResult` and a last pointer | throws `MathError` | raises `MathError` | throws `MathException` | `Result<T, MathError>` | throws `MathError` |
| `int` and a last pointer, such as a raycast | `std::optional` | `None` | `T?` | `Option` | `null` |
| `int` alone | `bool` | `bool` | `bool` | `bool` | `boolean` |
| two last pointers | a tuple | a tuple | a tuple | a tuple | an array |

The error's `kind` says why: `Argument`, `Degenerate`, `Format`, `Memory` or `Outside`. Rust's is the error
itself. A wrong shape a plain answer cannot report, such as triangles not in threes, panics in Rust.

## Lists

| | C++ | Python | C# | Rust | TypeScript |
|---|---|---|---|---|---|
| In | `Span`: a vector, array or `{ ... }` | any iterable or numpy | `ReadOnlySpan` | a slice | an array or `Float64Array` |
| Out | `std::vector` | `list`, numpy for numpy | an array | `Vec` | an array, `Float64Array` for `Float64Array` |
| Masks | `std::vector<uint8_t>` | `bytes`, numpy for numpy | `byte[]` | `Vec<u8>` | `Uint8Array` |

- A call asks a list's size for you, so `*count` never shows.
- A C view of a list is the list: `polygon2d_area(points)`, `mesh_area(vertices, triangles)`.
- In C++ and Python, numpy and `std::span` go in with no copy. A `Float64Array` holds a point's numbers in a row.

## Shapes

`shape_from_sphere` and its kin give a `Shape`. It keeps its own copy of any list, except in Rust, where
`Shape<'a>` borrows it.

```python
line = m.shape_from_polyline(points)
m.shape_distance(line, m.Double3(1, 1, 0))
```

## Images, clouds and grids

- A call that writes a new picture hands back a new `Image`: `image_convert(src, format)`,
  `image_resize(src, filter, width, height)`. `image_crop` owns its pixels too.
- One that changes a value changes it where it is: `image_draw`, `point_cloud_write_points`, `grid_write_value`.
- `point_cloud_layout` gives a cloud with its fields set: give `data` its size.

## Options

`FitOptions` and `AlignOptions` hold what C's do. Give `inliers` room, one per point, and the call fills it.
`AlignOptions.error` holds the error after the call, and in C++ and Rust it is where to write it.

```cpp
std::vector<uint8_t> marks(points.size());
rant::math::FitOptions options;
options.tolerance = 0.01;
options.inliers = marks;
auto plane = rant::math::points_fit_plane(points, options);
```

## Other libraries

| Language | With | Has |
|---|---|---|
| C++ | Eigen | `to_eigen`, `from_eigen`, `points_to_eigen` |
| C++ | OpenCV | a `cv::Mat` goes in as an image, `to_mat`, `from_mat` |
| C++ | PCL | a `pcl::PointCloud` goes in as a cloud, `to_pcl`, `from_pcl` |
| Python | numpy | `image_to_numpy`, `cloud_to_numpy`, `points_to_numpy` and their `from` forms |
| C# | System.Numerics | `v.ToVector3()`, `q.ToNumerics()`, `v.ToDouble3()` |
| Unity | UnityEngine | `v.ToUnity()`, `v.ToRant()`, for `Vector3`, `Quaternion` and `Pose`, in Unity's axes |
| Rust | nalgebra, the feature `nalgebra` | `v.to_nalgebra()`, `m.to_rant()` |

C++ finds each with no setting. `RANT_MATH_NO_EIGEN`, `RANT_MATH_NO_OPENCV` and `RANT_MATH_NO_PCL` leave one out.

C++ also has `+ - * /` on `Double2`, `Double3` and `Double4`, and `*` on `Quaternion`, `Pose` and `Pose2D`.

## Testing a binding

`tools/bindgen/gen.py` writes each binding from the C headers. `tests/bindings/vectors.json` holds the answers
the C library gave, and every binding answers the same, errors included.
