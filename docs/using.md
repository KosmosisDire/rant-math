# Using Rant Math

Math on Rant's standard types, in C99. It needs nothing but the types, so it works with Rant or
without it.

## Add it

In a RANT workspace:

```sh
rant add math
```

With CMake and CPM:

```cmake
CPMAddPackage(NAME math GITHUB_REPOSITORY KosmosisDire/rant-math GIT_TAG v0.0.1)
target_link_libraries(my_node PRIVATE rant_math)
```

As one file: take `rant_math.h` from a release, and build it in one source file:

```c
#define RANT_MATH_IMPLEMENTATION
#include "rant_math.h"
```

- `rant_math.h` carries Rant's types, so it goes beside `rant.h` in either order.
- Link `m` on Linux.

## First call

```c
#include "rant_math.h"

RantPose robot = { { 2, 0, 0 }, { 0, 0, 0, 1 } };
RantDouble3 tool = rant_pose_apply(robot, rant_double3(0.5, 0, 0));   /* 2.5, 0, 0 */
```

## Rules every call keeps

| | |
|---|---|
| Units | meters, radians, seconds |
| Axes | x forward, y left, z up. Images: x right, y down, from the top left |
| Results | `RantMathResult`, `RANT_MATH_OK` is 0, `rant_math_result_text` names one |
| Answers | through the last pointer |
| Lists | `*count` holds the room and gets back the need |
| Masks | one `uint8_t` per point, 0 or 1 |
| Views | a polygon, mesh, image or cloud is a view of your memory, never copied |

Ask a list's size first, then fill it:

```c
uint32_t count = 0;
rant_polygon2d_triangulate(square, NULL, &count);       /* RANT_MATH_ERR_SPACE, count is the need */
uint32_t *triangles = malloc(3 * count * sizeof *triangles);
rant_polygon2d_triangulate(square, triangles, &count);
```

## The parts

| Page | Headers |
|---|---|
| [Frames](frames.md) | vectors, rotations, matrices, poses, motion, axes, places on the Earth |
| [Shapes](shapes.md) | lines and planes, 2D regions, solids, curves, surfaces, any shape, point sets, fits |
| [Sensors](sensors.md) | colours, cameras, images, point clouds, neighbors, grids, detections |

Each header has one call per job, named `rant_<type>_<job>`, as `rant_sphere_distance`.
