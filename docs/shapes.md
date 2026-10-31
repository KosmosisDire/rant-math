# Shapes

Measuring, testing and fitting shapes.

| Header | For |
|---|---|
| `line.h` | planes, segments and rays in 3D and 2D, and where they meet |
| `region.h` | 2D boxes, circles, capsules and polygons |
| `solid.h` | boxes, spheres, capsules, cylinders, cones and prisms |
| `curve.h` | polylines and paths of poses: length, the point at a distance, resample, simplify |
| `surface.h` | flat polygons in 3D and triangle meshes |
| `shape.h` | any shape behind one type: distances of many points, gaps, samples, meshes |
| `points.h` | a point set's bounds, mean, covariance, hull and the boxes and balls that hold it |
| `fit.h` | planes, lines, circles, spheres, poses and polynomials fitted to points |

## What every shape answers

```c
double d = rant_sphere_distance(ball, point);     /* negative inside */
RantSurfacePoint near = rant_sphere_closest(ball, point);
```

- `distance` is signed, negative inside. A curve or a surface has no inside, so its distance is never negative.
- `closest` is the nearest point of the boundary, from inside or out, with its outward normal.
- `contains` counts the boundary as inside.
- `raycast` finds where a ray first meets the boundary at or beyond its origin. From inside, that is where it leaves.
- `inertia` is that of a body of even density, about its center in the shape's frame.
- `iou` is the intersection over union of two boxes, for rotated boxes too.

## Orders

- 2D corners, edges and planes run counterclockwise. Plane i holds edge i, its normal facing out.
- A box's corners run counterclockwise round the bottom seen from above, then the same round the top. A prism's follow its base.
- 3D edges are the bottom ones, the top ones, then the uprights at each bottom corner.
- 3D planes are the sides in edge order, then the bottom and the top.
- A polygon's normal follows the right hand rule round its corners. A triangle's faces the side its corners run counterclockwise.

## Any shape

```c
RantShape zone = rant_shape_from_aligned_box(box);
rant_shape_distances(&zone, points, distances, count);   /* inside where distance <= 0 */
```

- A shape made from a view points at your data.
- `rant_shape_gap` is how far apart two shapes are, 0 when they touch.
- `rant_shape_mesh` makes a triangle mesh of any shape, closed and facing out.
- The typed calls, as `rant_sphere_distance`, answer the same and cost less.

## Lines and planes

- A plane's normal and a ray's direction need not be unit length. Every one returned is.
- A Plane2D is a line.

## Fits

```c
RantFitOptions ransac = { 0 };
ransac.tolerance = 0.01;                                  /* RANSAC, outliers past 1 cm */
RantPlane floor;
rant_points_fit_plane(points, count, &ransac, &floor);
```

- Without a tolerance a fit is least squares over every point.
- With one, RANSAC keeps the model most points lie near, then fits those points alone.
- A fitted plane faces the origin, where the sensor that saw the points sits.
- `RANT_MATH_ERR_DEGENERATE` means the points fix no single answer, as points on one line for a plane.
