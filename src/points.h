/* Sets of points: their bounds, mean and spread, and the shapes that hold them. Each call
 * fails with RANT_MATH_ERR_ARGUMENT for a set of no points. */
#ifndef RANT_MATH_POINTS_H
#define RANT_MATH_POINTS_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API RantMathResult rant_points_bounds(const RantDouble3 *points, uint32_t count, RantAlignedBox *out);
RANT_MATH_API RantMathResult rant_points_mean(const RantDouble3 *points, uint32_t count, RantDouble3 *out);
/* Divided by the count, as PCL does, not by one less. */
RANT_MATH_API RantMathResult rant_points_covariance(const RantDouble3 *points, uint32_t count, RantCovariance3 *out);
/* A box along the points' principal axes, x the one they spread along most: close to the
 * smallest box, not always it. */
RANT_MATH_API RantMathResult rant_points_oriented_box(const RantDouble3 *points, uint32_t count, RantOrientedBox *out);
/* The smallest ball holding every point, by Welzl's method. */
RANT_MATH_API RantMathResult rant_points_enclosing_sphere(const RantDouble3 *points, uint32_t count, RantSphere *out);

RANT_MATH_API RantMathResult rant_points2d_bounds(const RantDouble2 *points, uint32_t count, RantAlignedBox2D *out);
RANT_MATH_API RantMathResult rant_points2d_mean(const RantDouble2 *points, uint32_t count, RantDouble2 *out);
/* The convex hull, counterclockwise from the lowest leftmost point, with no corner on a
 * straight edge. out_count follows the count rule. */
RANT_MATH_API RantMathResult rant_points2d_hull(const RantDouble2 *points, uint32_t count, RantDouble2 *out, uint32_t *out_count);
/* The smallest rectangle at any angle holding every point, with its angle in (-pi/2, pi/2]. */
RANT_MATH_API RantMathResult rant_points2d_oriented_box(const RantDouble2 *points, uint32_t count, RantOrientedBox2D *out);
RANT_MATH_API RantMathResult rant_points2d_enclosing_circle(const RantDouble2 *points, uint32_t count, RantCircle *out);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_POINTS_H */
