/* Curves: polylines and paths of poses, in 3D and 2D. Distance along one starts at 0 at its
 * first point, and a path runs through its poses' positions. */
#ifndef RANT_MATH_CURVE_H
#define RANT_MATH_CURVE_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API double         rant_polyline_length(RantPolylineView l);
/* HUGE_VAL for a line of no points. A line of no length has a zero tangent. */
RANT_MATH_API double         rant_polyline_distance(RantPolylineView l, RantDouble3 point);
/* The nearest point of the line, the way the line runs there and how far along it lies. */
RANT_MATH_API RantCurvePoint rant_polyline_closest(RantPolylineView l, RantDouble3 point);
/* The point distance along the line, held to its ends. */
RANT_MATH_API RantCurvePoint rant_polyline_at(RantPolylineView l, double distance);
RANT_MATH_API RantAlignedBox rant_polyline_bounds(RantPolylineView l);
/* One segment per step of nonzero length. */
RANT_MATH_API RantMathResult rant_polyline_edges(RantPolylineView l, RantSegment *out, uint32_t *count);
/* count points spread evenly along the line, the first at its start and the last at its end. */
RANT_MATH_API RantMathResult rant_polyline_resample(RantPolylineView l, RantDouble3 *out, uint32_t count);
/* The fewest of its points that keep the line within tolerance of every point it drops,
 * found by Douglas and Peucker's method. The two ends always stay. */
RANT_MATH_API RantMathResult rant_polyline_simplify(RantPolylineView l, double tolerance, RantDouble3 *out, uint32_t *count);

RANT_MATH_API double           rant_polyline2d_length(RantPolyline2DView l);
RANT_MATH_API double           rant_polyline2d_distance(RantPolyline2DView l, RantDouble2 point);
RANT_MATH_API RantCurvePoint2D rant_polyline2d_closest(RantPolyline2DView l, RantDouble2 point);
RANT_MATH_API RantCurvePoint2D rant_polyline2d_at(RantPolyline2DView l, double distance);
RANT_MATH_API RantAlignedBox2D rant_polyline2d_bounds(RantPolyline2DView l);
RANT_MATH_API RantMathResult   rant_polyline2d_edges(RantPolyline2DView l, RantSegment2D *out, uint32_t *count);
/* One plane per segment of nonzero length, its normal on the right of the way the line runs,
 * as a wall's face seen walking along it. */
RANT_MATH_API RantMathResult   rant_polyline2d_planes(RantPolyline2DView l, RantPlane2D *out, uint32_t *count);
RANT_MATH_API RantMathResult   rant_polyline2d_resample(RantPolyline2DView l, RantDouble2 *out, uint32_t count);
RANT_MATH_API RantMathResult   rant_polyline2d_simplify(RantPolyline2DView l, double tolerance, RantDouble2 *out, uint32_t *count);

RANT_MATH_API double         rant_path_length(RantPathView p);
RANT_MATH_API double         rant_path_distance(RantPathView p, RantDouble3 point);
RANT_MATH_API RantCurvePoint rant_path_closest(RantPathView p, RantDouble3 point);
/* The pose distance along the path, its orientation turned between its two neighbors' by
 * slerp. A turn in place takes no distance, so at passes it at once. */
RANT_MATH_API RantPose       rant_path_at(RantPathView p, double distance);
RANT_MATH_API RantAlignedBox rant_path_bounds(RantPathView p);
RANT_MATH_API RantMathResult rant_path_resample(RantPathView p, RantPose *out, uint32_t count);

RANT_MATH_API double           rant_path2d_length(RantPath2DView p);
RANT_MATH_API double           rant_path2d_distance(RantPath2DView p, RantDouble2 point);
RANT_MATH_API RantCurvePoint2D rant_path2d_closest(RantPath2DView p, RantDouble2 point);
/* The heading turns the shorter way between neighbors. */
RANT_MATH_API RantPose2D       rant_path2d_at(RantPath2DView p, double distance);
RANT_MATH_API RantAlignedBox2D rant_path2d_bounds(RantPath2DView p);
RANT_MATH_API RantMathResult   rant_path2d_resample(RantPath2DView p, RantPose2D *out, uint32_t count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_CURVE_H */
