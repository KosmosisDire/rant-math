/* 2D regions: boxes, circles, capsules and polygons. Each answers the questions docs/shapes.md
 * lists, distance signed and negative inside, its corners and planes counterclockwise. */
#ifndef RANT_MATH_REGION_H
#define RANT_MATH_REGION_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API double             rant_aligned_box2d_distance(RantAlignedBox2D b, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_aligned_box2d_closest(RantAlignedBox2D b, RantDouble2 point);
RANT_MATH_API int                rant_aligned_box2d_contains(RantAlignedBox2D b, RantDouble2 point);
RANT_MATH_API int                rant_aligned_box2d_raycast(RantAlignedBox2D b, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantDouble2        rant_aligned_box2d_centroid(RantAlignedBox2D b);
RANT_MATH_API double             rant_aligned_box2d_area(RantAlignedBox2D b);
RANT_MATH_API double             rant_aligned_box2d_perimeter(RantAlignedBox2D b);
/* From min, counterclockwise. */
RANT_MATH_API void               rant_aligned_box2d_corners(RantAlignedBox2D b, RantDouble2 out[4]);
RANT_MATH_API void               rant_aligned_box2d_edges(RantAlignedBox2D b, RantSegment2D out[4]);
RANT_MATH_API void               rant_aligned_box2d_planes(RantAlignedBox2D b, RantPlane2D out[4]);
/* b given in pose's frame, returned in pose's parent, where it may no longer line up. */
RANT_MATH_API RantOrientedBox2D  rant_aligned_box2d_transform(RantAlignedBox2D b, RantPose2D pose);
/* The smallest box holding both. */
RANT_MATH_API RantAlignedBox2D   rant_aligned_box2d_merge(RantAlignedBox2D a, RantAlignedBox2D b);
/* 1 when they overlap or touch, with the box they share. out may be NULL. */
RANT_MATH_API int                rant_aligned_box2d_overlap(RantAlignedBox2D a, RantAlignedBox2D b, RantAlignedBox2D *out);
/* Each side moved out by margin, or in for a negative one, never past the center. */
RANT_MATH_API RantAlignedBox2D   rant_aligned_box2d_grow(RantAlignedBox2D b, double margin);
/* Intersection over union: the area both cover over the area either covers, 0 when they share none. */
RANT_MATH_API double             rant_aligned_box2d_iou(RantAlignedBox2D a, RantAlignedBox2D b);

RANT_MATH_API double             rant_oriented_box2d_distance(RantOrientedBox2D b, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_oriented_box2d_closest(RantOrientedBox2D b, RantDouble2 point);
RANT_MATH_API int                rant_oriented_box2d_contains(RantOrientedBox2D b, RantDouble2 point);
RANT_MATH_API int                rant_oriented_box2d_raycast(RantOrientedBox2D b, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantAlignedBox2D   rant_oriented_box2d_bounds(RantOrientedBox2D b);
RANT_MATH_API double             rant_oriented_box2d_area(RantOrientedBox2D b);
RANT_MATH_API double             rant_oriented_box2d_perimeter(RantOrientedBox2D b);
/* From the corner at -x -y of its own frame, counterclockwise. */
RANT_MATH_API void               rant_oriented_box2d_corners(RantOrientedBox2D b, RantDouble2 out[4]);
RANT_MATH_API void               rant_oriented_box2d_edges(RantOrientedBox2D b, RantSegment2D out[4]);
RANT_MATH_API void               rant_oriented_box2d_planes(RantOrientedBox2D b, RantPlane2D out[4]);
RANT_MATH_API RantOrientedBox2D  rant_oriented_box2d_transform(RantOrientedBox2D b, RantPose2D pose);
RANT_MATH_API RantOrientedBox2D  rant_oriented_box2d_from_aligned_box2d(RantAlignedBox2D b);
RANT_MATH_API double             rant_oriented_box2d_iou(RantOrientedBox2D a, RantOrientedBox2D b);

RANT_MATH_API double             rant_circle_distance(RantCircle c, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_circle_closest(RantCircle c, RantDouble2 point);
RANT_MATH_API int                rant_circle_contains(RantCircle c, RantDouble2 point);
RANT_MATH_API int                rant_circle_raycast(RantCircle c, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantAlignedBox2D   rant_circle_bounds(RantCircle c);
RANT_MATH_API double             rant_circle_area(RantCircle c);
RANT_MATH_API double             rant_circle_perimeter(RantCircle c);
RANT_MATH_API RantCircle         rant_circle_transform(RantCircle c, RantPose2D pose);

RANT_MATH_API double             rant_capsule2d_distance(RantCapsule2D c, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_capsule2d_closest(RantCapsule2D c, RantDouble2 point);
RANT_MATH_API int                rant_capsule2d_contains(RantCapsule2D c, RantDouble2 point);
RANT_MATH_API int                rant_capsule2d_raycast(RantCapsule2D c, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantAlignedBox2D   rant_capsule2d_bounds(RantCapsule2D c);
RANT_MATH_API double             rant_capsule2d_area(RantCapsule2D c);
RANT_MATH_API double             rant_capsule2d_perimeter(RantCapsule2D c);
RANT_MATH_API RantCapsule2D      rant_capsule2d_transform(RantCapsule2D c, RantPose2D pose);

/* A polygon may wind either way and must not cross itself. One with no points is outside
 * everywhere: its distance is HUGE_VAL and its bounds are empty, min above max. */
RANT_MATH_API double             rant_polygon2d_distance(RantPolygon2DView p, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_polygon2d_closest(RantPolygon2DView p, RantDouble2 point);
RANT_MATH_API int                rant_polygon2d_contains(RantPolygon2DView p, RantDouble2 point);
RANT_MATH_API int                rant_polygon2d_raycast(RantPolygon2DView p, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantAlignedBox2D   rant_polygon2d_bounds(RantPolygon2DView p);
RANT_MATH_API RantDouble2        rant_polygon2d_centroid(RantPolygon2DView p);
RANT_MATH_API double             rant_polygon2d_area(RantPolygon2DView p);
/* Positive when the corners run counterclockwise. */
RANT_MATH_API double             rant_polygon2d_signed_area(RantPolygon2DView p);
RANT_MATH_API double             rant_polygon2d_perimeter(RantPolygon2DView p);
/* 1 when every corner turns the same way and the edges go around once. */
RANT_MATH_API int                rant_polygon2d_is_convex(RantPolygon2DView p);
/* One per edge of nonzero length, in the corners' order, by the count rule. */
RANT_MATH_API RantMathResult     rant_polygon2d_edges(RantPolygon2DView p, RantSegment2D *out, uint32_t *count);
RANT_MATH_API RantMathResult     rant_polygon2d_planes(RantPolygon2DView p, RantPlane2D *out, uint32_t *count);
/* Triangles over the polygon as three indices into its points each, counterclockwise, the
 * Mesh layout. *count counts triangles: a polygon of n corners makes n - 2. */
RANT_MATH_API RantMathResult     rant_polygon2d_triangulate(RantPolygon2DView p, uint32_t *triangles, uint32_t *count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_REGION_H */
