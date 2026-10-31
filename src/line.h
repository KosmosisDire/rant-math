/* Planes, segments and rays in 3D and 2D, and where they meet. A Plane2D is a line. A normal or
 * direction need not be unit length, and every one returned is. */
#ifndef RANT_MATH_LINE_H
#define RANT_MATH_LINE_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The plane through three points, its normal by the right hand rule from a to b to c. */
RANT_MATH_API RantMathResult rant_plane_from_points(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantPlane *out);
/* Positive on the side the normal points to. A plane is the boundary of the half space
 * behind it, so the distance is signed as a solid's is. */
RANT_MATH_API double      rant_plane_distance(RantPlane p, RantDouble3 point);
RANT_MATH_API RantDouble3 rant_plane_closest(RantPlane p, RantDouble3 point);
/* 1 when the ray meets the plane from either side. hit may be NULL. */
RANT_MATH_API int         rant_plane_raycast(RantPlane p, RantRay ray, RantRayHit *hit);
/* p given in pose's frame, returned in pose's parent. */
RANT_MATH_API RantPlane   rant_plane_transform(RantPlane p, RantPose pose);
/* The line where two planes meet, as a ray along a's normal cross b's from the point of the
 * line nearest the frame's origin. */
RANT_MATH_API RantMathResult rant_plane_intersect(RantPlane a, RantPlane b, RantRay *out);
/* The one point where three planes meet. */
RANT_MATH_API RantMathResult rant_plane_intersect3(RantPlane a, RantPlane b, RantPlane c, RantDouble3 *out);

RANT_MATH_API double      rant_segment_length(RantSegment s);
RANT_MATH_API RantDouble3 rant_segment_closest(RantSegment s, RantDouble3 point);
RANT_MATH_API double      rant_segment_distance(RantSegment s, RantDouble3 point);
/* The shortest segment joining the two: its a lies on a and its b on b. */
RANT_MATH_API RantSegment rant_segment_closest_segment(RantSegment a, RantSegment b);
RANT_MATH_API RantSegment rant_segment_transform(RantSegment s, RantPose pose);
RANT_MATH_API RantAlignedBox rant_segment_bounds(RantSegment s);

/* The point distance along the ray from its origin. */
RANT_MATH_API RantDouble3 rant_ray_at(RantRay ray, double distance);
RANT_MATH_API RantDouble3 rant_ray_closest(RantRay ray, RantDouble3 point);
RANT_MATH_API double      rant_ray_distance(RantRay ray, RantDouble3 point);
/* The shortest segment joining the two rays, such as where two cameras' rays to one target
 * pass nearest: its middle is the best guess at the target. */
RANT_MATH_API RantSegment rant_ray_closest_ray(RantRay a, RantRay b);
RANT_MATH_API RantRay     rant_ray_transform(RantRay ray, RantPose pose);

/* The line through a and b, its normal on the right going from a to b, which faces out of a
 * counterclockwise polygon. */
RANT_MATH_API RantMathResult rant_plane2d_from_points(RantDouble2 a, RantDouble2 b, RantPlane2D *out);
RANT_MATH_API double      rant_plane2d_distance(RantPlane2D p, RantDouble2 point);
RANT_MATH_API RantDouble2 rant_plane2d_closest(RantPlane2D p, RantDouble2 point);
RANT_MATH_API int         rant_plane2d_raycast(RantPlane2D p, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantPlane2D rant_plane2d_transform(RantPlane2D p, RantPose2D pose);
/* The point where two lines cross. */
RANT_MATH_API RantMathResult rant_plane2d_intersect(RantPlane2D a, RantPlane2D b, RantDouble2 *out);

RANT_MATH_API double        rant_segment2d_length(RantSegment2D s);
RANT_MATH_API RantDouble2   rant_segment2d_closest(RantSegment2D s, RantDouble2 point);
RANT_MATH_API double        rant_segment2d_distance(RantSegment2D s, RantDouble2 point);
RANT_MATH_API RantSegment2D rant_segment2d_closest_segment(RantSegment2D a, RantSegment2D b);
/* 1 when a and b cross or touch, with point where. Segments that overlap along one line
 * give the shared point nearest a.a. point may be NULL. */
RANT_MATH_API int           rant_segment2d_intersect(RantSegment2D a, RantSegment2D b, RantDouble2 *point);
/* A segment has no outside, so the hit normal faces back toward the ray. */
RANT_MATH_API int           rant_segment2d_raycast(RantSegment2D s, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantSegment2D rant_segment2d_transform(RantSegment2D s, RantPose2D pose);
RANT_MATH_API RantAlignedBox2D rant_segment2d_bounds(RantSegment2D s);

RANT_MATH_API RantDouble2   rant_ray2d_at(RantRay2D ray, double distance);
RANT_MATH_API RantDouble2   rant_ray2d_closest(RantRay2D ray, RantDouble2 point);
RANT_MATH_API double        rant_ray2d_distance(RantRay2D ray, RantDouble2 point);
RANT_MATH_API RantRay2D     rant_ray2d_transform(RantRay2D ray, RantPose2D pose);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_LINE_H */
