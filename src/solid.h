/* 3D solids: boxes, spheres, capsules, cylinders, cones and prisms. They answer what the 2D
 * regions do, in the corner, edge and plane orders docs/shapes.md gives. */
#ifndef RANT_MATH_SOLID_H
#define RANT_MATH_SOLID_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API double           rant_aligned_box_distance(RantAlignedBox b, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_aligned_box_closest(RantAlignedBox b, RantDouble3 point);
RANT_MATH_API int              rant_aligned_box_contains(RantAlignedBox b, RantDouble3 point);
RANT_MATH_API int              rant_aligned_box_raycast(RantAlignedBox b, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantDouble3      rant_aligned_box_centroid(RantAlignedBox b);
RANT_MATH_API double           rant_aligned_box_volume(RantAlignedBox b);
RANT_MATH_API double           rant_aligned_box_area(RantAlignedBox b);
RANT_MATH_API RantInertia      rant_aligned_box_inertia(RantAlignedBox b, double mass);
/* The bottom from min, counterclockwise, then the top. */
RANT_MATH_API void             rant_aligned_box_corners(RantAlignedBox b, RantDouble3 out[8]);
RANT_MATH_API void             rant_aligned_box_edges(RantAlignedBox b, RantSegment out[12]);
RANT_MATH_API void             rant_aligned_box_planes(RantAlignedBox b, RantPlane out[6]);
/* b given in pose's frame, returned in pose's parent, where it may no longer line up. */
RANT_MATH_API RantOrientedBox  rant_aligned_box_transform(RantAlignedBox b, RantPose pose);
/* The smallest box holding both. */
RANT_MATH_API RantAlignedBox   rant_aligned_box_merge(RantAlignedBox a, RantAlignedBox b);
/* 1 when they overlap or touch, with the box they share. out may be NULL. */
RANT_MATH_API int              rant_aligned_box_overlap(RantAlignedBox a, RantAlignedBox b, RantAlignedBox *out);
/* Each side moved out by margin, or in for a negative one, never past the center. */
RANT_MATH_API RantAlignedBox   rant_aligned_box_grow(RantAlignedBox b, double margin);
/* Intersection over union: the volume both hold over the volume either holds, 0 when they share none. */
RANT_MATH_API double           rant_aligned_box_iou(RantAlignedBox a, RantAlignedBox b);

RANT_MATH_API double           rant_oriented_box_distance(RantOrientedBox b, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_oriented_box_closest(RantOrientedBox b, RantDouble3 point);
RANT_MATH_API int              rant_oriented_box_contains(RantOrientedBox b, RantDouble3 point);
RANT_MATH_API int              rant_oriented_box_raycast(RantOrientedBox b, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_oriented_box_bounds(RantOrientedBox b);
RANT_MATH_API double           rant_oriented_box_volume(RantOrientedBox b);
RANT_MATH_API double           rant_oriented_box_area(RantOrientedBox b);
RANT_MATH_API RantInertia      rant_oriented_box_inertia(RantOrientedBox b, double mass);
/* From the corner at -x -y -z of its own frame, as for an aligned box. */
RANT_MATH_API void             rant_oriented_box_corners(RantOrientedBox b, RantDouble3 out[8]);
RANT_MATH_API void             rant_oriented_box_edges(RantOrientedBox b, RantSegment out[12]);
RANT_MATH_API void             rant_oriented_box_planes(RantOrientedBox b, RantPlane out[6]);
RANT_MATH_API RantOrientedBox  rant_oriented_box_transform(RantOrientedBox b, RantPose pose);
RANT_MATH_API RantOrientedBox  rant_oriented_box_from_aligned_box(RantAlignedBox b);
RANT_MATH_API double           rant_oriented_box_iou(RantOrientedBox a, RantOrientedBox b);

RANT_MATH_API double           rant_sphere_distance(RantSphere s, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_sphere_closest(RantSphere s, RantDouble3 point);
RANT_MATH_API int              rant_sphere_contains(RantSphere s, RantDouble3 point);
RANT_MATH_API int              rant_sphere_raycast(RantSphere s, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_sphere_bounds(RantSphere s);
RANT_MATH_API double           rant_sphere_volume(RantSphere s);
RANT_MATH_API double           rant_sphere_area(RantSphere s);
RANT_MATH_API RantInertia      rant_sphere_inertia(RantSphere s, double mass);
RANT_MATH_API RantSphere       rant_sphere_transform(RantSphere s, RantPose pose);

RANT_MATH_API double           rant_capsule_distance(RantCapsule c, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_capsule_closest(RantCapsule c, RantDouble3 point);
RANT_MATH_API int              rant_capsule_contains(RantCapsule c, RantDouble3 point);
RANT_MATH_API int              rant_capsule_raycast(RantCapsule c, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_capsule_bounds(RantCapsule c);
RANT_MATH_API RantDouble3      rant_capsule_centroid(RantCapsule c);
RANT_MATH_API double           rant_capsule_volume(RantCapsule c);
RANT_MATH_API double           rant_capsule_area(RantCapsule c);
RANT_MATH_API RantInertia      rant_capsule_inertia(RantCapsule c, double mass);
RANT_MATH_API RantCapsule      rant_capsule_transform(RantCapsule c, RantPose pose);

RANT_MATH_API double           rant_cylinder_distance(RantCylinder c, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_cylinder_closest(RantCylinder c, RantDouble3 point);
RANT_MATH_API int              rant_cylinder_contains(RantCylinder c, RantDouble3 point);
RANT_MATH_API int              rant_cylinder_raycast(RantCylinder c, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_cylinder_bounds(RantCylinder c);
RANT_MATH_API RantDouble3      rant_cylinder_centroid(RantCylinder c);
RANT_MATH_API double           rant_cylinder_volume(RantCylinder c);
RANT_MATH_API double           rant_cylinder_area(RantCylinder c);
RANT_MATH_API RantInertia      rant_cylinder_inertia(RantCylinder c, double mass);
RANT_MATH_API RantCylinder     rant_cylinder_transform(RantCylinder c, RantPose pose);

RANT_MATH_API double           rant_cone_distance(RantCone c, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_cone_closest(RantCone c, RantDouble3 point);
RANT_MATH_API int              rant_cone_contains(RantCone c, RantDouble3 point);
RANT_MATH_API int              rant_cone_raycast(RantCone c, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_cone_bounds(RantCone c);
/* A quarter of the way from the base to the tip. */
RANT_MATH_API RantDouble3      rant_cone_centroid(RantCone c);
RANT_MATH_API double           rant_cone_volume(RantCone c);
RANT_MATH_API double           rant_cone_area(RantCone c);
RANT_MATH_API RantInertia      rant_cone_inertia(RantCone c, double mass);
RANT_MATH_API RantCone         rant_cone_transform(RantCone c, RantPose pose);

/* The base may wind either way and must not cross itself. A height below 0 rises along -z. */
RANT_MATH_API double           rant_prism_distance(RantPrismView p, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_prism_closest(RantPrismView p, RantDouble3 point);
RANT_MATH_API int              rant_prism_contains(RantPrismView p, RantDouble3 point);
RANT_MATH_API int              rant_prism_raycast(RantPrismView p, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_prism_bounds(RantPrismView p);
RANT_MATH_API RantDouble3      rant_prism_centroid(RantPrismView p);
RANT_MATH_API double           rant_prism_volume(RantPrismView p);
RANT_MATH_API double           rant_prism_area(RantPrismView p);
RANT_MATH_API RantInertia      rant_prism_inertia(RantPrismView p, double mass);
/* A base of n corners makes 2n corners, 3n edges and n + 2 planes, by the count rule. */
RANT_MATH_API RantMathResult   rant_prism_corners(RantPrismView p, RantDouble3 *out, uint32_t *count);
RANT_MATH_API RantMathResult   rant_prism_edges(RantPrismView p, RantSegment *out, uint32_t *count);
RANT_MATH_API RantMathResult   rant_prism_planes(RantPrismView p, RantPlane *out, uint32_t *count);
/* The same base, its pose moved into pose's parent. */
RANT_MATH_API RantPrismView    rant_prism_transform(RantPrismView p, RantPose pose);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_SOLID_H */
