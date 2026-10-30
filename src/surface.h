/* Surfaces: flat polygons in 3D and triangle meshes. A surface has no inside, so its distance is
 * never negative, but a closed mesh tells inside from out with contains. */
#ifndef RANT_MATH_SURFACE_H
#define RANT_MATH_SURFACE_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A polygon whose corners are not quite flat counts as its shadow on its plane. */
RANT_MATH_API RantMathResult   rant_polygon_plane(RantPolygonView p, RantPlane *out);
/* The polygon in its own plane, for the Polygon2D calls: frame's xy plane holds it, at its first
 * corner with +x toward the next, and out gets its p.count corners in that frame. */
RANT_MATH_API RantMathResult   rant_polygon_flatten(RantPolygonView p, RantPose *frame, RantDouble2 *out);
RANT_MATH_API double           rant_polygon_distance(RantPolygonView p, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_polygon_closest(RantPolygonView p, RantDouble3 point);
/* Either side of the polygon counts. */
RANT_MATH_API int              rant_polygon_raycast(RantPolygonView p, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_polygon_bounds(RantPolygonView p);
RANT_MATH_API RantDouble3      rant_polygon_centroid(RantPolygonView p);
RANT_MATH_API double           rant_polygon_area(RantPolygonView p);
RANT_MATH_API double           rant_polygon_perimeter(RantPolygonView p);
RANT_MATH_API RantMathResult   rant_polygon_edges(RantPolygonView p, RantSegment *out, uint32_t *count);

/* Fails when an index runs past the vertices. Every other mesh call skips such a triangle. */
RANT_MATH_API RantMathResult   rant_mesh_check(RantMeshView m);
RANT_MATH_API double           rant_mesh_distance(RantMeshView m, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_mesh_closest(RantMeshView m, RantDouble3 point);
/* 1 when the triangles of a closed mesh wind round the point, which puts it inside. */
RANT_MATH_API int              rant_mesh_contains(RantMeshView m, RantDouble3 point);
/* Either side of a triangle counts. */
RANT_MATH_API int              rant_mesh_raycast(RantMeshView m, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_mesh_bounds(RantMeshView m);
RANT_MATH_API double           rant_mesh_area(RantMeshView m);
/* What a closed mesh holds, from the signed solids its triangles make with one point. The
 * centroid is of that volume, or of the surface when the mesh holds none. */
RANT_MATH_API double           rant_mesh_volume(RantMeshView m);
RANT_MATH_API RantDouble3      rant_mesh_centroid(RantMeshView m);
RANT_MATH_API RantInertia      rant_mesh_inertia(RantMeshView m, double mass);
/* The unit normal at each vertex, the sum of its triangles' normals by their area, NaN for a
 * vertex in none. normals holds vertex_count. */
RANT_MATH_API RantMathResult   rant_mesh_normals(RantMeshView m, RantDouble3 *normals);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_SURFACE_H */
