/* Any shape behind one type, made by its rant_shape_from_ call, for work that mixes kinds. A
 * shape made from a view points at the caller's data. */
#ifndef RANT_MATH_SHAPE_H
#define RANT_MATH_SHAPE_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RANT_SHAPE_ALIGNED_BOX, RANT_SHAPE_ORIENTED_BOX, RANT_SHAPE_SPHERE, RANT_SHAPE_CAPSULE,
    RANT_SHAPE_CYLINDER, RANT_SHAPE_CONE, RANT_SHAPE_PRISM, RANT_SHAPE_SEGMENT,
    RANT_SHAPE_POLYLINE, RANT_SHAPE_POLYGON, RANT_SHAPE_MESH
} RantShapeKind;

typedef struct {
    RantShapeKind kind;
    union {
        RantAlignedBox aligned_box; RantOrientedBox oriented_box; RantSphere sphere; RantCapsule capsule;
        RantCylinder cylinder; RantCone cone; RantPrismView prism; RantSegment segment;
        RantPolylineView polyline; RantPolygonView polygon; RantMeshView mesh;
    } as;
} RantShape;

RANT_MATH_API RantShape rant_shape_from_aligned_box(RantAlignedBox b);
RANT_MATH_API RantShape rant_shape_from_oriented_box(RantOrientedBox b);
RANT_MATH_API RantShape rant_shape_from_sphere(RantSphere s);
RANT_MATH_API RantShape rant_shape_from_capsule(RantCapsule c);
RANT_MATH_API RantShape rant_shape_from_cylinder(RantCylinder c);
RANT_MATH_API RantShape rant_shape_from_cone(RantCone c);
RANT_MATH_API RantShape rant_shape_from_prism(RantPrismView p);
RANT_MATH_API RantShape rant_shape_from_segment(RantSegment s);
RANT_MATH_API RantShape rant_shape_from_polyline(RantPolylineView l);
RANT_MATH_API RantShape rant_shape_from_polygon(RantPolygonView p);
RANT_MATH_API RantShape rant_shape_from_mesh(RantMeshView m);

/* Signed for a solid, never negative for a curve or surface. A curve's closest normal points to
 * the point, zero on the curve, and a curve contains nothing and meets no ray. */
RANT_MATH_API double           rant_shape_distance(const RantShape *s, RantDouble3 point);
RANT_MATH_API RantSurfacePoint rant_shape_closest(const RantShape *s, RantDouble3 point);
RANT_MATH_API int              rant_shape_contains(const RantShape *s, RantDouble3 point);
RANT_MATH_API int              rant_shape_raycast(const RantShape *s, RantRay ray, RantRayHit *hit);
RANT_MATH_API RantAlignedBox   rant_shape_bounds(const RantShape *s);
/* rant_shape_distance at each of count points. */
RANT_MATH_API void             rant_shape_distances(const RantShape *s, const RantDouble3 *points, double *out, uint32_t count);

/* How far apart two shapes are by GJK, 0 when they touch, a solid filled and a mesh or polygon
 * hollow. between, when not NULL, gets the nearest point of each, a on its a. */
RANT_MATH_API RantMathResult   rant_shape_gap(const RantShape *a, const RantShape *b, double *gap, RantSegment *between);
/* 1 when they touch or overlap. */
RANT_MATH_API int              rant_shape_overlaps(const RantShape *a, const RantShape *b);

/* count points at random, evenly through a solid or over any shape's surface (along a curve),
 * the same for the same seed. Only a solid or a closed mesh has an inside to sample. */
RANT_MATH_API RantMathResult   rant_shape_sample_inside(const RantShape *s, uint64_t seed, RantDouble3 *out, uint32_t count);
RANT_MATH_API RantMathResult   rant_shape_sample_surface(const RantShape *s, uint64_t seed, RantDouble3 *out, uint32_t count);
/* The surface as triangles counterclockwise seen from outside, closed for a solid. A round part
 * takes segments round it, 0 for 32. Each count holds its room and gets its need, triangles in threes. */
RANT_MATH_API RantMathResult   rant_shape_mesh(const RantShape *s, uint32_t segments, RantDouble3 *vertices,
                                               uint32_t *vertex_count, uint32_t *triangles, uint32_t *triangle_count);

typedef enum {
    RANT_SHAPE2D_ALIGNED_BOX, RANT_SHAPE2D_ORIENTED_BOX, RANT_SHAPE2D_CIRCLE, RANT_SHAPE2D_CAPSULE,
    RANT_SHAPE2D_POLYGON, RANT_SHAPE2D_SEGMENT, RANT_SHAPE2D_POLYLINE
} RantShape2DKind;

typedef struct {
    RantShape2DKind kind;
    union {
        RantAlignedBox2D aligned_box; RantOrientedBox2D oriented_box; RantCircle circle; RantCapsule2D capsule;
        RantPolygon2DView polygon; RantSegment2D segment; RantPolyline2DView polyline;
    } as;
} RantShape2D;

RANT_MATH_API RantShape2D rant_shape2d_from_aligned_box2d(RantAlignedBox2D b);
RANT_MATH_API RantShape2D rant_shape2d_from_oriented_box2d(RantOrientedBox2D b);
RANT_MATH_API RantShape2D rant_shape2d_from_circle(RantCircle c);
RANT_MATH_API RantShape2D rant_shape2d_from_capsule2d(RantCapsule2D c);
RANT_MATH_API RantShape2D rant_shape2d_from_polygon2d(RantPolygon2DView p);
RANT_MATH_API RantShape2D rant_shape2d_from_segment2d(RantSegment2D s);
RANT_MATH_API RantShape2D rant_shape2d_from_polyline2d(RantPolyline2DView l);

RANT_MATH_API double             rant_shape2d_distance(const RantShape2D *s, RantDouble2 point);
RANT_MATH_API RantSurfacePoint2D rant_shape2d_closest(const RantShape2D *s, RantDouble2 point);
RANT_MATH_API int                rant_shape2d_contains(const RantShape2D *s, RantDouble2 point);
/* A ray meets a segment or a polyline from either side, the normal facing back toward it. */
RANT_MATH_API int                rant_shape2d_raycast(const RantShape2D *s, RantRay2D ray, RantRayHit2D *hit);
RANT_MATH_API RantAlignedBox2D   rant_shape2d_bounds(const RantShape2D *s);
RANT_MATH_API void               rant_shape2d_distances(const RantShape2D *s, const RantDouble2 *points, double *out, uint32_t count);
RANT_MATH_API RantMathResult     rant_shape2d_gap(const RantShape2D *a, const RantShape2D *b, double *gap, RantSegment2D *between);
RANT_MATH_API int                rant_shape2d_overlaps(const RantShape2D *a, const RantShape2D *b);
RANT_MATH_API RantMathResult     rant_shape2d_sample_inside(const RantShape2D *s, uint64_t seed, RantDouble2 *out, uint32_t count);
RANT_MATH_API RantMathResult     rant_shape2d_sample_surface(const RantShape2D *s, uint64_t seed, RantDouble2 *out, uint32_t count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_SHAPE_H */
