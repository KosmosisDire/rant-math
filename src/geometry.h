/* What the shape parts share: the points they answer with, and views of the shapes whose
 * standard type holds a list, which have no C mirror. A view only points at its data. */
#ifndef RANT_MATH_GEOMETRY_H
#define RANT_MATH_GEOMETRY_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The nearest point of a shape's surface and the outward unit normal there. For a solid
 * the normal is the way the signed distance grows, so the query is point + distance * normal. */
typedef struct { RantDouble3 point, normal; } RantSurfacePoint;
typedef struct { RantDouble2 point, normal; } RantSurfacePoint2D;

/* Where a ray first meets a shape at or beyond its origin, the shape's outward normal there,
 * and how far along the ray that is. */
typedef struct { RantDouble3 point, normal; double distance; } RantRayHit;
typedef struct { RantDouble2 point, normal; double distance; } RantRayHit2D;

/* A point on a curve, the unit direction the curve runs there, and how far along it lies. */
typedef struct { RantDouble3 point, tangent; double distance; } RantCurvePoint;
typedef struct { RantDouble2 point, tangent; double distance; } RantCurvePoint2D;

/* A polygon's corners in order, the last joined back to the first. A repeated closing
 * point is allowed. */
typedef struct { const RantDouble3 *points; uint32_t count; } RantPolygonView;
typedef struct { const RantDouble2 *points; uint32_t count; } RantPolygon2DView;
typedef struct { const RantDouble3 *points; uint32_t count; } RantPolylineView;
typedef struct { const RantDouble2 *points; uint32_t count; } RantPolyline2DView;
typedef struct { const RantPose *poses; uint32_t count; } RantPathView;
typedef struct { const RantPose2D *poses; uint32_t count; } RantPath2DView;
/* base lies in the xy plane of pose and rises height along its +z. */
typedef struct { RantPose pose; RantPolygon2DView base; double height; } RantPrismView;
/* Three indices into vertices per triangle, counterclockwise seen from outside. */
typedef struct {
    const RantDouble3 *vertices; uint32_t vertex_count;
    const uint32_t *triangles; uint32_t triangle_count;
} RantMeshView;

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_GEOMETRY_H */
