#include "internal.h"

RantShape rant_shape_from_aligned_box(RantAlignedBox b){ RantShape s; s.kind = RANT_SHAPE_ALIGNED_BOX; s.as.aligned_box = b; return s; }
RantShape rant_shape_from_oriented_box(RantOrientedBox b){ RantShape s; s.kind = RANT_SHAPE_ORIENTED_BOX; s.as.oriented_box = b; return s; }
RantShape rant_shape_from_sphere(RantSphere x){ RantShape s; s.kind = RANT_SHAPE_SPHERE; s.as.sphere = x; return s; }
RantShape rant_shape_from_capsule(RantCapsule c){ RantShape s; s.kind = RANT_SHAPE_CAPSULE; s.as.capsule = c; return s; }
RantShape rant_shape_from_cylinder(RantCylinder c){ RantShape s; s.kind = RANT_SHAPE_CYLINDER; s.as.cylinder = c; return s; }
RantShape rant_shape_from_cone(RantCone c){ RantShape s; s.kind = RANT_SHAPE_CONE; s.as.cone = c; return s; }
RantShape rant_shape_from_prism(RantPrismView p){ RantShape s; s.kind = RANT_SHAPE_PRISM; s.as.prism = p; return s; }
RantShape rant_shape_from_segment(RantSegment x){ RantShape s; s.kind = RANT_SHAPE_SEGMENT; s.as.segment = x; return s; }
RantShape rant_shape_from_polyline(RantPolylineView l){ RantShape s; s.kind = RANT_SHAPE_POLYLINE; s.as.polyline = l; return s; }
RantShape rant_shape_from_polygon(RantPolygonView p){ RantShape s; s.kind = RANT_SHAPE_POLYGON; s.as.polygon = p; return s; }
RantShape rant_shape_from_mesh(RantMeshView m){ RantShape s; s.kind = RANT_SHAPE_MESH; s.as.mesh = m; return s; }

/* The closest point of a curve, with the normal from it toward the point. */
static RantSurfacePoint toward(RantDouble3 on, RantDouble3 point){
    RantSurfacePoint r;
    r.point = on;
    r.normal = rant_double3_normalize(rant_double3_subtract(point, on));
    return r;
}

double rant_shape_distance(const RantShape *s, RantDouble3 point){
    if (!s) return HUGE_VAL;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:  return rant_aligned_box_distance(s->as.aligned_box, point);
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_distance(s->as.oriented_box, point);
    case RANT_SHAPE_SPHERE:       return rant_sphere_distance(s->as.sphere, point);
    case RANT_SHAPE_CAPSULE:      return rant_capsule_distance(s->as.capsule, point);
    case RANT_SHAPE_CYLINDER:     return rant_cylinder_distance(s->as.cylinder, point);
    case RANT_SHAPE_CONE:         return rant_cone_distance(s->as.cone, point);
    case RANT_SHAPE_PRISM:        return rant_prism_distance(s->as.prism, point);
    case RANT_SHAPE_SEGMENT:      return rant_segment_distance(s->as.segment, point);
    case RANT_SHAPE_POLYLINE:     return rant_polyline_distance(s->as.polyline, point);
    case RANT_SHAPE_POLYGON:      return rant_polygon_distance(s->as.polygon, point);
    case RANT_SHAPE_MESH:         return rant_mesh_distance(s->as.mesh, point);
    }
    return HUGE_VAL;
}

RantSurfacePoint rant_shape_closest(const RantShape *s, RantDouble3 point){
    RantSurfacePoint none;
    none.point = point;
    none.normal = rant_double3(0, 0, 0);
    if (!s) return none;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:  return rant_aligned_box_closest(s->as.aligned_box, point);
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_closest(s->as.oriented_box, point);
    case RANT_SHAPE_SPHERE:       return rant_sphere_closest(s->as.sphere, point);
    case RANT_SHAPE_CAPSULE:      return rant_capsule_closest(s->as.capsule, point);
    case RANT_SHAPE_CYLINDER:     return rant_cylinder_closest(s->as.cylinder, point);
    case RANT_SHAPE_CONE:         return rant_cone_closest(s->as.cone, point);
    case RANT_SHAPE_PRISM:        return rant_prism_closest(s->as.prism, point);
    case RANT_SHAPE_SEGMENT:      return toward(rant_segment_closest(s->as.segment, point), point);
    case RANT_SHAPE_POLYLINE:
        return s->as.polyline.points && s->as.polyline.count ? toward(rant_polyline_closest(s->as.polyline, point).point, point) : none;
    case RANT_SHAPE_POLYGON:      return rant_polygon_closest(s->as.polygon, point);
    case RANT_SHAPE_MESH:         return rant_mesh_closest(s->as.mesh, point);
    }
    return none;
}

int rant_shape_contains(const RantShape *s, RantDouble3 point){
    if (!s) return 0;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:  return rant_aligned_box_contains(s->as.aligned_box, point);
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_contains(s->as.oriented_box, point);
    case RANT_SHAPE_SPHERE:       return rant_sphere_contains(s->as.sphere, point);
    case RANT_SHAPE_CAPSULE:      return rant_capsule_contains(s->as.capsule, point);
    case RANT_SHAPE_CYLINDER:     return rant_cylinder_contains(s->as.cylinder, point);
    case RANT_SHAPE_CONE:         return rant_cone_contains(s->as.cone, point);
    case RANT_SHAPE_PRISM:        return rant_prism_contains(s->as.prism, point);
    case RANT_SHAPE_MESH:         return rant_mesh_contains(s->as.mesh, point);
    case RANT_SHAPE_SEGMENT: case RANT_SHAPE_POLYLINE: case RANT_SHAPE_POLYGON: return 0;
    }
    return 0;
}

int rant_shape_raycast(const RantShape *s, RantRay ray, RantRayHit *hit){
    if (!s) return 0;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:  return rant_aligned_box_raycast(s->as.aligned_box, ray, hit);
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_raycast(s->as.oriented_box, ray, hit);
    case RANT_SHAPE_SPHERE:       return rant_sphere_raycast(s->as.sphere, ray, hit);
    case RANT_SHAPE_CAPSULE:      return rant_capsule_raycast(s->as.capsule, ray, hit);
    case RANT_SHAPE_CYLINDER:     return rant_cylinder_raycast(s->as.cylinder, ray, hit);
    case RANT_SHAPE_CONE:         return rant_cone_raycast(s->as.cone, ray, hit);
    case RANT_SHAPE_PRISM:        return rant_prism_raycast(s->as.prism, ray, hit);
    case RANT_SHAPE_POLYGON:      return rant_polygon_raycast(s->as.polygon, ray, hit);
    case RANT_SHAPE_MESH:         return rant_mesh_raycast(s->as.mesh, ray, hit);
    case RANT_SHAPE_SEGMENT: case RANT_SHAPE_POLYLINE: return 0;
    }
    return 0;
}

RantAlignedBox rant_shape_bounds(const RantShape *s){
    RantAlignedBox empty;
    empty.min = rant_double3(HUGE_VAL, HUGE_VAL, HUGE_VAL);
    empty.max = rant_double3(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL);
    if (!s) return empty;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:  return s->as.aligned_box;
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_bounds(s->as.oriented_box);
    case RANT_SHAPE_SPHERE:       return rant_sphere_bounds(s->as.sphere);
    case RANT_SHAPE_CAPSULE:      return rant_capsule_bounds(s->as.capsule);
    case RANT_SHAPE_CYLINDER:     return rant_cylinder_bounds(s->as.cylinder);
    case RANT_SHAPE_CONE:         return rant_cone_bounds(s->as.cone);
    case RANT_SHAPE_PRISM:        return rant_prism_bounds(s->as.prism);
    case RANT_SHAPE_SEGMENT:      return rant_segment_bounds(s->as.segment);
    case RANT_SHAPE_POLYLINE:     return rant_polyline_bounds(s->as.polyline);
    case RANT_SHAPE_POLYGON:      return rant_polygon_bounds(s->as.polygon);
    case RANT_SHAPE_MESH:         return rant_mesh_bounds(s->as.mesh);
    }
    return empty;
}

void rant_shape_distances(const RantShape *s, const RantDouble3 *points, double *out, uint32_t count){
    uint32_t i;
    if (!points || !out) return;
    for (i = 0; i < count; i++) out[i] = rant_shape_distance(s, points[i]);
}

/* A convex piece GJK can read: the hull of up to six points, a box, a cylinder or a cone, each
 * grown by margin, which is how a ball or a capsule enters as a point or a segment. */
typedef enum { I_POINTS, I_BOX, I_CYLINDER, I_CONE } i_PieceKind;

typedef struct {
    i_PieceKind kind;
    RantDouble3 p[6];
    int n;
    RantPose pose;
    RantDouble3 half;
    RantDouble3 a, b;
    double r, margin;
    RantAlignedBox bounds;
} i_Piece;

/* The core's farthest point along d. */
static RantDouble3 support(const i_Piece *s, RantDouble3 d){
    switch (s->kind){
    case I_BOX: {
        RantDouble3 l = rant_quaternion_rotate(rant_quaternion_inverse(s->pose.orientation), d);
        return rant_pose_apply(s->pose, rant_double3(l.x >= 0.0 ? s->half.x : -s->half.x, l.y >= 0.0 ? s->half.y : -s->half.y,
                                                     l.z >= 0.0 ? s->half.z : -s->half.z));
    }
    case I_CYLINDER: case I_CONE: {
        /* Along the axis the side part is only rounding, and any point of the cap will do. */
        RantDouble3 u = i_math_axis(s->a, s->b), dp = rant_double3_subtract(d, rant_double3_scale(u, rant_double3_dot(u, d)));
        double l = rant_double3_length(dp);
        RantDouble3 rim = l > 1e-9 * rant_double3_length(d) ? rant_double3_scale(dp, s->r / l) : rant_double3(0, 0, 0);
        if (s->kind == I_CONE){
            RantDouble3 base = rant_double3_add(s->a, rim);
            return rant_double3_dot(base, d) >= rant_double3_dot(s->b, d) ? base : s->b;
        }
        return rant_double3_add(rant_double3_dot(u, d) >= 0.0 ? s->b : s->a, rim);
    }
    case I_POINTS: default: {
        RantDouble3 best = s->p[0];
        double farther = rant_double3_dot(best, d);
        int i;
        for (i = 1; i < s->n; i++){
            double k = rant_double3_dot(s->p[i], d);
            if (k > farther){ farther = k; best = s->p[i]; }
        }
        return best;
    }
    }
}

static void piece_bounds(i_Piece *s){
    RantDouble3 lo, hi;
    double m = s->margin;
    lo = rant_double3(support(s, rant_double3(-1, 0, 0)).x, support(s, rant_double3(0, -1, 0)).y, support(s, rant_double3(0, 0, -1)).z);
    hi = rant_double3(support(s, rant_double3(1, 0, 0)).x, support(s, rant_double3(0, 1, 0)).y, support(s, rant_double3(0, 0, 1)).z);
    s->bounds.min = rant_double3_subtract(lo, rant_double3(m, m, m));
    s->bounds.max = rant_double3_add(hi, rant_double3(m, m, m));
}

static i_Piece points_piece(const RantDouble3 *p, int n, double margin){
    i_Piece s;
    int i;
    memset(&s, 0, sizeof s);
    s.kind = I_POINTS;
    s.n = n;
    for (i = 0; i < n; i++) s.p[i] = p[i];
    s.margin = margin;
    piece_bounds(&s);
    return s;
}

static i_Piece box_piece(RantPose pose, RantDouble3 half){
    i_Piece s;
    memset(&s, 0, sizeof s);
    s.kind = I_BOX;
    s.pose = pose;
    s.half = rant_double3(fabs(half.x), fabs(half.y), fabs(half.z));
    piece_bounds(&s);
    return s;
}

static i_Piece round_piece(i_PieceKind kind, RantDouble3 a, RantDouble3 b, double r){
    i_Piece s;
    memset(&s, 0, sizeof s);
    s.kind = kind;
    s.a = a; s.b = b; s.r = fabs(r);
    piece_bounds(&s);
    return s;
}

typedef struct { i_Piece *list; uint32_t count; i_Piece one; } i_Pieces;

static void pieces_free(i_Pieces *p){ if (p->list != &p->one) free(p->list); }

static RantMathResult pieces_alloc(i_Pieces *p, uint32_t n){
    p->count = 0;
    p->list = n <= 1 ? &p->one : (i_Piece *)malloc(sizeof(i_Piece) * n);
    return p->list ? RANT_MATH_OK : RANT_MATH_ERR_MEMORY;
}

/* Triangles over a 2D polygon, each lifted by place: z 0 and h of a prism's frame, or z 0 of a
 * polygon's own. A triangle of the prism is six corners, of a flat polygon three. */
static RantMathResult polygon_pieces(RantPolygon2DView base, RantPose frame, int prism, double lo, double hi, i_Pieces *out){
    uint32_t n = 0, *tri, t;
    RantMathResult r = rant_polygon2d_triangulate(base, NULL, &n);
    if (r != RANT_MATH_OK && r != RANT_MATH_ERR_SPACE) return r;
    if (!n) return RANT_MATH_ERR_ARGUMENT;
    tri = (uint32_t *)malloc(sizeof(uint32_t) * 3 * n);
    if (!tri) return RANT_MATH_ERR_MEMORY;
    if ((r = rant_polygon2d_triangulate(base, tri, &n)) != RANT_MATH_OK || (r = pieces_alloc(out, n)) != RANT_MATH_OK){ free(tri); return r; }
    for (t = 0; t < n; t++){
        RantDouble3 c[6];
        int k;
        for (k = 0; k < 3; k++){
            RantDouble2 v = base.points[tri[3 * t + k]];
            c[k] = rant_pose_apply(frame, rant_double3(v.x, v.y, lo));
            c[k + 3] = rant_pose_apply(frame, rant_double3(v.x, v.y, hi));
        }
        out->list[out->count++] = points_piece(c, prism ? 6 : 3, 0.0);
    }
    free(tri);
    return RANT_MATH_OK;
}

static RantMathResult line_pieces(const RantDouble3 *p, uint32_t n, i_Pieces *out){
    uint32_t i;
    RantMathResult r;
    if (!p || !n) return RANT_MATH_ERR_ARGUMENT;
    if ((r = pieces_alloc(out, n > 1 ? n - 1 : 1)) != RANT_MATH_OK) return r;
    if (n == 1) out->list[out->count++] = points_piece(p, 1, 0.0);
    for (i = 0; i + 1 < n; i++) out->list[out->count++] = points_piece(p + i, 2, 0.0);
    return RANT_MATH_OK;
}

static RantMathResult pieces_of(const RantShape *s, i_Pieces *out){
    RantPose at = rant_pose_identity();
    RantDouble3 two[2];
    RantMathResult r;
    out->list = &out->one;
    out->count = 1;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX:
        at.position = rant_aligned_box_centroid(s->as.aligned_box);
        out->one = box_piece(at, rant_double3_scale(rant_double3_subtract(s->as.aligned_box.max, s->as.aligned_box.min), 0.5));
        return RANT_MATH_OK;
    case RANT_SHAPE_ORIENTED_BOX:
        out->one = box_piece(s->as.oriented_box.pose, rant_double3_scale(s->as.oriented_box.size, 0.5));
        return RANT_MATH_OK;
    case RANT_SHAPE_SPHERE:
        out->one = points_piece(&s->as.sphere.center, 1, fabs(s->as.sphere.radius));
        return RANT_MATH_OK;
    case RANT_SHAPE_CAPSULE:
        two[0] = s->as.capsule.axis.a; two[1] = s->as.capsule.axis.b;
        out->one = points_piece(two, 2, fabs(s->as.capsule.radius));
        return RANT_MATH_OK;
    case RANT_SHAPE_CYLINDER:
        out->one = round_piece(I_CYLINDER, s->as.cylinder.axis.a, s->as.cylinder.axis.b, s->as.cylinder.radius);
        return RANT_MATH_OK;
    case RANT_SHAPE_CONE:
        out->one = round_piece(I_CONE, s->as.cone.base, s->as.cone.tip, s->as.cone.radius);
        return RANT_MATH_OK;
    case RANT_SHAPE_SEGMENT:
        two[0] = s->as.segment.a; two[1] = s->as.segment.b;
        out->one = points_piece(two, 2, 0.0);
        return RANT_MATH_OK;
    case RANT_SHAPE_PRISM:
        return polygon_pieces(s->as.prism.base, s->as.prism.pose, 1, i_math_min(0.0, s->as.prism.height),
                              i_math_max(0.0, s->as.prism.height), out);
    case RANT_SHAPE_POLYLINE:
        return line_pieces(s->as.polyline.points, s->as.polyline.count, out);
    case RANT_SHAPE_POLYGON: {
        RantPolygonView p = s->as.polygon;
        RantDouble2 *flat;
        RantPose frame;
        if (!p.points || !p.count) return RANT_MATH_ERR_ARGUMENT;
        flat = (RantDouble2 *)malloc(sizeof(RantDouble2) * p.count);
        if (!flat) return RANT_MATH_ERR_MEMORY;
        if (rant_polygon_flatten(p, &frame, flat) == RANT_MATH_OK){
            RantPolygon2DView v;
            v.points = flat; v.count = p.count;
            r = polygon_pieces(v, frame, 0, 0.0, 0.0, out);
        } else {
            /* with no plane the polygon is only its edges, the last back to the first */
            RantDouble3 *ring = (RantDouble3 *)malloc(sizeof(RantDouble3) * (p.count + 1));
            r = RANT_MATH_ERR_MEMORY;
            if (ring){
                memcpy(ring, p.points, sizeof(RantDouble3) * p.count);
                ring[p.count] = p.points[0];
                r = line_pieces(ring, p.count + 1, out);
                free(ring);
            }
        }
        free(flat);
        return r;
    }
    case RANT_SHAPE_MESH: {
        RantMeshView m = s->as.mesh;
        uint32_t t, n = m.vertices && m.triangles ? m.triangle_count : 0;
        if ((r = pieces_alloc(out, n)) != RANT_MATH_OK) return r;
        for (t = 0; t < n; t++){
            const uint32_t *i = m.triangles + 3 * (size_t)t;
            RantDouble3 c[3];
            if (i[0] >= m.vertex_count || i[1] >= m.vertex_count || i[2] >= m.vertex_count) continue;
            c[0] = m.vertices[i[0]]; c[1] = m.vertices[i[1]]; c[2] = m.vertices[i[2]];
            out->list[out->count++] = points_piece(c, 3, 0.0);
        }
        if (!out->count){ pieces_free(out); return RANT_MATH_ERR_ARGUMENT; }
        return RANT_MATH_OK;
    }
    }
    return RANT_MATH_ERR_ARGUMENT;
}

/* A corner of the Minkowski difference a - b and the support points that made it. */
typedef struct { RantDouble3 w, a, b; } i_Corner;

/* The point of the simplex nearest the origin, as weights on its corners, then only the
 * corners that carry weight are kept. Returns 1 when the origin lies inside it. */
static int nearest_on_simplex(i_Corner *s, int *n, double *lambda){
    int i, k = 0;
    double w[4] = { 1.0, 0.0, 0.0, 0.0 };
    RantDouble3 o = rant_double3(0, 0, 0);
    if (*n == 2){
        RantDouble3 e = rant_double3_subtract(s[1].w, s[0].w);
        double ee = rant_double3_dot(e, e), t = ee > 0.0 ? i_math_clamp(-rant_double3_dot(s[0].w, e) / ee, 0.0, 1.0) : 0.0;
        w[0] = 1.0 - t; w[1] = t;
    } else if (*n == 3){
        i_math_triangle_closest(s[0].w, s[1].w, s[2].w, o, w);
    } else if (*n == 4){
        /* inside when the origin's weights from signed volumes are none of them negative,
         * else the nearest of the four faces */
        static const int face[4][3] = { { 1, 2, 3 }, { 0, 2, 3 }, { 0, 1, 3 }, { 0, 1, 2 } };
        RantDouble3 e1 = rant_double3_subtract(s[1].w, s[0].w), e2 = rant_double3_subtract(s[2].w, s[0].w);
        RantDouble3 e3 = rant_double3_subtract(s[3].w, s[0].w), m = rant_double3_scale(s[0].w, -1.0);
        double vol = rant_double3_dot(e1, rant_double3_cross(e2, e3)), best = HUGE_VAL;
        int f;
        if (fabs(vol) > 1e-9 * rant_double3_length(e1) * rant_double3_length(e2) * rant_double3_length(e3)){
            w[1] = rant_double3_dot(m, rant_double3_cross(e2, e3)) / vol;
            w[2] = rant_double3_dot(e1, rant_double3_cross(m, e3)) / vol;
            w[3] = rant_double3_dot(e1, rant_double3_cross(e2, m)) / vol;
            w[0] = 1.0 - w[1] - w[2] - w[3];
            if (w[0] >= 0.0 && w[1] >= 0.0 && w[2] >= 0.0 && w[3] >= 0.0){
                for (i = 0; i < 4; i++) lambda[i] = w[i];
                return 1;
            }
        }
        for (f = 0; f < 4; f++){
            const int *v = face[f];
            double fw[3], d;
            i_math_triangle_closest(s[v[0]].w, s[v[1]].w, s[v[2]].w, o, fw);
            d = rant_double3_length(rant_double3_add(rant_double3_add(rant_double3_scale(s[v[0]].w, fw[0]),
                                                                      rant_double3_scale(s[v[1]].w, fw[1])),
                                                     rant_double3_scale(s[v[2]].w, fw[2])));
            if (d < best){
                best = d;
                w[0] = w[1] = w[2] = w[3] = 0.0;
                w[v[0]] = fw[0]; w[v[1]] = fw[1]; w[v[2]] = fw[2];
            }
        }
    }
    for (i = 0; i < *n; i++)
        if (w[i] > 0.0){
            s[k] = s[i];
            lambda[k++] = w[i];
        }
    *n = k;
    return 0;
}

/* GJK's distance between two convex cores, and the point of each nearest the other. */
static double gjk(const i_Piece *pa, const i_Piece *pb, RantDouble3 *on_a, RantDouble3 *on_b){
    i_Corner s[4];
    double lambda[4] = { 1.0, 0.0, 0.0, 0.0 }, reach = 0.0;
    int n = 1, iter, i, inside = 0;
    RantDouble3 v;
    s[0].a = support(pa, rant_double3(1, 0, 0));
    s[0].b = support(pb, rant_double3(-1, 0, 0));
    s[0].w = v = rant_double3_subtract(s[0].a, s[0].b);
    for (iter = 0; iter < 128; iter++){
        double vv = rant_double3_dot(v, v);
        i_Corner c;
        reach = i_math_max(reach, vv);
        if (vv <= 1e-26 * reach){ inside = 1; break; }
        c.a = support(pa, rant_double3_scale(v, -1.0));
        c.b = support(pb, v);
        c.w = rant_double3_subtract(c.a, c.b);
        reach = i_math_max(reach, rant_double3_dot(c.w, c.w));
        if (vv - rant_double3_dot(v, c.w) <= 1e-11 * vv) break;
        for (i = 0; i < n; i++) if (rant_double3_distance(s[i].w, c.w) == 0.0) break;
        if (i < n) break;
        {   /* rounding near the answer can make a step that gains nothing: keep the last one */
            i_Corner keep[4];
            double kept[4];
            int was = n;
            RantDouble3 next = rant_double3(0, 0, 0);
            memcpy(keep, s, sizeof keep);
            memcpy(kept, lambda, sizeof kept);
            s[n++] = c;
            inside = nearest_on_simplex(s, &n, lambda);
            for (i = 0; i < n; i++) next = rant_double3_add(next, rant_double3_scale(s[i].w, lambda[i]));
            if (!inside && rant_double3_dot(next, next) >= vv){
                memcpy(s, keep, sizeof keep);
                memcpy(lambda, kept, sizeof kept);
                n = was;
                break;
            }
            v = next;
        }
        if (inside) break;
    }
    *on_a = *on_b = rant_double3(0, 0, 0);
    for (i = 0; i < n; i++){
        *on_a = rant_double3_add(*on_a, rant_double3_scale(s[i].a, lambda[i]));
        *on_b = rant_double3_add(*on_b, rant_double3_scale(s[i].b, lambda[i]));
    }
    return inside ? 0.0 : rant_double3_length(v);
}

static double box_gap(RantAlignedBox a, RantAlignedBox b){
    RantDouble3 d = rant_double3(i_math_max(0.0, i_math_max(a.min.x - b.max.x, b.min.x - a.max.x)),
                                 i_math_max(0.0, i_math_max(a.min.y - b.max.y, b.min.y - a.max.y)),
                                 i_math_max(0.0, i_math_max(a.min.z - b.max.z, b.min.z - a.max.z)));
    return rant_double3_length(d);
}

/* The gap of two lists of pieces, the margins taken off the cores' gap. A pair whose bounds
 * are already farther apart than the best so far is skipped. */
static double pieces_gap(const i_Pieces *a, const i_Pieces *b, RantDouble3 *on_a, RantDouble3 *on_b){
    double best = HUGE_VAL;
    uint32_t i, j;
    for (i = 0; i < a->count && best > 0.0; i++)
        for (j = 0; j < b->count && best > 0.0; j++){
            const i_Piece *pa = &a->list[i], *pb = &b->list[j];
            RantDouble3 ca, cb;
            double core, gap, m = pa->margin + pb->margin;
            if (box_gap(pa->bounds, pb->bounds) >= best) continue;
            core = gjk(pa, pb, &ca, &cb);
            gap = i_math_max(0.0, core - m);
            if (gap < best){
                RantDouble3 dir = core > 0.0 ? rant_double3_scale(rant_double3_subtract(cb, ca), 1.0 / core) : rant_double3(0, 0, 0);
                best = gap;
                if (gap > 0.0){
                    *on_a = rant_double3_add(ca, rant_double3_scale(dir, pa->margin));
                    *on_b = rant_double3_subtract(cb, rant_double3_scale(dir, pb->margin));
                } else {
                    /* a point in both: along the cores' gap, split by the margins */
                    *on_a = *on_b = m > 0.0 ? rant_double3_add(ca, rant_double3_scale(dir, core * pa->margin / m)) : ca;
                }
            }
        }
    return best;
}

RantMathResult rant_shape_gap(const RantShape *a, const RantShape *b, double *gap, RantSegment *between){
    i_Pieces pa, pb;
    RantDouble3 on_a, on_b;
    RantMathResult r;
    double g;
    if (!a || !b || !gap) return RANT_MATH_ERR_ARGUMENT;
    if ((r = pieces_of(a, &pa)) != RANT_MATH_OK) return r;
    if ((r = pieces_of(b, &pb)) != RANT_MATH_OK){ pieces_free(&pa); return r; }
    g = pieces_gap(&pa, &pb, &on_a, &on_b);
    pieces_free(&pa);
    pieces_free(&pb);
    *gap = g;
    if (between){ between->a = on_a; between->b = on_b; }
    return RANT_MATH_OK;
}

int rant_shape_overlaps(const RantShape *a, const RantShape *b){
    double g;
    return rant_shape_gap(a, b, &g, NULL) == RANT_MATH_OK && g == 0.0;
}

/* Draws in order, one at a time. A call's arguments have no order of their own, so drawing inside
 * them would give each compiler its own answer for one seed. */
static RantDouble3 draw3(i_MathRng *r){
    RantDouble3 v;
    v.x = i_math_rng_unit(r);
    v.y = i_math_rng_unit(r);
    v.z = i_math_rng_unit(r);
    return v;
}

static RantDouble2 draw2(i_MathRng *r){
    RantDouble2 v;
    v.x = i_math_rng_unit(r);
    v.y = i_math_rng_unit(r);
    return v;
}

static RantDouble3 on_ball(i_MathRng *r){
    double z = 2.0 * i_math_rng_unit(r) - 1.0, a = 2.0 * I_MATH_PI * i_math_rng_unit(r), s = sqrt(i_math_max(0.0, 1.0 - z * z));
    return rant_double3(s * cos(a), s * sin(a), z);
}

static RantDouble3 in_triangle(i_MathRng *r, RantDouble3 a, RantDouble3 b, RantDouble3 c){
    double u = i_math_rng_unit(r), v = i_math_rng_unit(r);
    if (u + v > 1.0){ u = 1.0 - u; v = 1.0 - v; }
    return rant_double3_add(a, rant_double3_add(rant_double3_scale(rant_double3_subtract(b, a), u),
                                                rant_double3_scale(rant_double3_subtract(c, a), v)));
}

/* The first index whose running total passes u times the whole. */
static uint32_t pick(const double *total, uint32_t n, double u){
    uint32_t lo = 0, hi = n - 1;
    double want = u * total[n - 1];
    while (lo < hi){
        uint32_t mid = (lo + hi) / 2;
        if (total[mid] > want) hi = mid; else lo = mid + 1;
    }
    return lo;
}

/* A frame along a unit axis: the level side and the third axis. */
static void axis_frame(RantDouble3 u, RantDouble3 *s1, RantDouble3 *s2){
    *s1 = i_math_side(u);
    *s2 = rant_double3_cross(u, *s1);
}

static RantDouble3 around(RantDouble3 c, RantDouble3 s1, RantDouble3 s2, double radius, double angle){
    return rant_double3_add(c, rant_double3_add(rant_double3_scale(s1, radius * cos(angle)), rant_double3_scale(s2, radius * sin(angle))));
}

/* Triangles weighted by area: those of a mesh or of a pieces list of triangles or prisms. */
static RantMathResult sample_pieces(const i_Pieces *p, int top, i_MathRng *r, RantDouble3 *out, uint32_t count){
    double *total = (double *)malloc(sizeof(double) * p->count), sum = 0.0;
    uint32_t i, k;
    if (!total) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i < p->count; i++){
        const RantDouble3 *c = p->list[i].p;
        sum += 0.5 * rant_double3_length(rant_double3_cross(rant_double3_subtract(c[1], c[0]), rant_double3_subtract(c[2], c[0])));
        total[i] = sum;
    }
    if (!(sum > 0.0)){ free(total); return RANT_MATH_ERR_DEGENERATE; }
    for (k = 0; k < count; k++){
        const RantDouble3 *c = p->list[pick(total, p->count, i_math_rng_unit(r))].p;
        RantDouble3 a = in_triangle(r, c[0], c[1], c[2]);
        if (top){
            /* a prism's piece: the same spot on its top face, then a height between */
            double w[3], t = i_math_rng_unit(r);
            RantDouble3 b;
            i_math_triangle_closest(c[0], c[1], c[2], a, w);
            b = rant_double3_add(rant_double3_add(rant_double3_scale(c[3], w[0]), rant_double3_scale(c[4], w[1])), rant_double3_scale(c[5], w[2]));
            a = rant_double3_lerp(a, b, t);
        }
        out[k] = a;
    }
    free(total);
    return RANT_MATH_OK;
}

RantMathResult rant_shape_sample_inside(const RantShape *s, uint64_t seed, RantDouble3 *out, uint32_t count){
    i_MathRng r = i_math_rng(seed);
    uint32_t k;
    if (!s || !out) return RANT_MATH_ERR_ARGUMENT;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX: {
        RantAlignedBox b = s->as.aligned_box;
        for (k = 0; k < count; k++){
            RantDouble3 d = draw3(&r);
            out[k] = rant_double3(b.min.x + (b.max.x - b.min.x) * d.x, b.min.y + (b.max.y - b.min.y) * d.y, b.min.z + (b.max.z - b.min.z) * d.z);
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_ORIENTED_BOX: {
        RantOrientedBox b = s->as.oriented_box;
        for (k = 0; k < count; k++){
            RantDouble3 d = draw3(&r);
            out[k] = rant_pose_apply(b.pose, rant_double3(b.size.x * (d.x - 0.5), b.size.y * (d.y - 0.5), b.size.z * (d.z - 0.5)));
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_SPHERE:
        for (k = 0; k < count; k++){
            RantDouble3 v;
            do v = rant_double3_subtract(rant_double3_scale(draw3(&r), 2.0), rant_double3(1.0, 1.0, 1.0));
            while (rant_double3_length_squared(v) > 1.0);
            out[k] = rant_double3_add(s->as.sphere.center, rant_double3_scale(v, fabs(s->as.sphere.radius)));
        }
        return RANT_MATH_OK;
    case RANT_SHAPE_CAPSULE: {
        RantCapsule c = s->as.capsule;
        RantDouble3 u = i_math_axis(c.axis.a, c.axis.b), s1, s2;
        double rad = fabs(c.radius), len = rant_segment_length(c.axis);
        if (!(rad > 0.0)) return RANT_MATH_ERR_DEGENERATE;
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            RantDouble3 q;
            do {
                RantDouble3 d = draw3(&r);
                q = rant_double3_add(c.axis.a, rant_double3_scale(u, -rad + (len + 2.0 * rad) * d.x));
                q = rant_double3_add(q, rant_double3_add(rant_double3_scale(s1, rad * (2.0 * d.y - 1.0)),
                                                         rant_double3_scale(s2, rad * (2.0 * d.z - 1.0))));
            } while (rant_segment_distance(c.axis, q) > rad);
            out[k] = q;
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_CYLINDER: {
        RantCylinder c = s->as.cylinder;
        RantDouble3 u = i_math_axis(c.axis.a, c.axis.b), s1, s2;
        double len = rant_segment_length(c.axis);
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            RantDouble3 d = draw3(&r);
            out[k] = around(rant_double3_add(c.axis.a, rant_double3_scale(u, len * d.x)), s1, s2,
                            fabs(c.radius) * sqrt(d.y), 2.0 * I_MATH_PI * d.z);
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_CONE: {
        /* the cut at height y holds (1 - y/h)^2 of the base, so y comes from a cube root */
        RantCone c = s->as.cone;
        RantDouble3 u = i_math_axis(c.base, c.tip), s1, s2;
        double h = rant_double3_distance(c.base, c.tip);
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            double left = cbrt(i_math_rng_unit(&r));
            RantDouble2 d = draw2(&r);
            out[k] = around(rant_double3_add(c.base, rant_double3_scale(u, h * (1.0 - left))), s1, s2,
                            fabs(c.radius) * left * sqrt(d.x), 2.0 * I_MATH_PI * d.y);
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_PRISM: {
        i_Pieces p;
        RantMathResult res = pieces_of(s, &p);
        if (res != RANT_MATH_OK) return res;
        res = sample_pieces(&p, 1, &r, out, count);
        pieces_free(&p);
        return res;
    }
    case RANT_SHAPE_MESH: {
        /* a guess in the bounds, kept when the mesh winds round it. An open mesh may hold none. */
        RantAlignedBox b = rant_mesh_bounds(s->as.mesh);
        uint32_t tries = 0;
        if (!(b.min.x <= b.max.x)) return RANT_MATH_ERR_DEGENERATE;
        for (k = 0; k < count; tries++){
            RantDouble3 d = draw3(&r);
            RantDouble3 q = rant_double3(b.min.x + (b.max.x - b.min.x) * d.x, b.min.y + (b.max.y - b.min.y) * d.y, b.min.z + (b.max.z - b.min.z) * d.z);
            if (tries > 1000u + 100u * count) return RANT_MATH_ERR_DEGENERATE;
            if (rant_mesh_contains(s->as.mesh, q)) out[k++] = q;
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_SEGMENT: case RANT_SHAPE_POLYLINE: case RANT_SHAPE_POLYGON:
        return RANT_MATH_ERR_ARGUMENT;
    }
    return RANT_MATH_ERR_ARGUMENT;
}

/* A box's faces by area, in its own frame from its full sizes. */
static RantDouble3 on_box(i_MathRng *r, RantDouble3 size){
    double a[6], sum = 0.0, u, v;
    uint32_t f;
    int axis;
    a[0] = a[1] = fabs(size.y * size.z);
    a[2] = a[3] = fabs(size.x * size.z);
    a[4] = a[5] = fabs(size.x * size.y);
    for (f = 0; f < 6; f++){ sum += a[f]; a[f] = sum; }
    f = sum > 0.0 ? pick(a, 6, i_math_rng_unit(r)) : 0;
    axis = (int)(f / 2);
    u = i_math_rng_unit(r) - 0.5;
    v = i_math_rng_unit(r) - 0.5;
    if (axis == 0) return rant_double3(f % 2 ? 0.5 * size.x : -0.5 * size.x, u * size.y, v * size.z);
    if (axis == 1) return rant_double3(u * size.x, f % 2 ? 0.5 * size.y : -0.5 * size.y, v * size.z);
    return rant_double3(u * size.x, v * size.y, f % 2 ? 0.5 * size.z : -0.5 * size.z);
}

static RantMathResult along_points(const RantDouble3 *p, uint32_t n, i_MathRng *r, RantDouble3 *out, uint32_t count){
    double *total, sum = 0.0;
    uint32_t i, k;
    if (!p || !n) return RANT_MATH_ERR_ARGUMENT;
    if (n == 1){
        for (k = 0; k < count; k++) out[k] = p[0];
        return RANT_MATH_OK;
    }
    total = (double *)malloc(sizeof(double) * (n - 1));
    if (!total) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i + 1 < n; i++){ sum += rant_double3_distance(p[i], p[i + 1]); total[i] = sum; }
    for (k = 0; k < count; k++){
        i = sum > 0.0 ? pick(total, n - 1, i_math_rng_unit(r)) : 0;
        out[k] = rant_double3_lerp(p[i], p[i + 1], i_math_rng_unit(r));
    }
    free(total);
    return RANT_MATH_OK;
}

RantMathResult rant_shape_sample_surface(const RantShape *s, uint64_t seed, RantDouble3 *out, uint32_t count){
    i_MathRng r = i_math_rng(seed);
    uint32_t k;
    if (!s || !out) return RANT_MATH_ERR_ARGUMENT;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX: {
        RantAlignedBox b = s->as.aligned_box;
        RantDouble3 c = rant_aligned_box_centroid(b), size = rant_double3_subtract(b.max, b.min);
        for (k = 0; k < count; k++) out[k] = rant_double3_add(c, on_box(&r, size));
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_ORIENTED_BOX:
        for (k = 0; k < count; k++) out[k] = rant_pose_apply(s->as.oriented_box.pose, on_box(&r, s->as.oriented_box.size));
        return RANT_MATH_OK;
    case RANT_SHAPE_SPHERE:
        for (k = 0; k < count; k++) out[k] = rant_double3_add(s->as.sphere.center, rant_double3_scale(on_ball(&r), fabs(s->as.sphere.radius)));
        return RANT_MATH_OK;
    case RANT_SHAPE_CAPSULE: {
        /* the side by its area, else a point of a ball on the end whose outer half it falls in */
        RantCapsule c = s->as.capsule;
        RantDouble3 u = i_math_axis(c.axis.a, c.axis.b), s1, s2;
        double rad = fabs(c.radius), len = rant_segment_length(c.axis), side = 2.0 * I_MATH_PI * rad * len, ends = 4.0 * I_MATH_PI * rad * rad;
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            if (i_math_rng_unit(&r) * (side + ends) < side){
                RantDouble2 d = draw2(&r);
                out[k] = around(rant_double3_add(c.axis.a, rant_double3_scale(u, len * d.x)), s1, s2, rad, 2.0 * I_MATH_PI * d.y);
            } else {
                RantDouble3 v = on_ball(&r);
                out[k] = rant_double3_add(rant_double3_dot(v, u) < 0.0 ? c.axis.a : c.axis.b, rant_double3_scale(v, rad));
            }
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_CYLINDER: {
        RantCylinder c = s->as.cylinder;
        RantDouble3 u = i_math_axis(c.axis.a, c.axis.b), s1, s2;
        double rad = fabs(c.radius), len = rant_segment_length(c.axis), side = 2.0 * I_MATH_PI * rad * len, cap = I_MATH_PI * rad * rad;
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            double pickd = i_math_rng_unit(&r) * (side + 2.0 * cap), a = 2.0 * I_MATH_PI * i_math_rng_unit(&r);
            if (pickd < side) out[k] = around(rant_double3_add(c.axis.a, rant_double3_scale(u, len * i_math_rng_unit(&r))), s1, s2, rad, a);
            else out[k] = around(pickd < side + cap ? c.axis.a : c.axis.b, s1, s2, rad * sqrt(i_math_rng_unit(&r)), a);
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_CONE: {
        /* on the slope the share at a distance from the tip grows with it, so a square root */
        RantCone c = s->as.cone;
        RantDouble3 u = i_math_axis(c.base, c.tip), s1, s2;
        double rad = fabs(c.radius), h = rant_double3_distance(c.base, c.tip);
        double side = I_MATH_PI * rad * sqrt(rad * rad + h * h), base = I_MATH_PI * rad * rad;
        axis_frame(u, &s1, &s2);
        for (k = 0; k < count; k++){
            double a = 2.0 * I_MATH_PI * i_math_rng_unit(&r);
            if (i_math_rng_unit(&r) * (side + base) < side){
                double f = sqrt(i_math_rng_unit(&r));
                out[k] = around(rant_double3_add(c.base, rant_double3_scale(u, h * (1.0 - f))), s1, s2, rad * f, a);
            } else {
                out[k] = around(c.base, s1, s2, rad * sqrt(i_math_rng_unit(&r)), a);
            }
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE_PRISM: {
        /* the two caps by the base's area, the sides by their edges' lengths */
        RantPrismView p = s->as.prism;
        double area = rant_polygon2d_area(p.base), height = fabs(p.height), lo = i_math_min(0.0, p.height), hi = i_math_max(0.0, p.height);
        double ring = rant_polygon2d_perimeter(p.base) * height;
        i_Pieces caps;
        RantMathResult res = pieces_of(s, &caps);
        uint32_t n = p.base.count;
        if (res != RANT_MATH_OK) return res;
        for (k = 0; k < count && res == RANT_MATH_OK; k++){
            double u = i_math_rng_unit(&r) * (2.0 * area + ring);
            if (u < 2.0 * area){
                RantDouble3 q;
                RantDouble3 local;
                res = sample_pieces(&caps, 0, &r, &q, 1);
                local = rant_pose_apply_inverse(p.pose, q);
                out[k] = rant_pose_apply(p.pose, rant_double3(local.x, local.y, u < area ? lo : hi));
            } else {
                double want = (u - 2.0 * area) / (height > 0.0 ? height : 1.0), along = 0.0;
                uint32_t i;
                for (i = 0; i < n; i++){
                    RantDouble2 a = p.base.points[i], b = p.base.points[i + 1 == n ? 0 : i + 1];
                    double len = rant_double2_distance(a, b);
                    if (along + len >= want || i + 1 == n){
                        RantDouble2 at = rant_double2_lerp(a, b, len > 0.0 ? i_math_clamp((want - along) / len, 0.0, 1.0) : 0.0);
                        out[k] = rant_pose_apply(p.pose, rant_double3(at.x, at.y, lo + (hi - lo) * i_math_rng_unit(&r)));
                        break;
                    }
                    along += len;
                }
            }
        }
        pieces_free(&caps);
        return res;
    }
    case RANT_SHAPE_SEGMENT: {
        RantDouble3 two[2];
        two[0] = s->as.segment.a; two[1] = s->as.segment.b;
        return along_points(two, 2, &r, out, count);
    }
    case RANT_SHAPE_POLYLINE:
        return along_points(s->as.polyline.points, s->as.polyline.count, &r, out, count);
    case RANT_SHAPE_POLYGON: case RANT_SHAPE_MESH: {
        i_Pieces p;
        RantMathResult res = pieces_of(s, &p);
        if (res != RANT_MATH_OK) return res;
        res = p.count && p.list[0].n == 3 ? sample_pieces(&p, 0, &r, out, count) : RANT_MATH_ERR_DEGENERATE;
        pieces_free(&p);
        return res;
    }
    }
    return RANT_MATH_ERR_ARGUMENT;
}

/* A mesh written into room checked beforehand. */
typedef struct { RantDouble3 *v; uint32_t *t; uint32_t nv, nt; } i_Build;

static uint32_t put_vertex(i_Build *b, RantDouble3 p){ b->v[b->nv] = p; return b->nv++; }
static void put_triangle(i_Build *b, uint32_t x, uint32_t y, uint32_t z){
    uint32_t *t = b->t + 3 * (size_t)b->nt++;
    t[0] = x; t[1] = y; t[2] = z;
}

/* s vertices round c, turning from s1 toward s2, so counterclockwise about s1 x s2. */
static uint32_t put_ring(i_Build *b, RantDouble3 c, RantDouble3 s1, RantDouble3 s2, double radius, uint32_t s){
    uint32_t first = b->nv, j;
    for (j = 0; j < s; j++) put_vertex(b, around(c, s1, s2, radius, 2.0 * I_MATH_PI * j / s));
    return first;
}

/* From ring a to ring c, a the one farther along their axis. */
static void put_band(i_Build *b, uint32_t a, uint32_t c, uint32_t s){
    uint32_t j;
    for (j = 0; j < s; j++){
        uint32_t k = (j + 1) % s;
        put_triangle(b, a + j, c + j, c + k);
        put_triangle(b, a + j, c + k, a + k);
    }
}

/* From one vertex to a ring, facing along the ring's axis, or against it when down. */
static void put_fan(i_Build *b, uint32_t tip, uint32_t ring, uint32_t s, int down){
    uint32_t j;
    for (j = 0; j < s; j++){
        uint32_t k = (j + 1) % s;
        if (down) put_triangle(b, tip, ring + k, ring + j);
        else put_triangle(b, tip, ring + j, ring + k);
    }
}

static void put_box(i_Build *b, const RantDouble3 c[8]){
    static const uint32_t t[12][3] = { {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                       {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7} };
    int i;
    for (i = 0; i < 8; i++) put_vertex(b, c[i]);
    for (i = 0; i < 12; i++) put_triangle(b, t[i][0], t[i][1], t[i][2]);
}

/* The corners of an outline that start an edge, numbered in order into number when it is not
 * NULL, a repeated corner left out. Returns how many. */
static uint32_t outline_corners(RantPolygon2DView p, uint32_t *number){
    uint32_t i, e = 0, n = p.points ? p.count : 0;
    for (i = 0; i < n; i++){
        RantDouble2 a = p.points[i], c = p.points[i + 1 == n ? 0 : i + 1];
        int edge = a.x != c.x || a.y != c.y;
        if (number) number[i] = edge ? e : UINT32_MAX;
        e += (uint32_t)edge;
    }
    return e;
}

/* Triangles over an outline of e corners by their numbers, facing +z, or -z when down. */
static RantMathResult put_outline(i_Build *b, RantPolygon2DView p, const uint32_t *number, uint32_t e, uint32_t base,
                                  int down){
    uint32_t t = e - 2, k, *tri = (uint32_t *)malloc(sizeof(uint32_t) * 3 * t);
    RantMathResult r;
    if (!tri) return RANT_MATH_ERR_MEMORY;
    if ((r = rant_polygon2d_triangulate(p, tri, &t)) == RANT_MATH_OK)
        for (k = 0; k < t; k++){
            uint32_t x = base + number[tri[3 * k]], y = base + number[tri[3 * k + 1]], z = base + number[tri[3 * k + 2]];
            if (down) put_triangle(b, x, z, y);
            else put_triangle(b, x, y, z);
        }
    free(tri);
    return r;
}

static RantMathResult prism_mesh(i_Build *b, RantPrismView p, uint32_t e){
    double lo = i_math_min(0.0, p.height), hi = i_math_max(0.0, p.height);
    int ccw = rant_polygon2d_signed_area(p.base) >= 0.0;
    uint32_t *number = (uint32_t *)malloc(sizeof(uint32_t) * p.base.count), i, k;
    RantMathResult r;
    if (!number) return RANT_MATH_ERR_MEMORY;
    outline_corners(p.base, number);
    for (k = 0; k < 2; k++)
        for (i = 0; i < p.base.count; i++)
            if (number[i] != UINT32_MAX)
                put_vertex(b, rant_pose_apply(p.pose, rant_double3(p.base.points[i].x, p.base.points[i].y, k ? hi : lo)));
    r = put_outline(b, p.base, number, e, 0, 1);
    if (r == RANT_MATH_OK) r = put_outline(b, p.base, number, e, e, 0);
    for (k = 0; k < e && r == RANT_MATH_OK; k++){
        uint32_t j = (k + 1) % e;
        if (ccw){ put_triangle(b, k, j, e + j); put_triangle(b, k, e + j, e + k); }
        else { put_triangle(b, k, e + j, j); put_triangle(b, k, e + k, e + j); }
    }
    free(number);
    return r;
}

/* The corners of p flattened into flat, which holds p.count. */
static RantMathResult polygon_mesh(i_Build *b, RantPolygonView p, RantPolygon2DView flat, uint32_t e){
    uint32_t *number = (uint32_t *)malloc(sizeof(uint32_t) * p.count), i;
    RantMathResult r;
    if (!number) return RANT_MATH_ERR_MEMORY;
    outline_corners(flat, number);
    for (i = 0; i < p.count; i++) if (number[i] != UINT32_MAX) put_vertex(b, p.points[i]);
    /* the face is the polygon's own, by the right hand rule round its corners */
    r = put_outline(b, flat, number, e, 0, rant_polygon2d_signed_area(flat) < 0.0);
    free(number);
    return r;
}

RantMathResult rant_shape_mesh(const RantShape *s, uint32_t segments, RantDouble3 *vertices, uint32_t *vertex_count,
                               uint32_t *triangles, uint32_t *triangle_count){
    uint32_t n = segments ? segments : 32, rings = n / 2 < 2 ? 2 : n / 2, half = n / 4 < 1 ? 1 : n / 4, e = 0, i, k;
    uint64_t nv = 0, nt = 0;
    RantDouble2 *flat = NULL;
    RantPolygon2DView fv;
    RantMathResult r = RANT_MATH_OK;
    i_Build b;
    if (!s || !vertex_count || !triangle_count || (segments && segments < 3)) return RANT_MATH_ERR_ARGUMENT;
    fv.points = NULL;
    fv.count = 0;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX: case RANT_SHAPE_ORIENTED_BOX: nv = 8; nt = 12; break;
    case RANT_SHAPE_SPHERE: nv = 2 + (uint64_t)(rings - 1) * n; nt = 2ull * n * (rings - 1); break;
    case RANT_SHAPE_CAPSULE: nv = 2 + 2ull * half * n; nt = 4ull * half * n; break;
    case RANT_SHAPE_CYLINDER: nv = 2 + 2ull * n; nt = 4ull * n; break;
    case RANT_SHAPE_CONE: nv = 2 + (uint64_t)n; nt = 2ull * n; break;
    case RANT_SHAPE_PRISM:
        if ((e = outline_corners(s->as.prism.base, NULL)) < 3) return RANT_MATH_ERR_DEGENERATE;
        nv = 2ull * e;
        nt = 4ull * e - 4;
        break;
    case RANT_SHAPE_POLYGON: {
        RantPose frame;
        RantPolygonView p = s->as.polygon;
        if (!p.points || p.count < 3) return RANT_MATH_ERR_DEGENERATE;
        flat = (RantDouble2 *)malloc(sizeof(RantDouble2) * p.count);
        if (!flat) return RANT_MATH_ERR_MEMORY;
        if ((r = rant_polygon_flatten(p, &frame, flat)) != RANT_MATH_OK){ free(flat); return r; }
        fv.points = flat;
        fv.count = p.count;
        if ((e = outline_corners(fv, NULL)) < 3){ free(flat); return RANT_MATH_ERR_DEGENERATE; }
        nv = e;
        nt = e - 2;
        break;
    }
    case RANT_SHAPE_MESH:
        if (s->as.mesh.vertex_count && !s->as.mesh.vertices) return RANT_MATH_ERR_ARGUMENT;
        if (s->as.mesh.triangle_count && !s->as.mesh.triangles) return RANT_MATH_ERR_ARGUMENT;
        nv = s->as.mesh.vertex_count;
        nt = s->as.mesh.triangle_count;
        break;
    default: return RANT_MATH_ERR_ARGUMENT;
    }
    if (nv > UINT32_MAX || nt > UINT32_MAX / 3){ free(flat); return RANT_MATH_ERR_ARGUMENT; }
    i = *vertex_count;
    k = *triangle_count;
    *vertex_count = (uint32_t)nv;
    *triangle_count = (uint32_t)nt;
    if (nv > i || nt > k || (nv && !vertices) || (nt && !triangles)){ free(flat); return RANT_MATH_ERR_SPACE; }
    b.v = vertices; b.t = triangles; b.nv = 0; b.nt = 0;
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX: {
        RantAlignedBox x;
        RantDouble3 c[8];
        x.min = rant_double3_min(s->as.aligned_box.min, s->as.aligned_box.max);
        x.max = rant_double3_max(s->as.aligned_box.min, s->as.aligned_box.max);
        rant_aligned_box_corners(x, c);
        put_box(&b, c);
        break;
    }
    case RANT_SHAPE_ORIENTED_BOX: {
        RantOrientedBox x = s->as.oriented_box;
        RantDouble3 c[8];
        x.size = rant_double3(fabs(x.size.x), fabs(x.size.y), fabs(x.size.z));
        rant_oriented_box_corners(x, c);
        put_box(&b, c);
        break;
    }
    case RANT_SHAPE_SPHERE: {
        RantSphere x = s->as.sphere;
        RantDouble3 up = rant_double3(0.0, 0.0, 1.0), s1, s2;
        double radius = fabs(x.radius);
        uint32_t top, bottom, ring = 0, last = 0;
        axis_frame(up, &s1, &s2);
        top = put_vertex(&b, rant_double3_add(x.center, rant_double3_scale(up, radius)));
        for (i = 1; i < rings; i++){
            double a = I_MATH_PI * i / rings;
            ring = put_ring(&b, rant_double3_add(x.center, rant_double3_scale(up, radius * cos(a))), s1, s2, radius * sin(a), n);
            if (i == 1) put_fan(&b, top, ring, n, 0);
            else put_band(&b, last, ring, n);
            last = ring;
        }
        bottom = put_vertex(&b, rant_double3_add(x.center, rant_double3_scale(up, -radius)));
        put_fan(&b, bottom, last, n, 1);
        break;
    }
    case RANT_SHAPE_CAPSULE: {
        RantCapsule x = s->as.capsule;
        RantDouble3 up = i_math_axis(x.axis.a, x.axis.b), s1, s2;
        double radius = fabs(x.radius);
        uint32_t top, bottom, ring, last = 0;
        axis_frame(up, &s1, &s2);
        top = put_vertex(&b, rant_double3_add(x.axis.b, rant_double3_scale(up, radius)));
        /* down the top half from its pole to the equator, then the bottom half from its equator */
        for (i = 1; i <= 2 * half; i++){
            int low = i > half;
            double a = 0.5 * I_MATH_PI * (low ? 2 * half + 1 - i : i) / half;
            RantDouble3 c = rant_double3_add(low ? x.axis.a : x.axis.b, rant_double3_scale(up, (low ? -radius : radius) * cos(a)));
            ring = put_ring(&b, c, s1, s2, radius * sin(a), n);
            if (i == 1) put_fan(&b, top, ring, n, 0);
            else put_band(&b, last, ring, n);
            last = ring;
        }
        bottom = put_vertex(&b, rant_double3_add(x.axis.a, rant_double3_scale(up, -radius)));
        put_fan(&b, bottom, last, n, 1);
        break;
    }
    case RANT_SHAPE_CYLINDER: {
        RantCylinder x = s->as.cylinder;
        RantDouble3 up = i_math_axis(x.axis.a, x.axis.b), s1, s2;
        uint32_t ca, cb, ra, rb;
        axis_frame(up, &s1, &s2);
        ca = put_vertex(&b, x.axis.a);
        cb = put_vertex(&b, x.axis.b);
        ra = put_ring(&b, x.axis.a, s1, s2, fabs(x.radius), n);
        rb = put_ring(&b, x.axis.b, s1, s2, fabs(x.radius), n);
        put_fan(&b, ca, ra, n, 1);
        put_fan(&b, cb, rb, n, 0);
        put_band(&b, rb, ra, n);
        break;
    }
    case RANT_SHAPE_CONE: {
        RantCone x = s->as.cone;
        RantDouble3 up = i_math_axis(x.base, x.tip), s1, s2;
        uint32_t c, ring, tip;
        axis_frame(up, &s1, &s2);
        c = put_vertex(&b, x.base);
        ring = put_ring(&b, x.base, s1, s2, fabs(x.radius), n);
        tip = put_vertex(&b, x.tip);
        put_fan(&b, c, ring, n, 1);
        put_fan(&b, tip, ring, n, 0);
        break;
    }
    case RANT_SHAPE_PRISM: r = prism_mesh(&b, s->as.prism, e); break;
    case RANT_SHAPE_POLYGON: r = polygon_mesh(&b, s->as.polygon, fv, e); break;
    case RANT_SHAPE_MESH:
        memcpy(vertices, s->as.mesh.vertices, sizeof(RantDouble3) * s->as.mesh.vertex_count);
        memcpy(triangles, s->as.mesh.triangles, sizeof(uint32_t) * 3 * s->as.mesh.triangle_count);
        break;
    default: break;
    }
    free(flat);
    return r;
}

RantShape2D rant_shape2d_from_aligned_box2d(RantAlignedBox2D b){ RantShape2D s; s.kind = RANT_SHAPE2D_ALIGNED_BOX; s.as.aligned_box = b; return s; }
RantShape2D rant_shape2d_from_oriented_box2d(RantOrientedBox2D b){ RantShape2D s; s.kind = RANT_SHAPE2D_ORIENTED_BOX; s.as.oriented_box = b; return s; }
RantShape2D rant_shape2d_from_circle(RantCircle c){ RantShape2D s; s.kind = RANT_SHAPE2D_CIRCLE; s.as.circle = c; return s; }
RantShape2D rant_shape2d_from_capsule2d(RantCapsule2D c){ RantShape2D s; s.kind = RANT_SHAPE2D_CAPSULE; s.as.capsule = c; return s; }
RantShape2D rant_shape2d_from_polygon2d(RantPolygon2DView p){ RantShape2D s; s.kind = RANT_SHAPE2D_POLYGON; s.as.polygon = p; return s; }
RantShape2D rant_shape2d_from_segment2d(RantSegment2D x){ RantShape2D s; s.kind = RANT_SHAPE2D_SEGMENT; s.as.segment = x; return s; }
RantShape2D rant_shape2d_from_polyline2d(RantPolyline2DView l){ RantShape2D s; s.kind = RANT_SHAPE2D_POLYLINE; s.as.polyline = l; return s; }

static RantSurfacePoint2D toward2(RantDouble2 on, RantDouble2 point){
    RantSurfacePoint2D r;
    r.point = on;
    r.normal = rant_double2_normalize(rant_double2_subtract(point, on));
    return r;
}

double rant_shape2d_distance(const RantShape2D *s, RantDouble2 point){
    if (!s) return HUGE_VAL;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:  return rant_aligned_box2d_distance(s->as.aligned_box, point);
    case RANT_SHAPE2D_ORIENTED_BOX: return rant_oriented_box2d_distance(s->as.oriented_box, point);
    case RANT_SHAPE2D_CIRCLE:       return rant_circle_distance(s->as.circle, point);
    case RANT_SHAPE2D_CAPSULE:      return rant_capsule2d_distance(s->as.capsule, point);
    case RANT_SHAPE2D_POLYGON:      return rant_polygon2d_distance(s->as.polygon, point);
    case RANT_SHAPE2D_SEGMENT:      return rant_segment2d_distance(s->as.segment, point);
    case RANT_SHAPE2D_POLYLINE:     return rant_polyline2d_distance(s->as.polyline, point);
    }
    return HUGE_VAL;
}

RantSurfacePoint2D rant_shape2d_closest(const RantShape2D *s, RantDouble2 point){
    RantSurfacePoint2D none;
    none.point = point;
    none.normal = rant_double2(0, 0);
    if (!s) return none;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:  return rant_aligned_box2d_closest(s->as.aligned_box, point);
    case RANT_SHAPE2D_ORIENTED_BOX: return rant_oriented_box2d_closest(s->as.oriented_box, point);
    case RANT_SHAPE2D_CIRCLE:       return rant_circle_closest(s->as.circle, point);
    case RANT_SHAPE2D_CAPSULE:      return rant_capsule2d_closest(s->as.capsule, point);
    case RANT_SHAPE2D_POLYGON:      return rant_polygon2d_closest(s->as.polygon, point);
    case RANT_SHAPE2D_SEGMENT:      return toward2(rant_segment2d_closest(s->as.segment, point), point);
    case RANT_SHAPE2D_POLYLINE:
        return s->as.polyline.points && s->as.polyline.count ? toward2(rant_polyline2d_closest(s->as.polyline, point).point, point) : none;
    }
    return none;
}

int rant_shape2d_contains(const RantShape2D *s, RantDouble2 point){
    if (!s) return 0;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:  return rant_aligned_box2d_contains(s->as.aligned_box, point);
    case RANT_SHAPE2D_ORIENTED_BOX: return rant_oriented_box2d_contains(s->as.oriented_box, point);
    case RANT_SHAPE2D_CIRCLE:       return rant_circle_contains(s->as.circle, point);
    case RANT_SHAPE2D_CAPSULE:      return rant_capsule2d_contains(s->as.capsule, point);
    case RANT_SHAPE2D_POLYGON:      return rant_polygon2d_contains(s->as.polygon, point);
    case RANT_SHAPE2D_SEGMENT: case RANT_SHAPE2D_POLYLINE: return 0;
    }
    return 0;
}

int rant_shape2d_raycast(const RantShape2D *s, RantRay2D ray, RantRayHit2D *hit){
    if (!s) return 0;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:  return rant_aligned_box2d_raycast(s->as.aligned_box, ray, hit);
    case RANT_SHAPE2D_ORIENTED_BOX: return rant_oriented_box2d_raycast(s->as.oriented_box, ray, hit);
    case RANT_SHAPE2D_CIRCLE:       return rant_circle_raycast(s->as.circle, ray, hit);
    case RANT_SHAPE2D_CAPSULE:      return rant_capsule2d_raycast(s->as.capsule, ray, hit);
    case RANT_SHAPE2D_POLYGON:      return rant_polygon2d_raycast(s->as.polygon, ray, hit);
    case RANT_SHAPE2D_SEGMENT:      return rant_segment2d_raycast(s->as.segment, ray, hit);
    case RANT_SHAPE2D_POLYLINE: {
        RantPolyline2DView l = s->as.polyline;
        RantRayHit2D best, h;
        uint32_t i;
        int met = 0;
        memset(&best, 0, sizeof best);
        for (i = 0; l.points && i + 1 < l.count; i++){
            RantSegment2D seg;
            seg.a = l.points[i]; seg.b = l.points[i + 1];
            if (rant_segment2d_raycast(seg, ray, &h) && (!met || h.distance < best.distance)){ best = h; met = 1; }
        }
        if (met && hit) *hit = best;
        return met;
    }
    }
    return 0;
}

RantAlignedBox2D rant_shape2d_bounds(const RantShape2D *s){
    RantAlignedBox2D empty;
    empty.min = rant_double2(HUGE_VAL, HUGE_VAL);
    empty.max = rant_double2(-HUGE_VAL, -HUGE_VAL);
    if (!s) return empty;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:  return s->as.aligned_box;
    case RANT_SHAPE2D_ORIENTED_BOX: return rant_oriented_box2d_bounds(s->as.oriented_box);
    case RANT_SHAPE2D_CIRCLE:       return rant_circle_bounds(s->as.circle);
    case RANT_SHAPE2D_CAPSULE:      return rant_capsule2d_bounds(s->as.capsule);
    case RANT_SHAPE2D_POLYGON:      return rant_polygon2d_bounds(s->as.polygon);
    case RANT_SHAPE2D_SEGMENT:      return rant_segment2d_bounds(s->as.segment);
    case RANT_SHAPE2D_POLYLINE:     return rant_polyline2d_bounds(s->as.polyline);
    }
    return empty;
}

void rant_shape2d_distances(const RantShape2D *s, const RantDouble2 *points, double *out, uint32_t count){
    uint32_t i;
    if (!points || !out) return;
    for (i = 0; i < count; i++) out[i] = rant_shape2d_distance(s, points[i]);
}

/* A 2D shape's pieces are those of its flat 3D twin on z = 0, whose gaps are the 2D ones. */
static RantMathResult pieces2d_of(const RantShape2D *s, i_Pieces *out){
    RantShape t;
    RantDouble3 *lifted;
    uint32_t i, n;
    RantMathResult r;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX: {
        RantAlignedBox2D b = s->as.aligned_box;
        RantPose at = rant_pose_identity();
        at.position = i_math_lift(rant_aligned_box2d_centroid(b));
        out->list = &out->one; out->count = 1;
        out->one = box_piece(at, rant_double3(0.5 * (b.max.x - b.min.x), 0.5 * (b.max.y - b.min.y), 0.0));
        return RANT_MATH_OK;
    }
    case RANT_SHAPE2D_ORIENTED_BOX:
        out->list = &out->one; out->count = 1;
        out->one = box_piece(rant_pose_from_pose2d(s->as.oriented_box.pose),
                             rant_double3(0.5 * fabs(s->as.oriented_box.size.x), 0.5 * fabs(s->as.oriented_box.size.y), 0.0));
        return RANT_MATH_OK;
    case RANT_SHAPE2D_CIRCLE: {
        RantSphere b;
        b.center = i_math_lift(s->as.circle.center); b.radius = s->as.circle.radius;
        t = rant_shape_from_sphere(b);
        return pieces_of(&t, out);
    }
    case RANT_SHAPE2D_CAPSULE: {
        RantCapsule c;
        c.axis.a = i_math_lift(s->as.capsule.axis.a); c.axis.b = i_math_lift(s->as.capsule.axis.b); c.radius = s->as.capsule.radius;
        t = rant_shape_from_capsule(c);
        return pieces_of(&t, out);
    }
    case RANT_SHAPE2D_SEGMENT: {
        RantSegment g;
        g.a = i_math_lift(s->as.segment.a); g.b = i_math_lift(s->as.segment.b);
        t = rant_shape_from_segment(g);
        return pieces_of(&t, out);
    }
    case RANT_SHAPE2D_POLYGON:
        return polygon_pieces(s->as.polygon, rant_pose_identity(), 0, 0.0, 0.0, out);
    case RANT_SHAPE2D_POLYLINE:
        n = s->as.polyline.points ? s->as.polyline.count : 0;
        if (!n) return RANT_MATH_ERR_ARGUMENT;
        lifted = (RantDouble3 *)malloc(sizeof(RantDouble3) * n);
        if (!lifted) return RANT_MATH_ERR_MEMORY;
        for (i = 0; i < n; i++) lifted[i] = i_math_lift(s->as.polyline.points[i]);
        r = line_pieces(lifted, n, out);
        free(lifted);
        return r;
    }
    return RANT_MATH_ERR_ARGUMENT;
}

RantMathResult rant_shape2d_gap(const RantShape2D *a, const RantShape2D *b, double *gap, RantSegment2D *between){
    i_Pieces pa, pb;
    RantDouble3 on_a, on_b;
    RantMathResult r;
    double g;
    if (!a || !b || !gap) return RANT_MATH_ERR_ARGUMENT;
    if ((r = pieces2d_of(a, &pa)) != RANT_MATH_OK) return r;
    if ((r = pieces2d_of(b, &pb)) != RANT_MATH_OK){ pieces_free(&pa); return r; }
    g = pieces_gap(&pa, &pb, &on_a, &on_b);
    pieces_free(&pa);
    pieces_free(&pb);
    *gap = g;
    if (between){ between->a = i_math_flat(on_a); between->b = i_math_flat(on_b); }
    return RANT_MATH_OK;
}

int rant_shape2d_overlaps(const RantShape2D *a, const RantShape2D *b){
    double g;
    return rant_shape2d_gap(a, b, &g, NULL) == RANT_MATH_OK && g == 0.0;
}

RantMathResult rant_shape2d_sample_inside(const RantShape2D *s, uint64_t seed, RantDouble2 *out, uint32_t count){
    i_MathRng r = i_math_rng(seed);
    uint32_t k;
    if (!s || !out) return RANT_MATH_ERR_ARGUMENT;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX: {
        RantAlignedBox2D b = s->as.aligned_box;
        for (k = 0; k < count; k++){
            RantDouble2 d = draw2(&r);
            out[k] = rant_double2(b.min.x + (b.max.x - b.min.x) * d.x, b.min.y + (b.max.y - b.min.y) * d.y);
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE2D_ORIENTED_BOX: {
        RantOrientedBox2D b = s->as.oriented_box;
        for (k = 0; k < count; k++){
            RantDouble2 d = draw2(&r);
            out[k] = rant_pose2d_apply(b.pose, rant_double2(b.size.x * (d.x - 0.5), b.size.y * (d.y - 0.5)));
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE2D_CIRCLE:
        for (k = 0; k < count; k++){
            double rad = fabs(s->as.circle.radius) * sqrt(i_math_rng_unit(&r)), a = 2.0 * I_MATH_PI * i_math_rng_unit(&r);
            out[k] = rant_double2_add(s->as.circle.center, rant_double2(rad * cos(a), rad * sin(a)));
        }
        return RANT_MATH_OK;
    case RANT_SHAPE2D_CAPSULE: {
        RantCapsule2D c = s->as.capsule;
        RantAlignedBox2D b = rant_capsule2d_bounds(c);
        if (!(fabs(c.radius) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
        for (k = 0; k < count;){
            RantDouble2 d = draw2(&r);
            RantDouble2 q = rant_double2(b.min.x + (b.max.x - b.min.x) * d.x, b.min.y + (b.max.y - b.min.y) * d.y);
            if (rant_capsule2d_contains(c, q)) out[k++] = q;
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE2D_POLYGON: {
        i_Pieces p;
        RantDouble3 q;
        RantMathResult res = pieces2d_of(s, &p);
        if (res != RANT_MATH_OK) return res;
        for (k = 0; k < count && res == RANT_MATH_OK; k++){
            res = sample_pieces(&p, 0, &r, &q, 1);
            out[k] = i_math_flat(q);
        }
        pieces_free(&p);
        return res;
    }
    case RANT_SHAPE2D_SEGMENT: case RANT_SHAPE2D_POLYLINE:
        return RANT_MATH_ERR_ARGUMENT;
    }
    return RANT_MATH_ERR_ARGUMENT;
}

/* Corners round a 2D outline for the walk by length, lifted to z = 0, with the first again
 * at the end when it closes. */
static RantMathResult along2d(const RantDouble2 *p, uint32_t n, int closed, i_MathRng *r, RantDouble2 *out, uint32_t count){
    RantDouble3 *ring, *q;
    uint32_t i, m = closed ? n + 1 : n;
    RantMathResult res;
    if (!p || !n) return RANT_MATH_ERR_ARGUMENT;
    ring = (RantDouble3 *)malloc(sizeof(RantDouble3) * (m + count));
    if (!ring) return RANT_MATH_ERR_MEMORY;
    q = ring + m;
    for (i = 0; i < n; i++) ring[i] = i_math_lift(p[i]);
    if (closed) ring[n] = ring[0];
    res = along_points(ring, m, r, q, count);
    for (i = 0; i < count && res == RANT_MATH_OK; i++) out[i] = i_math_flat(q[i]);
    free(ring);
    return res;
}

RantMathResult rant_shape2d_sample_surface(const RantShape2D *s, uint64_t seed, RantDouble2 *out, uint32_t count){
    i_MathRng r = i_math_rng(seed);
    RantDouble2 c[4];
    uint32_t k;
    if (!s || !out) return RANT_MATH_ERR_ARGUMENT;
    switch (s->kind){
    case RANT_SHAPE2D_ALIGNED_BOX:
        rant_aligned_box2d_corners(s->as.aligned_box, c);
        return along2d(c, 4, 1, &r, out, count);
    case RANT_SHAPE2D_ORIENTED_BOX:
        rant_oriented_box2d_corners(s->as.oriented_box, c);
        return along2d(c, 4, 1, &r, out, count);
    case RANT_SHAPE2D_CIRCLE:
        for (k = 0; k < count; k++){
            double a = 2.0 * I_MATH_PI * i_math_rng_unit(&r);
            out[k] = rant_double2_add(s->as.circle.center, rant_double2_scale(rant_double2(cos(a), sin(a)), fabs(s->as.circle.radius)));
        }
        return RANT_MATH_OK;
    case RANT_SHAPE2D_CAPSULE: {
        /* the two sides by length, else a point of a circle on the end whose outer half it falls in */
        RantCapsule2D cap = s->as.capsule;
        RantDouble2 u = rant_double2_normalize(rant_double2_subtract(cap.axis.b, cap.axis.a)), side;
        double rad = fabs(cap.radius), len = rant_segment2d_length(cap.axis), sides = 2.0 * len, ends = 2.0 * I_MATH_PI * rad;
        if (rant_double2_length_squared(u) == 0.0) u = rant_double2(1, 0);
        side = rant_double2_perpendicular(u);
        for (k = 0; k < count; k++){
            if (i_math_rng_unit(&r) * (sides + ends) < sides){
                double t = i_math_rng_unit(&r);
                out[k] = rant_double2_add(rant_double2_lerp(cap.axis.a, cap.axis.b, i_math_rng_unit(&r)), rant_double2_scale(side, t < 0.5 ? rad : -rad));
            } else {
                double a = 2.0 * I_MATH_PI * i_math_rng_unit(&r);
                RantDouble2 v = rant_double2(cos(a), sin(a));
                out[k] = rant_double2_add(rant_double2_dot(v, u) < 0.0 ? cap.axis.a : cap.axis.b, rant_double2_scale(v, rad));
            }
        }
        return RANT_MATH_OK;
    }
    case RANT_SHAPE2D_POLYGON:
        return along2d(s->as.polygon.points, s->as.polygon.points ? s->as.polygon.count : 0, 1, &r, out, count);
    case RANT_SHAPE2D_SEGMENT:
        c[0] = s->as.segment.a; c[1] = s->as.segment.b;
        return along2d(c, 2, 0, &r, out, count);
    case RANT_SHAPE2D_POLYLINE:
        return along2d(s->as.polyline.points, s->as.polyline.points ? s->as.polyline.count : 0, 0, &r, out, count);
    }
    return RANT_MATH_ERR_ARGUMENT;
}
