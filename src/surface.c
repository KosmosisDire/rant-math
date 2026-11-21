#include "internal.h"

/* The projection onto the face when it falls inside, else the nearest of the three edges.
 * Region tests on dot products lose their digits on thin triangles, which GJK makes. */
void i_math_triangle_closest(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantDouble3 p, double w[3]){
    RantDouble3 ab = rant_double3_subtract(b, a), ac = rant_double3_subtract(c, a), ap = rant_double3_subtract(p, a);
    RantDouble3 n = rant_double3_cross(ab, ac), corner[3];
    double nn = rant_double3_dot(n, n), best = HUGE_VAL;
    int i;
    if (nn > 1e-24 * rant_double3_dot(ab, ab) * rant_double3_dot(ac, ac)){
        double u = rant_double3_dot(rant_double3_cross(ap, ac), n) / nn, v = rant_double3_dot(rant_double3_cross(ab, ap), n) / nn;
        if (u >= 0.0 && v >= 0.0 && u + v <= 1.0){
            w[0] = 1.0 - u - v; w[1] = u; w[2] = v;
            return;
        }
    }
    corner[0] = a; corner[1] = b; corner[2] = c;
    w[0] = 1.0; w[1] = w[2] = 0.0;
    for (i = 0; i < 3; i++){
        RantDouble3 s0 = corner[i], e = rant_double3_subtract(corner[(i + 1) % 3], s0);
        double ee = rant_double3_dot(e, e);
        double t = ee > 0.0 ? i_math_clamp(rant_double3_dot(rant_double3_subtract(p, s0), e) / ee, 0.0, 1.0) : 0.0;
        double d = rant_double3_distance(rant_double3_add(s0, rant_double3_scale(e, t)), p);
        if (d < best){
            best = d;
            w[0] = w[1] = w[2] = 0.0;
            w[i] = 1.0 - t;
            w[(i + 1) % 3] = t;
        }
    }
}

static RantDouble3 weighted(RantDouble3 a, RantDouble3 b, RantDouble3 c, const double w[3]){
    return rant_double3_add(rant_double3_add(rant_double3_scale(a, w[0]), rant_double3_scale(b, w[1])), rant_double3_scale(c, w[2]));
}

/* Moller and Trumbore: the distance along the unit direction to where the ray crosses the
 * triangle from either side, edges counted in so a ray through a shared edge never slips by. */
static int triangle_raycast(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantDouble3 o, RantDouble3 d, double *t){
    RantDouble3 e1 = rant_double3_subtract(b, a), e2 = rant_double3_subtract(c, a), pv = rant_double3_cross(d, e2), tv, qv;
    double det = rant_double3_dot(e1, pv), u, v;
    if (!(fabs(det) > I_MATH_TINY * rant_double3_length(e1) * rant_double3_length(e2))) return 0;
    tv = rant_double3_subtract(o, a);
    u = rant_double3_dot(tv, pv) / det;
    if (u < -1e-12 || u > 1.0 + 1e-12) return 0;
    qv = rant_double3_cross(tv, e1);
    v = rant_double3_dot(d, qv) / det;
    if (v < -1e-12 || u + v > 1.0 + 1e-12) return 0;
    *t = rant_double3_dot(e2, qv) / det;
    return *t >= 0.0;
}

static uint32_t corners_of(RantPolygonView p){ return p.points ? p.count : 0; }
static RantDouble3 next_of(RantPolygonView p, uint32_t i){ return p.points[i + 1 == p.count ? 0 : i + 1]; }

/* Newell's normal, twice the area long, measured from the first corner. */
static RantDouble3 newell(RantPolygonView p){
    RantDouble3 n = rant_double3(0, 0, 0);
    uint32_t i, k = corners_of(p);
    for (i = 1; i + 1 < k; i++)
        n = rant_double3_add(n, rant_double3_cross(rant_double3_subtract(p.points[i], p.points[0]),
                                                   rant_double3_subtract(p.points[i + 1], p.points[0])));
    return n;
}

double rant_polygon_perimeter(RantPolygonView p){
    uint32_t i, k = corners_of(p);
    double s = 0.0;
    for (i = 0; i < k; i++) s += rant_double3_distance(p.points[i], next_of(p, i));
    return s;
}

RantMathResult rant_polygon_plane(RantPolygonView p, RantPlane *out){
    RantDouble3 n = newell(p), mean = rant_double3(0, 0, 0);
    double edge = rant_polygon_perimeter(p);
    uint32_t i, k = corners_of(p);
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(rant_double3_length(n) > I_MATH_TINY * edge * edge)) return RANT_MATH_ERR_DEGENERATE;
    for (i = 0; i < k; i++) mean = rant_double3_add(mean, p.points[i]);
    out->position = rant_double3_scale(mean, 1.0 / k);
    out->normal = rant_double3_normalize(n);
    return RANT_MATH_OK;
}

static RantMathResult polygon_frame(RantPolygonView p, RantPose *frame){
    RantPlane plane;
    RantMathResult r = rant_polygon_plane(p, &plane);
    uint32_t i;
    if (r != RANT_MATH_OK) return r;
    frame->position = rant_plane_closest(plane, p.points[0]);
    for (i = 1; i < p.count; i++){
        RantDouble3 x = rant_double3_subtract(p.points[i], p.points[0]);
        x = rant_double3_subtract(x, rant_double3_scale(plane.normal, rant_double3_dot(plane.normal, x)));
        if (rant_double3_length(x) > 0.0){
            frame->orientation = rant_quaternion_look(x, plane.normal);
            return RANT_MATH_OK;
        }
    }
    return RANT_MATH_ERR_DEGENERATE;
}

RantMathResult rant_polygon_flatten(RantPolygonView p, RantPose *frame, RantDouble2 *out){
    RantPose f;
    RantMathResult r;
    uint32_t i;
    if (!frame) return RANT_MATH_ERR_ARGUMENT;
    if ((r = polygon_frame(p, &f)) != RANT_MATH_OK) return r;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < p.count; i++) out[i] = i_math_flat(rant_pose_apply_inverse(f, p.points[i]));
    *frame = f;
    return RANT_MATH_OK;
}

/* A polygon flattened for the 2D calls, on the stack when it is small. ok is 0 when it has no
 * plane, or no memory. */
typedef struct { RantPose frame; RantPolygon2DView view; RantDouble2 *heap; RantDouble2 on_stack[64]; int ok; } i_Flat;

static void flat_open(RantPolygonView p, i_Flat *f){
    RantDouble2 *pts = f->on_stack;
    f->heap = NULL;
    f->ok = 0;
    if (corners_of(p) > 64){
        pts = f->heap = (RantDouble2 *)malloc(sizeof(RantDouble2) * p.count);
        if (!pts) return;
    }
    f->ok = rant_polygon_flatten(p, &f->frame, pts) == RANT_MATH_OK;
    f->view.points = pts;
    f->view.count = p.count;
}

static void flat_close(i_Flat *f){ free(f->heap); }

/* With no plane the polygon is only its edges. */
static RantSurfacePoint edges_closest(RantPolygonView p, RantDouble3 q){
    RantSurfacePoint r;
    double best = HUGE_VAL;
    uint32_t i, k = corners_of(p);
    r.point = k ? p.points[0] : q;
    r.normal = rant_double3(0, 0, 0);
    for (i = 0; i < k; i++){
        RantSegment s;
        RantDouble3 c;
        s.a = p.points[i]; s.b = next_of(p, i);
        c = rant_segment_closest(s, q);
        if (rant_double3_distance(c, q) < best){ best = rant_double3_distance(c, q); r.point = c; }
    }
    return r;
}

RantSurfacePoint rant_polygon_closest(RantPolygonView p, RantDouble3 point){
    RantSurfacePoint r;
    i_Flat f;
    flat_open(p, &f);
    if (!f.ok){
        flat_close(&f);
        return edges_closest(p, point);
    }
    {   RantDouble3 local = rant_pose_apply_inverse(f.frame, point);
        RantDouble2 q = i_math_flat(local);
        if (!rant_polygon2d_contains(f.view, q)) q = rant_polygon2d_closest(f.view, q).point;
        r.point = rant_pose_apply(f.frame, i_math_lift(q));
        r.normal = rant_quaternion_rotate(f.frame.orientation, rant_double3(0, 0, 1));
    }
    flat_close(&f);
    return r;
}

double rant_polygon_distance(RantPolygonView p, RantDouble3 point){
    if (!corners_of(p)) return HUGE_VAL;
    return rant_double3_distance(rant_polygon_closest(p, point).point, point);
}

int rant_polygon_raycast(RantPolygonView p, RantRay ray, RantRayHit *hit){
    RantPlane plane;
    RantRayHit h;
    i_Flat f;
    int met = 0;
    memset(&h, 0, sizeof h);
    flat_open(p, &f);
    if (f.ok){
        plane.position = f.frame.position;
        plane.normal = rant_quaternion_rotate(f.frame.orientation, rant_double3(0, 0, 1));
        met = rant_plane_raycast(plane, ray, &h)
              && rant_polygon2d_contains(f.view, i_math_flat(rant_pose_apply_inverse(f.frame, h.point)));
    }
    flat_close(&f);
    if (!met) return 0;
    if (hit) *hit = h;
    return 1;
}

RantAlignedBox rant_polygon_bounds(RantPolygonView p){
    RantPolylineView l;
    l.points = p.points;
    l.count = p.count;
    return rant_polyline_bounds(l);
}

RantDouble3 rant_polygon_centroid(RantPolygonView p){
    RantDouble3 c = rant_double3(0, 0, 0);
    uint32_t i, k = corners_of(p);
    i_Flat f;
    flat_open(p, &f);
    if (f.ok) c = rant_pose_apply(f.frame, i_math_lift(rant_polygon2d_centroid(f.view)));
    else if (k){
        for (i = 0; i < k; i++) c = rant_double3_add(c, p.points[i]);
        c = rant_double3_scale(c, 1.0 / k);
    }
    flat_close(&f);
    return c;
}

double rant_polygon_area(RantPolygonView p){ return 0.5 * rant_double3_length(newell(p)); }

RantMathResult rant_polygon_edges(RantPolygonView p, RantSegment *out, uint32_t *count){
    uint32_t i, k = corners_of(p), need = 0, w = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < k; i++) need += (uint32_t)(rant_double3_distance(p.points[i], next_of(p, i)) > 0.0);
    if ((r = i_math_room(count, need, out)) != RANT_MATH_OK) return r;
    for (i = 0; i < k; i++){
        if (!(rant_double3_distance(p.points[i], next_of(p, i)) > 0.0)) continue;
        out[w].a = p.points[i];
        out[w++].b = next_of(p, i);
    }
    return RANT_MATH_OK;
}

static uint32_t triangles_of(RantMeshView m){ return m.vertices && m.triangles ? m.triangle_count : 0; }

static int triangle_of(RantMeshView m, uint32_t t, RantDouble3 *a, RantDouble3 *b, RantDouble3 *c){
    const uint32_t *i = m.triangles + 3 * (size_t)t;
    if (i[0] >= m.vertex_count || i[1] >= m.vertex_count || i[2] >= m.vertex_count) return 0;
    *a = m.vertices[i[0]];
    *b = m.vertices[i[1]];
    *c = m.vertices[i[2]];
    return 1;
}

RantMathResult rant_mesh_check(RantMeshView m){
    uint32_t t;
    if ((m.vertex_count && !m.vertices) || (m.triangle_count && !m.triangles)) return RANT_MATH_ERR_ARGUMENT;
    for (t = 0; t < m.triangle_count; t++){
        const uint32_t *i = m.triangles + 3 * (size_t)t;
        if (i[0] >= m.vertex_count || i[1] >= m.vertex_count || i[2] >= m.vertex_count) return RANT_MATH_ERR_ARGUMENT;
    }
    return RANT_MATH_OK;
}

RantSurfacePoint rant_mesh_closest(RantMeshView m, RantDouble3 point){
    RantSurfacePoint r;
    double best = HUGE_VAL;
    uint32_t t, n = triangles_of(m);
    r.point = point;
    r.normal = rant_double3(0, 0, 0);
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c, q, normal;
        double w[3], d;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        i_math_triangle_closest(a, b, c, point, w);
        q = weighted(a, b, c, w);
        d = rant_double3_distance(q, point);
        normal = rant_double3_normalize(rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a)));
        if (d < best || (d == best && rant_double3_length_squared(r.normal) == 0.0)){
            best = d;
            r.point = q;
            r.normal = normal;
        }
    }
    return r;
}

double rant_mesh_distance(RantMeshView m, RantDouble3 point){
    uint32_t t, n = triangles_of(m);
    double best = HUGE_VAL;
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c;
        double w[3], d;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        i_math_triangle_closest(a, b, c, point, w);
        d = rant_double3_distance(weighted(a, b, c, w), point);
        if (d < best) best = d;
    }
    return best;
}

/* The winding number: the solid angle each triangle fills seen from the point, by Van
 * Oosterom and Strackee, added up over a whole turn. */
int rant_mesh_contains(RantMeshView m, RantDouble3 point){
    uint32_t t, n = triangles_of(m);
    double total = 0.0;
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c;
        double la, lb, lc;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        a = rant_double3_subtract(a, point);
        b = rant_double3_subtract(b, point);
        c = rant_double3_subtract(c, point);
        la = rant_double3_length(a); lb = rant_double3_length(b); lc = rant_double3_length(c);
        total += 2.0 * atan2(rant_double3_dot(a, rant_double3_cross(b, c)),
                             la * lb * lc + rant_double3_dot(a, b) * lc + rant_double3_dot(a, c) * lb + rant_double3_dot(b, c) * la);
    }
    return total >= 2.0 * I_MATH_PI;
}

int rant_mesh_raycast(RantMeshView m, RantRay ray, RantRayHit *hit){
    RantDouble3 d = rant_double3_normalize(ray.direction), normal = rant_double3(0, 0, 0);
    double best = HUGE_VAL;
    uint32_t t, n = triangles_of(m);
    if (rant_double3_length_squared(d) == 0.0) return 0;
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c;
        double at;
        if (!triangle_of(m, t, &a, &b, &c) || !triangle_raycast(a, b, c, ray.origin, d, &at) || !(at < best)) continue;
        best = at;
        normal = rant_double3_normalize(rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a)));
    }
    if (!(best < HUGE_VAL)) return 0;
    if (hit){
        hit->point = rant_double3_add(ray.origin, rant_double3_scale(d, best));
        hit->normal = normal;
        hit->distance = best;
    }
    return 1;
}

RantAlignedBox rant_mesh_bounds(RantMeshView m){
    RantPolylineView l;
    l.points = m.vertices;
    l.count = m.vertex_count;
    return rant_polyline_bounds(l);
}

double rant_mesh_area(RantMeshView m){
    uint32_t t, n = triangles_of(m);
    double s = 0.0;
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c;
        if (triangle_of(m, t, &a, &b, &c))
            s += 0.5 * rant_double3_length(rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a)));
    }
    return s;
}

/* Volume, the sum of x dV and of x x^T dV about o, from the signed tetrahedra o a b c. One with a
 * corner at the origin holds v / 20 (a a^T + b b^T + c c^T + s s^T) of the last, s = a + b + c. */
typedef struct { double volume; RantDouble3 first; double second[3][3]; RantDouble3 o; } i_Moments;

static i_Moments moments(RantMeshView m){
    i_Moments k;
    uint32_t t, n = triangles_of(m);
    memset(&k, 0, sizeof k);
    if (m.vertices && m.vertex_count) k.o = m.vertices[0];
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c, s;
        double v, p[4][3];
        int i, j, q;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        a = rant_double3_subtract(a, k.o);
        b = rant_double3_subtract(b, k.o);
        c = rant_double3_subtract(c, k.o);
        s = rant_double3_add(rant_double3_add(a, b), c);
        v = rant_double3_dot(a, rant_double3_cross(b, c)) / 6.0;
        k.volume += v;
        k.first = rant_double3_add(k.first, rant_double3_scale(s, v / 4.0));
        p[0][0] = a.x; p[0][1] = a.y; p[0][2] = a.z;
        p[1][0] = b.x; p[1][1] = b.y; p[1][2] = b.z;
        p[2][0] = c.x; p[2][1] = c.y; p[2][2] = c.z;
        p[3][0] = s.x; p[3][1] = s.y; p[3][2] = s.z;
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++){
                double sum = 0.0;
                for (q = 0; q < 4; q++) sum += p[q][i] * p[q][j];
                k.second[i][j] += v / 20.0 * sum;
            }
    }
    return k;
}

static int holds_volume(RantMeshView m, double volume){
    RantAlignedBox b = rant_mesh_bounds(m);
    double reach = b.min.x <= b.max.x ? rant_double3_distance(b.min, b.max) : 0.0;
    return fabs(volume) > I_MATH_TINY * reach * reach * reach;
}

double rant_mesh_volume(RantMeshView m){ return moments(m).volume; }

RantDouble3 rant_mesh_centroid(RantMeshView m){
    i_Moments k = moments(m);
    RantDouble3 sum = rant_double3(0, 0, 0);
    double area = 0.0;
    uint32_t t, n = triangles_of(m);
    if (holds_volume(m, k.volume)) return rant_double3_add(k.o, rant_double3_scale(k.first, 1.0 / k.volume));
    for (t = 0; t < n; t++){
        RantDouble3 a, b, c;
        double s;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        s = 0.5 * rant_double3_length(rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a)));
        sum = rant_double3_add(sum, rant_double3_scale(rant_double3_add(rant_double3_add(a, b), c), s / 3.0));
        area += s;
    }
    return area > 0.0 ? rant_double3_scale(sum, 1.0 / area) : k.o;
}

/* An inside out mesh has a negative volume and density, which cancel. */
RantInertia rant_mesh_inertia(RantMeshView m, double mass){
    i_Moments k = moments(m);
    RantInertia r;
    memset(&r, 0, sizeof r);
    r.mass = mass;
    r.center = rant_mesh_centroid(m);
    if (holds_volume(m, k.volume)){
        RantDouble3 c = rant_double3_scale(k.first, 1.0 / k.volume);
        double cc[3], ks = mass / k.volume, t[3][3], trace;
        int i, j;
        cc[0] = c.x; cc[1] = c.y; cc[2] = c.z;
        for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) t[i][j] = k.second[i][j] - k.volume * cc[i] * cc[j];
        trace = t[0][0] + t[1][1] + t[2][2];
        r.ixx = ks * (trace - t[0][0]); r.iyy = ks * (trace - t[1][1]); r.izz = ks * (trace - t[2][2]);
        r.ixy = -ks * t[0][1]; r.ixz = -ks * t[0][2]; r.iyz = -ks * t[1][2];
    }
    return r;
}

RantMathResult rant_mesh_normals(RantMeshView m, RantDouble3 *normals){
    uint32_t i, t;
    if ((m.vertex_count && (!m.vertices || !normals)) || (m.triangle_count && !m.triangles)) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < m.vertex_count; i++) normals[i] = rant_double3(0.0, 0.0, 0.0);
    for (t = 0; t < m.triangle_count; t++){
        const uint32_t *k = m.triangles + 3 * (size_t)t;
        RantDouble3 a, b, c, n;
        int j;
        if (!triangle_of(m, t, &a, &b, &c)) continue;
        /* twice the area along the face's normal, so a larger face counts for more */
        n = rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a));
        for (j = 0; j < 3; j++) normals[k[j]] = rant_double3_add(normals[k[j]], n);
    }
    for (i = 0; i < m.vertex_count; i++){
        double l = rant_double3_length(normals[i]);
        normals[i] = l > 0.0 ? rant_double3_scale(normals[i], 1.0 / l) : rant_double3(NAN, NAN, NAN);
    }
    return RANT_MATH_OK;
}
