#include "internal.h"

/* Rounding at a rim or a seam counts this much in or out, relative to the shape's size. */
#define I_SEAM 1e-12

/* p[i] - hi[i] or lo[i] - p[i], whichever is larger, has the right sign exactly, which
 * |p - center| - half would not. */
double i_math_box_closest(int dim, const double *lo, const double *hi, const double *p, double *point, double *normal){
    double q[3], best = -HUGE_VAL, len = 0.0;
    int i, k = 0, outside = 0;
    for (i = 0; i < dim; i++){
        q[i] = i_math_max(lo[i] - p[i], p[i] - hi[i]);
        if (q[i] > 0.0) outside = 1;
        if (q[i] > best){ best = q[i]; k = i; }
    }
    if (outside){
        for (i = 0; i < dim; i++){
            point[i] = i_math_clamp(p[i], lo[i], hi[i]);
            normal[i] = p[i] - point[i];
            len += normal[i] * normal[i];
        }
        len = sqrt(len);
        if (len > 0.0){
            for (i = 0; i < dim; i++) normal[i] /= len;
            return len;
        }
    }
    for (i = 0; i < dim; i++){ point[i] = p[i]; normal[i] = 0.0; }
    if (p[k] - hi[k] >= lo[k] - p[k]){ point[k] = hi[k]; normal[k] = 1.0; }
    else { point[k] = lo[k]; normal[k] = -1.0; }
    return best;
}

/* The slab method: the ray is inside the box between its last entry and its first exit. */
int i_math_box_raycast(int dim, const double *lo, const double *hi, const double *o, const double *d,
                       double *t, double *normal){
    double enter = -HUGE_VAL, leave = HUGE_VAL;
    int i, enter_k = -1, leave_k = -1;
    for (i = 0; i < dim; i++){
        double t1, t2;
        if (d[i] == 0.0){
            if (o[i] < lo[i] || o[i] > hi[i]) return 0;
            continue;
        }
        t1 = (lo[i] - o[i]) / d[i];
        t2 = (hi[i] - o[i]) / d[i];
        if (t1 > t2){ double s = t1; t1 = t2; t2 = s; }
        if (t1 > enter){ enter = t1; enter_k = i; }
        if (t2 < leave){ leave = t2; leave_k = i; }
    }
    if (enter_k < 0 || enter > leave || leave < 0.0) return 0;
    for (i = 0; i < dim; i++) normal[i] = 0.0;
    if (enter >= 0.0){
        *t = enter;
        normal[enter_k] = d[enter_k] > 0.0 ? -1.0 : 1.0;
    } else {
        *t = leave;
        normal[leave_k] = d[leave_k] > 0.0 ? 1.0 : -1.0;
    }
    return 1;
}

void i_math_box_grow(int dim, double *lo, double *hi, double margin){
    int i;
    for (i = 0; i < dim; i++){
        double c = 0.5 * (lo[i] + hi[i]);
        lo[i] -= margin;
        hi[i] += margin;
        if (lo[i] > hi[i]) lo[i] = hi[i] = c;
    }
}

/* The roots of a t^2 + 2 b t + c, the smaller first, found without losing digits. A ray that
 * just grazes, such as one through a cone's tip, can round to a slightly negative discriminant. */
static int quadratic(double a, double b, double c, double t[2]){
    double disc, q;
    if (a == 0.0){
        if (b == 0.0) return 0;
        t[0] = t[1] = -c / (2.0 * b);
        return 1;
    }
    disc = b * b - a * c;
    if (disc < 0.0 && disc > -1e-12 * (b * b + fabs(a * c))) disc = 0.0;
    if (disc < 0.0) return 0;
    q = -(b + copysign(sqrt(disc), b));
    t[0] = q / a;
    t[1] = q != 0.0 ? c / q : t[0];
    if (t[0] > t[1]){ double s = t[0]; t[0] = t[1]; t[1] = s; }
    return 2;
}

/* The nearest hit so far of a raycast that tries each part of a surface. */
typedef struct { double t; RantDouble3 normal; } i_RayHit;

static i_RayHit no_hit(void){
    i_RayHit h;
    h.t = HUGE_VAL;
    h.normal = rant_double3(0, 0, 0);
    return h;
}

static void consider(i_RayHit *h, double t, RantDouble3 normal){
    if (t >= 0.0 && t < h->t){ h->t = t; h->normal = normal; }
}

static int report(const i_RayHit *h, RantRay ray, RantDouble3 d, RantRayHit *hit){
    if (!(h->t < HUGE_VAL)) return 0;
    if (hit){
        hit->point = rant_double3_add(ray.origin, rant_double3_scale(d, h->t));
        hit->normal = rant_double3_normalize(h->normal);
        hit->distance = h->t;
    }
    return 1;
}

static RantAlignedBox box3(RantDouble3 min, RantDouble3 max){
    RantAlignedBox b;
    b.min = min; b.max = max;
    return b;
}

static RantSegment seg3(RantDouble3 a, RantDouble3 b){
    RantSegment s;
    s.a = a; s.b = b;
    return s;
}

static RantPlane plane3(RantDouble3 position, RantDouble3 normal){
    RantPlane p;
    p.position = position; p.normal = normal;
    return p;
}

static RantInertia inertia_diagonal(double mass, double ixx, double iyy, double izz){
    RantInertia i;
    memset(&i, 0, sizeof i);
    i.mass = mass;
    i.ixx = ixx; i.iyy = iyy; i.izz = izz;
    return i;
}

RantDouble3 i_math_axis(RantDouble3 a, RantDouble3 b){
    RantDouble3 u = rant_double3_normalize(rant_double3_subtract(b, a));
    return rant_double3_length_squared(u) > 0.0 ? u : rant_double3(0, 0, 1);
}

RantDouble3 i_math_side(RantDouble3 axis){
    RantDouble3 s = rant_double3(axis.y, -axis.x, 0.0);
    double l = rant_double3_length(s);
    return l > 1e-12 ? rant_double3_scale(s, 1.0 / l) : rant_double3(1, 0, 0);
}

/* A body of revolution about the unit axis up through base, given as a local inertia whose
 * own z is that axis. */
static RantInertia inertia_along(RantInertia local, RantDouble3 center, RantDouble3 up){
    RantPose p;
    p.position = center;
    p.orientation = rant_quaternion_between(rant_double3(0, 0, 1), up);
    return rant_inertia_transform(local, p);
}

/* Edges of corners laid out as solid.h says, n around the bottom then n around the top. */
static void ring_edges(const RantDouble3 *c, uint32_t n, RantSegment *out){
    uint32_t i;
    for (i = 0; i < n; i++){
        out[i] = seg3(c[i], c[(i + 1) % n]);
        out[n + i] = seg3(c[n + i], c[n + (i + 1) % n]);
        out[2 * n + i] = seg3(c[i], c[n + i]);
    }
}

static void to3(RantDouble3 v, double *a){ a[0] = v.x; a[1] = v.y; a[2] = v.z; }
static RantDouble3 of3(const double *a){ return rant_double3(a[0], a[1], a[2]); }

double rant_aligned_box_distance(RantAlignedBox b, RantDouble3 point){
    double lo[3], hi[3], p[3], c[3], n[3];
    to3(b.min, lo); to3(b.max, hi); to3(point, p);
    return i_math_box_closest(3, lo, hi, p, c, n);
}

RantSurfacePoint rant_aligned_box_closest(RantAlignedBox b, RantDouble3 point){
    double lo[3], hi[3], p[3], c[3], n[3];
    RantSurfacePoint r;
    to3(b.min, lo); to3(b.max, hi); to3(point, p);
    i_math_box_closest(3, lo, hi, p, c, n);
    r.point = of3(c);
    r.normal = of3(n);
    return r;
}

int rant_aligned_box_contains(RantAlignedBox b, RantDouble3 point){
    return point.x >= b.min.x && point.x <= b.max.x && point.y >= b.min.y && point.y <= b.max.y
        && point.z >= b.min.z && point.z <= b.max.z;
}

int rant_aligned_box_raycast(RantAlignedBox b, RantRay ray, RantRayHit *hit){
    double lo[3], hi[3], o[3], d[3], n[3], t;
    RantDouble3 dir = rant_double3_normalize(ray.direction);
    to3(b.min, lo); to3(b.max, hi); to3(ray.origin, o); to3(dir, d);
    if (!i_math_box_raycast(3, lo, hi, o, d, &t, n)) return 0;
    if (hit){
        hit->point = rant_double3_add(ray.origin, rant_double3_scale(dir, t));
        hit->normal = of3(n);
        hit->distance = t;
    }
    return 1;
}

RantDouble3 rant_aligned_box_centroid(RantAlignedBox b){ return rant_double3_lerp(b.min, b.max, 0.5); }

double rant_aligned_box_volume(RantAlignedBox b){
    RantDouble3 s = rant_double3_subtract(b.max, b.min);
    return s.x * s.y * s.z;
}

double rant_aligned_box_area(RantAlignedBox b){
    RantDouble3 s = rant_double3_subtract(b.max, b.min);
    return 2.0 * (s.x * s.y + s.y * s.z + s.z * s.x);
}

RantInertia rant_aligned_box_inertia(RantAlignedBox b, double mass){
    RantDouble3 s = rant_double3_subtract(b.max, b.min);
    RantInertia i = inertia_diagonal(mass, mass * (s.y * s.y + s.z * s.z) / 12.0,
                                     mass * (s.x * s.x + s.z * s.z) / 12.0, mass * (s.x * s.x + s.y * s.y) / 12.0);
    i.center = rant_aligned_box_centroid(b);
    return i;
}

void rant_aligned_box_corners(RantAlignedBox b, RantDouble3 out[8]){
    int i;
    if (!out) return;
    for (i = 0; i < 8; i++){
        int k = i & 3;
        out[i] = rant_double3(k == 1 || k == 2 ? b.max.x : b.min.x, k >= 2 ? b.max.y : b.min.y, i >= 4 ? b.max.z : b.min.z);
    }
}

void rant_aligned_box_edges(RantAlignedBox b, RantSegment out[12]){
    RantDouble3 c[8];
    if (!out) return;
    rant_aligned_box_corners(b, c);
    ring_edges(c, 4, out);
}

/* Set from the axes rather than from the faces, so a box of no thickness still faces out. */
void rant_aligned_box_planes(RantAlignedBox b, RantPlane out[6]){
    if (!out) return;
    out[0] = plane3(b.min, rant_double3(0, -1, 0));
    out[1] = plane3(b.max, rant_double3(1, 0, 0));
    out[2] = plane3(b.max, rant_double3(0, 1, 0));
    out[3] = plane3(b.min, rant_double3(-1, 0, 0));
    out[4] = plane3(b.min, rant_double3(0, 0, -1));
    out[5] = plane3(b.max, rant_double3(0, 0, 1));
}

RantOrientedBox rant_aligned_box_transform(RantAlignedBox b, RantPose pose){
    return rant_oriented_box_transform(rant_oriented_box_from_aligned_box(b), pose);
}

/* A convex solid as its faces, each counterclockwise seen from outside. Two boxes meet in at
 * most 12 faces, each with at most 12 corners. */
#define I_FACES 14
#define I_CORNERS 16
typedef struct { int n; RantDouble3 v[I_CORNERS]; } i_Face;
typedef struct { int n; i_Face f[I_FACES]; } i_Solid;

static double coord(RantDouble3 p, int axis){ return axis == 0 ? p.x : (axis == 1 ? p.y : p.z); }
static void set_coord(RantDouble3 *p, int axis, double v){ if (axis == 0) p->x = v; else if (axis == 1) p->y = v; else p->z = v; }

static RantDouble3 face_normal(const i_Face *f){
    RantDouble3 n = rant_double3(0.0, 0.0, 0.0);
    int k;
    for (k = 1; k + 1 < f->n; k++)
        n = rant_double3_add(n, rant_double3_cross(rant_double3_subtract(f->v[k], f->v[0]), rant_double3_subtract(f->v[k + 1], f->v[0])));
    return n;
}

/* Turns about the cap's middle, seen from outside, for the sort below. */
typedef struct { RantDouble3 p; double angle; } i_Around;
static int by_angle(const void *a, const void *b){
    double x = ((const i_Around *)a)->angle, y = ((const i_Around *)b)->angle;
    return x < y ? -1 : (x > y);
}

/* Keeps the part of s with sign times coordinate axis no more than limit, and closes the cut. A
 * corner within eps of the plane moves onto it, so faces lying on it are told apart exactly. */
static void cut(i_Solid *s, int axis, double sign, double limit, double eps){
    i_Solid out;
    i_Around cap[I_CORNERS];
    RantDouble3 mid = rant_double3(0.0, 0.0, 0.0);
    double at = sign * limit;
    int f, k, j, n = 0, u = (axis + (sign > 0.0 ? 1 : 2)) % 3, w = (axis + (sign > 0.0 ? 2 : 1)) % 3;
    out.n = 0;
    for (f = 0; f < s->n; f++){
        const i_Face *F = &s->f[f];
        i_Face G;
        double d[I_CORNERS];
        int on = 0;
        for (k = 0; k < F->n; k++){
            d[k] = sign * coord(F->v[k], axis) - limit;
            if (fabs(d[k]) <= eps){ d[k] = 0.0; on++; }
        }
        /* a face on the plane facing out is the cut itself, which the cap makes again */
        if (on == F->n && sign * coord(face_normal(F), axis) > 0.0) continue;
        G.n = 0;
        for (k = 0; k < F->n; k++){
            j = (k + 1) % F->n;
            if (d[k] <= 0.0){
                G.v[G.n] = F->v[k];
                if (d[k] == 0.0) set_coord(&G.v[G.n], axis, at);
                G.n++;
            }
            if ((d[k] < 0.0 && d[j] > 0.0) || (d[k] > 0.0 && d[j] < 0.0)){
                /* from the inside corner, so the two faces sharing the edge cut it at one point */
                int in = d[k] < 0.0 ? k : j, ex = in == k ? j : k;
                G.v[G.n] = rant_double3_lerp(F->v[in], F->v[ex], d[in] / (d[in] - d[ex]));
                set_coord(&G.v[G.n++], axis, at);
            }
        }
        if (G.n >= 3) out.f[out.n++] = G;
    }
    for (f = 0; f < out.n; f++)
        for (k = 0; k < out.f[f].n; k++){
            RantDouble3 p = out.f[f].v[k];
            if (coord(p, axis) != at) continue;
            for (j = 0; j < n && rant_double3_distance(cap[j].p, p) > eps; j++){}
            if (j == n && n < I_CORNERS){ cap[n++].p = p; mid = rant_double3_add(mid, p); }
        }
    if (n >= 3 && out.n < I_FACES){
        i_Face *C = &out.f[out.n++];
        mid = rant_double3_scale(mid, 1.0 / n);
        for (k = 0; k < n; k++)
            cap[k].angle = atan2(coord(cap[k].p, w) - coord(mid, w), coord(cap[k].p, u) - coord(mid, u));
        qsort(cap, (size_t)n, sizeof *cap, by_angle);
        C->n = n;
        for (k = 0; k < n; k++) C->v[k] = cap[k].p;
    }
    *s = out;
}

double rant_oriented_box_iou(RantOrientedBox a, RantOrientedBox b){
    static const int faces[6][4] = { {1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1} };
    RantDouble3 ha = rant_double3(0.5 * fabs(a.size.x), 0.5 * fabs(a.size.y), 0.5 * fabs(a.size.z));
    RantDouble3 hb = rant_double3(0.5 * fabs(b.size.x), 0.5 * fabs(b.size.y), 0.5 * fabs(b.size.z)), c[8];
    double lim[3], volume = 0.0, eps;
    RantPose rel;
    i_Solid s;
    int i, k;
    if (!rant_aligned_box_overlap(rant_oriented_box_bounds(a), rant_oriented_box_bounds(b), NULL)) return 0.0;
    /* b in a's own frame, where a is an aligned box about the origin to cut it by */
    rel = rant_pose_between(a.pose, b.pose);
    for (i = 0; i < 8; i++)
        c[i] = rant_pose_apply(rel, rant_double3(i & 1 ? hb.x : -hb.x, i & 2 ? hb.y : -hb.y, i & 4 ? hb.z : -hb.z));
    s.n = 6;
    for (i = 0; i < 6; i++){
        s.f[i].n = 4;
        for (k = 0; k < 4; k++) s.f[i].v[k] = c[faces[i][k]];
    }
    to3(ha, lim);
    eps = 1e-12 * (rant_double3_length(rel.position) + rant_double3_length(ha) + rant_double3_length(hb));
    for (k = 0; k < 3 && s.n; k++){
        cut(&s, k, 1.0, lim[k], eps);
        cut(&s, k, -1.0, lim[k], eps);
    }
    for (i = 0; i < s.n; i++)
        for (k = 1; k + 1 < s.f[i].n; k++)
            volume += rant_double3_dot(s.f[i].v[0], rant_double3_cross(s.f[i].v[k], s.f[i].v[k + 1]));
    return i_math_iou(volume / 6.0, 8.0 * ha.x * ha.y * ha.z, 8.0 * hb.x * hb.y * hb.z);
}

RantAlignedBox rant_aligned_box_merge(RantAlignedBox a, RantAlignedBox b){
    return box3(rant_double3_min(a.min, b.min), rant_double3_max(a.max, b.max));
}

int rant_aligned_box_overlap(RantAlignedBox a, RantAlignedBox b, RantAlignedBox *out){
    RantAlignedBox r = box3(rant_double3_max(a.min, b.min), rant_double3_min(a.max, b.max));
    if (!(r.min.x <= r.max.x && r.min.y <= r.max.y && r.min.z <= r.max.z)) return 0;
    if (out) *out = r;
    return 1;
}

RantAlignedBox rant_aligned_box_grow(RantAlignedBox b, double margin){
    double lo[3], hi[3];
    to3(b.min, lo); to3(b.max, hi);
    i_math_box_grow(3, lo, hi, margin);
    return box3(of3(lo), of3(hi));
}

double rant_aligned_box_iou(RantAlignedBox a, RantAlignedBox b){
    double both = 1.0, sa = 1.0, sb = 1.0, lo[3], hi[3], alo[3], ahi[3], blo[3], bhi[3];
    int k;
    to3(a.min, alo); to3(a.max, ahi); to3(b.min, blo); to3(b.max, bhi);
    for (k = 0; k < 3; k++){
        lo[k] = i_math_max(alo[k], blo[k]);
        hi[k] = i_math_min(ahi[k], bhi[k]);
        both *= hi[k] > lo[k] ? hi[k] - lo[k] : 0.0;
        sa *= i_math_max(0.0, ahi[k] - alo[k]);
        sb *= i_math_max(0.0, bhi[k] - blo[k]);
    }
    return i_math_iou(both, sa, sb);
}

/* An oriented box is an aligned box about the origin of its own frame. */
static RantAlignedBox local_box(RantOrientedBox b){
    RantDouble3 h = rant_double3_scale(b.size, 0.5);
    return box3(rant_double3_scale(h, -1.0), h);
}

double rant_oriented_box_distance(RantOrientedBox b, RantDouble3 point){
    return rant_aligned_box_distance(local_box(b), rant_pose_apply_inverse(b.pose, point));
}

RantSurfacePoint rant_oriented_box_closest(RantOrientedBox b, RantDouble3 point){
    RantSurfacePoint r = rant_aligned_box_closest(local_box(b), rant_pose_apply_inverse(b.pose, point));
    r.point = rant_pose_apply(b.pose, r.point);
    r.normal = rant_quaternion_rotate(b.pose.orientation, r.normal);
    return r;
}

int rant_oriented_box_contains(RantOrientedBox b, RantDouble3 point){
    return rant_aligned_box_contains(local_box(b), rant_pose_apply_inverse(b.pose, point));
}

int rant_oriented_box_raycast(RantOrientedBox b, RantRay ray, RantRayHit *hit){
    RantRay local;
    RantRayHit h;
    local.origin = rant_pose_apply_inverse(b.pose, ray.origin);
    local.direction = rant_quaternion_rotate(rant_quaternion_inverse(b.pose.orientation), ray.direction);
    if (!rant_aligned_box_raycast(local_box(b), local, &h)) return 0;
    if (hit){
        hit->point = rant_double3_add(ray.origin, rant_double3_scale(rant_double3_normalize(ray.direction), h.distance));
        hit->normal = rant_quaternion_rotate(b.pose.orientation, h.normal);
        hit->distance = h.distance;
    }
    return 1;
}

RantAlignedBox rant_oriented_box_bounds(RantOrientedBox b){
    i_MathMat3 r = i_math_mat3_from_quaternion(b.pose.orientation);
    double h[3], e[3];
    int i, j;
    h[0] = 0.5 * fabs(b.size.x); h[1] = 0.5 * fabs(b.size.y); h[2] = 0.5 * fabs(b.size.z);
    for (i = 0; i < 3; i++){
        e[i] = 0.0;
        for (j = 0; j < 3; j++) e[i] += fabs(r.m[i][j]) * h[j];
    }
    return box3(rant_double3_subtract(b.pose.position, of3(e)), rant_double3_add(b.pose.position, of3(e)));
}

double rant_oriented_box_volume(RantOrientedBox b){ return fabs(b.size.x * b.size.y * b.size.z); }
double rant_oriented_box_area(RantOrientedBox b){
    double x = fabs(b.size.x), y = fabs(b.size.y), z = fabs(b.size.z);
    return 2.0 * (x * y + y * z + z * x);
}

RantInertia rant_oriented_box_inertia(RantOrientedBox b, double mass){
    return rant_inertia_transform(rant_aligned_box_inertia(local_box(b), mass), b.pose);
}

void rant_oriented_box_corners(RantOrientedBox b, RantDouble3 out[8]){
    if (!out) return;
    rant_aligned_box_corners(local_box(b), out);
    rant_pose_apply_points(b.pose, out, out, 8);
}

void rant_oriented_box_edges(RantOrientedBox b, RantSegment out[12]){
    RantDouble3 c[8];
    if (!out) return;
    rant_oriented_box_corners(b, c);
    ring_edges(c, 4, out);
}

void rant_oriented_box_planes(RantOrientedBox b, RantPlane out[6]){
    int i;
    if (!out) return;
    rant_aligned_box_planes(local_box(b), out);
    for (i = 0; i < 6; i++) out[i] = rant_plane_transform(out[i], b.pose);
}

RantOrientedBox rant_oriented_box_transform(RantOrientedBox b, RantPose pose){
    b.pose = rant_pose_compose(pose, b.pose);
    return b;
}

RantOrientedBox rant_oriented_box_from_aligned_box(RantAlignedBox b){
    RantOrientedBox r;
    r.pose.position = rant_aligned_box_centroid(b);
    r.pose.orientation = rant_quaternion_identity();
    r.size = rant_double3_subtract(b.max, b.min);
    return r;
}

double rant_sphere_distance(RantSphere s, RantDouble3 point){
    return rant_double3_distance(s.center, point) - s.radius;
}

/* A point at the very center takes +x, which stays in the plane of a circle. */
RantSurfacePoint rant_sphere_closest(RantSphere s, RantDouble3 point){
    RantSurfacePoint r;
    RantDouble3 v = rant_double3_subtract(point, s.center);
    double d = rant_double3_length(v);
    r.normal = d > 0.0 ? rant_double3_scale(v, 1.0 / d) : rant_double3(1, 0, 0);
    r.point = rant_double3_add(s.center, rant_double3_scale(r.normal, s.radius));
    return r;
}

int rant_sphere_contains(RantSphere s, RantDouble3 point){ return rant_sphere_distance(s, point) <= 0.0; }

int rant_sphere_raycast(RantSphere s, RantRay ray, RantRayHit *hit){
    RantDouble3 d = rant_double3_normalize(ray.direction), oc = rant_double3_subtract(ray.origin, s.center);
    i_RayHit h = no_hit();
    double t[2];
    int n, i;
    if (rant_double3_length_squared(d) == 0.0 || !(s.radius > 0.0)) return 0;
    n = quadratic(1.0, rant_double3_dot(oc, d), rant_double3_dot(oc, oc) - s.radius * s.radius, t);
    for (i = 0; i < n; i++) consider(&h, t[i], rant_double3_add(oc, rant_double3_scale(d, t[i])));
    return report(&h, ray, d, hit);
}

RantAlignedBox rant_sphere_bounds(RantSphere s){
    RantDouble3 r = rant_double3(fabs(s.radius), fabs(s.radius), fabs(s.radius));
    return box3(rant_double3_subtract(s.center, r), rant_double3_add(s.center, r));
}

double rant_sphere_volume(RantSphere s){ return 4.0 / 3.0 * I_MATH_PI * s.radius * s.radius * fabs(s.radius); }
double rant_sphere_area(RantSphere s){ return 4.0 * I_MATH_PI * s.radius * s.radius; }

RantInertia rant_sphere_inertia(RantSphere s, double mass){
    double k = 0.4 * mass * s.radius * s.radius;
    RantInertia i = inertia_diagonal(mass, k, k, k);
    i.center = s.center;
    return i;
}

RantSphere rant_sphere_transform(RantSphere s, RantPose pose){
    s.center = rant_pose_apply(pose, s.center);
    return s;
}

double rant_capsule_distance(RantCapsule c, RantDouble3 point){
    return rant_segment_distance(c.axis, point) - c.radius;
}

RantSurfacePoint rant_capsule_closest(RantCapsule c, RantDouble3 point){
    RantSurfacePoint r;
    RantDouble3 on = rant_segment_closest(c.axis, point), v = rant_double3_subtract(point, on);
    double d = rant_double3_length(v);
    r.normal = d > 0.0 ? rant_double3_scale(v, 1.0 / d) : i_math_side(i_math_axis(c.axis.a, c.axis.b));
    r.point = rant_double3_add(on, rant_double3_scale(r.normal, c.radius));
    return r;
}

int rant_capsule_contains(RantCapsule c, RantDouble3 point){ return rant_capsule_distance(c, point) <= 0.0; }

/* The side's crossings count between the end centers, each end's only on its outer half. */
int rant_capsule_raycast(RantCapsule c, RantRay ray, RantRayHit *hit){
    RantDouble3 d = rant_double3_normalize(ray.direction), ba = rant_double3_subtract(c.axis.b, c.axis.a);
    double length = rant_double3_length(ba), r = c.radius, t[2];
    i_RayHit h = no_hit();
    int n, i, end;
    if (rant_double3_length_squared(d) == 0.0 || !(r > 0.0)) return 0;
    if (length > 0.0){
        RantDouble3 u = rant_double3_scale(ba, 1.0 / length), oa = rant_double3_subtract(ray.origin, c.axis.a);
        RantDouble3 op = rant_double3_subtract(oa, rant_double3_scale(u, rant_double3_dot(u, oa)));
        RantDouble3 dp = rant_double3_subtract(d, rant_double3_scale(u, rant_double3_dot(u, d)));
        n = quadratic(rant_double3_dot(dp, dp), rant_double3_dot(op, dp), rant_double3_dot(op, op) - r * r, t);
        for (i = 0; i < n; i++){
            double along = rant_double3_dot(u, rant_double3_add(oa, rant_double3_scale(d, t[i])));
            if (along >= 0.0 && along <= length) consider(&h, t[i], rant_double3_add(op, rant_double3_scale(dp, t[i])));
        }
    }
    for (end = 0; end < 2; end++){
        RantDouble3 e = end ? c.axis.b : c.axis.a, oc = rant_double3_subtract(ray.origin, e);
        n = quadratic(1.0, rant_double3_dot(oc, d), rant_double3_dot(oc, oc) - r * r, t);
        for (i = 0; i < n; i++){
            RantDouble3 v = rant_double3_add(oc, rant_double3_scale(d, t[i]));
            double out = rant_double3_dot(v, ba);
            if (length == 0.0 || (end ? out >= 0.0 : out <= 0.0)) consider(&h, t[i], v);
        }
    }
    return report(&h, ray, d, hit);
}

RantAlignedBox rant_capsule_bounds(RantCapsule c){
    RantAlignedBox b = rant_segment_bounds(c.axis);
    RantDouble3 r = rant_double3(fabs(c.radius), fabs(c.radius), fabs(c.radius));
    return box3(rant_double3_subtract(b.min, r), rant_double3_add(b.max, r));
}

RantDouble3 rant_capsule_centroid(RantCapsule c){ return rant_double3_lerp(c.axis.a, c.axis.b, 0.5); }

double rant_capsule_volume(RantCapsule c){
    double r = fabs(c.radius);
    return I_MATH_PI * r * r * (rant_segment_length(c.axis) + 4.0 / 3.0 * r);
}

double rant_capsule_area(RantCapsule c){
    double r = fabs(c.radius);
    return 2.0 * I_MATH_PI * r * (rant_segment_length(c.axis) + 2.0 * r);
}

/* The side and the two half balls by their share of the volume. A half ball's center sits
 * 3r/8 from its flat face and it holds 83/320 m r^2 about a level axis through that center. */
RantInertia rant_capsule_inertia(RantCapsule c, double mass){
    double r = fabs(c.radius), l = rant_segment_length(c.axis);
    double side = r * r * l, ends = 4.0 / 3.0 * r * r * r, total = side + ends;
    double ms = total > 0.0 ? mass * side / total : 0.0, me = total > 0.0 ? mass * ends / total : mass;
    double lift = 0.5 * l + 0.375 * r;
    double level = ms * (3.0 * r * r + l * l) / 12.0 + me * (83.0 / 320.0 * r * r + lift * lift);
    double spin = 0.5 * ms * r * r + 0.4 * me * r * r;
    return inertia_along(inertia_diagonal(mass, level, level, spin), rant_capsule_centroid(c), i_math_axis(c.axis.a, c.axis.b));
}

RantCapsule rant_capsule_transform(RantCapsule c, RantPose pose){
    c.axis = rant_segment_transform(c.axis, pose);
    return c;
}

/* A body of revolution seen in the half plane through its axis and a point: x is the
 * distance from the axis, y the height above the base. side gets the way out from the axis. */
static RantDouble2 meridian(RantDouble3 base, RantDouble3 up, RantDouble3 point, RantDouble3 *side){
    RantDouble3 v = rant_double3_subtract(point, base);
    double z = rant_double3_dot(v, up);
    RantDouble3 radial = rant_double3_subtract(v, rant_double3_scale(up, z));
    double rho = rant_double3_length(radial);
    *side = rho > 0.0 ? rant_double3_scale(radial, 1.0 / rho) : i_math_side(up);
    return rant_double2(rho, z);
}

/* The nearest point of an outline running counterclockwise in that half plane from the axis
 * round to the axis, and the signed distance. The axis itself is not surface. */
static double outline_closest(const RantDouble2 *line, uint32_t n, RantDouble2 q, int inside, RantDouble2 *point, RantDouble2 *normal){
    double d = i_math_edges_closest(line, n, 0, q, 1.0, inside, point, normal);
    return inside ? -d : d;
}

static RantSurfacePoint revolve(RantDouble3 base, RantDouble3 up, RantDouble3 side, RantDouble2 point, RantDouble2 normal){
    RantSurfacePoint r;
    r.point = rant_double3_add(base, rant_double3_add(rant_double3_scale(side, point.x), rant_double3_scale(up, point.y)));
    r.normal = rant_double3_normalize(rant_double3_add(rant_double3_scale(side, normal.x), rant_double3_scale(up, normal.y)));
    return r;
}

static double cylinder_closest(RantCylinder c, RantDouble3 point, RantSurfacePoint *out){
    RantDouble3 up = i_math_axis(c.axis.a, c.axis.b), side;
    double h = rant_segment_length(c.axis), r = fabs(c.radius), d;
    RantDouble2 q = meridian(c.axis.a, up, point, &side), line[4], at, n;
    line[0] = rant_double2(0, 0); line[1] = rant_double2(r, 0); line[2] = rant_double2(r, h); line[3] = rant_double2(0, h);
    d = outline_closest(line, 4, q, q.y >= 0.0 && q.y <= h && q.x <= r, &at, &n);
    if (out) *out = revolve(c.axis.a, up, side, at, n);
    return d;
}

double rant_cylinder_distance(RantCylinder c, RantDouble3 point){ return cylinder_closest(c, point, NULL); }

RantSurfacePoint rant_cylinder_closest(RantCylinder c, RantDouble3 point){
    RantSurfacePoint r;
    cylinder_closest(c, point, &r);
    return r;
}

int rant_cylinder_contains(RantCylinder c, RantDouble3 point){ return cylinder_closest(c, point, NULL) <= 0.0; }

int rant_cylinder_raycast(RantCylinder c, RantRay ray, RantRayHit *hit){
    RantDouble3 d = rant_double3_normalize(ray.direction), up = i_math_axis(c.axis.a, c.axis.b);
    RantDouble3 oa = rant_double3_subtract(ray.origin, c.axis.a);
    double h = rant_segment_length(c.axis), r = fabs(c.radius), oz = rant_double3_dot(oa, up), dz = rant_double3_dot(d, up);
    double seam = I_SEAM * (h > r ? h : r), t[2];
    RantDouble3 op = rant_double3_subtract(oa, rant_double3_scale(up, oz)), dp = rant_double3_subtract(d, rant_double3_scale(up, dz));
    i_RayHit hh = no_hit();
    int n, i;
    if (rant_double3_length_squared(d) == 0.0 || !(r > 0.0)) return 0;
    n = quadratic(rant_double3_dot(dp, dp), rant_double3_dot(op, dp), rant_double3_dot(op, op) - r * r, t);
    for (i = 0; i < n; i++){
        double z = oz + t[i] * dz;
        if (z >= -seam && z <= h + seam) consider(&hh, t[i], rant_double3_add(op, rant_double3_scale(dp, t[i])));
    }
    if (dz != 0.0){
        for (i = 0; i < 2; i++){
            double tc = ((i ? h : 0.0) - oz) / dz;
            RantDouble3 at = rant_double3_add(op, rant_double3_scale(dp, tc));
            if (rant_double3_length(at) <= r + seam) consider(&hh, tc, rant_double3_scale(up, i ? 1.0 : -1.0));
        }
    }
    return report(&hh, ray, d, hit);
}

static RantDouble3 disk_reach(RantDouble3 up, double r){
    return rant_double3(r * sqrt(i_math_max(0.0, 1.0 - up.x * up.x)), r * sqrt(i_math_max(0.0, 1.0 - up.y * up.y)),
                        r * sqrt(i_math_max(0.0, 1.0 - up.z * up.z)));
}

RantAlignedBox rant_cylinder_bounds(RantCylinder c){
    RantDouble3 e = disk_reach(i_math_axis(c.axis.a, c.axis.b), fabs(c.radius));
    RantAlignedBox b = rant_segment_bounds(c.axis);
    return box3(rant_double3_subtract(b.min, e), rant_double3_add(b.max, e));
}

RantDouble3 rant_cylinder_centroid(RantCylinder c){ return rant_double3_lerp(c.axis.a, c.axis.b, 0.5); }

double rant_cylinder_volume(RantCylinder c){ return I_MATH_PI * c.radius * c.radius * rant_segment_length(c.axis); }

double rant_cylinder_area(RantCylinder c){
    double r = fabs(c.radius);
    return 2.0 * I_MATH_PI * r * (r + rant_segment_length(c.axis));
}

RantInertia rant_cylinder_inertia(RantCylinder c, double mass){
    double r = c.radius, h = rant_segment_length(c.axis), level = mass * (3.0 * r * r + h * h) / 12.0;
    return inertia_along(inertia_diagonal(mass, level, level, 0.5 * mass * r * r), rant_cylinder_centroid(c),
                         i_math_axis(c.axis.a, c.axis.b));
}

RantCylinder rant_cylinder_transform(RantCylinder c, RantPose pose){
    c.axis = rant_segment_transform(c.axis, pose);
    return c;
}

static double cone_closest(RantCone c, RantDouble3 point, RantSurfacePoint *out){
    RantDouble3 up = i_math_axis(c.base, c.tip), side;
    double h = rant_double3_distance(c.base, c.tip), r = fabs(c.radius), d;
    RantDouble2 q = meridian(c.base, up, point, &side), line[3], at, n;
    line[0] = rant_double2(0, 0); line[1] = rant_double2(r, 0); line[2] = rant_double2(0, h);
    d = outline_closest(line, 3, q, q.y >= 0.0 && q.y <= h && q.x * h <= r * (h - q.y), &at, &n);
    if (out) *out = revolve(c.base, up, side, at, n);
    return d;
}

double rant_cone_distance(RantCone c, RantDouble3 point){ return cone_closest(c, point, NULL); }

RantSurfacePoint rant_cone_closest(RantCone c, RantDouble3 point){
    RantSurfacePoint r;
    cone_closest(c, point, &r);
    return r;
}

int rant_cone_contains(RantCone c, RantDouble3 point){ return cone_closest(c, point, NULL) <= 0.0; }

/* The side is where the distance from the axis is r (h - z) / h, a quadratic in t. */
int rant_cone_raycast(RantCone c, RantRay ray, RantRayHit *hit){
    RantDouble3 d = rant_double3_normalize(ray.direction), up = i_math_axis(c.base, c.tip);
    RantDouble3 oa = rant_double3_subtract(ray.origin, c.base);
    double h = rant_double3_distance(c.base, c.tip), r = fabs(c.radius), oz = rant_double3_dot(oa, up), dz = rant_double3_dot(d, up);
    double seam = I_SEAM * (h > r ? h : r), t[2];
    RantDouble3 op = rant_double3_subtract(oa, rant_double3_scale(up, oz)), dp = rant_double3_subtract(d, rant_double3_scale(up, dz));
    i_RayHit hh = no_hit();
    int n, i;
    if (rant_double3_length_squared(d) == 0.0 || !(r > 0.0)) return 0;
    if (h > 0.0){
        double k = r / h, w = h - oz;
        n = quadratic(rant_double3_dot(dp, dp) - k * k * dz * dz, rant_double3_dot(op, dp) + k * k * w * dz,
                      rant_double3_dot(op, op) - k * k * w * w, t);
        for (i = 0; i < n; i++){
            double z = oz + t[i] * dz;
            RantDouble3 radial = rant_double3_normalize(rant_double3_add(op, rant_double3_scale(dp, t[i])));
            if (rant_double3_length_squared(radial) == 0.0) radial = i_math_side(up);
            if (z >= -seam && z <= h + seam)
                consider(&hh, t[i], rant_double3_add(rant_double3_scale(radial, h), rant_double3_scale(up, r)));
        }
    }
    if (dz != 0.0){
        double tc = -oz / dz;
        if (rant_double3_length(rant_double3_add(op, rant_double3_scale(dp, tc))) <= r + seam)
            consider(&hh, tc, rant_double3_scale(up, -1.0));
    }
    return report(&hh, ray, d, hit);
}

RantAlignedBox rant_cone_bounds(RantCone c){
    RantDouble3 e = disk_reach(i_math_axis(c.base, c.tip), fabs(c.radius));
    return box3(rant_double3_min(rant_double3_subtract(c.base, e), c.tip), rant_double3_max(rant_double3_add(c.base, e), c.tip));
}

RantDouble3 rant_cone_centroid(RantCone c){ return rant_double3_lerp(c.base, c.tip, 0.25); }

double rant_cone_volume(RantCone c){ return I_MATH_PI * c.radius * c.radius * rant_double3_distance(c.base, c.tip) / 3.0; }

double rant_cone_area(RantCone c){
    double r = fabs(c.radius), h = rant_double3_distance(c.base, c.tip);
    return I_MATH_PI * r * (r + sqrt(r * r + h * h));
}

RantInertia rant_cone_inertia(RantCone c, double mass){
    double r = c.radius, h = rant_double3_distance(c.base, c.tip);
    double level = mass * (3.0 / 20.0 * r * r + 3.0 / 80.0 * h * h);
    return inertia_along(inertia_diagonal(mass, level, level, 0.3 * mass * r * r), rant_cone_centroid(c), i_math_axis(c.base, c.tip));
}

RantCone rant_cone_transform(RantCone c, RantPose pose){
    c.base = rant_pose_apply(pose, c.base);
    c.tip = rant_pose_apply(pose, c.tip);
    return c;
}

/* A prism spans z from lo to hi in its own frame. */
static void prism_span(RantPrismView p, double *lo, double *hi){
    *lo = i_math_min(0.0, p.height);
    *hi = i_math_max(0.0, p.height);
}

static uint32_t base_count(RantPrismView p){ return p.base.points ? p.base.count : 0; }

static int base_edge(RantPrismView p, uint32_t i, RantDouble2 *a, RantDouble2 *b){
    *a = p.base.points[i];
    *b = p.base.points[i + 1 == p.base.count ? 0 : i + 1];
    return a->x != b->x || a->y != b->y;
}

/* The extrusion of the base's 2D distance: the side, a cap, or the rim between them. */
static double prism_closest(RantPrismView p, RantDouble3 point, RantSurfacePoint *out){
    RantDouble3 q = rant_pose_apply_inverse(p.pose, point), at, n;
    RantSurfacePoint2D s;
    double lo, hi, d2, dz, d;
    if (!base_count(p)){
        if (out){ out->point = point; out->normal = rant_double3(0, 0, 0); }
        return HUGE_VAL;
    }
    prism_span(p, &lo, &hi);
    s = rant_polygon2d_closest(p.base, i_math_flat(q));
    d2 = rant_double2_dot(rant_double2_subtract(i_math_flat(q), s.point), s.normal);
    dz = i_math_max(lo - q.z, q.z - hi);
    if (d2 > 0.0 && dz > 0.0){
        at = rant_double3(s.point.x, s.point.y, i_math_clamp(q.z, lo, hi));
        d = rant_double3_distance(q, at);
        n = rant_double3_scale(rant_double3_subtract(q, at), 1.0 / d);
    } else if (d2 > 0.0 || (dz <= 0.0 && d2 >= dz)){
        at = rant_double3(s.point.x, s.point.y, q.z);
        n = rant_double3(s.normal.x, s.normal.y, 0.0);
        d = d2;
    } else {
        int top = q.z - hi >= lo - q.z;
        at = rant_double3(q.x, q.y, top ? hi : lo);
        n = rant_double3(0, 0, top ? 1.0 : -1.0);
        d = dz;
    }
    if (out){
        out->point = rant_pose_apply(p.pose, at);
        out->normal = rant_quaternion_rotate(p.pose.orientation, n);
    }
    return d;
}

double rant_prism_distance(RantPrismView p, RantDouble3 point){ return prism_closest(p, point, NULL); }

RantSurfacePoint rant_prism_closest(RantPrismView p, RantDouble3 point){
    RantSurfacePoint r;
    prism_closest(p, point, &r);
    return r;
}

int rant_prism_contains(RantPrismView p, RantDouble3 point){ return prism_closest(p, point, NULL) <= 0.0; }

int rant_prism_raycast(RantPrismView p, RantRay ray, RantRayHit *hit){
    RantDouble3 dw = rant_double3_normalize(ray.direction);
    RantDouble3 o = rant_pose_apply_inverse(p.pose, ray.origin);
    RantDouble3 d = rant_quaternion_rotate(rant_quaternion_inverse(p.pose.orientation), dw);
    RantDouble2 dxy = i_math_flat(d);
    double lo, hi, sign = rant_polygon2d_signed_area(p.base) < 0.0 ? -1.0 : 1.0, seam;
    uint32_t n = base_count(p), i;
    i_RayHit h = no_hit();
    if (!n || rant_double3_length_squared(dw) == 0.0) return 0;
    prism_span(p, &lo, &hi);
    seam = I_SEAM * (1.0 + hi - lo);
    if (d.z != 0.0){
        for (i = 0; i < 2; i++){
            double tc = ((i ? hi : lo) - o.z) / d.z;
            RantDouble2 at = rant_double2_add(i_math_flat(o), rant_double2_scale(dxy, tc));
            if (rant_polygon2d_contains(p.base, at)) consider(&h, tc, rant_double3(0, 0, i ? 1.0 : -1.0));
        }
    }
    for (i = 0; i < n; i++){
        RantDouble2 a, b, e, w;
        double denom, t, u, z;
        if (!base_edge(p, i, &a, &b)) continue;
        e = rant_double2_subtract(b, a);
        w = rant_double2_subtract(a, i_math_flat(o));
        denom = rant_double2_cross(dxy, e);
        if (!(fabs(denom) > I_MATH_TINY * rant_double2_length(e))) continue;
        t = rant_double2_cross(w, e) / denom;
        u = rant_double2_cross(w, dxy) / denom;
        z = o.z + t * d.z;
        if (u >= -1e-12 && u <= 1.0 + 1e-12 && z >= lo - seam && z <= hi + seam)
            consider(&h, t, rant_double3(sign * e.y, -sign * e.x, 0.0));
    }
    if (!(h.t < HUGE_VAL)) return 0;
    h.normal = rant_quaternion_rotate(p.pose.orientation, h.normal);
    return report(&h, ray, dw, hit);
}

RantAlignedBox rant_prism_bounds(RantPrismView p){
    RantAlignedBox b = box3(rant_double3(HUGE_VAL, HUGE_VAL, HUGE_VAL), rant_double3(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL));
    uint32_t n = base_count(p), i;
    int k;
    for (i = 0; i < n; i++)
        for (k = 0; k < 2; k++){
            RantDouble3 c = rant_pose_apply(p.pose, rant_double3(p.base.points[i].x, p.base.points[i].y, k ? p.height : 0.0));
            b.min = rant_double3_min(b.min, c);
            b.max = rant_double3_max(b.max, c);
        }
    return b;
}

RantDouble3 rant_prism_centroid(RantPrismView p){
    RantDouble2 c = rant_polygon2d_centroid(p.base);
    return rant_pose_apply(p.pose, rant_double3(c.x, c.y, 0.5 * p.height));
}

double rant_prism_volume(RantPrismView p){ return rant_polygon2d_area(p.base) * fabs(p.height); }

double rant_prism_area(RantPrismView p){
    return 2.0 * rant_polygon2d_area(p.base) + rant_polygon2d_perimeter(p.base) * fabs(p.height);
}

/* The base's second moments about its centroid from its edges, then the height's share. */
RantInertia rant_prism_inertia(RantPrismView p, double mass){
    RantDouble2 c = rant_polygon2d_centroid(p.base);
    double area = rant_polygon2d_signed_area(p.base), h = fabs(p.height), sxx = 0.0, syy = 0.0, sxy = 0.0, k;
    uint32_t n = base_count(p), i;
    RantInertia local = inertia_diagonal(mass, 0.0, 0.0, 0.0);
    local.center = rant_double3(c.x, c.y, 0.5 * p.height);
    if (area != 0.0){
        for (i = 0; i < n; i++){
            RantDouble2 a, b;
            double cr;
            base_edge(p, i, &a, &b);
            a = rant_double2_subtract(a, c);
            b = rant_double2_subtract(b, c);
            cr = rant_double2_cross(a, b);
            sxx += (a.x * a.x + a.x * b.x + b.x * b.x) * cr;
            syy += (a.y * a.y + a.y * b.y + b.y * b.y) * cr;
            sxy += (a.x * b.y + 2.0 * a.x * a.y + 2.0 * b.x * b.y + b.x * a.y) * cr;
        }
        k = mass / area;
        sxx *= k / 12.0;
        syy *= k / 12.0;
        sxy *= k / 24.0;
        local.ixx = syy + mass * h * h / 12.0;
        local.iyy = sxx + mass * h * h / 12.0;
        local.izz = sxx + syy;
        local.ixy = -sxy;
    }
    return rant_inertia_transform(local, p.pose);
}

static uint32_t prism_edges(RantPrismView p){
    uint32_t n = base_count(p), i, k = 0;
    RantDouble2 a, b;
    for (i = 0; i < n; i++) k += (uint32_t)base_edge(p, i, &a, &b);
    return k;
}

/* The bottom corner at the start of each edge, then the top ones. */
static void prism_corner_list(RantPrismView p, uint32_t m, RantDouble3 *out){
    uint32_t n = base_count(p), i, k = 0;
    RantDouble2 a, b;
    for (i = 0; i < n; i++){
        if (!base_edge(p, i, &a, &b)) continue;
        out[k] = rant_pose_apply(p.pose, rant_double3(a.x, a.y, 0.0));
        out[m + k] = rant_pose_apply(p.pose, rant_double3(a.x, a.y, p.height));
        k++;
    }
}

RantMathResult rant_prism_corners(RantPrismView p, RantDouble3 *out, uint32_t *count){
    uint32_t m = prism_edges(p);
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, 2 * m, out)) != RANT_MATH_OK) return r;
    prism_corner_list(p, m, out);
    return RANT_MATH_OK;
}

RantMathResult rant_prism_edges(RantPrismView p, RantSegment *out, uint32_t *count){
    uint32_t m = prism_edges(p);
    RantDouble3 *c;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, 3 * m, out)) != RANT_MATH_OK || !m) return r;
    c = (RantDouble3 *)malloc(sizeof(RantDouble3) * 2 * m);
    if (!c) return RANT_MATH_ERR_MEMORY;
    prism_corner_list(p, m, c);
    ring_edges(c, m, out);
    free(c);
    return RANT_MATH_OK;
}

RantMathResult rant_prism_planes(RantPrismView p, RantPlane *out, uint32_t *count){
    uint32_t m = prism_edges(p), n = base_count(p), i, k = 0;
    double lo, hi, sign = rant_polygon2d_signed_area(p.base) < 0.0 ? -1.0 : 1.0;
    RantDouble2 a, b, first = rant_double2(0, 0);
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, m ? m + 2 : 0, out)) != RANT_MATH_OK || !m) return r;
    prism_span(p, &lo, &hi);
    for (i = 0; i < n; i++){
        RantDouble2 e;
        if (!base_edge(p, i, &a, &b)) continue;
        if (!k) first = a;
        e = rant_double2_normalize(rant_double2_subtract(b, a));
        out[k++] = rant_plane_transform(plane3(rant_double3(a.x, a.y, 0.0), rant_double3(sign * e.y, -sign * e.x, 0.0)), p.pose);
    }
    out[k++] = rant_plane_transform(plane3(rant_double3(first.x, first.y, lo), rant_double3(0, 0, -1)), p.pose);
    out[k] = rant_plane_transform(plane3(rant_double3(first.x, first.y, hi), rant_double3(0, 0, 1)), p.pose);
    return RANT_MATH_OK;
}

RantPrismView rant_prism_transform(RantPrismView p, RantPose pose){
    p.pose = rant_pose_compose(pose, p.pose);
    return p;
}
