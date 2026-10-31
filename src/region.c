#include "internal.h"

/* A circle and a 2D capsule are the cut of a sphere and a capsule through their middle, so
 * those answer on z = 0. */
static RantSphere sphere_of(RantCircle c){
    RantSphere s;
    s.center = i_math_lift(c.center);
    s.radius = c.radius;
    return s;
}

static RantCapsule capsule_of(RantCapsule2D c){
    RantCapsule s;
    s.axis.a = i_math_lift(c.axis.a);
    s.axis.b = i_math_lift(c.axis.b);
    s.radius = c.radius;
    return s;
}

static RantRay ray_of(RantRay2D r){
    RantRay s;
    s.origin = i_math_lift(r.origin);
    s.direction = i_math_lift(r.direction);
    return s;
}

static RantSurfacePoint2D flat_surface(RantSurfacePoint s){
    RantSurfacePoint2D r;
    r.point = i_math_flat(s.point);
    r.normal = i_math_flat(s.normal);
    return r;
}

static void flat_hit(RantRayHit h, RantRayHit2D *hit){
    if (!hit) return;
    hit->point = i_math_flat(h.point);
    hit->normal = i_math_flat(h.normal);
    hit->distance = h.distance;
}

static RantAlignedBox2D box2(RantDouble2 min, RantDouble2 max){
    RantAlignedBox2D b;
    b.min = min; b.max = max;
    return b;
}

static RantSegment2D seg2(RantDouble2 a, RantDouble2 b){
    RantSegment2D s;
    s.a = a; s.b = b;
    return s;
}

static RantPlane2D plane2(RantDouble2 position, RantDouble2 normal){
    RantPlane2D p;
    p.position = position; p.normal = normal;
    return p;
}

double rant_aligned_box2d_distance(RantAlignedBox2D b, RantDouble2 point){
    double lo[2], hi[2], p[2], c[2], n[2];
    lo[0] = b.min.x; lo[1] = b.min.y; hi[0] = b.max.x; hi[1] = b.max.y; p[0] = point.x; p[1] = point.y;
    return i_math_box_closest(2, lo, hi, p, c, n);
}

RantSurfacePoint2D rant_aligned_box2d_closest(RantAlignedBox2D b, RantDouble2 point){
    double lo[2], hi[2], p[2], c[2], n[2];
    RantSurfacePoint2D r;
    lo[0] = b.min.x; lo[1] = b.min.y; hi[0] = b.max.x; hi[1] = b.max.y; p[0] = point.x; p[1] = point.y;
    i_math_box_closest(2, lo, hi, p, c, n);
    r.point = rant_double2(c[0], c[1]);
    r.normal = rant_double2(n[0], n[1]);
    return r;
}

int rant_aligned_box2d_contains(RantAlignedBox2D b, RantDouble2 point){
    return point.x >= b.min.x && point.x <= b.max.x && point.y >= b.min.y && point.y <= b.max.y;
}

int rant_aligned_box2d_raycast(RantAlignedBox2D b, RantRay2D ray, RantRayHit2D *hit){
    double lo[2], hi[2], o[2], d[2], n[2], t;
    RantDouble2 dir = rant_double2_normalize(ray.direction);
    lo[0] = b.min.x; lo[1] = b.min.y; hi[0] = b.max.x; hi[1] = b.max.y;
    o[0] = ray.origin.x; o[1] = ray.origin.y; d[0] = dir.x; d[1] = dir.y;
    if (!i_math_box_raycast(2, lo, hi, o, d, &t, n)) return 0;
    if (hit){
        hit->point = rant_double2_add(ray.origin, rant_double2_scale(dir, t));
        hit->normal = rant_double2(n[0], n[1]);
        hit->distance = t;
    }
    return 1;
}

RantDouble2 rant_aligned_box2d_centroid(RantAlignedBox2D b){ return rant_double2_lerp(b.min, b.max, 0.5); }
double rant_aligned_box2d_area(RantAlignedBox2D b){ return (b.max.x - b.min.x) * (b.max.y - b.min.y); }
double rant_aligned_box2d_perimeter(RantAlignedBox2D b){ return 2.0 * ((b.max.x - b.min.x) + (b.max.y - b.min.y)); }

void rant_aligned_box2d_corners(RantAlignedBox2D b, RantDouble2 out[4]){
    if (!out) return;
    out[0] = b.min;
    out[1] = rant_double2(b.max.x, b.min.y);
    out[2] = b.max;
    out[3] = rant_double2(b.min.x, b.max.y);
}

void rant_aligned_box2d_edges(RantAlignedBox2D b, RantSegment2D out[4]){
    RantDouble2 c[4];
    int i;
    if (!out) return;
    rant_aligned_box2d_corners(b, c);
    for (i = 0; i < 4; i++) out[i] = seg2(c[i], c[(i + 1) % 4]);
}

/* Set from the axes rather than from the edges, so a box of no width still faces out. */
void rant_aligned_box2d_planes(RantAlignedBox2D b, RantPlane2D out[4]){
    RantDouble2 c[4];
    if (!out) return;
    rant_aligned_box2d_corners(b, c);
    out[0] = plane2(c[0], rant_double2(0, -1));
    out[1] = plane2(c[1], rant_double2(1, 0));
    out[2] = plane2(c[2], rant_double2(0, 1));
    out[3] = plane2(c[3], rant_double2(-1, 0));
}

RantOrientedBox2D rant_aligned_box2d_transform(RantAlignedBox2D b, RantPose2D pose){
    return rant_oriented_box2d_transform(rant_oriented_box2d_from_aligned_box2d(b), pose);
}

RantAlignedBox2D rant_aligned_box2d_merge(RantAlignedBox2D a, RantAlignedBox2D b){
    return box2(rant_double2_min(a.min, b.min), rant_double2_max(a.max, b.max));
}

int rant_aligned_box2d_overlap(RantAlignedBox2D a, RantAlignedBox2D b, RantAlignedBox2D *out){
    RantAlignedBox2D r = box2(rant_double2_max(a.min, b.min), rant_double2_min(a.max, b.max));
    if (!(r.min.x <= r.max.x && r.min.y <= r.max.y)) return 0;
    if (out) *out = r;
    return 1;
}

RantAlignedBox2D rant_aligned_box2d_grow(RantAlignedBox2D b, double margin){
    double lo[2], hi[2];
    lo[0] = b.min.x; lo[1] = b.min.y; hi[0] = b.max.x; hi[1] = b.max.y;
    i_math_box_grow(2, lo, hi, margin);
    return box2(rant_double2(lo[0], lo[1]), rant_double2(hi[0], hi[1]));
}

double rant_aligned_box2d_iou(RantAlignedBox2D a, RantAlignedBox2D b){
    double w = i_math_min(a.max.x, b.max.x) - i_math_max(a.min.x, b.min.x);
    double h = i_math_min(a.max.y, b.max.y) - i_math_max(a.min.y, b.min.y);
    double sa = i_math_max(0.0, a.max.x - a.min.x) * i_math_max(0.0, a.max.y - a.min.y);
    double sb = i_math_max(0.0, b.max.x - b.min.x) * i_math_max(0.0, b.max.y - b.min.y);
    return i_math_iou(w > 0.0 && h > 0.0 ? w * h : 0.0, sa, sb);
}

/* An oriented box is an aligned box about the origin of its own frame. */
static RantAlignedBox2D local_box2(RantOrientedBox2D b){
    RantDouble2 h = rant_double2(0.5 * fabs(b.size.x), 0.5 * fabs(b.size.y));
    return box2(rant_double2_scale(h, -1.0), h);
}

double rant_oriented_box2d_distance(RantOrientedBox2D b, RantDouble2 point){
    return rant_aligned_box2d_distance(local_box2(b), rant_pose2d_apply_inverse(b.pose, point));
}

RantSurfacePoint2D rant_oriented_box2d_closest(RantOrientedBox2D b, RantDouble2 point){
    RantSurfacePoint2D r = rant_aligned_box2d_closest(local_box2(b), rant_pose2d_apply_inverse(b.pose, point));
    r.point = rant_pose2d_apply(b.pose, r.point);
    r.normal = rant_double2_rotate(r.normal, b.pose.angle);
    return r;
}

int rant_oriented_box2d_contains(RantOrientedBox2D b, RantDouble2 point){
    return rant_aligned_box2d_contains(local_box2(b), rant_pose2d_apply_inverse(b.pose, point));
}

int rant_oriented_box2d_raycast(RantOrientedBox2D b, RantRay2D ray, RantRayHit2D *hit){
    RantRay2D local;
    RantRayHit2D h;
    local.origin = rant_pose2d_apply_inverse(b.pose, ray.origin);
    local.direction = rant_double2_rotate(ray.direction, -b.pose.angle);
    if (!rant_aligned_box2d_raycast(local_box2(b), local, &h)) return 0;
    if (hit){
        hit->point = rant_pose2d_apply(b.pose, h.point);
        hit->normal = rant_double2_rotate(h.normal, b.pose.angle);
        hit->distance = h.distance;
    }
    return 1;
}

RantAlignedBox2D rant_oriented_box2d_bounds(RantOrientedBox2D b){
    double c = fabs(cos(b.pose.angle)), s = fabs(sin(b.pose.angle));
    double hx = 0.5 * fabs(b.size.x), hy = 0.5 * fabs(b.size.y);
    RantDouble2 e = rant_double2(c * hx + s * hy, s * hx + c * hy);
    return box2(rant_double2_subtract(b.pose.position, e), rant_double2_add(b.pose.position, e));
}

double rant_oriented_box2d_area(RantOrientedBox2D b){ return fabs(b.size.x * b.size.y); }
double rant_oriented_box2d_perimeter(RantOrientedBox2D b){ return 2.0 * (fabs(b.size.x) + fabs(b.size.y)); }

void rant_oriented_box2d_corners(RantOrientedBox2D b, RantDouble2 out[4]){
    if (!out) return;
    rant_aligned_box2d_corners(local_box2(b), out);
    rant_pose2d_apply_points(b.pose, out, out, 4);
}

void rant_oriented_box2d_edges(RantOrientedBox2D b, RantSegment2D out[4]){
    RantDouble2 c[4];
    int i;
    if (!out) return;
    rant_oriented_box2d_corners(b, c);
    for (i = 0; i < 4; i++) out[i] = seg2(c[i], c[(i + 1) % 4]);
}

void rant_oriented_box2d_planes(RantOrientedBox2D b, RantPlane2D out[4]){
    int i;
    if (!out) return;
    rant_aligned_box2d_planes(local_box2(b), out);
    for (i = 0; i < 4; i++) out[i] = rant_plane2d_transform(out[i], b.pose);
}

RantOrientedBox2D rant_oriented_box2d_transform(RantOrientedBox2D b, RantPose2D pose){
    b.pose = rant_pose2d_compose(pose, b.pose);
    return b;
}

RantOrientedBox2D rant_oriented_box2d_from_aligned_box2d(RantAlignedBox2D b){
    RantOrientedBox2D r;
    r.pose.position = rant_aligned_box2d_centroid(b);
    r.pose.angle = 0.0;
    r.size = rant_double2_subtract(b.max, b.min);
    return r;
}

/* The part of polygon p with sign times coordinate axis no more than limit. */
static int clip2(const RantDouble2 *p, int n, int axis, double sign, double limit, RantDouble2 *out){
    int i, m = 0;
    for (i = 0; i < n; i++){
        RantDouble2 u = p[i], v = p[(i + 1) % n];
        double du = sign * (axis ? u.y : u.x) - limit, dv = sign * (axis ? v.y : v.x) - limit;
        if (du <= 0.0) out[m++] = u;
        if ((du < 0.0 && dv > 0.0) || (du > 0.0 && dv < 0.0))
            out[m++] = rant_double2_lerp(u, v, du / (du - dv));
    }
    return m;
}

double rant_oriented_box2d_iou(RantOrientedBox2D a, RantOrientedBox2D b){
    RantDouble2 ha = rant_double2(0.5 * fabs(a.size.x), 0.5 * fabs(a.size.y));
    RantDouble2 hb = rant_double2(0.5 * fabs(b.size.x), 0.5 * fabs(b.size.y)), p[2][12];
    double area = 0.0;
    int n = 4, k, i;
    /* b in a's own frame, where a is an aligned box about the origin to clip it by */
    RantPose2D rel = rant_pose2d_between(a.pose, b.pose);
    p[0][0] = rant_double2(-hb.x, -hb.y); p[0][1] = rant_double2(hb.x, -hb.y);
    p[0][2] = hb; p[0][3] = rant_double2(-hb.x, hb.y);
    rant_pose2d_apply_points(rel, p[0], p[0], 4);
    for (k = 0; k < 4 && n; k++) n = clip2(p[k & 1], n, k >> 1, k & 1 ? -1.0 : 1.0, k >> 1 ? ha.y : ha.x, p[(k + 1) & 1]);
    for (i = 0; i < n; i++) area += rant_double2_cross(p[0][i], p[0][(i + 1) % n]);
    return i_math_iou(0.5 * fabs(area), 4.0 * ha.x * ha.y, 4.0 * hb.x * hb.y);
}

double rant_circle_distance(RantCircle c, RantDouble2 point){ return rant_sphere_distance(sphere_of(c), i_math_lift(point)); }
RantSurfacePoint2D rant_circle_closest(RantCircle c, RantDouble2 point){
    return flat_surface(rant_sphere_closest(sphere_of(c), i_math_lift(point)));
}
int rant_circle_contains(RantCircle c, RantDouble2 point){ return rant_sphere_contains(sphere_of(c), i_math_lift(point)); }
int rant_circle_raycast(RantCircle c, RantRay2D ray, RantRayHit2D *hit){
    RantRayHit h;
    if (!rant_sphere_raycast(sphere_of(c), ray_of(ray), &h)) return 0;
    flat_hit(h, hit);
    return 1;
}
RantAlignedBox2D rant_circle_bounds(RantCircle c){
    RantDouble2 r = rant_double2(fabs(c.radius), fabs(c.radius));
    return box2(rant_double2_subtract(c.center, r), rant_double2_add(c.center, r));
}
double rant_circle_area(RantCircle c){ return I_MATH_PI * c.radius * c.radius; }
double rant_circle_perimeter(RantCircle c){ return 2.0 * I_MATH_PI * fabs(c.radius); }
RantCircle rant_circle_transform(RantCircle c, RantPose2D pose){
    c.center = rant_pose2d_apply(pose, c.center);
    return c;
}

double rant_capsule2d_distance(RantCapsule2D c, RantDouble2 point){ return rant_capsule_distance(capsule_of(c), i_math_lift(point)); }
RantSurfacePoint2D rant_capsule2d_closest(RantCapsule2D c, RantDouble2 point){
    return flat_surface(rant_capsule_closest(capsule_of(c), i_math_lift(point)));
}
int rant_capsule2d_contains(RantCapsule2D c, RantDouble2 point){ return rant_capsule_contains(capsule_of(c), i_math_lift(point)); }
int rant_capsule2d_raycast(RantCapsule2D c, RantRay2D ray, RantRayHit2D *hit){
    RantRayHit h;
    if (!rant_capsule_raycast(capsule_of(c), ray_of(ray), &h)) return 0;
    flat_hit(h, hit);
    return 1;
}
RantAlignedBox2D rant_capsule2d_bounds(RantCapsule2D c){
    RantAlignedBox b = rant_capsule_bounds(capsule_of(c));
    return box2(i_math_flat(b.min), i_math_flat(b.max));
}
double rant_capsule2d_area(RantCapsule2D c){
    return I_MATH_PI * c.radius * c.radius + 2.0 * fabs(c.radius) * rant_segment2d_length(c.axis);
}
double rant_capsule2d_perimeter(RantCapsule2D c){
    return 2.0 * I_MATH_PI * fabs(c.radius) + 2.0 * rant_segment2d_length(c.axis);
}
RantCapsule2D rant_capsule2d_transform(RantCapsule2D c, RantPose2D pose){
    c.axis = rant_segment2d_transform(c.axis, pose);
    return c;
}

/* The polygon's corners, with a view of no points read as none. */
static uint32_t corners2d_of(RantPolygon2DView p){ return p.points ? p.count : 0; }
static RantDouble2 next2d_of(RantPolygon2DView p, uint32_t i){ return p.points[i + 1 == p.count ? 0 : i + 1]; }

double rant_polygon2d_signed_area(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i;
    double twice = 0.0;
    if (n < 3) return 0.0;
    /* Measured from the first corner, which keeps the sum exact far from the frame origin. */
    for (i = 1; i + 1 < n; i++)
        twice += rant_double2_cross(rant_double2_subtract(p.points[i], p.points[0]),
                                    rant_double2_subtract(p.points[i + 1], p.points[0]));
    return 0.5 * twice;
}

double rant_polygon2d_area(RantPolygon2DView p){ return fabs(rant_polygon2d_signed_area(p)); }

/* 1 for counterclockwise, -1 for clockwise, so the right of an edge times this faces out. */
static double winding_sign(RantPolygon2DView p){ return rant_polygon2d_signed_area(p) < 0.0 ? -1.0 : 1.0; }

/* Sunday's winding number: how many times the edges go around the point. */
static int winding(RantPolygon2DView p, RantDouble2 q){
    uint32_t n = corners2d_of(p), i;
    int w = 0;
    for (i = 0; i < n; i++){
        RantDouble2 a = p.points[i], b = next2d_of(p, i);
        double side = rant_double2_cross(rant_double2_subtract(b, a), rant_double2_subtract(q, a));
        if (a.y <= q.y){
            if (b.y > q.y && side > 0.0) w++;
        } else if (b.y <= q.y && side < 0.0){
            w--;
        }
    }
    return w;
}

/* Along the inside of an edge the normal is the edge's own, which stays exact where rounding
 * leaves the point a hair off the boundary. Only at a corner does it follow the way to the point. */
double i_math_edges_closest(const RantDouble2 *pts, uint32_t n, int closed, RantDouble2 q, double out, int inside,
                            RantDouble2 *point, RantDouble2 *normal){
    uint32_t edges = closed ? n : n - 1, i;
    RantDouble2 right = rant_double2(0, 0);
    double best = rant_double2_distance(pts[0], q);
    int corner = 1;
    *point = pts[0];
    for (i = 0; i < edges; i++){
        RantDouble2 a = pts[i], e = rant_double2_subtract(pts[i + 1 == n ? 0 : i + 1], a), c;
        double ee = rant_double2_dot(e, e), t, d;
        if (!(ee > 0.0)) continue;
        t = i_math_clamp(rant_double2_dot(rant_double2_subtract(q, a), e) / ee, 0.0, 1.0);
        c = rant_double2_add(a, rant_double2_scale(e, t));
        d = rant_double2_distance(c, q);
        if (d < best || (d == best && right.x == 0.0 && right.y == 0.0)){
            best = d;
            *point = c;
            corner = !(t > 0.0 && t < 1.0);
            right = rant_double2_normalize(rant_double2(e.y, -e.x));
        }
    }
    if (!corner || !(best > 0.0)) *normal = rant_double2_scale(right, out);
    else *normal = rant_double2_scale(rant_double2_subtract(q, *point), (inside ? -1.0 : 1.0) / best);
    return best;
}

double rant_polygon2d_distance(RantPolygon2DView p, RantDouble2 point){
    RantSurfacePoint2D s;
    double d;
    if (!corners2d_of(p)) return HUGE_VAL;
    d = i_math_edges_closest(p.points, p.count, 1, point, 1.0, 0, &s.point, &s.normal);
    return winding(p, point) ? -d : d;
}

RantSurfacePoint2D rant_polygon2d_closest(RantPolygon2DView p, RantDouble2 point){
    RantSurfacePoint2D r;
    if (!corners2d_of(p)){
        r.point = point;
        r.normal = rant_double2(0, 0);
        return r;
    }
    i_math_edges_closest(p.points, p.count, 1, point, winding_sign(p), winding(p, point) != 0, &r.point, &r.normal);
    return r;
}

int rant_polygon2d_contains(RantPolygon2DView p, RantDouble2 point){
    return rant_polygon2d_distance(p, point) <= 0.0;
}

int rant_polygon2d_raycast(RantPolygon2DView p, RantRay2D ray, RantRayHit2D *hit){
    uint32_t n = corners2d_of(p), i;
    RantDouble2 d = rant_double2_normalize(ray.direction), right = rant_double2(0, 0);
    double best = HUGE_VAL;
    for (i = 0; i < n; i++){
        RantDouble2 a = p.points[i], e = rant_double2_subtract(next2d_of(p, i), a), w = rant_double2_subtract(a, ray.origin);
        double denom = rant_double2_cross(d, e), t, u;
        if (!(fabs(denom) > I_MATH_TINY * rant_double2_length(e))) continue;
        t = rant_double2_cross(w, e) / denom;
        u = rant_double2_cross(w, d) / denom;
        if (t >= 0.0 && t < best && u >= -1e-12 && u <= 1.0 + 1e-12){
            best = t;
            right = rant_double2_normalize(rant_double2(e.y, -e.x));
        }
    }
    if (best == HUGE_VAL) return 0;
    if (hit){
        hit->point = rant_double2_add(ray.origin, rant_double2_scale(d, best));
        hit->normal = rant_double2_scale(right, winding_sign(p));
        hit->distance = best;
    }
    return 1;
}

RantAlignedBox2D rant_polygon2d_bounds(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i;
    RantAlignedBox2D b = box2(rant_double2(HUGE_VAL, HUGE_VAL), rant_double2(-HUGE_VAL, -HUGE_VAL));
    for (i = 0; i < n; i++){
        b.min = rant_double2_min(b.min, p.points[i]);
        b.max = rant_double2_max(b.max, p.points[i]);
    }
    return b;
}

/* The center of the area, or of the corners when the polygon has none. */
RantDouble2 rant_polygon2d_centroid(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i;
    RantDouble2 o, sum = rant_double2(0, 0);
    double twice = 0.0;
    if (!n) return sum;
    o = p.points[0];
    for (i = 1; i + 1 < n; i++){
        RantDouble2 a = rant_double2_subtract(p.points[i], o), b = rant_double2_subtract(p.points[i + 1], o);
        double c = rant_double2_cross(a, b);
        twice += c;
        sum = rant_double2_add(sum, rant_double2_scale(rant_double2_add(a, b), c));
    }
    if (fabs(twice) > I_MATH_TINY * rant_polygon2d_perimeter(p) * rant_polygon2d_perimeter(p))
        return rant_double2_add(o, rant_double2_scale(sum, 1.0 / (3.0 * twice)));
    for (i = 0; i < n; i++) sum = rant_double2_add(sum, rant_double2_subtract(p.points[i], o));
    return rant_double2_add(o, rant_double2_scale(sum, 1.0 / n));
}

double rant_polygon2d_perimeter(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i;
    double s = 0.0;
    for (i = 0; i < n; i++) s += rant_double2_distance(p.points[i], next2d_of(p, i));
    return s;
}

static int is_edge(RantPolygon2DView p, uint32_t i){
    RantDouble2 a = p.points[i], b = next2d_of(p, i);
    return a.x != b.x || a.y != b.y;
}

static uint32_t edge_count(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i, k = 0;
    for (i = 0; i < n; i++) k += (uint32_t)is_edge(p, i);
    return k;
}

int rant_polygon2d_is_convex(RantPolygon2DView p){
    uint32_t n = corners2d_of(p), i, edges = 0;
    RantDouble2 prev = rant_double2(0, 0);
    double turned = 0.0;
    int sign = 0;
    for (i = n; i-- > 0;)
        if (is_edge(p, i)){ prev = rant_double2_subtract(next2d_of(p, i), p.points[i]); break; }
    for (i = 0; i < n; i++){
        RantDouble2 e;
        double c;
        if (!is_edge(p, i)) continue;
        e = rant_double2_subtract(next2d_of(p, i), p.points[i]);
        c = rant_double2_cross(prev, e);
        if (c != 0.0){
            int s = c > 0.0 ? 1 : -1;
            if (sign && s != sign) return 0;
            sign = s;
        }
        turned += atan2(c, rant_double2_dot(prev, e));
        prev = e;
        edges++;
    }
    return edges >= 3 && sign != 0 && fabs(fabs(turned) - 2.0 * I_MATH_PI) < 1e-6;
}

RantMathResult rant_polygon2d_edges(RantPolygon2DView p, RantSegment2D *out, uint32_t *count){
    uint32_t n = corners2d_of(p), i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, edge_count(p), out)) != RANT_MATH_OK) return r;
    for (i = 0; i < n; i++) if (is_edge(p, i)) out[k++] = seg2(p.points[i], next2d_of(p, i));
    return RANT_MATH_OK;
}

RantMathResult rant_polygon2d_planes(RantPolygon2DView p, RantPlane2D *out, uint32_t *count){
    uint32_t n = corners2d_of(p), i, k = 0;
    double sign = winding_sign(p);
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, edge_count(p), out)) != RANT_MATH_OK) return r;
    for (i = 0; i < n; i++){
        if (!is_edge(p, i)) continue;
        rant_plane2d_from_points(p.points[i], next2d_of(p, i), &out[k]);
        out[k].normal = rant_double2_scale(out[k].normal, sign);
        k++;
    }
    return RANT_MATH_OK;
}

/* Ear clipping over the distinct corners. When rounding leaves no ear, the corner that turns
 * most the right way goes, so it always ends with n - 2 triangles. */
RantMathResult rant_polygon2d_triangulate(RantPolygon2DView p, uint32_t *triangles, uint32_t *count){
    uint32_t n = corners2d_of(p), i, m = 0, *ring, *prev, *next, k = 0, v, left;
    double sign = winding_sign(p);
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < n; i++) m += (uint32_t)is_edge(p, i);
    if ((r = i_math_room(count, m >= 3 ? m - 2 : 0, triangles)) != RANT_MATH_OK || m < 3) return r;
    ring = (uint32_t *)malloc(sizeof(uint32_t) * m * 3);
    if (!ring) return RANT_MATH_ERR_MEMORY;
    prev = ring + m;
    next = prev + m;
    for (i = 0; i < n; i++) if (is_edge(p, i)) ring[k++] = i;
    for (i = 0; i < m; i++){
        prev[i] = (i + m - 1) % m;
        next[i] = (i + 1) % m;
    }
    v = 0;
    k = 0;
    for (left = m; left > 3; left--){
        uint32_t tries, best = v, at = v;
        double best_turn = -HUGE_VAL;
        for (tries = 0; tries < left; tries++, at = next[at]){
            RantDouble2 a = p.points[ring[prev[at]]], b = p.points[ring[at]], c = p.points[ring[next[at]]];
            double turn = sign * rant_double2_cross(rant_double2_subtract(b, a), rant_double2_subtract(c, b));
            uint32_t o;
            int clear = 1;
            if (turn > best_turn){ best_turn = turn; best = at; }
            if (!(turn > 0.0)) continue;
            for (o = next[next[at]]; o != prev[at]; o = next[o]){
                RantDouble2 q = p.points[ring[o]];
                if ((q.x == a.x && q.y == a.y) || (q.x == b.x && q.y == b.y) || (q.x == c.x && q.y == c.y)) continue;
                if (sign * rant_double2_cross(rant_double2_subtract(b, a), rant_double2_subtract(q, a)) >= 0.0
                    && sign * rant_double2_cross(rant_double2_subtract(c, b), rant_double2_subtract(q, b)) >= 0.0
                    && sign * rant_double2_cross(rant_double2_subtract(a, c), rant_double2_subtract(q, c)) >= 0.0){
                    clear = 0;
                    break;
                }
            }
            if (clear){ best = at; break; }
        }
        at = best;
        triangles[3 * k + 0] = ring[prev[at]];
        triangles[3 * k + 1] = sign > 0.0 ? ring[at] : ring[next[at]];
        triangles[3 * k + 2] = sign > 0.0 ? ring[next[at]] : ring[at];
        k++;
        next[prev[at]] = next[at];
        prev[next[at]] = prev[at];
        v = prev[at];
    }
    triangles[3 * k + 0] = ring[prev[v]];
    triangles[3 * k + 1] = sign > 0.0 ? ring[v] : ring[next[v]];
    triangles[3 * k + 2] = sign > 0.0 ? ring[next[v]] : ring[v];
    free(ring);
    return RANT_MATH_OK;
}
