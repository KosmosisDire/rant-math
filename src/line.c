#include "internal.h"

static RantSegment lift_segment(RantSegment2D s){
    RantSegment r;
    r.a = i_math_lift(s.a); r.b = i_math_lift(s.b);
    return r;
}

/* Ericson's segment to segment method, with an endless end for a ray. */
void i_math_closest_lines(RantDouble3 p1, RantDouble3 d1, double hi1,
                          RantDouble3 p2, RantDouble3 d2, double hi2, double *s_out, double *t_out){
    RantDouble3 r = rant_double3_subtract(p1, p2);
    double a = rant_double3_dot(d1, d1), e = rant_double3_dot(d2, d2), f = rant_double3_dot(d2, r);
    double s, t;
    if (a <= 0.0 && e <= 0.0){
        s = t = 0.0;
    } else if (a <= 0.0){
        s = 0.0;
        t = i_math_clamp(f / e, 0.0, hi2);
    } else {
        double c = rant_double3_dot(d1, r);
        if (e <= 0.0){
            t = 0.0;
            s = i_math_clamp(-c / a, 0.0, hi1);
        } else {
            double b = rant_double3_dot(d1, d2), denom = a * e - b * b;
            s = denom > I_MATH_TINY * a * e ? i_math_clamp((b * f - c * e) / denom, 0.0, hi1) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0){
                t = 0.0;
                s = i_math_clamp(-c / a, 0.0, hi1);
            } else if (t > hi2){
                t = hi2;
                s = i_math_clamp((b * hi2 - c) / a, 0.0, hi1);
            }
        }
    }
    *s_out = s;
    *t_out = t;
}

RantMathResult rant_plane_from_points(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantPlane *out){
    RantDouble3 u = rant_double3_subtract(b, a), v = rant_double3_subtract(c, a);
    RantDouble3 n = rant_double3_cross(u, v);
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(rant_double3_length(n) > I_MATH_TINY * rant_double3_length(u) * rant_double3_length(v)))
        return RANT_MATH_ERR_DEGENERATE;
    out->position = a;
    out->normal = rant_double3_normalize(n);
    return RANT_MATH_OK;
}

double rant_plane_distance(RantPlane p, RantDouble3 point){
    return rant_double3_dot(rant_double3_normalize(p.normal), rant_double3_subtract(point, p.position));
}

RantDouble3 rant_plane_closest(RantPlane p, RantDouble3 point){
    RantDouble3 n = rant_double3_normalize(p.normal);
    return rant_double3_subtract(point, rant_double3_scale(n, rant_double3_dot(n, rant_double3_subtract(point, p.position))));
}

int rant_plane_raycast(RantPlane p, RantRay ray, RantRayHit *hit){
    RantDouble3 n = rant_double3_normalize(p.normal), d = rant_double3_normalize(ray.direction);
    double along = rant_double3_dot(n, d), t;
    if (fabs(along) <= I_MATH_TINY) return 0;
    t = rant_double3_dot(n, rant_double3_subtract(p.position, ray.origin)) / along;
    if (!(t >= 0.0)) return 0;
    if (hit){
        hit->point = rant_double3_add(ray.origin, rant_double3_scale(d, t));
        hit->normal = n;
        hit->distance = t;
    }
    return 1;
}

RantPlane rant_plane_transform(RantPlane p, RantPose pose){
    RantPlane r;
    r.position = rant_pose_apply(pose, p.position);
    r.normal = rant_double3_normalize(rant_quaternion_rotate(pose.orientation, p.normal));
    return r;
}

/* Each row a plane's unit normal, each right side its offset, solved by Cramer's rule. */
static int solve3(RantDouble3 r0, RantDouble3 r1, RantDouble3 r2, RantDouble3 rhs, RantDouble3 *x){
    RantDouble3 c12 = rant_double3_cross(r1, r2), c20 = rant_double3_cross(r2, r0), c01 = rant_double3_cross(r0, r1);
    double det = rant_double3_dot(r0, c12);
    if (!(fabs(det) > I_MATH_TINY)) return 0;
    *x = rant_double3_scale(rant_double3_add(rant_double3_add(rant_double3_scale(c12, rhs.x),
                                                              rant_double3_scale(c20, rhs.y)),
                                             rant_double3_scale(c01, rhs.z)), 1.0 / det);
    return 1;
}

RantMathResult rant_plane_intersect(RantPlane a, RantPlane b, RantRay *out){
    RantDouble3 na = rant_double3_normalize(a.normal), nb = rant_double3_normalize(b.normal);
    RantDouble3 d = rant_double3_cross(na, nb), x;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(rant_double3_length(d) > 1e-9)) return RANT_MATH_ERR_DEGENERATE;
    d = rant_double3_normalize(d);
    if (!solve3(na, nb, d, rant_double3(rant_double3_dot(na, a.position), rant_double3_dot(nb, b.position), 0.0), &x))
        return RANT_MATH_ERR_DEGENERATE;
    out->origin = x;
    out->direction = d;
    return RANT_MATH_OK;
}

RantMathResult rant_plane_intersect3(RantPlane a, RantPlane b, RantPlane c, RantDouble3 *out){
    RantDouble3 na = rant_double3_normalize(a.normal), nb = rant_double3_normalize(b.normal);
    RantDouble3 nc = rant_double3_normalize(c.normal), x;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!solve3(na, nb, nc, rant_double3(rant_double3_dot(na, a.position), rant_double3_dot(nb, b.position),
                                         rant_double3_dot(nc, c.position)), &x))
        return RANT_MATH_ERR_DEGENERATE;
    *out = x;
    return RANT_MATH_OK;
}

double rant_segment_length(RantSegment s){ return rant_double3_distance(s.a, s.b); }

RantDouble3 rant_segment_closest(RantSegment s, RantDouble3 point){
    RantDouble3 d = rant_double3_subtract(s.b, s.a);
    double dd = rant_double3_dot(d, d), t;
    if (!(dd > 0.0)) return s.a;
    t = i_math_clamp(rant_double3_dot(rant_double3_subtract(point, s.a), d) / dd, 0.0, 1.0);
    return rant_double3_add(s.a, rant_double3_scale(d, t));
}

double rant_segment_distance(RantSegment s, RantDouble3 point){
    return rant_double3_distance(rant_segment_closest(s, point), point);
}

RantSegment rant_segment_closest_segment(RantSegment a, RantSegment b){
    RantDouble3 da = rant_double3_subtract(a.b, a.a), db = rant_double3_subtract(b.b, b.a);
    RantSegment r;
    double s, t;
    i_math_closest_lines(a.a, da, 1.0, b.a, db, 1.0, &s, &t);
    r.a = rant_double3_add(a.a, rant_double3_scale(da, s));
    r.b = rant_double3_add(b.a, rant_double3_scale(db, t));
    return r;
}

RantSegment rant_segment_transform(RantSegment s, RantPose pose){
    RantSegment r;
    r.a = rant_pose_apply(pose, s.a);
    r.b = rant_pose_apply(pose, s.b);
    return r;
}

RantAlignedBox rant_segment_bounds(RantSegment s){
    RantAlignedBox b;
    b.min = rant_double3_min(s.a, s.b);
    b.max = rant_double3_max(s.a, s.b);
    return b;
}

RantDouble3 rant_ray_at(RantRay ray, double distance){
    return rant_double3_add(ray.origin, rant_double3_scale(rant_double3_normalize(ray.direction), distance));
}

RantDouble3 rant_ray_closest(RantRay ray, RantDouble3 point){
    RantDouble3 d = rant_double3_normalize(ray.direction);
    double t = rant_double3_dot(rant_double3_subtract(point, ray.origin), d);
    return rant_double3_add(ray.origin, rant_double3_scale(d, t > 0.0 ? t : 0.0));
}

double rant_ray_distance(RantRay ray, RantDouble3 point){
    return rant_double3_distance(rant_ray_closest(ray, point), point);
}

RantSegment rant_ray_closest_ray(RantRay a, RantRay b){
    RantDouble3 da = rant_double3_normalize(a.direction), db = rant_double3_normalize(b.direction);
    RantSegment r;
    double s, t;
    i_math_closest_lines(a.origin, da, HUGE_VAL, b.origin, db, HUGE_VAL, &s, &t);
    r.a = rant_double3_add(a.origin, rant_double3_scale(da, s));
    r.b = rant_double3_add(b.origin, rant_double3_scale(db, t));
    return r;
}

RantRay rant_ray_transform(RantRay ray, RantPose pose){
    RantRay r;
    r.origin = rant_pose_apply(pose, ray.origin);
    r.direction = rant_double3_normalize(rant_quaternion_rotate(pose.orientation, ray.direction));
    return r;
}

RantMathResult rant_plane2d_from_points(RantDouble2 a, RantDouble2 b, RantPlane2D *out){
    RantDouble2 d = rant_double2_subtract(b, a);
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(rant_double2_length(d) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    out->position = a;
    out->normal = rant_double2_normalize(rant_double2(d.y, -d.x));
    return RANT_MATH_OK;
}

double rant_plane2d_distance(RantPlane2D p, RantDouble2 point){
    return rant_double2_dot(rant_double2_normalize(p.normal), rant_double2_subtract(point, p.position));
}

RantDouble2 rant_plane2d_closest(RantPlane2D p, RantDouble2 point){
    RantDouble2 n = rant_double2_normalize(p.normal);
    return rant_double2_subtract(point, rant_double2_scale(n, rant_double2_dot(n, rant_double2_subtract(point, p.position))));
}

int rant_plane2d_raycast(RantPlane2D p, RantRay2D ray, RantRayHit2D *hit){
    RantPlane p3;
    RantRay r3;
    RantRayHit h;
    p3.position = i_math_lift(p.position); p3.normal = i_math_lift(p.normal);
    r3.origin = i_math_lift(ray.origin); r3.direction = i_math_lift(ray.direction);
    if (!rant_plane_raycast(p3, r3, &h)) return 0;
    if (hit){
        hit->point = i_math_flat(h.point);
        hit->normal = i_math_flat(h.normal);
        hit->distance = h.distance;
    }
    return 1;
}

RantPlane2D rant_plane2d_transform(RantPlane2D p, RantPose2D pose){
    RantPlane2D r;
    r.position = rant_pose2d_apply(pose, p.position);
    r.normal = rant_double2_normalize(rant_double2_rotate(p.normal, pose.angle));
    return r;
}

RantMathResult rant_plane2d_intersect(RantPlane2D a, RantPlane2D b, RantDouble2 *out){
    RantDouble2 na = rant_double2_normalize(a.normal), nb = rant_double2_normalize(b.normal);
    double det = rant_double2_cross(na, nb);
    double ha = rant_double2_dot(na, a.position), hb = rant_double2_dot(nb, b.position);
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(fabs(det) > 1e-9)) return RANT_MATH_ERR_DEGENERATE;
    *out = rant_double2((ha * nb.y - hb * na.y) / det, (hb * na.x - ha * nb.x) / det);
    return RANT_MATH_OK;
}

double rant_segment2d_length(RantSegment2D s){ return rant_double2_distance(s.a, s.b); }

RantDouble2 rant_segment2d_closest(RantSegment2D s, RantDouble2 point){
    return i_math_flat(rant_segment_closest(lift_segment(s), i_math_lift(point)));
}

double rant_segment2d_distance(RantSegment2D s, RantDouble2 point){
    return rant_double2_distance(rant_segment2d_closest(s, point), point);
}

RantSegment2D rant_segment2d_closest_segment(RantSegment2D a, RantSegment2D b){
    RantSegment c = rant_segment_closest_segment(lift_segment(a), lift_segment(b));
    RantSegment2D r;
    r.a = i_math_flat(c.a); r.b = i_math_flat(c.b);
    return r;
}

/* Parameters within this of an end still count, so segments that share an end touch. */
#define I_TOUCH 1e-12

int rant_segment2d_intersect(RantSegment2D a, RantSegment2D b, RantDouble2 *point){
    RantDouble2 r = rant_double2_subtract(a.b, a.a), s = rant_double2_subtract(b.b, b.a);
    RantDouble2 w = rant_double2_subtract(b.a, a.a), at;
    double rr = rant_double2_dot(r, r), ss = rant_double2_dot(s, s), denom = rant_double2_cross(r, s);
    if (!(rr > 0.0) || !(ss > 0.0)){
        /* A segment that is one point meets the other only where it lies on it. */
        RantDouble2 p = rr > 0.0 ? b.a : a.a;
        RantSegment2D other = rr > 0.0 ? a : b;
        if (rant_segment2d_distance(other, p) > I_TOUCH * (1.0 + sqrt(rr > ss ? rr : ss))) return 0;
        at = p;
    } else if (fabs(denom) > I_TOUCH * sqrt(rr * ss)){
        double t = rant_double2_cross(w, s) / denom, u = rant_double2_cross(w, r) / denom;
        if (t < -I_TOUCH || t > 1.0 + I_TOUCH || u < -I_TOUCH || u > 1.0 + I_TOUCH) return 0;
        at = rant_double2_add(a.a, rant_double2_scale(r, i_math_clamp(t, 0.0, 1.0)));
    } else {
        double t0, t1, lo, hi;
        if (fabs(rant_double2_cross(w, r)) > I_TOUCH * rr * (1.0 + sqrt(rant_double2_dot(w, w) / rr))) return 0;
        t0 = rant_double2_dot(w, r) / rr;
        t1 = rant_double2_dot(rant_double2_subtract(b.b, a.a), r) / rr;
        lo = i_math_max(0.0, i_math_min(t0, t1));
        hi = i_math_min(1.0, i_math_max(t0, t1));
        if (lo > hi + I_TOUCH) return 0;
        at = rant_double2_add(a.a, rant_double2_scale(r, i_math_min(lo, 1.0)));
    }
    if (point) *point = at;
    return 1;
}

int rant_segment2d_raycast(RantSegment2D s, RantRay2D ray, RantRayHit2D *hit){
    RantDouble2 d = rant_double2_normalize(ray.direction), e = rant_double2_subtract(s.b, s.a);
    RantDouble2 w = rant_double2_subtract(s.a, ray.origin), n;
    double denom = rant_double2_cross(d, e), t, u;
    if (!(fabs(denom) > I_MATH_TINY * rant_double2_length(e))) return 0;
    t = rant_double2_cross(w, e) / denom;
    u = rant_double2_cross(w, d) / denom;
    if (!(t >= 0.0) || u < -I_TOUCH || u > 1.0 + I_TOUCH) return 0;
    if (hit){
        n = rant_double2_normalize(rant_double2(e.y, -e.x));
        if (rant_double2_dot(n, d) > 0.0) n = rant_double2_scale(n, -1.0);
        hit->point = rant_double2_add(ray.origin, rant_double2_scale(d, t));
        hit->normal = n;
        hit->distance = t;
    }
    return 1;
}

RantSegment2D rant_segment2d_transform(RantSegment2D s, RantPose2D pose){
    RantSegment2D r;
    r.a = rant_pose2d_apply(pose, s.a);
    r.b = rant_pose2d_apply(pose, s.b);
    return r;
}

RantAlignedBox2D rant_segment2d_bounds(RantSegment2D s){
    RantAlignedBox2D b;
    b.min = rant_double2_min(s.a, s.b);
    b.max = rant_double2_max(s.a, s.b);
    return b;
}

RantDouble2 rant_ray2d_at(RantRay2D ray, double distance){
    return rant_double2_add(ray.origin, rant_double2_scale(rant_double2_normalize(ray.direction), distance));
}

RantDouble2 rant_ray2d_closest(RantRay2D ray, RantDouble2 point){
    RantRay r3;
    r3.origin = i_math_lift(ray.origin); r3.direction = i_math_lift(ray.direction);
    return i_math_flat(rant_ray_closest(r3, i_math_lift(point)));
}

double rant_ray2d_distance(RantRay2D ray, RantDouble2 point){
    return rant_double2_distance(rant_ray2d_closest(ray, point), point);
}

RantRay2D rant_ray2d_transform(RantRay2D ray, RantPose2D pose){
    RantRay2D r;
    r.origin = rant_pose2d_apply(pose, ray.origin);
    r.direction = rant_double2_normalize(rant_double2_rotate(ray.direction, pose.angle));
    return r;
}
