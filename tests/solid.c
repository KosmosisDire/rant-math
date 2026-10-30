#include <stdlib.h>
#include "check.h"
#include "reference.h"

/* One solid behind function pointers, so every kind runs through the same checks. */
typedef struct {
    const char *name;
    const void *shape;
    double (*distance)(const void *, RantDouble3);
    RantSurfacePoint (*closest)(const void *, RantDouble3);
    int (*contains)(const void *, RantDouble3);
    int (*raycast)(const void *, RantRay, RantRayHit *);
    RantAlignedBox (*bounds)(const void *);
    RantDouble3 (*centroid)(const void *);
    double (*volume)(const void *);
    RantInertia (*inertia)(const void *, double);
} Solid;

#define ADAPT(T, K, bounds_body, centroid_body) \
    static double K##_d(const void *s, RantDouble3 p){ return rant_##K##_distance(*(const T *)s, p); } \
    static RantSurfacePoint K##_c(const void *s, RantDouble3 p){ return rant_##K##_closest(*(const T *)s, p); } \
    static int K##_in(const void *s, RantDouble3 p){ return rant_##K##_contains(*(const T *)s, p); } \
    static int K##_r(const void *s, RantRay r, RantRayHit *h){ return rant_##K##_raycast(*(const T *)s, r, h); } \
    static RantAlignedBox K##_b(const void *s){ const T *x = (const T *)s; bounds_body } \
    static RantDouble3 K##_ce(const void *s){ const T *x = (const T *)s; centroid_body } \
    static double K##_v(const void *s){ return rant_##K##_volume(*(const T *)s); } \
    static RantInertia K##_i(const void *s, double m){ return rant_##K##_inertia(*(const T *)s, m); } \
    static Solid K##_solid(const T *s){ \
        Solid r; r.name = #K; r.shape = s; r.distance = K##_d; r.closest = K##_c; r.contains = K##_in; \
        r.raycast = K##_r; r.bounds = K##_b; r.centroid = K##_ce; r.volume = K##_v; r.inertia = K##_i; \
        return r; }

ADAPT(RantAlignedBox, aligned_box, return *x;, return rant_aligned_box_centroid(*x);)
ADAPT(RantOrientedBox, oriented_box, return rant_oriented_box_bounds(*x);, return x->pose.position;)
ADAPT(RantSphere, sphere, return rant_sphere_bounds(*x);, return x->center;)
ADAPT(RantCapsule, capsule, return rant_capsule_bounds(*x);, return rant_capsule_centroid(*x);)
ADAPT(RantCylinder, cylinder, return rant_cylinder_bounds(*x);, return rant_cylinder_centroid(*x);)
ADAPT(RantCone, cone, return rant_cone_bounds(*x);, return rant_cone_centroid(*x);)
ADAPT(RantPrismView, prism, return rant_prism_bounds(*x);, return rant_prism_centroid(*x);)

static RantDouble3 in_box(RantAlignedBox b, double grow){
    return rant_double3(random_range(b.min.x - grow, b.max.x + grow), random_range(b.min.y - grow, b.max.y + grow),
                        random_range(b.min.z - grow, b.max.z + grow));
}

static RantRay ray_from(RantDouble3 origin, RantDouble3 toward){
    RantRay r;
    r.origin = origin;
    r.direction = rant_double3_normalize(rant_double3_subtract(toward, origin));
    return r;
}

/* Marching by the distance never steps past the surface, so it finds the first crossing on
 * its own: 1 with where, 0 for a miss, -1 when a grazing ray does not settle. */
static int march(const Solid *s, RantRay r, double far, double *at){
    double t = 0.0;
    int i;
    for (i = 0; i < 4000; i++){
        double d = fabs(s->distance(s->shape, rant_ray_at(r, t)));
        if (d < 1e-10){ *at = t; return 1; }
        t += d;
        if (t > far) return 0;
    }
    return -1;
}

#define SURFACE 1500

static void check_solid(const Solid *s){
    static RantDouble3 surface[SURFACE];
    RantAlignedBox b = s->bounds(s->shape);
    RantDouble3 size = rant_double3_subtract(b.max, b.min), c = s->centroid(s->shape);
    double reach = rant_double3_length(size), tol = 1e-9 * (1.0 + reach);
    int n = 0, i, k, inside = 0, total = 0;
    RantDouble3 sum = rant_double3(0, 0, 0);
    double second[3][3];
    memset(second, 0, sizeof second);

    /* Surface points from rays aimed in from all round, which the distance must agree with. */
    for (i = 0; i < 3 * SURFACE && n < SURFACE; i++){
        RantDouble3 from = rant_double3_add(c, rant_double3_scale(rant_double3_normalize(random_double3(1.0)), 2.0 * reach));
        RantRayHit h;
        RantRay r = ray_from(from, in_box(b, 0.0));
        if (!s->raycast(s->shape, r, &h)) continue;
        CHECK(fabs(s->distance(s->shape, h.point)) <= tol, "%s: a hit lies on the surface", s->name);
        CHECK(near3(h.point, rant_ray_at(r, h.distance), tol), "%s: a hit lies along its ray", s->name);
        CHECK(near(rant_double3_length(h.normal), 1.0, 1e-12) && rant_double3_dot(h.normal, r.direction) <= 1e-9,
              "%s: a hit from outside has a unit normal facing back", s->name);
        CHECK(h.point.x >= b.min.x - tol && h.point.x <= b.max.x + tol && h.point.y >= b.min.y - tol
              && h.point.y <= b.max.y + tol && h.point.z >= b.min.z - tol && h.point.z <= b.max.z + tol,
              "%s: the surface lies in the bounds", s->name);
        surface[n++] = h.point;
    }
    CHECK(n > SURFACE / 3, "%s: rays aimed at it hit it", s->name);

    for (i = 0; i < 300; i++){
        RantDouble3 q = i % 3 ? in_box(b, 0.3 * reach) : rant_double3_lerp(c, in_box(b, 0.0), random_unit());
        RantSurfacePoint p = s->closest(s->shape, q);
        double d = s->distance(s->shape, q), best = HUGE_VAL;
        CHECK(near3(rant_double3_add(p.point, rant_double3_scale(p.normal, d)), q, tol),
              "%s: the point is the closest plus the distance along the normal", s->name);
        CHECK(near(rant_double3_length(p.normal), 1.0, 1e-12), "%s: the normal is unit", s->name);
        CHECK(fabs(s->distance(s->shape, p.point)) <= tol, "%s: the closest point lies on the surface", s->name);
        CHECK(s->contains(s->shape, q) == (d <= 0.0), "%s: contains is distance <= 0", s->name);
        for (k = 0; k < n; k++){
            double e = rant_double3_distance(q, surface[k]);
            if (e < best) best = e;
        }
        CHECK(fabs(d) <= best + tol, "%s: no surface point is nearer than the distance", s->name);
        if (fabs(d) > 1e-6){
            RantRayHit h;
            int met = s->raycast(s->shape, ray_from(q, p.point), &h);
            /* A ray at a cone's tip meets a double root, good to the square root of rounding. */
            CHECK(met && near(h.distance, fabs(d), 1e-6 * (1.0 + reach)),
                  "%s: a ray toward the closest point first meets the surface there", s->name);
        }
    }

    for (i = 0; i < 300; i++){
        RantDouble3 o = i % 2 ? in_box(b, 0.5 * reach) : rant_double3_lerp(c, in_box(b, 0.0), random_unit());
        RantRay r;
        RantRayHit h;
        double at;
        int met, traced;
        r.origin = o;
        r.direction = i % 3 ? rant_double3_normalize(random_double3(1.0)) : rant_double3_normalize(rant_double3_subtract(in_box(b, 0.0), o));
        if (rant_double3_length_squared(r.direction) == 0.0) continue;
        met = s->raycast(s->shape, r, &h);
        traced = march(s, r, 10.0 * reach, &at);
        if (traced == 1) CHECK(met && near(h.distance, at, 1e-7 * (1.0 + reach)), "%s: the raycast meets where marching does", s->name);
        if (traced == 0) CHECK(!met, "%s: the raycast misses where marching does", s->name);
        if (met){
            RantSurfacePoint p = s->closest(s->shape, h.point);
            CHECK(fabs(s->distance(s->shape, h.point)) <= tol, "%s: every hit lies on the surface", s->name);
            CHECK(rant_double3_dot(p.normal, h.normal) > 0.999, "%s: the hit normal is the surface's", s->name);
        }
    }

    /* Volume, center and inertia against a count of random points inside the bounds. */
    for (i = 0; i < 60000; i++){
        RantDouble3 q = in_box(b, 0.0);
        total++;
        if (!s->contains(s->shape, q)) continue;
        inside++;
        sum = rant_double3_add(sum, q);
        {   double v[3];
            int r, cc;
            v[0] = q.x; v[1] = q.y; v[2] = q.z;
            for (r = 0; r < 3; r++) for (cc = 0; cc < 3; cc++) second[r][cc] += v[r] * v[cc];
        }
    }
    {   double box = size.x * size.y * size.z, p = (double)inside / total;
        double want = s->volume(s->shape), spread = 4.0 * box * sqrt(p * (1.0 - p) / total) + 1e-12;
        CHECK(fabs(box * p - want) <= spread, "%s: volume %g against a count of %g", s->name, want, box * p);
        if (inside > 1000){
            RantDouble3 mean = rant_double3_scale(sum, 1.0 / inside);
            RantInertia in = s->inertia(s->shape, 2.0);
            double cov[3][3], mc[3], t[3][3], got[3][3], scale;
            int r, cc;
            mc[0] = mean.x; mc[1] = mean.y; mc[2] = mean.z;
            CHECK(near3(mean, in.center, 0.03 * reach), "%s: the inertia's center is the mean of the inside", s->name);
            CHECK(near3(in.center, c, 1e-9 * (1.0 + reach)), "%s: centroid and the inertia's center agree", s->name);
            for (r = 0; r < 3; r++) for (cc = 0; cc < 3; cc++) cov[r][cc] = second[r][cc] / inside - mc[r] * mc[cc];
            for (r = 0; r < 3; r++)
                for (cc = 0; cc < 3; cc++)
                    t[r][cc] = 2.0 * ((r == cc ? cov[0][0] + cov[1][1] + cov[2][2] : 0.0) - cov[r][cc]);
            got[0][0] = in.ixx; got[0][1] = got[1][0] = in.ixy; got[0][2] = got[2][0] = in.ixz;
            got[1][1] = in.iyy; got[1][2] = got[2][1] = in.iyz; got[2][2] = in.izz;
            scale = t[0][0] + t[1][1] + t[2][2];
            for (r = 0; r < 3; r++)
                for (cc = 0; cc < 3; cc++)
                    CHECK(fabs(got[r][cc] - t[r][cc]) <= 0.03 * scale, "%s: inertia [%d][%d] %g against a count of %g",
                          s->name, r, cc, got[r][cc], t[r][cc]);
        }
    }
}

static RantSegment random_segment(double r){
    RantSegment s;
    s.a = random_double3(r);
    s.b = random_double3(r);
    return s;
}

static void random_solids(void){
    int i;
    for (i = 0; i < 6; i++){
        RantAlignedBox ab;
        RantOrientedBox ob;
        RantSphere sp;
        RantCapsule ca;
        RantCylinder cy;
        RantCone co;
        RantPrismView pr;
        RantDouble2 base[40];
        Solid s;
        ab.min = random_double3(3.0);
        ab.max = rant_double3_add(ab.min, rant_double3(random_range(0.2, 3), random_range(0.2, 3), random_range(0.2, 3)));
        s = aligned_box_solid(&ab); check_solid(&s);
        ob.pose = random_pose(3.0);
        ob.size = rant_double3(random_range(0.2, 3), random_range(0.2, 3), random_range(0.2, 3));
        s = oriented_box_solid(&ob); check_solid(&s);
        sp.center = random_double3(3.0); sp.radius = random_range(0.2, 2.0);
        s = sphere_solid(&sp); check_solid(&s);
        ca.axis = random_segment(2.0); ca.radius = random_range(0.2, 1.5);
        if (i == 0) ca.axis.b = ca.axis.a;
        s = capsule_solid(&ca); check_solid(&s);
        cy.axis = random_segment(2.0); cy.radius = random_range(0.2, 1.5);
        if (i == 0){ cy.axis.b = cy.axis.a; cy.axis.b.z += 1.5; }
        s = cylinder_solid(&cy); check_solid(&s);
        co.base = random_double3(2.0); co.tip = random_double3(2.0); co.radius = random_range(0.2, 1.5);
        if (i == 0){ co.tip = co.base; co.tip.z -= 2.0; }
        s = cone_solid(&co); check_solid(&s);
        pr.pose = random_pose(2.0);
        pr.base.points = base;
        pr.base.count = random_polygon(base, 20);
        if (i == 1) base[pr.base.count++] = base[0];
        pr.height = i == 2 ? -random_range(0.3, 2.0) : random_range(0.3, 2.0);
        s = prism_solid(&pr); check_solid(&s);
    }
}

static void boxes(void){
    RantAlignedBox b, o;
    RantDouble3 c[8];
    RantSegment e[12];
    RantPlane p[6];
    RantRayHit h;
    RantRay r;
    int i, j;
    b.min = rant_double3(0, 0, 0); b.max = rant_double3(2, 4, 6);
    CHECK(near(rant_aligned_box_distance(b, rant_double3(1, 2, 3)), -1.0, 0.0), "the center of a box is 1 from its nearest face");
    CHECK(near(rant_aligned_box_distance(b, rant_double3(5, 8, 3)), 5.0, 1e-15), "a point past an edge is measured to it");
    CHECK(near(rant_aligned_box_volume(b), 48.0, 0.0) && near(rant_aligned_box_area(b), 88.0, 0.0), "volume and area");
    rant_aligned_box_corners(b, c);
    CHECK(near3(c[0], b.min, 0.0) && near3(c[1], rant_double3(2, 0, 0), 0.0) && near3(c[2], rant_double3(2, 4, 0), 0.0)
          && near3(c[3], rant_double3(0, 4, 0), 0.0) && near3(c[4], rant_double3(0, 0, 6), 0.0) && near3(c[6], b.max, 0.0),
          "corners run round the bottom from min, then the top");
    rant_aligned_box_edges(b, e);
    for (i = 0; i < 12; i++)
        CHECK(fabs(rant_aligned_box_distance(b, rant_double3_lerp(e[i].a, e[i].b, 0.37))) < 1e-15, "edge %d lies on the box", i);
    CHECK(near(rant_segment_length(e[8]), 6.0, 0.0) && near3(e[8].a, c[0], 0.0) && near3(e[8].b, c[4], 0.0), "the uprights come last");
    rant_aligned_box_planes(b, p);
    for (i = 0; i < 6; i++){
        CHECK(near(rant_plane_distance(p[i], rant_double3(1, 2, 3)), i < 4 ? (i % 2 ? -1.0 : -2.0) : -3.0, 0.0),
              "plane %d faces out", i);
        for (j = 0; j < 8; j++) CHECK(rant_plane_distance(p[i], c[j]) <= 0.0, "every corner is behind every plane");
    }
    CHECK(near(rant_plane_distance(p[0], rant_double3_lerp(e[0].a, e[0].b, 0.5)), 0.0, 0.0)
          && near(rant_plane_distance(p[1], rant_double3_lerp(e[1].a, e[1].b, 0.5)), 0.0, 0.0), "plane i holds edge i");
    r.origin = rant_double3(1, 2, 3); r.direction = rant_double3(0, 0, 1);
    CHECK(rant_aligned_box_raycast(b, r, &h) && near(h.distance, 3.0, 0.0) && near3(h.normal, rant_double3(0, 0, 1), 0.0),
          "a ray from inside leaves through the top");
    r.origin = rant_double3(-1, 2, 3); r.direction = rant_double3(1, 0, 0);
    CHECK(rant_aligned_box_raycast(b, r, &h) && near(h.distance, 1.0, 0.0) && near3(h.normal, rant_double3(-1, 0, 0), 0.0),
          "a ray from outside enters through the near face");
    r.direction = rant_double3(-1, 0, 0);
    CHECK(!rant_aligned_box_raycast(b, r, NULL), "a ray going away misses");
    o.min = rant_double3(1, 1, 1); o.max = rant_double3(9, 9, 9);
    CHECK(rant_aligned_box_overlap(b, o, &o) && near3(o.min, rant_double3(1, 1, 1), 0.0) && near3(o.max, rant_double3(2, 4, 6), 0.0),
          "overlap is the shared box");
    o.min = rant_double3(3, 0, 0); o.max = rant_double3(4, 1, 1);
    CHECK(!rant_aligned_box_overlap(b, o, NULL), "boxes apart do not overlap");
    o = rant_aligned_box_merge(b, o);
    CHECK(near3(o.min, b.min, 0.0) && near3(o.max, rant_double3(4, 4, 6), 0.0), "merge");
    o = rant_aligned_box_grow(b, -1.5);
    CHECK(near3(o.min, rant_double3(1, 1.5, 1.5), 0.0) && near3(o.max, rant_double3(1, 2.5, 4.5), 0.0),
          "shrinking stops at the center");
    {   RantInertia in = rant_aligned_box_inertia(b, 12.0);
        CHECK(near(in.ixx, 52.0, 1e-13) && near(in.iyy, 40.0, 1e-13) && near(in.izz, 20.0, 1e-13) && near3(in.center, rant_double3(1, 2, 3), 0.0),
              "a box's inertia");
    }
    for (i = 0; i < 200; i++){
        RantPose pose = random_pose(4.0);
        RantOrientedBox ob = rant_aligned_box_transform(b, pose), same = rant_oriented_box_from_aligned_box(b);
        RantDouble3 q = random_double3(8.0), oc[8];
        RantPlane op[6];
        RantInertia moved = rant_oriented_box_inertia(ob, 3.0), want = rant_inertia_transform(rant_aligned_box_inertia(b, 3.0), pose);
        CHECK(near(rant_oriented_box_distance(ob, rant_pose_apply(pose, q)), rant_aligned_box_distance(b, q), 1e-12),
              "a moved box keeps every distance");
        CHECK(near(rant_oriented_box_distance(same, q), rant_aligned_box_distance(b, q), 1e-14), "an unturned oriented box is the aligned one");
        rant_oriented_box_corners(ob, oc);
        rant_oriented_box_planes(ob, op);
        for (j = 0; j < 8; j++) CHECK(near3(oc[j], rant_pose_apply(pose, c[j]), 1e-12), "oriented corners in the same order");
        for (j = 0; j < 6; j++) CHECK(near(rant_plane_distance(op[j], rant_pose_apply(pose, q)), rant_plane_distance(p[j], q), 1e-12),
                                      "oriented planes in the same order");
        CHECK(near(moved.ixx, want.ixx, 1e-12) && near(moved.ixy, want.ixy, 1e-12) && near(moved.izz, want.izz, 1e-12), "oriented inertia");
        CHECK(near(rant_oriented_box_area(ob), 88.0, 1e-12) && near(rant_oriented_box_volume(ob), 48.0, 1e-12), "oriented volume and area");
        {   RantAlignedBox ob_b = rant_oriented_box_bounds(ob), cb;
            cb.min = cb.max = oc[0];
            for (j = 1; j < 8; j++){ cb.min = rant_double3_min(cb.min, oc[j]); cb.max = rant_double3_max(cb.max, oc[j]); }
            CHECK(near3(ob_b.min, cb.min, 1e-12) && near3(ob_b.max, cb.max, 1e-12), "oriented bounds are the corners' box");
        }
        {   RantOrientedBox twice = rant_oriented_box_transform(ob, pose);
            CHECK(same_pose(twice.pose, rant_pose_compose(pose, ob.pose), 1e-12), "transform composes the pose");
        }
    }
}

static void round_solids(void){
    RantSphere s;
    RantCapsule c;
    RantCylinder y;
    RantCone k;
    RantRay r;
    RantRayHit h;
    int i;
    s.center = rant_double3(1, 2, 3); s.radius = 2.0;
    CHECK(near(rant_sphere_distance(s, rant_double3(1, 2, 3)), -2.0, 0.0) && near(rant_sphere_distance(s, rant_double3(1, 2, 8)), 3.0, 0.0),
          "sphere distance");
    CHECK(near3(rant_sphere_closest(s, s.center).normal, rant_double3(1, 0, 0), 0.0), "the center takes +x");
    CHECK(near(rant_sphere_volume(s), 32.0 / 3.0 * PI, 1e-13) && near(rant_sphere_area(s), 16.0 * PI, 1e-13), "sphere volume and area");
    r.origin = rant_double3(1, 2, -7); r.direction = rant_double3(0, 0, 1);
    CHECK(rant_sphere_raycast(s, r, &h) && near(h.distance, 8.0, 0.0) && near3(h.normal, rant_double3(0, 0, -1), 0.0), "a ray into a ball");
    r.origin = s.center;
    CHECK(rant_sphere_raycast(s, r, &h) && near(h.distance, 2.0, 0.0) && near3(h.point, rant_double3(1, 2, 5), 0.0), "and out of it");
    c.axis.a = rant_double3(0, 0, 0); c.axis.b = rant_double3(0, 0, 2); c.radius = 0.5;
    CHECK(near(rant_capsule_distance(c, rant_double3(0, 0, 3)), 0.5, 0.0) && near(rant_capsule_distance(c, rant_double3(2, 0, 1)), 1.5, 0.0),
          "capsule distance");
    CHECK(near3(rant_capsule_closest(c, rant_double3(0, 0, 1)).normal, rant_double3(1, 0, 0), 0.0), "a point on an upright axis takes +x");
    c.axis.b = rant_double3(3, 4, 0);
    CHECK(near(rant_capsule_closest(c, rant_double3(1.5, 2, 0)).normal.z, 0.0, 0.0), "a point on a level axis keeps a level normal");
    CHECK(near(rant_capsule_volume(c), PI * 0.25 * (5.0 + 2.0 / 3.0), 1e-13), "capsule volume");
    c.axis.b = c.axis.a;
    {   RantInertia ci = rant_capsule_inertia(c, 5.0), si;
        s.center = c.axis.a; s.radius = c.radius;
        si = rant_sphere_inertia(s, 5.0);
        CHECK(near(ci.ixx, si.ixx, 1e-15) && near(ci.izz, si.izz, 1e-15), "a capsule of no length is a ball");
    }
    y.axis.a = rant_double3(0, 0, 0); y.axis.b = rant_double3(0, 0, 4); y.radius = 1.0;
    CHECK(near(rant_cylinder_distance(y, rant_double3(0, 0, 2)), -1.0, 0.0) && near(rant_cylinder_distance(y, rant_double3(0, 0, 1)), -1.0, 0.0)
          && near(rant_cylinder_distance(y, rant_double3(0, 0, 0.5)), -0.5, 0.0), "inside a cylinder, the nearer of side and cap");
    CHECK(near(rant_cylinder_distance(y, rant_double3(4, 0, 7)), sqrt(18.0), 1e-15), "past the rim, to the rim");
    CHECK(near3(rant_cylinder_closest(y, rant_double3(0.5, 0, 5)).point, rant_double3(0.5, 0, 4), 0.0), "above the top, to the top");
    CHECK(near(rant_cylinder_area(y), 10.0 * PI, 1e-13) && near(rant_cylinder_volume(y), 4.0 * PI, 1e-13), "cylinder area and volume");
    {   RantAlignedBox bb;
        y.axis.b = rant_double3(3, 0, 4);
        bb = rant_cylinder_bounds(y);
        CHECK(near3(bb.min, rant_double3(-0.8, -1, -0.6), 1e-15) && near3(bb.max, rant_double3(3.8, 1, 4.6), 1e-15),
              "a tilted cylinder's bounds reach its rims");
    }
    k.base = rant_double3(0, 0, 0); k.tip = rant_double3(0, 0, 3); k.radius = 4.0;
    CHECK(near(rant_cone_distance(k, rant_double3(0, 0, 4)), 1.0, 1e-15) && near(rant_cone_distance(k, rant_double3(0, 0, -2)), 2.0, 0.0),
          "above the tip and below the base");
    CHECK(near(rant_cone_distance(k, rant_double3(4, 0, 3)), 2.4, 1e-15), "beside the slope, square to it");
    CHECK(near(rant_cone_area(k), 36.0 * PI, 1e-13) && near(rant_cone_volume(k), 16.0 * PI, 1e-13), "cone area and volume");
    CHECK(near3(rant_cone_centroid(k), rant_double3(0, 0, 0.75), 0.0), "a quarter of the way up");
    {   RantAlignedBox bb = rant_cone_bounds(k);
        CHECK(near3(bb.min, rant_double3(-4, -4, 0), 0.0) && near3(bb.max, rant_double3(4, 4, 3), 0.0), "cone bounds");
    }
    r.origin = rant_double3(0, 0, 10); r.direction = rant_double3(0, 0, -1);
    CHECK(rant_cone_raycast(k, r, &h) && near(h.distance, 7.0, 1e-15), "a ray down the axis meets the tip");
    for (i = 0; i < 300; i++){
        RantPose pose = random_pose(3.0);
        RantDouble3 q = random_double3(5.0), w = rant_pose_apply(pose, q);
        RantSphere s2;
        RantCapsule c2;
        RantCylinder y2;
        RantCone k2;
        s2.center = random_double3(2.0); s2.radius = random_range(0.1, 2.0);
        c2.axis.a = random_double3(2.0); c2.axis.b = random_double3(2.0); c2.radius = random_range(0.1, 2.0);
        y2.axis = c2.axis; y2.radius = c2.radius;
        k2.base = c2.axis.a; k2.tip = c2.axis.b; k2.radius = c2.radius;
        CHECK(near(rant_sphere_distance(rant_sphere_transform(s2, pose), w), rant_sphere_distance(s2, q), 1e-12), "sphere transform");
        CHECK(near(rant_capsule_distance(rant_capsule_transform(c2, pose), w), rant_capsule_distance(c2, q), 1e-12), "capsule transform");
        CHECK(near(rant_cylinder_distance(rant_cylinder_transform(y2, pose), w), rant_cylinder_distance(y2, q), 1e-12), "cylinder transform");
        CHECK(near(rant_cone_distance(rant_cone_transform(k2, pose), w), rant_cone_distance(k2, q), 1e-12), "cone transform");
        CHECK(rant_capsule_distance(c2, q) <= rant_cylinder_distance(y2, q) + 1e-12, "a capsule holds the cylinder on its axis");
        CHECK(rant_cylinder_distance(y2, q) <= rant_cone_distance(k2, q) + 1e-12, "and the cylinder holds the cone");
    }
}

/* A cylinder is the limit of a prism over a many sided base. */
static void prisms(void){
    RantDouble2 base[400], square[5];
    RantPrismView p, sq;
    RantCylinder y;
    RantOrientedBox ob;
    uint32_t i, count;
    RantDouble3 corners[16];
    RantSegment edges[16];
    RantPlane planes[8];
    for (i = 0; i < 400; i++) base[i] = rant_double2(cos(2.0 * PI * i / 400), sin(2.0 * PI * i / 400));
    p.pose = random_pose(2.0); p.base.points = base; p.base.count = 400; p.height = 2.0;
    y.axis.a = p.pose.position; y.axis.b = rant_pose_apply(p.pose, rant_double3(0, 0, 2)); y.radius = 1.0;
    CHECK(near(rant_prism_volume(p), rant_cylinder_volume(y), 1e-3) && near(rant_prism_area(p), rant_cylinder_area(y), 1e-3),
          "a 400 sided prism has the cylinder's volume and area");
    {   RantInertia a = rant_prism_inertia(p, 3.0), b = rant_cylinder_inertia(y, 3.0);
        CHECK(near(a.ixx, b.ixx, 1e-4) && near(a.ixy, b.ixy, 1e-4) && near(a.iyz, b.iyz, 1e-4) && near(a.izz, b.izz, 1e-4)
              && near3(a.center, b.center, 1e-12), "and its inertia");
    }
    for (i = 0; i < 200; i++){
        RantDouble3 q = rant_double3_add(y.axis.a, random_double3(3.0));
        CHECK(near(rant_prism_distance(p, q), rant_cylinder_distance(y, q), 1e-4), "and its distance");
    }
    /* a square base clockwise, with the closing corner repeated, is the oriented box */
    square[0] = rant_double2(-1, -2); square[1] = rant_double2(-1, 2); square[2] = rant_double2(1, 2);
    square[3] = rant_double2(1, -2); square[4] = square[0];
    sq.pose = random_pose(2.0); sq.base.points = square; sq.base.count = 5; sq.height = -3.0;
    ob.pose = rant_pose_compose(sq.pose, rant_pose_identity());
    ob.pose.position = rant_pose_apply(sq.pose, rant_double3(0, 0, -1.5));
    ob.size = rant_double3(2, 4, 3);
    for (i = 0; i < 300; i++){
        RantDouble3 q = rant_double3_add(ob.pose.position, random_double3(4.0));
        RantRay r;
        RantRayHit a, b;
        CHECK(near(rant_prism_distance(sq, q), rant_oriented_box_distance(ob, q), 1e-12), "a box shaped prism is the box");
        r.origin = q; r.direction = random_double3(1.0);
        CHECK(rant_prism_raycast(sq, r, &a) == rant_oriented_box_raycast(ob, r, &b), "it meets rays where the box does");
        if (rant_oriented_box_raycast(ob, r, &b) && rant_prism_raycast(sq, r, &a))
            CHECK(near(a.distance, b.distance, 1e-12) && near3(a.normal, b.normal, 1e-12), "at the same points and normals");
    }
    CHECK(near(rant_prism_volume(sq), 24.0, 1e-14) && near(rant_prism_area(sq), 52.0, 1e-14), "box shaped volume and area");
    {   RantInertia a = rant_prism_inertia(sq, 2.0), b = rant_oriented_box_inertia(ob, 2.0);
        CHECK(near(a.ixx, b.ixx, 1e-12) && near(a.ixy, b.ixy, 1e-12) && near(a.ixz, b.ixz, 1e-12) && near(a.iyy, b.iyy, 1e-12)
              && near(a.iyz, b.iyz, 1e-12) && near(a.izz, b.izz, 1e-12) && near3(a.center, b.center, 1e-12), "box shaped inertia");
    }
    count = 0;
    CHECK(rant_prism_corners(sq, NULL, &count) == RANT_MATH_ERR_SPACE && count == 8, "asking with no room gives the count");
    count = 7;
    CHECK(rant_prism_corners(sq, corners, &count) == RANT_MATH_ERR_SPACE && count == 8, "too little room gives the count");
    count = 16;
    CHECK(rant_prism_corners(sq, corners, &count) == RANT_MATH_OK && count == 8, "corners skip the repeated one");
    CHECK(near3(corners[0], rant_pose_apply(sq.pose, rant_double3(-1, -2, 0)), 1e-14)
          && near3(corners[4], rant_pose_apply(sq.pose, rant_double3(-1, -2, -3)), 1e-14), "the bottom then the top");
    count = 16;
    CHECK(rant_prism_edges(sq, edges, &count) == RANT_MATH_OK && count == 12, "a box shaped prism has 12 edges");
    for (i = 0; i < 12; i++) CHECK(fabs(rant_prism_distance(sq, rant_double3_lerp(edges[i].a, edges[i].b, 0.3))) < 1e-12, "edges lie on it");
    count = 8;
    CHECK(rant_prism_planes(sq, planes, &count) == RANT_MATH_OK && count == 6, "and 6 planes");
    for (i = 0; i < 300; i++){
        RantDouble3 q = rant_double3_add(ob.pose.position, random_double3(1.0));
        double worst = -HUGE_VAL;
        uint32_t j;
        for (j = 0; j < 6; j++) worst = fmax(worst, rant_plane_distance(planes[j], q));
        if (rant_prism_contains(sq, q)) CHECK(near(worst, rant_prism_distance(sq, q), 1e-12), "inside a convex prism the distance is the nearest plane's");
        else CHECK(worst <= rant_prism_distance(sq, q) + 1e-12 && worst > 0.0, "outside it some plane faces the point");
    }
    CHECK(near3(planes[4].normal, rant_quaternion_rotate(sq.pose.orientation, rant_double3(0, 0, -1)), 1e-15)
          && near(rant_plane_distance(planes[4], rant_pose_apply(sq.pose, rant_double3(0, 0, -3))), 0.0, 1e-14),
          "a prism rising along -z has its bottom at the height");
    {   RantPrismView moved = rant_prism_transform(sq, random_pose(2.0)), empty = sq;
        RantDouble3 q = random_double3(2.0);
        CHECK(near(rant_prism_distance(moved, rant_pose_apply(rant_pose_compose(moved.pose, rant_pose_inverse(sq.pose)), q)),
                   rant_prism_distance(sq, q), 1e-11), "prism transform");
        empty.base.count = 0;
        CHECK(rant_prism_distance(empty, q) == HUGE_VAL && !rant_prism_contains(empty, q), "a prism over no base is nowhere");
    }
}

static RantOrientedBox obox(RantDouble3 c, RantQuaternion q, RantDouble3 size){
    RantOrientedBox b;
    b.pose.position = c;
    b.pose.orientation = q;
    b.size = size;
    return b;
}

/* Against qhull, and where faces lie on each other, which is where clipping goes wrong. */
static void iou(void){
    RantOrientedBox a, b;
    RantAlignedBox p, q;
    RantQuaternion yaw;
    double worst = 0.0;
    int i, bad = 0, odd = 0, flat = 0;
    for (i = 0; i < REF_IOU3D_CASES; i++){
        const double *r = REF_IOU3D + 21 * i;
        double got;
        a = obox(rant_double3(r[0], r[1], r[2]), rant_quaternion(r[3], r[4], r[5], r[6]), rant_double3(r[7], r[8], r[9]));
        b = obox(rant_double3(r[10], r[11], r[12]), rant_quaternion(r[13], r[14], r[15], r[16]), rant_double3(r[17], r[18], r[19]));
        got = rant_oriented_box_iou(a, b);
        if (fabs(got - r[20]) > worst) worst = fabs(got - r[20]);
        if (!near(got, r[20], 1e-9)) bad++;
    }
    CHECK(bad == 0, "%d IoU of %d off qhull's, worst %.3g", bad, REF_IOU3D_CASES, worst);
    for (i = 0; i < 200; i++){
        RantPose move = random_pose(30.0);
        double ab, ba, moved;
        a = obox(random_double3(1.0), random_quaternion(), rant_double3(random_range(0.2, 3), random_range(0.2, 3), random_range(0.2, 3)));
        b = obox(random_double3(1.0), random_quaternion(), rant_double3(random_range(0.2, 3), random_range(0.2, 3), random_range(0.2, 3)));
        ab = rant_oriented_box_iou(a, b);
        ba = rant_oriented_box_iou(b, a);
        moved = rant_oriented_box_iou(rant_oriented_box_transform(a, move), rant_oriented_box_transform(b, move));
        if (!near(ab, ba, 1e-10) || !near(ab, moved, 1e-10) || ab < 0.0 || ab > 1.0) odd++;
    }
    CHECK(odd == 0, "%d pairs differ swapped or moved together", odd);
    /* boxes standing on one floor with one height, as a detector reports cars: the top view's IoU */
    for (i = 0; i < 200; i++){
        RantOrientedBox2D fa, fb;
        double h = random_range(0.5, 2), turn_a = random_range(-PI, PI), turn_b = i % 4 ? random_range(-PI, PI) : turn_a + PI / 2 * (i % 3);
        fa.pose.position = random_double2(1.0); fa.pose.angle = turn_a; fa.size = rant_double2(random_range(0.5, 3), random_range(0.5, 3));
        fb.pose.position = i % 5 ? random_double2(1.0) : fa.pose.position;
        fb.pose.angle = turn_b; fb.size = i % 7 ? rant_double2(random_range(0.5, 3), random_range(0.5, 3)) : fa.size;
        a = obox(rant_double3(fa.pose.position.x, fa.pose.position.y, h / 2), rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), turn_a),
                 rant_double3(fa.size.x, fa.size.y, h));
        b = obox(rant_double3(fb.pose.position.x, fb.pose.position.y, h / 2), rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), turn_b),
                 rant_double3(fb.size.x, fb.size.y, h));
        if (!near(rant_oriented_box_iou(a, b), rant_oriented_box2d_iou(fa, fb), 1e-10)) flat++;
    }
    CHECK(flat == 0, "%d boxes on one floor differ from their top view", flat);
    a = obox(rant_double3(1, 2, 3), random_quaternion(), rant_double3(1, 2, 3));
    CHECK(near(rant_oriented_box_iou(a, a), 1.0, 1e-12), "a box with itself: %.17g", rant_oriented_box_iou(a, a));
    b = a;
    b.pose.orientation = rant_quaternion_compose(a.pose.orientation, rant_quaternion_from_axis_angle(rant_double3(0, 1, 0), PI));
    CHECK(near(rant_oriented_box_iou(a, b), 1.0, 1e-12), "turned half round, the same box");
    b = a;
    b.size = rant_double3(0.5, 1, 1.5);
    CHECK(near(rant_oriented_box_iou(a, b), 0.125, 1e-12), "an eighth of it inside it");
    b = a;
    b.pose.position = rant_pose_apply(a.pose, rant_double3(1, 0, 0));
    CHECK(near(rant_oriented_box_iou(a, b), 0.0, 1e-12), "touching face to face shares nothing");
    b.pose.position = rant_pose_apply(a.pose, rant_double3(0.5, 0, 0));
    CHECK(near(rant_oriented_box_iou(a, b), 1.0 / 3.0, 1e-12), "half along x shares a third");
    b.pose.position = rant_pose_apply(a.pose, rant_double3(0.5, 1.0, 0));
    CHECK(near(rant_oriented_box_iou(a, b), 1.0 / 7.0, 1e-12), "half along x and y shares a seventh");
    yaw = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), 0.3);
    a = obox(rant_double3(0, 0, 0), yaw, rant_double3(2, 2, 2));
    b = obox(rant_double3(0, 0, 0), rant_quaternion_compose(yaw, rant_quaternion_from_axis_angle(rant_double3(1, 0, 0), 1e-13)), rant_double3(2, 2, 2));
    CHECK(near(rant_oriented_box_iou(a, b), 1.0, 1e-9), "all but the same box");
    b = obox(rant_double3(0, 0, 0), yaw, rant_double3(2, 0, 2));
    CHECK(rant_oriented_box_iou(a, b) == 0.0, "a box of no volume");
    b = obox(rant_double3(9, 0, 0), yaw, rant_double3(2, 2, 2));
    CHECK(rant_oriented_box_iou(a, b) == 0.0, "far apart");
    p.min = rant_double3(0, 0, 0); p.max = rant_double3(2, 2, 2);
    q.min = rant_double3(1, 1, 1); q.max = rant_double3(3, 3, 4);
    CHECK(near(rant_aligned_box_iou(p, q), 1.0 / 19.0, 1e-15), "aligned: 1 shared of 19");
    CHECK(near(rant_aligned_box_iou(p, q), rant_oriented_box_iou(rant_oriented_box_from_aligned_box(p), rant_oriented_box_from_aligned_box(q)), 1e-12),
          "as the oriented boxes");
    q.min = rant_double3(2, 0, 0);
    CHECK(rant_aligned_box_iou(p, q) == 0.0, "aligned and touching");
}

void test_solid(void){
    boxes();
    round_solids();
    prisms();
    random_solids();
    iou();
}
