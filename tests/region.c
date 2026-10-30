#include <stdlib.h>
#include "check.h"
#include "reference.h"

/* One 2D region behind function pointers, so every kind runs through the same checks. */
typedef struct {
    const char *name;
    const void *shape;
    double (*distance)(const void *, RantDouble2);
    RantSurfacePoint2D (*closest)(const void *, RantDouble2);
    int (*contains)(const void *, RantDouble2);
    int (*raycast)(const void *, RantRay2D, RantRayHit2D *);
    RantAlignedBox2D (*bounds)(const void *);
    RantDouble2 (*centroid)(const void *);
    double (*area)(const void *);
} Region;

#define ADAPT(T, K, bounds_body, centroid_body) \
    static double K##_d(const void *s, RantDouble2 p){ return rant_##K##_distance(*(const T *)s, p); } \
    static RantSurfacePoint2D K##_c(const void *s, RantDouble2 p){ return rant_##K##_closest(*(const T *)s, p); } \
    static int K##_in(const void *s, RantDouble2 p){ return rant_##K##_contains(*(const T *)s, p); } \
    static int K##_r(const void *s, RantRay2D r, RantRayHit2D *h){ return rant_##K##_raycast(*(const T *)s, r, h); } \
    static RantAlignedBox2D K##_b(const void *s){ const T *x = (const T *)s; bounds_body } \
    static RantDouble2 K##_ce(const void *s){ const T *x = (const T *)s; centroid_body } \
    static double K##_a(const void *s){ return rant_##K##_area(*(const T *)s); } \
    static Region K##_region(const T *s){ \
        Region r; r.name = #K; r.shape = s; r.distance = K##_d; r.closest = K##_c; r.contains = K##_in; \
        r.raycast = K##_r; r.bounds = K##_b; r.centroid = K##_ce; r.area = K##_a; \
        return r; }

ADAPT(RantAlignedBox2D, aligned_box2d, return *x;, return rant_aligned_box2d_centroid(*x);)
ADAPT(RantOrientedBox2D, oriented_box2d, return rant_oriented_box2d_bounds(*x);, return x->pose.position;)
ADAPT(RantCircle, circle, return rant_circle_bounds(*x);, return x->center;)
ADAPT(RantCapsule2D, capsule2d, return rant_capsule2d_bounds(*x);, return rant_double2_lerp(x->axis.a, x->axis.b, 0.5);)
ADAPT(RantPolygon2DView, polygon2d, return rant_polygon2d_bounds(*x);, return rant_polygon2d_centroid(*x);)

static RantDouble2 in_box2(RantAlignedBox2D b, double grow){
    return rant_double2(random_range(b.min.x - grow, b.max.x + grow), random_range(b.min.y - grow, b.max.y + grow));
}

static RantRay2D ray2_from(RantDouble2 origin, RantDouble2 toward){
    RantRay2D r;
    r.origin = origin;
    r.direction = rant_double2_normalize(rant_double2_subtract(toward, origin));
    return r;
}

static int march2(const Region *s, RantRay2D r, double far, double *at){
    double t = 0.0;
    int i;
    for (i = 0; i < 4000; i++){
        double d = fabs(s->distance(s->shape, rant_ray2d_at(r, t)));
        if (d < 1e-10){ *at = t; return 1; }
        t += d;
        if (t > far) return 0;
    }
    return -1;
}

#define BOUNDARY 1500

static void check_region(const Region *s){
    static RantDouble2 edge[BOUNDARY];
    RantAlignedBox2D b = s->bounds(s->shape);
    RantDouble2 size = rant_double2_subtract(b.max, b.min), c = s->centroid(s->shape), sum = rant_double2(0, 0);
    double reach = rant_double2_length(size), tol = 1e-9 * (1.0 + reach);
    int n = 0, i, k, inside = 0, total = 0;
    for (i = 0; i < 3 * BOUNDARY && n < BOUNDARY; i++){
        double a = random_range(0.0, 2.0 * PI);
        RantDouble2 from = rant_double2_add(c, rant_double2_scale(rant_double2(cos(a), sin(a)), 2.0 * reach));
        RantRayHit2D h;
        RantRay2D r = ray2_from(from, in_box2(b, 0.0));
        if (!s->raycast(s->shape, r, &h)) continue;
        CHECK(fabs(s->distance(s->shape, h.point)) <= tol, "%s: a hit lies on the boundary", s->name);
        CHECK(near2(h.point, rant_ray2d_at(r, h.distance), tol), "%s: a hit lies along its ray", s->name);
        CHECK(near(rant_double2_length(h.normal), 1.0, 1e-12) && rant_double2_dot(h.normal, r.direction) <= 1e-9,
              "%s: a hit from outside has a unit normal facing back", s->name);
        CHECK(h.point.x >= b.min.x - tol && h.point.x <= b.max.x + tol && h.point.y >= b.min.y - tol && h.point.y <= b.max.y + tol,
              "%s: the boundary lies in the bounds", s->name);
        edge[n++] = h.point;
    }
    CHECK(n > BOUNDARY / 3, "%s: rays aimed at it hit it", s->name);
    for (i = 0; i < 400; i++){
        RantDouble2 q = i % 3 ? in_box2(b, 0.3 * reach) : rant_double2_lerp(c, in_box2(b, 0.0), random_unit());
        RantSurfacePoint2D p = s->closest(s->shape, q);
        double d = s->distance(s->shape, q), best = HUGE_VAL;
        CHECK(near2(rant_double2_add(p.point, rant_double2_scale(p.normal, d)), q, tol),
              "%s: the point is the closest plus the distance along the normal", s->name);
        CHECK(near(rant_double2_length(p.normal), 1.0, 1e-12), "%s: the normal is unit", s->name);
        CHECK(fabs(s->distance(s->shape, p.point)) <= tol, "%s: the closest point lies on the boundary", s->name);
        CHECK(s->contains(s->shape, q) == (d <= 0.0), "%s: contains is distance <= 0", s->name);
        for (k = 0; k < n; k++){
            double e = rant_double2_distance(q, edge[k]);
            if (e < best) best = e;
        }
        CHECK(fabs(d) <= best + tol, "%s: no boundary point is nearer than the distance", s->name);
        if (fabs(d) > 1e-6){
            RantRayHit2D h;
            CHECK(s->raycast(s->shape, ray2_from(q, p.point), &h) && near(h.distance, fabs(d), 1e-7 * (1.0 + reach)),
                  "%s: a ray toward the closest point first meets the boundary there", s->name);
        }
    }
    for (i = 0; i < 400; i++){
        RantRay2D r;
        RantRayHit2D h;
        double at;
        int met, traced;
        r.origin = i % 2 ? in_box2(b, 0.5 * reach) : rant_double2_lerp(c, in_box2(b, 0.0), random_unit());
        r.direction = rant_double2_normalize(i % 3 ? random_double2(1.0) : rant_double2_subtract(in_box2(b, 0.0), r.origin));
        if (rant_double2_length_squared(r.direction) == 0.0) continue;
        met = s->raycast(s->shape, r, &h);
        traced = march2(s, r, 10.0 * reach, &at);
        if (traced == 1) CHECK(met && near(h.distance, at, 1e-7 * (1.0 + reach)), "%s: the raycast meets where marching does", s->name);
        if (traced == 0) CHECK(!met, "%s: the raycast misses where marching does", s->name);
        if (met) CHECK(rant_double2_dot(s->closest(s->shape, h.point).normal, h.normal) > 0.999, "%s: the hit normal is the boundary's", s->name);
    }
    for (i = 0; i < 60000; i++){
        RantDouble2 q = in_box2(b, 0.0);
        total++;
        if (s->contains(s->shape, q)){ inside++; sum = rant_double2_add(sum, q); }
    }
    {   double box = size.x * size.y, p = (double)inside / total, want = s->area(s->shape);
        CHECK(fabs(box * p - want) <= 4.0 * box * sqrt(p * (1.0 - p) / total) + 1e-12, "%s: area %g against a count of %g", s->name, want, box * p);
        if (inside > 1000) CHECK(near2(rant_double2_scale(sum, 1.0 / inside), c, 0.02 * reach), "%s: the centroid is the mean of the inside", s->name);
    }
}

static void random_regions(void){
    int i;
    for (i = 0; i < 8; i++){
        RantAlignedBox2D ab;
        RantOrientedBox2D ob;
        RantCircle ci;
        RantCapsule2D ca;
        RantPolygon2DView pg;
        RantDouble2 pts[40];
        Region s;
        ab.min = random_double2(3.0);
        ab.max = rant_double2_add(ab.min, rant_double2(random_range(0.2, 3), random_range(0.2, 3)));
        s = aligned_box2d_region(&ab); check_region(&s);
        ob.pose.position = random_double2(3.0); ob.pose.angle = random_range(-PI, PI);
        ob.size = rant_double2(random_range(0.2, 3), random_range(0.2, 3));
        s = oriented_box2d_region(&ob); check_region(&s);
        ci.center = random_double2(3.0); ci.radius = random_range(0.2, 2.0);
        s = circle_region(&ci); check_region(&s);
        ca.axis.a = random_double2(2.0); ca.axis.b = i ? random_double2(2.0) : ca.axis.a; ca.radius = random_range(0.2, 1.5);
        s = capsule2d_region(&ca); check_region(&s);
        pg.points = pts;
        pg.count = random_polygon(pts, 30);
        if (i == 1) pts[pg.count++] = pts[0];
        if (i == 2){ pts[pg.count] = pts[pg.count - 1]; pg.count++; }
        s = polygon2d_region(&pg); check_region(&s);
    }
}

static void boxes(void){
    RantAlignedBox2D b, o;
    RantDouble2 c[4];
    RantSegment2D e[4];
    RantPlane2D p[4];
    RantRayHit2D h;
    RantRay2D r;
    int i;
    b.min = rant_double2(0, 0); b.max = rant_double2(2, 4);
    CHECK(near(rant_aligned_box2d_distance(b, rant_double2(1, 1)), -1.0, 0.0) && near(rant_aligned_box2d_distance(b, rant_double2(5, 8)), 5.0, 0.0),
          "box2d distance in and out");
    CHECK(near(rant_aligned_box2d_area(b), 8.0, 0.0) && near(rant_aligned_box2d_perimeter(b), 12.0, 0.0), "area and perimeter");
    rant_aligned_box2d_corners(b, c);
    rant_aligned_box2d_edges(b, e);
    rant_aligned_box2d_planes(b, p);
    CHECK(near2(c[0], b.min, 0.0) && near2(c[1], rant_double2(2, 0), 0.0) && near2(c[2], b.max, 0.0) && near2(c[3], rant_double2(0, 4), 0.0),
          "corners counterclockwise from min");
    for (i = 0; i < 4; i++){
        RantDouble2 mid = rant_double2_lerp(e[i].a, e[i].b, 0.5);
        RantPlane2D from;
        CHECK(near2(e[i].a, c[i], 0.0) && near2(e[i].b, c[(i + 1) % 4], 0.0), "edge %d runs from corner %d", i, i);
        CHECK(near(rant_plane2d_distance(p[i], mid), 0.0, 0.0) && rant_plane2d_distance(p[i], rant_double2(1, 2)) < 0.0,
              "plane %d holds its edge and faces out", i);
        rant_plane2d_from_points(e[i].a, e[i].b, &from);
        CHECK(near2(from.normal, p[i].normal, 1e-15), "the planes are the edges' own lines");
    }
    r.origin = rant_double2(1, 1); r.direction = rant_double2(1, 0);
    CHECK(rant_aligned_box2d_raycast(b, r, &h) && near(h.distance, 1.0, 0.0) && near2(h.normal, rant_double2(1, 0), 0.0), "out through the right side");
    o.min = rant_double2(1, 3); o.max = rant_double2(5, 5);
    CHECK(rant_aligned_box2d_overlap(b, o, &o) && near2(o.min, rant_double2(1, 3), 0.0) && near2(o.max, rant_double2(2, 4), 0.0), "box2d overlap");
    o.min = rant_double2(2, 4); o.max = rant_double2(3, 5);
    CHECK(rant_aligned_box2d_overlap(b, o, NULL), "boxes touching at a corner overlap");
    o = rant_aligned_box2d_merge(b, o);
    CHECK(near2(o.max, rant_double2(3, 5), 0.0) && near2(o.min, b.min, 0.0), "box2d merge");
    o = rant_aligned_box2d_grow(b, 1.0);
    CHECK(near2(o.min, rant_double2(-1, -1), 0.0) && near2(o.max, rant_double2(3, 5), 0.0), "box2d grow");
    o = rant_aligned_box2d_grow(b, -1.5);
    CHECK(near2(o.min, rant_double2(1, 1.5), 0.0) && near2(o.max, rant_double2(1, 2.5), 0.0), "box2d shrink stops at the center");
    for (i = 0; i < 200; i++){
        RantPose2D pose;
        RantOrientedBox2D ob;
        RantDouble2 q = random_double2(6.0), oc[4];
        RantPlane2D op[4];
        int j;
        pose.position = random_double2(4.0); pose.angle = random_range(-PI, PI);
        ob = rant_aligned_box2d_transform(b, pose);
        CHECK(near(rant_oriented_box2d_distance(ob, rant_pose2d_apply(pose, q)), rant_aligned_box2d_distance(b, q), 1e-12),
              "a moved box2d keeps every distance");
        rant_oriented_box2d_corners(ob, oc);
        rant_oriented_box2d_planes(ob, op);
        for (j = 0; j < 4; j++){
            CHECK(near2(oc[j], rant_pose2d_apply(pose, c[j]), 1e-12), "oriented corners in the same order");
            CHECK(near(rant_plane2d_distance(op[j], rant_pose2d_apply(pose, q)), rant_plane2d_distance(p[j], q), 1e-12),
                  "oriented planes in the same order");
        }
        CHECK(near(rant_oriented_box2d_area(ob), 8.0, 1e-12) && near(rant_oriented_box2d_perimeter(ob), 12.0, 1e-12), "oriented area and perimeter");
        {   RantAlignedBox2D bb = rant_oriented_box2d_bounds(ob), cb;
            cb.min = cb.max = oc[0];
            for (j = 1; j < 4; j++){ cb.min = rant_double2_min(cb.min, oc[j]); cb.max = rant_double2_max(cb.max, oc[j]); }
            CHECK(near2(bb.min, cb.min, 1e-12) && near2(bb.max, cb.max, 1e-12), "oriented bounds are the corners' box");
        }
    }
    {   RantCircle ci;
        RantCapsule2D ca;
        ci.center = rant_double2(1, 1); ci.radius = 2.0;
        ca.axis.a = rant_double2(0, 0); ca.axis.b = rant_double2(3, 4); ca.radius = 1.0;
        CHECK(near(rant_circle_area(ci), 4.0 * PI, 1e-14) && near(rant_circle_perimeter(ci), 4.0 * PI, 1e-14), "circle area and perimeter");
        CHECK(near(rant_capsule2d_area(ca), PI + 10.0, 1e-14) && near(rant_capsule2d_perimeter(ca), 2.0 * PI + 10.0, 1e-14), "capsule2d area and perimeter");
        CHECK(near2(rant_circle_closest(ci, ci.center).normal, rant_double2(1, 0), 0.0), "a circle's center takes +x");
        CHECK(near(rant_double2_length(rant_capsule2d_closest(ca, rant_double2(1.5, 2)).normal), 1.0, 1e-15), "a point on the axis has a normal in the plane");
    }
}

static double triangle_area(RantDouble2 a, RantDouble2 b, RantDouble2 c){
    return 0.5 * rant_double2_cross(rant_double2_subtract(b, a), rant_double2_subtract(c, a));
}

static void check_triangulation(RantPolygon2DView p, uint32_t distinct, const char *what){
    uint32_t tri[3 * 64], count = 64, k;
    double sum = 0.0;
    RantMathResult r = rant_polygon2d_triangulate(p, tri, &count);
    CHECK(r == RANT_MATH_OK && count == distinct - 2, "%s: triangulate makes n - 2 triangles", what);
    for (k = 0; k < count && r == RANT_MATH_OK; k++){
        RantDouble2 a = p.points[tri[3 * k]], b = p.points[tri[3 * k + 1]], c = p.points[tri[3 * k + 2]];
        double area = triangle_area(a, b, c);
        RantDouble2 mid = rant_double2_scale(rant_double2_add(rant_double2_add(a, b), c), 1.0 / 3.0);
        CHECK(area >= -1e-12, "%s: every triangle runs counterclockwise", what);
        CHECK(area < 1e-12 || rant_polygon2d_distance(p, mid) <= 1e-12, "%s: every triangle lies inside", what);
        CHECK(tri[3 * k] < p.count && tri[3 * k + 1] < p.count && tri[3 * k + 2] < p.count, "%s: indices are the polygon's", what);
        sum += area;
    }
    CHECK(near(sum, rant_polygon2d_area(p), 1e-10), "%s: the triangles cover the area", what);
}

static void polygons(void){
    RantDouble2 comb[40], pts[64];
    RantPolygon2DView p;
    uint32_t i, n, count;
    RantPlane2D planes[64];
    RantSegment2D edges[64];
    /* a comb: deep teeth, both windings */
    n = 0;
    for (i = 0; i < 8; i++){
        comb[n++] = rant_double2(2.0 * i, 0.0);
        comb[n++] = rant_double2(2.0 * i + 1.0, 5.0);
    }
    comb[n++] = rant_double2(16, 0);
    comb[n++] = rant_double2(16, -1);
    comb[n++] = rant_double2(0, -1);
    p.points = comb; p.count = n;
    CHECK(near(rant_polygon2d_signed_area(p), -56.0, 1e-12), "a comb running clockwise has a negative area");
    check_triangulation(p, n, "comb");
    CHECK(!rant_polygon2d_is_convex(p), "a comb is not convex");
    for (i = 0; i < n / 2; i++){ RantDouble2 t = comb[i]; comb[i] = comb[n - 1 - i]; comb[n - 1 - i] = t; }
    check_triangulation(p, n, "reversed comb");
    {   RantDouble2 sq[6];
        sq[0] = rant_double2(0, 0); sq[1] = rant_double2(1, 0); sq[2] = rant_double2(2, 0);
        sq[3] = rant_double2(2, 2); sq[4] = rant_double2(0, 2); sq[5] = sq[0];
        p.points = sq; p.count = 6;
        CHECK(near(rant_polygon2d_signed_area(p), 4.0, 0.0) && near(rant_polygon2d_perimeter(p), 8.0, 0.0), "square area and perimeter");
        CHECK(near2(rant_polygon2d_centroid(p), rant_double2(1, 1), 1e-15), "square centroid");
        CHECK(rant_polygon2d_is_convex(p), "a square with a corner on an edge and a repeated end is convex");
        check_triangulation(p, 5, "square with a point on an edge");
        count = 0;
        CHECK(rant_polygon2d_planes(p, NULL, &count) == RANT_MATH_ERR_SPACE && count == 5, "the repeated end makes no plane");
        sq[2] = rant_double2(1, -1);
        CHECK(!rant_polygon2d_is_convex(p), "a dented square is not");
    }
    {   RantDouble2 star[5];
        for (i = 0; i < 5; i++) star[i] = rant_double2(cos(4.0 * PI * i / 5), sin(4.0 * PI * i / 5));
        p.points = star; p.count = 5;
        CHECK(!rant_polygon2d_is_convex(p), "a pentagram turns one way but twice round");
        CHECK(rant_polygon2d_distance(p, rant_double2(0, 0)) < 0.0, "its middle counts as inside by winding");
    }
    for (i = 0; i < 300; i++){
        uint32_t k, m;
        double sum = 0.0;
        RantPose2D pose;
        p.points = pts;
        p.count = random_polygon(pts, 40);
        m = p.count;
        check_triangulation(p, m, "random");
        count = 64;
        CHECK(rant_polygon2d_planes(p, planes, &count) == RANT_MATH_OK && count == m, "one plane per edge");
        count = 64;
        CHECK(rant_polygon2d_edges(p, edges, &count) == RANT_MATH_OK && count == m, "one edge per side");
        for (k = 0; k < m; k++){
            RantDouble2 mid = rant_double2_lerp(edges[k].a, edges[k].b, 0.5);
            double l = rant_segment2d_length(edges[k]);
            sum += l;
            CHECK(near(rant_plane2d_distance(planes[k], edges[k].a), 0.0, 1e-12) && near(rant_plane2d_distance(planes[k], edges[k].b), 0.0, 1e-12),
                  "plane k holds edge k");
            CHECK(!rant_polygon2d_contains(p, rant_double2_add(mid, rant_double2_scale(planes[k].normal, 1e-6 * l)))
                  && rant_polygon2d_contains(p, rant_double2_subtract(mid, rant_double2_scale(planes[k].normal, 1e-6 * l))),
                  "every plane faces out whichever way the polygon winds");
        }
        CHECK(near(sum, rant_polygon2d_perimeter(p), 1e-12), "the edges add to the perimeter");
        pose.position = random_double2(3.0); pose.angle = random_range(-PI, PI);
        {   RantDouble2 moved[64], q = random_double2(3.0);
            RantPolygon2DView mp;
            rant_pose2d_apply_points(pose, pts, moved, p.count);
            mp.points = moved; mp.count = p.count;
            CHECK(near(rant_polygon2d_distance(mp, rant_pose2d_apply(pose, q)), rant_polygon2d_distance(p, q), 1e-12), "moving a polygon keeps distances");
            CHECK(near(rant_polygon2d_signed_area(mp), rant_polygon2d_signed_area(p), 1e-12), "and its area");
        }
        {   RantDouble2 hull[4];
            RantPolygon2DView t;
            hull[0] = rant_double2(0, 0); hull[1] = rant_double2(random_range(0.1, 3), 0); hull[2] = random_double2(3.0);
            t.points = hull; t.count = 3;
            CHECK(rant_polygon2d_is_convex(t) == (fabs(rant_polygon2d_signed_area(t)) > 1e-12), "every real triangle is convex");
        }
    }
    {   RantPolygon2DView empty;
        uint32_t tri[3], c0 = 1;
        empty.points = NULL; empty.count = 5;
        CHECK(rant_polygon2d_distance(empty, rant_double2(0, 0)) == HUGE_VAL && !rant_polygon2d_contains(empty, rant_double2(0, 0)),
              "a polygon of no points is nowhere");
        CHECK(rant_polygon2d_bounds(empty).min.x > rant_polygon2d_bounds(empty).max.x, "and its bounds are empty");
        CHECK(rant_polygon2d_triangulate(empty, tri, &c0) == RANT_MATH_OK && c0 == 0, "and it makes no triangles");
        CHECK(rant_polygon2d_triangulate(empty, tri, NULL) == RANT_MATH_ERR_ARGUMENT, "a list call needs its count");
    }
}

static RantOrientedBox2D obox2(double x, double y, double angle, double w, double h){
    RantOrientedBox2D b;
    b.pose.position = rant_double2(x, y);
    b.pose.angle = angle;
    b.size = rant_double2(w, h);
    return b;
}

/* Against qhull's intersection of the half planes, and the cases where the answer is known. */
static void iou(void){
    RantOrientedBox2D a, b;
    RantAlignedBox2D p, q;
    double worst = 0.0;
    int i, bad = 0, odd = 0;
    for (i = 0; i < REF_IOU2D_CASES; i++){
        const double *r = REF_IOU2D + 11 * i;
        double got;
        a = obox2(r[0], r[1], r[2], r[3], r[4]);
        b = obox2(r[5], r[6], r[7], r[8], r[9]);
        got = rant_oriented_box2d_iou(a, b);
        if (fabs(got - r[10]) > worst) worst = fabs(got - r[10]);
        if (!near(got, r[10], 1e-9)) bad++;
    }
    CHECK(bad == 0, "%d IoU of %d off qhull's, worst %.3g", bad, REF_IOU2D_CASES, worst);
    for (i = 0; i < 300; i++){
        RantPose2D move;
        double ab, ba, moved;
        a = obox2(random_range(-1, 1), random_range(-1, 1), random_range(-PI, PI), random_range(0.2, 3), random_range(0.2, 3));
        b = obox2(random_range(-1, 1), random_range(-1, 1), random_range(-PI, PI), random_range(0.2, 3), random_range(0.2, 3));
        move.position = random_double2(50.0);
        move.angle = random_range(-PI, PI);
        ab = rant_oriented_box2d_iou(a, b);
        ba = rant_oriented_box2d_iou(b, a);
        moved = rant_oriented_box2d_iou(rant_oriented_box2d_transform(a, move), rant_oriented_box2d_transform(b, move));
        if (!near(ab, ba, 1e-12) || !near(ab, moved, 1e-10) || ab < 0.0 || ab > 1.0) odd++;
    }
    CHECK(odd == 0, "%d pairs differ swapped or moved together", odd);
    a = obox2(1, 2, 0.7, 2, 3);
    CHECK(near(rant_oriented_box2d_iou(a, a), 1.0, 1e-12), "a box with itself");
    b = a; b.pose.angle += PI;
    CHECK(near(rant_oriented_box2d_iou(a, b), 1.0, 1e-12), "turned half round it is the same box");
    b = a; b.size = rant_double2(-2, -3);
    CHECK(near(rant_oriented_box2d_iou(a, b), 1.0, 1e-12), "negative sizes count by their length");
    b = obox2(1, 2, 0.7, 1, 1.5);
    CHECK(near(rant_oriented_box2d_iou(a, b), 0.25, 1e-12), "a quarter of it inside it");
    b = obox2(1 + 2 * cos(0.7), 2 + 2 * sin(0.7), 0.7, 2, 3);
    CHECK(near(rant_oriented_box2d_iou(a, b), 0.0, 1e-12), "touching along an edge shares nothing");
    b = obox2(40, 0, 0, 2, 2);
    CHECK(rant_oriented_box2d_iou(a, b) == 0.0, "far apart");
    b = obox2(1, 2, 0.7, 0, 3);
    CHECK(rant_oriented_box2d_iou(a, b) == 0.0 && rant_oriented_box2d_iou(b, b) == 0.0, "a box of no area");
    a = obox2(0, 0, 0, 2, 2);
    b = obox2(1, 0, 1e-13, 2, 2);
    CHECK(near(rant_oriented_box2d_iou(a, b), 1.0 / 3.0, 1e-12), "all but lined up");
    p.min = rant_double2(0, 0); p.max = rant_double2(2, 2);
    q.min = rant_double2(1, 1); q.max = rant_double2(3, 4);
    CHECK(near(rant_aligned_box2d_iou(p, q), 1.0 / 9.0, 1e-15), "aligned: 1 shared of 9");
    CHECK(near(rant_aligned_box2d_iou(p, q), rant_oriented_box2d_iou(rant_oriented_box2d_from_aligned_box2d(p),
                                                                   rant_oriented_box2d_from_aligned_box2d(q)), 1e-12), "as the oriented boxes");
    q.min = rant_double2(2, 0); q.max = rant_double2(3, 2);
    CHECK(rant_aligned_box2d_iou(p, q) == 0.0, "aligned and touching");
    q.min = rant_double2(3, 3); q.max = rant_double2(1, 1);
    CHECK(rant_aligned_box2d_iou(p, q) == 0.0, "an inverted box is empty");
}

void test_region(void){
    boxes();
    polygons();
    random_regions();
    iou();
}
