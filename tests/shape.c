#include <stdlib.h>
#include "check.h"

static const uint32_t box_triangles[36] = {
    0, 2, 1, 0, 3, 2,   4, 5, 6, 4, 6, 7,   0, 1, 5, 0, 5, 4,
    1, 2, 6, 1, 6, 5,   2, 3, 7, 2, 7, 6,   3, 0, 4, 3, 4, 7
};

/* Storage that the views of one random shape point into. */
typedef struct {
    RantDouble2 base[40];
    RantDouble3 pts[40];
    RantDouble3 corners[8];
} Store;

static int is_solid(RantShapeKind k){
    return k != RANT_SHAPE_SEGMENT && k != RANT_SHAPE_POLYLINE && k != RANT_SHAPE_POLYGON;
}

static RantShape random_shape(RantShapeKind kind, Store *st, RantDouble3 at){
    RantShape s;
    uint32_t i, n;
    RantPose pose = random_pose(0.0);
    pose.position = at;
    switch (kind){
    case RANT_SHAPE_ALIGNED_BOX: {
        RantAlignedBox b;
        b.min = rant_double3_add(at, random_double3(1.0));
        b.max = rant_double3_add(b.min, rant_double3(random_range(0.2, 2), random_range(0.2, 2), random_range(0.2, 2)));
        return rant_shape_from_aligned_box(b);
    }
    case RANT_SHAPE_ORIENTED_BOX: {
        RantOrientedBox b;
        b.pose = pose;
        b.size = rant_double3(random_range(0.2, 2), random_range(0.2, 2), random_range(0.2, 2));
        return rant_shape_from_oriented_box(b);
    }
    case RANT_SHAPE_SPHERE: {
        RantSphere b;
        b.center = at; b.radius = random_range(0.2, 1.2);
        return rant_shape_from_sphere(b);
    }
    case RANT_SHAPE_CAPSULE: {
        RantCapsule c;
        c.axis.a = rant_double3_add(at, random_double3(1.0)); c.axis.b = rant_double3_add(at, random_double3(1.0)); c.radius = random_range(0.2, 0.8);
        return rant_shape_from_capsule(c);
    }
    case RANT_SHAPE_CYLINDER: {
        RantCylinder c;
        c.axis.a = rant_double3_add(at, random_double3(1.0)); c.axis.b = rant_double3_add(at, random_double3(1.0)); c.radius = random_range(0.2, 0.8);
        return rant_shape_from_cylinder(c);
    }
    case RANT_SHAPE_CONE: {
        RantCone c;
        c.base = rant_double3_add(at, random_double3(1.0)); c.tip = rant_double3_add(at, random_double3(1.0)); c.radius = random_range(0.2, 0.8);
        return rant_shape_from_cone(c);
    }
    case RANT_SHAPE_PRISM: {
        RantPrismView p;
        p.pose = pose;
        p.base.points = st->base;
        p.base.count = random_polygon(st->base, 12);
        for (i = 0; i < p.base.count; i++) st->base[i] = rant_double2_scale(st->base[i], 0.6);
        p.height = random_range(0.3, 1.5);
        return rant_shape_from_prism(p);
    }
    case RANT_SHAPE_SEGMENT: {
        RantSegment g;
        g.a = rant_double3_add(at, random_double3(1.5)); g.b = rant_double3_add(at, random_double3(1.5));
        return rant_shape_from_segment(g);
    }
    case RANT_SHAPE_POLYLINE: {
        RantPolylineView l;
        n = 2 + (uint32_t)(random_unit() * 6);
        for (i = 0; i < n; i++) st->pts[i] = rant_double3_add(at, random_double3(1.5));
        l.points = st->pts; l.count = n;
        return rant_shape_from_polyline(l);
    }
    case RANT_SHAPE_POLYGON: {
        RantPolygonView p;
        n = random_polygon(st->base, 12);
        for (i = 0; i < n; i++) st->pts[i] = rant_pose_apply(pose, rant_double3(st->base[i].x, st->base[i].y, 0.0));
        p.points = st->pts; p.count = n;
        return rant_shape_from_polygon(p);
    }
    case RANT_SHAPE_MESH: {
        RantMeshView m;
        RantOrientedBox b;
        b.pose = pose;
        b.size = rant_double3(random_range(0.3, 2), random_range(0.3, 2), random_range(0.3, 2));
        rant_oriented_box_corners(b, st->corners);
        m.vertices = st->corners; m.vertex_count = 8; m.triangles = box_triangles; m.triangle_count = 12;
        return rant_shape_from_mesh(m);
    }
    }
    s.kind = RANT_SHAPE_SPHERE;
    return s;
}

static double typed_distance(const RantShape *s, RantDouble3 q){
    switch (s->kind){
    case RANT_SHAPE_ALIGNED_BOX: return rant_aligned_box_distance(s->as.aligned_box, q);
    case RANT_SHAPE_ORIENTED_BOX: return rant_oriented_box_distance(s->as.oriented_box, q);
    case RANT_SHAPE_SPHERE: return rant_sphere_distance(s->as.sphere, q);
    case RANT_SHAPE_CAPSULE: return rant_capsule_distance(s->as.capsule, q);
    case RANT_SHAPE_CYLINDER: return rant_cylinder_distance(s->as.cylinder, q);
    case RANT_SHAPE_CONE: return rant_cone_distance(s->as.cone, q);
    case RANT_SHAPE_PRISM: return rant_prism_distance(s->as.prism, q);
    case RANT_SHAPE_SEGMENT: return rant_segment_distance(s->as.segment, q);
    case RANT_SHAPE_POLYLINE: return rant_polyline_distance(s->as.polyline, q);
    case RANT_SHAPE_POLYGON: return rant_polygon_distance(s->as.polygon, q);
    case RANT_SHAPE_MESH: return rant_mesh_distance(s->as.mesh, q);
    }
    return 0.0;
}

#define KINDS 11
#define SAMPLES 600

static void dispatch(void){
    int k, i;
    for (k = 0; k < KINDS; k++){
        Store st;
        RantShape s = random_shape((RantShapeKind)k, &st, random_double3(1.0));
        RantDouble3 q[50];
        double d[50];
        for (i = 0; i < 50; i++) q[i] = random_double3(3.0);
        rant_shape_distances(&s, q, d, 50);
        for (i = 0; i < 50; i++){
            RantSurfacePoint c = rant_shape_closest(&s, q[i]);
            CHECK(d[i] == typed_distance(&s, q[i]) && rant_shape_distance(&s, q[i]) == d[i], "kind %d: one distance, typed or not", k);
            CHECK(near(fabs(d[i]), rant_double3_distance(c.point, q[i]), 1e-12), "kind %d: closest lies at the distance", k);
            if (!is_solid(s.kind) && s.kind != RANT_SHAPE_POLYGON)
                CHECK(near3(rant_double3_add(c.point, rant_double3_scale(c.normal, d[i])), q[i], 1e-12), "kind %d: a curve's normal points at the point", k);
            CHECK(rant_shape_contains(&s, q[i]) == (is_solid(s.kind) && (s.kind == RANT_SHAPE_MESH ? rant_mesh_contains(s.as.mesh, q[i]) : d[i] <= 0.0)),
                  "kind %d: contains", k);
        }
    }
}

/* A shape's surface, or the shape itself for a curve, as sampled points. */
static void surface_of(const RantShape *s, uint64_t seed, RantDouble3 *out){
    RantMathResult r = rant_shape_sample_surface(s, seed, out, SAMPLES);
    CHECK(r == RANT_MATH_OK, "kind %d: surface sampling runs", (int)s->kind);
}

static void gaps(void){
    static RantDouble3 sa[SAMPLES], sb[SAMPLES], ia[SAMPLES];
    int i, ka, kb;
    for (i = 0; i < 4; i++)
        for (ka = 0; ka < KINDS; ka++)
            for (kb = 0; kb < KINDS; kb++){
                Store sta, stb;
                RantDouble3 at = rant_double3(random_range(0.0, 3.5), random_range(-1, 1), random_range(-1, 1));
                RantShape a = random_shape((RantShapeKind)ka, &sta, rant_double3(0, 0, 0));
                RantShape b = random_shape((RantShapeKind)kb, &stb, at);
                RantSegment between;
                double gap, tol = 1e-7;
                int k, over = 0;
                CHECK(rant_shape_gap(&a, &b, &gap, &between) == RANT_MATH_OK, "%d %d: gap runs", ka, kb);
                CHECK(near(rant_double3_distance(between.a, between.b), gap, 1e-9), "%d %d: the nearest points are the gap apart", ka, kb);
                CHECK(fabs(rant_shape_distance(&a, between.a)) <= tol || (is_solid(a.kind) && rant_shape_distance(&a, between.a) <= tol),
                      "%d %d: the nearest point of a lies on a", ka, kb);
                CHECK(fabs(rant_shape_distance(&b, between.b)) <= tol || (is_solid(b.kind) && rant_shape_distance(&b, between.b) <= tol),
                      "%d %d: the nearest point of b lies on b", ka, kb);
                surface_of(&a, 11 + (uint64_t)i, sa);
                surface_of(&b, 22 + (uint64_t)i, sb);
                {   double best = HUGE_VAL;
                    for (k = 0; k < SAMPLES; k += 3){
                        int m;
                        for (m = 0; m < SAMPLES; m += 3){
                            double e = rant_double3_distance(sa[k], sb[m]);
                            if (e < best) best = e;
                        }
                    }
                    CHECK(gap <= best + 1e-9, "%d %d: no two surface points are nearer than the gap", ka, kb);
                }
                if (b.kind == RANT_SHAPE_SPHERE){
                    double want = typed_distance(&a, b.as.sphere.center) - b.as.sphere.radius;
                    CHECK(near(gap, want > 0.0 ? want : 0.0, 1e-8), "%d: the gap to a ball is the distance to its center less its radius", ka);
                }
                if (is_solid(a.kind) && rant_shape_sample_inside(&a, 33 + (uint64_t)i, ia, SAMPLES) == RANT_MATH_OK)
                    for (k = 0; k < SAMPLES && !over; k++) over = rant_shape_contains(&b, ia[k]) || rant_shape_distance(&b, ia[k]) <= 0.0;
                if (over) CHECK(gap == 0.0 && rant_shape_overlaps(&a, &b), "%d %d: shapes sharing a point overlap", ka, kb);
                if (gap > 1e-6 && ka <= RANT_SHAPE_CONE && kb <= RANT_SHAPE_CONE){
                    /* two convex shapes apart: the plane square to the gap separates them. The nearest
                     * points of curved shapes are good to about the root of the gap's accuracy. */
                    RantDouble3 n = rant_double3_scale(rant_double3_subtract(between.b, between.a), 1.0 / gap);
                    double top = rant_double3_dot(between.a, n), bottom = rant_double3_dot(between.b, n);
                    for (k = 0; k < SAMPLES; k++){
                        CHECK(rant_double3_dot(sa[k], n) <= top + 1e-5, "%d %d: all of a lies behind the plane", ka, kb);
                        CHECK(rant_double3_dot(sb[k], n) >= bottom - 1e-5, "%d %d: all of b lies past it", ka, kb);
                    }
                }
                if (gap == 0.0 && is_solid(a.kind) && is_solid(b.kind) && a.kind != RANT_SHAPE_MESH && b.kind != RANT_SHAPE_MESH)
                    CHECK(rant_shape_distance(&a, between.a) <= tol && rant_shape_distance(&b, between.a) <= tol,
                          "%d %d: overlapping solids give a point in both", ka, kb);
            }
    {   RantSphere x, y;
        RantShape a, b;
        RantSegment between;
        double gap;
        x.center = rant_double3(0, 0, 0); x.radius = 1.0;
        y.center = rant_double3(5, 0, 0); y.radius = 2.0;
        a = rant_shape_from_sphere(x); b = rant_shape_from_sphere(y);
        CHECK(rant_shape_gap(&a, &b, &gap, &between) == RANT_MATH_OK && near(gap, 2.0, 1e-12)
              && near3(between.a, rant_double3(1, 0, 0), 1e-12) && near3(between.b, rant_double3(3, 0, 0), 1e-12), "two balls two apart");
        y.center = rant_double3(2, 0, 0);
        b = rant_shape_from_sphere(y);
        CHECK(rant_shape_gap(&a, &b, &gap, &between) == RANT_MATH_OK && gap == 0.0 && near3(between.a, rant_double3(2.0 / 3.0, 0, 0), 1e-12),
              "overlapping balls share the point their radii split the gap at");
        CHECK(rant_shape_gap(&a, NULL, &gap, NULL) == RANT_MATH_ERR_ARGUMENT, "a gap needs two shapes");
    }
}

static void sampling(void){
    static RantDouble3 p[4000], q[4000];
    int k, i;
    for (k = 0; k < KINDS; k++){
        Store st;
        RantShape s = random_shape((RantShapeKind)k, &st, random_double3(1.0));
        RantMathResult r = rant_shape_sample_inside(&s, 7, p, 4000);
        if (!is_solid(s.kind)){
            CHECK(r == RANT_MATH_ERR_ARGUMENT, "kind %d: a curve or a surface has no inside", k);
        } else {
            RantDouble3 mean = rant_double3(0, 0, 0), want;
            CHECK(r == RANT_MATH_OK, "kind %d: inside sampling runs", k);
            for (i = 0; i < 4000; i++){
                CHECK(rant_shape_distance(&s, p[i]) <= 1e-9 || (s.kind == RANT_SHAPE_MESH && rant_shape_contains(&s, p[i])), "kind %d: inside points are inside", k);
                mean = rant_double3_add(mean, rant_double3_scale(p[i], 1.0 / 4000));
            }
            switch (s.kind){
            case RANT_SHAPE_ALIGNED_BOX: want = rant_aligned_box_centroid(s.as.aligned_box); break;
            case RANT_SHAPE_ORIENTED_BOX: want = s.as.oriented_box.pose.position; break;
            case RANT_SHAPE_SPHERE: want = s.as.sphere.center; break;
            case RANT_SHAPE_CAPSULE: want = rant_capsule_centroid(s.as.capsule); break;
            case RANT_SHAPE_CYLINDER: want = rant_cylinder_centroid(s.as.cylinder); break;
            case RANT_SHAPE_CONE: want = rant_cone_centroid(s.as.cone); break;
            case RANT_SHAPE_PRISM: want = rant_prism_centroid(s.as.prism); break;
            default: want = rant_mesh_centroid(s.as.mesh); break;
            }
            CHECK(near3(mean, want, 0.04), "kind %d: inside points spread evenly round the centroid", k);
            rant_shape_sample_inside(&s, 7, q, 4000);
            CHECK(memcmp(p, q, sizeof p) == 0, "kind %d: the same seed gives the same points", k);
            rant_shape_sample_inside(&s, 8, q, 4000);
            CHECK(memcmp(p, q, sizeof p) != 0, "kind %d: another seed gives others", k);
        }
        CHECK(rant_shape_sample_surface(&s, 9, p, 4000) == RANT_MATH_OK, "kind %d: surface sampling runs", k);
        for (i = 0; i < 4000; i++) CHECK(fabs(rant_shape_distance(&s, p[i])) <= 1e-9, "kind %d: surface points lie on the surface", k);
    }
    {   /* a cylinder's caps and side get their share by area */
        RantCylinder c;
        RantShape s;
        int caps = 0;
        c.axis.a = rant_double3(0, 0, 0); c.axis.b = rant_double3(0, 0, 3); c.radius = 1.0;
        s = rant_shape_from_cylinder(c);
        rant_shape_sample_surface(&s, 5, p, 4000);
        for (i = 0; i < 4000; i++) caps += p[i].z < 1e-12 || p[i].z > 3.0 - 1e-12;
        CHECK(fabs(caps / 4000.0 - 0.25) < 0.03, "a cylinder twice as tall as wide is a quarter caps");
    }
    {   /* a cone's slope gets more points away from the tip, so half lie below half the slant */
        RantCone c;
        RantShape s;
        int low = 0, side = 0;
        c.base = rant_double3(0, 0, 0); c.tip = rant_double3(0, 0, 2); c.radius = 1.0;
        s = rant_shape_from_cone(c);
        rant_shape_sample_surface(&s, 6, p, 4000);
        for (i = 0; i < 4000; i++){
            if (p[i].z < 1e-12) continue;
            side++;
            low += p[i].z < 2.0 * (1.0 - sqrt(0.5));
        }
        CHECK(fabs((double)low / side - 0.5) < 0.04, "a cone's slope is sampled by area");
        CHECK(fabs(side / 4000.0 - sqrt(5.0) / (1.0 + sqrt(5.0))) < 0.03, "and its base by its share");
    }
    {   RantMeshView m;
        RantShape s;
        RantDouble3 v[8];
        RantAlignedBox b;
        b.min = rant_double3(0, 0, 0); b.max = rant_double3(1, 1, 1);
        rant_aligned_box_corners(b, v);
        m.vertices = v; m.vertex_count = 8; m.triangles = box_triangles; m.triangle_count = 2;
        s = rant_shape_from_mesh(m);
        CHECK(rant_shape_sample_inside(&s, 1, p, 100) == RANT_MATH_ERR_DEGENERATE, "one face of a box has no inside to find");
    }
}

static RantShape2D random_shape2d(RantShape2DKind kind, RantDouble2 *store, RantDouble2 at){
    uint32_t i;
    switch (kind){
    case RANT_SHAPE2D_ALIGNED_BOX: {
        RantAlignedBox2D b;
        b.min = rant_double2_add(at, random_double2(1.0));
        b.max = rant_double2_add(b.min, rant_double2(random_range(0.2, 2), random_range(0.2, 2)));
        return rant_shape2d_from_aligned_box2d(b);
    }
    case RANT_SHAPE2D_ORIENTED_BOX: {
        RantOrientedBox2D b;
        b.pose.position = at; b.pose.angle = random_range(-PI, PI);
        b.size = rant_double2(random_range(0.2, 2), random_range(0.2, 2));
        return rant_shape2d_from_oriented_box2d(b);
    }
    case RANT_SHAPE2D_CIRCLE: {
        RantCircle c;
        c.center = at; c.radius = random_range(0.2, 1.2);
        return rant_shape2d_from_circle(c);
    }
    case RANT_SHAPE2D_CAPSULE: {
        RantCapsule2D c;
        c.axis.a = rant_double2_add(at, random_double2(1.0)); c.axis.b = rant_double2_add(at, random_double2(1.0)); c.radius = random_range(0.2, 0.8);
        return rant_shape2d_from_capsule2d(c);
    }
    case RANT_SHAPE2D_POLYGON: {
        RantPolygon2DView p;
        p.points = store;
        p.count = random_polygon(store, 12);
        for (i = 0; i < p.count; i++) store[i] = rant_double2_add(at, store[i]);
        return rant_shape2d_from_polygon2d(p);
    }
    case RANT_SHAPE2D_SEGMENT: {
        RantSegment2D g;
        g.a = rant_double2_add(at, random_double2(1.5)); g.b = rant_double2_add(at, random_double2(1.5));
        return rant_shape2d_from_segment2d(g);
    }
    case RANT_SHAPE2D_POLYLINE: default: {
        RantPolyline2DView l;
        l.points = store;
        l.count = 2 + (uint32_t)(random_unit() * 6);
        for (i = 0; i < l.count; i++) store[i] = rant_double2_add(at, random_double2(1.5));
        return rant_shape2d_from_polyline2d(l);
    }
    }
}

static void two_d(void){
    static RantDouble2 sa[SAMPLES], sb[SAMPLES];
    int i, ka, kb, k;
    for (i = 0; i < 6; i++)
        for (ka = 0; ka < 7; ka++)
            for (kb = 0; kb < 7; kb++){
                RantDouble2 pa[40], pb[40];
                RantShape2D a = random_shape2d((RantShape2DKind)ka, pa, rant_double2(0, 0));
                RantShape2D b = random_shape2d((RantShape2DKind)kb, pb, rant_double2(random_range(0, 3.5), random_range(-1, 1)));
                RantSegment2D between;
                double gap, best = HUGE_VAL;
                int m;
                CHECK(rant_shape2d_gap(&a, &b, &gap, &between) == RANT_MATH_OK && near(rant_double2_distance(between.a, between.b), gap, 1e-9),
                      "2D %d %d: gap runs and its points are the gap apart", ka, kb);
                CHECK(rant_shape2d_distance(&a, between.a) <= 1e-7 && rant_shape2d_distance(&b, between.b) <= 1e-7, "2D %d %d: on both shapes", ka, kb);
                CHECK(rant_shape2d_sample_surface(&a, 3, sa, SAMPLES) == RANT_MATH_OK && rant_shape2d_sample_surface(&b, 4, sb, SAMPLES) == RANT_MATH_OK,
                      "2D %d %d: boundary sampling runs", ka, kb);
                for (k = 0; k < SAMPLES; k++){
                    CHECK(fabs(rant_shape2d_distance(&a, sa[k])) <= 1e-9, "2D %d: boundary points lie on it", ka);
                    for (m = 0; m < SAMPLES; m += 4){
                        double e = rant_double2_distance(sa[k], sb[m]);
                        if (e < best) best = e;
                    }
                }
                CHECK(gap <= best + 1e-9, "2D %d %d: no two boundary points are nearer than the gap", ka, kb);
                if (b.kind == RANT_SHAPE2D_CIRCLE){
                    double want = rant_shape2d_distance(&a, b.as.circle.center) - b.as.circle.radius;
                    CHECK(near(gap, want > 0.0 ? want : 0.0, 1e-8), "2D %d: the gap to a circle is the distance to its center less its radius", ka);
                }
                if (ka <= RANT_SHAPE2D_POLYGON){
                    RantDouble2 in[200];
                    int over = 0;
                    CHECK(rant_shape2d_sample_inside(&a, 5, in, 200) == RANT_MATH_OK, "2D %d: inside sampling runs", ka);
                    for (k = 0; k < 200; k++){
                        CHECK(rant_shape2d_distance(&a, in[k]) <= 1e-9, "2D %d: inside points are inside", ka);
                        over |= rant_shape2d_distance(&b, in[k]) <= 0.0;
                    }
                    if (over) CHECK(rant_shape2d_overlaps(&a, &b), "2D %d %d: regions sharing a point overlap", ka, kb);
                } else {
                    RantDouble2 in[2];
                    CHECK(rant_shape2d_sample_inside(&a, 5, in, 2) == RANT_MATH_ERR_ARGUMENT, "2D %d: a line has no inside", ka);
                }
            }
    {   RantDouble2 wall[3];
        RantPolyline2DView l;
        RantShape2D s;
        RantRay2D r;
        RantRayHit2D h;
        wall[0] = rant_double2(2, -1); wall[1] = rant_double2(2, 1); wall[2] = rant_double2(4, 1);
        l.points = wall; l.count = 3;
        s = rant_shape2d_from_polyline2d(l);
        r.origin = rant_double2(0, 0.5); r.direction = rant_double2(1, 0);
        CHECK(rant_shape2d_raycast(&s, r, &h) && near(h.distance, 2.0, 0.0) && near2(h.normal, rant_double2(-1, 0), 0.0), "a laser meets a 2D wall");
        CHECK(!rant_shape2d_contains(&s, rant_double2(3, 0)), "a wall contains nothing");
    }
}

static int edge_order(const void *a, const void *b){
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return x < y ? -1 : (x > y);
}

/* Closed and wound one way: each edge runs once each way, never twice the same way. */
static int closed(const uint32_t *t, uint32_t nt){
    uint64_t *e = (uint64_t *)malloc(sizeof(uint64_t) * 3 * (nt ? nt : 1));
    uint32_t i, k;
    int ok = 1;
    if (!e) return 0;
    for (i = 0; i < nt; i++)
        for (k = 0; k < 3; k++) e[3 * i + k] = (uint64_t)t[3 * i + k] << 32 | t[3 * i + (k + 1) % 3];
    qsort(e, 3 * (size_t)nt, sizeof *e, edge_order);
    for (i = 0; i < 3 * nt && ok; i++){
        uint64_t back = e[i] >> 32 | e[i] << 32;
        ok = (i == 0 || e[i] != e[i - 1]) && bsearch(&back, e, 3 * (size_t)nt, sizeof *e, edge_order) != NULL;
    }
    free(e);
    return ok;
}

/* Each kind meshed: closed for a solid, on its surface, facing out, and its volume near the solid's. */
static void meshes(void){
    static RantDouble3 v[20000];
    static uint32_t t[3 * 40000];
    RantDouble2 base[7];
    RantDouble3 corners[5];
    RantShape s[9];
    double volume[9];
    RantAlignedBox ab;
    RantOrientedBox ob;
    RantSphere sp;
    RantCapsule cp;
    RantCylinder cy;
    RantCone co;
    RantPrismView pr;
    RantPolygonView pg;
    RantMeshView mv;
    uint32_t nv, nt, i, k;
    int bad;
    ab.min = rant_double3(-1, 0, 2); ab.max = rant_double3(2, 1.5, 3);
    ob.pose = random_pose(2.0); ob.size = rant_double3(1, 2, 0.5);
    sp.center = rant_double3(1, 2, 3); sp.radius = 1.5;
    cp.axis.a = rant_double3(0, 0, 0); cp.axis.b = rant_double3(1, 2, 2); cp.radius = 0.5;
    cy.axis.a = rant_double3(1, 1, 1); cy.axis.b = rant_double3(1, 1, -2); cy.radius = 0.7;
    co.base = rant_double3(0, 0, 0); co.tip = rant_double3(-1, 0.5, 2); co.radius = 1.0;
    base[0] = rant_double2(0, 0); base[1] = rant_double2(2, 0); base[2] = rant_double2(2, 2); base[3] = rant_double2(1, 0.8);
    base[4] = rant_double2(1, 0.8); base[5] = rant_double2(0, 2); base[6] = rant_double2(0, 0);
    /* clockwise, with a repeated corner and the first again at the end */
    for (i = 0; i < 3; i++){ RantDouble2 x = base[1 + i]; base[1 + i] = base[5 - i]; base[5 - i] = x; }
    pr.pose = random_pose(2.0); pr.base.points = base; pr.base.count = 7; pr.height = -1.25;
    for (i = 0; i < 5; i++) corners[i] = rant_pose_apply(pr.pose, rant_double3(cos(1.2 * i), sin(1.2 * i) * 0.5, 0.0));
    pg.points = corners; pg.count = 5;
    s[0] = rant_shape_from_aligned_box(ab); volume[0] = rant_aligned_box_volume(ab);
    s[1] = rant_shape_from_oriented_box(ob); volume[1] = rant_oriented_box_volume(ob);
    s[2] = rant_shape_from_sphere(sp); volume[2] = rant_sphere_volume(sp);
    s[3] = rant_shape_from_capsule(cp); volume[3] = rant_capsule_volume(cp);
    s[4] = rant_shape_from_cylinder(cy); volume[4] = rant_cylinder_volume(cy);
    s[5] = rant_shape_from_cone(co); volume[5] = rant_cone_volume(co);
    s[6] = rant_shape_from_prism(pr); volume[6] = rant_prism_volume(pr);
    s[7] = rant_shape_from_polygon(pg); volume[7] = 0.0;
    for (k = 0; k < 8; k++){
        double err[2] = { 0.0, 0.0 };
        int round;
        for (round = 0; round < 2; round++){
            uint32_t segments = round ? 128 : 64;
            RantMeshView m;
            nv = 0; nt = 0;
            CHECK(rant_shape_mesh(&s[k], segments, NULL, &nv, NULL, &nt) == RANT_MATH_ERR_SPACE && nv && nt, "shape %u: asks its room", k);
            CHECK(rant_shape_mesh(&s[k], segments, v, &nv, t, &nt) == RANT_MATH_OK, "shape %u: meshes", k);
            m.vertices = v; m.vertex_count = nv; m.triangles = t; m.triangle_count = nt;
            CHECK(rant_mesh_check(m) == RANT_MATH_OK, "shape %u: every index is a vertex", k);
            for (i = 0, bad = 0; i < nv; i++) bad += !(fabs(rant_shape_distance(&s[k], v[i])) < 1e-9);
            CHECK(bad == 0, "shape %u: %d vertices off the surface", k, bad);
            if (k < 7){
                CHECK(closed(t, nt), "shape %u: closed and wound one way", k);
                err[round] = fabs(rant_mesh_volume(m) / volume[k] - 1.0);
            } else {
                RantPlane plane;
                rant_polygon_plane(pg, &plane);
                for (i = 0, bad = 0; i < nt; i++){
                    RantDouble3 a = v[t[3 * i]], b = v[t[3 * i + 1]], c = v[t[3 * i + 2]];
                    bad += !(rant_double3_dot(rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a)), plane.normal) > 0.0);
                }
                CHECK(bad == 0 && nv == 5 && nt == 3 && near(rant_mesh_area(m), rant_polygon_area(pg), 1e-12), "a polygon faces its own normal");
            }
        }
        if (k < 2 || k == 6) CHECK(err[0] < 1e-12 && err[1] < 1e-12, "shape %u: flat, so its volume is exact: %.3g", k, err[0]);
        else if (k < 7) CHECK(err[0] < 0.01 && err[1] < 0.3 * err[0], "shape %u: volume off %.3g, then %.3g with twice the segments", k, err[0], err[1]);
    }
    nv = 20000; nt = 40000;
    CHECK(rant_shape_mesh(&s[2], 0, v, &nv, t, &nt) == RANT_MATH_OK && nv == 2 + 15 * 32 && nt == 2 * 32 * 15, "0 segments is 32");
    CHECK(rant_shape_mesh(&s[2], 2, v, &nv, t, &nt) == RANT_MATH_ERR_ARGUMENT, "2 segments make no ring");
    CHECK(rant_shape_mesh(&s[5], 3, v, &nv, t, &nt) == RANT_MATH_OK && nv == 5 && nt == 6 && closed(t, nt), "a cone of 3 is a closed pyramid");
    nv = 4; nt = 100; t[0] = 77;
    CHECK(rant_shape_mesh(&s[0], 0, v, &nv, t, &nt) == RANT_MATH_ERR_SPACE && nv == 8 && nt == 12 && t[0] == 77,
          "room for one list but not the other writes neither");
    mv.vertices = v; mv.vertex_count = 8; mv.triangles = t; mv.triangle_count = 12;
    nv = 8; nt = 12;
    rant_shape_mesh(&s[0], 0, v, &nv, t, &nt);
    s[8] = rant_shape_from_mesh(mv);
    {   RantDouble3 v2[8];
        uint32_t t2[36];
        nv = 8; nt = 12;
        CHECK(rant_shape_mesh(&s[8], 0, v2, &nv, t2, &nt) == RANT_MATH_OK && !memcmp(v2, v, sizeof v2) && !memcmp(t2, t, sizeof t2),
              "a mesh is its own copy");
    }
    {   RantSegment seg;
        RantShape line;
        seg.a = rant_double3(0, 0, 0); seg.b = rant_double3(1, 0, 0);
        line = rant_shape_from_segment(seg);
        CHECK(rant_shape_mesh(&line, 0, v, &nv, t, &nt) == RANT_MATH_ERR_ARGUMENT, "a segment has no surface");
    }
    pr.base.count = 2;
    s[6] = rant_shape_from_prism(pr);
    CHECK(rant_shape_mesh(&s[6], 0, v, &nv, t, &nt) == RANT_MATH_ERR_DEGENERATE, "a prism on two corners");
}

void test_shape(void){
    dispatch();
    gaps();
    sampling();
    two_d();
    meshes();
}
