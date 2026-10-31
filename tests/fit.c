#include <stdlib.h>
#include "check.h"
#include "reference.h"

#define MOST 600

static double weight_of(const double *w, uint32_t i){ return w ? w[i] : 1.0; }

static void load3(const double *flat, RantDouble3 *out, uint32_t n){
    uint32_t i;
    for (i = 0; i < n; i++) out[i] = rant_double3(flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]);
}
static void load2(const double *flat, RantDouble2 *out, uint32_t n){
    uint32_t i;
    for (i = 0; i < n; i++) out[i] = rant_double2(flat[2 * i], flat[2 * i + 1]);
}

/* Each fit's weighted sum of squared errors, written out here apart from the library's own. */
static double plane_cost(const RantDouble3 *p, const double *w, uint32_t n, RantPlane pl){
    double s = 0.0, d;
    uint32_t i;
    for (i = 0; i < n; i++){
        d = rant_double3_dot(rant_double3_subtract(p[i], pl.position), pl.normal);
        s += weight_of(w, i) * d * d;
    }
    return s;
}
static double line_cost(const RantDouble3 *p, const double *w, uint32_t n, RantDouble3 a, RantDouble3 dir){
    double s = 0.0;
    uint32_t i;
    dir = rant_double3_normalize(dir);
    for (i = 0; i < n; i++) s += weight_of(w, i) * rant_double3_length_squared(rant_double3_cross(rant_double3_subtract(p[i], a), dir));
    return s;
}
static double ball_cost(const RantDouble3 *p, const double *w, uint32_t n, RantDouble3 c, double r){
    double s = 0.0, e;
    uint32_t i;
    for (i = 0; i < n; i++){
        e = rant_double3_distance(p[i], c) - r;
        s += weight_of(w, i) * e * e;
    }
    return s;
}
static double pose_cost(const RantDouble3 *a, const RantDouble3 *b, const double *w, uint32_t n, RantPose t){
    double s = 0.0;
    uint32_t i;
    for (i = 0; i < n; i++) s += weight_of(w, i) * rant_double3_length_squared(rant_double3_subtract(rant_pose_apply(t, a[i]), b[i]));
    return s;
}
static double pose2d_cost(const RantDouble2 *a, const RantDouble2 *b, const double *w, uint32_t n, RantPose2D t){
    double s = 0.0;
    uint32_t i;
    for (i = 0; i < n; i++) s += weight_of(w, i) * rant_double2_length_squared(rant_double2_subtract(rant_pose2d_apply(t, a[i]), b[i]));
    return s;
}
static double poly_value(const double *c, uint32_t degree, double x){
    double s = 0.0;
    uint32_t e;
    for (e = 0; e <= degree; e++) s += c[e] * pow(x, (double)e);
    return s;
}
static double poly_cost(const double *x, const double *y, const double *w, uint32_t n, const double *c, uint32_t degree){
    double s = 0.0, e;
    uint32_t i;
    for (i = 0; i < n; i++){
        e = y[i] - poly_value(c, degree, x[i]);
        s += weight_of(w, i) * e * e;
    }
    return s;
}

static RantPlane pl_of(RantDouble3 position, RantDouble3 normal){
    RantPlane p;
    p.position = position;
    p.normal = rant_double3_normalize(normal);
    return p;
}

static RantFitOptions weighted(const double *w){
    RantFitOptions o;
    memset(&o, 0, sizeof o);
    o.weights = w;
    return o;
}

/* Random noise of about size s. */
static double noise(double s){ return random_range(-s, s); }

static void plane(void){
    RantDouble3 p[MOST], dup[3 * MOST];
    RantPlane got, again;
    RantFitOptions o;
    double w[MOST], intw[MOST], cost;
    uint32_t i, k, n, m, r;
    load3(REF_FIT_PLANE_POINTS, p, REF_FIT_COUNT);
    CHECK(rant_points_fit_plane(p, REF_FIT_COUNT, NULL, &got) == RANT_MATH_OK, "the plane fit runs");
    CHECK(near3(got.position, rant_double3(REF_FIT_PLANE[0], REF_FIT_PLANE[1], REF_FIT_PLANE[2]), 1e-12), "through the mean numpy finds");
    CHECK(fabs(rant_double3_dot(got.normal, rant_double3(REF_FIT_PLANE[3], REF_FIT_PLANE[4], REF_FIT_PLANE[5]))) > 1.0 - 1e-13,
          "along the normal of numpy's SVD");
    for (k = 0; k < 200; k++){
        RantDouble3 normal = rant_double3_normalize(random_double3(1.0)), u, v, at = random_double3(5.0);
        double off;
        n = 3 + (uint32_t)(random_unit() * 60);
        u = rant_double3_normalize(rant_double3_perpendicular(normal));
        v = rant_double3_cross(normal, u);
        for (i = 0; i < n; i++)
            p[i] = rant_double3_add(at, rant_double3_add(rant_double3_add(rant_double3_scale(u, random_range(-2, 2)), rant_double3_scale(v, random_range(-2, 2))),
                                                         rant_double3_scale(normal, k < 100 ? 0.0 : noise(0.05))));
        for (i = 0; i < n; i++) w[i] = random_range(0.1, 3.0);
        if (rant_points_fit_plane(p, n, NULL, &got) != RANT_MATH_OK){ CHECK(0, "a fit of a random plane runs"); continue; }
        CHECK(near(rant_double3_length(got.normal), 1.0, 1e-14), "a unit normal");
        /* the origin's side counts only beyond the points' own distance from the plane */
        off = -rant_double3_dot(got.normal, got.position);
        if (fabs(off) > sqrt(plane_cost(p, NULL, n, got) / n) + 1e-12 * rant_double3_length(got.position))
            CHECK(off > 0.0, "the normal faces the origin");
        else CHECK(got.normal.z >= 0.0 || fabs(got.normal.z) <= 1e-9, "up when the origin lies within the points' spread");
        if (k < 100){
            CHECK(fabs(rant_double3_dot(got.normal, normal)) > 1.0 - 1e-12 && fabs(rant_plane_distance(got, at)) < 1e-11,
                  "points on a plane give that plane");
        }
        /* no small turn or shift of the answer fits better */
        cost = plane_cost(p, NULL, n, got);
        for (i = 0; i < 6; i++){
            RantPlane moved = pl_of(rant_double3_add(got.position, random_double3(1e-3)), rant_double3_add(got.normal, random_double3(1e-3)));
            CHECK(plane_cost(p, NULL, n, moved) >= cost, "the fit is the least squares plane");
        }
        o = weighted(w);
        CHECK(rant_points_fit_plane(p, n, &o, &got) == RANT_MATH_OK, "a weighted fit runs");
        cost = plane_cost(p, w, n, got);
        for (i = 0; i < 6; i++){
            RantPlane moved = pl_of(rant_double3_add(got.position, random_double3(1e-3)), rant_double3_add(got.normal, random_double3(1e-3)));
            CHECK(plane_cost(p, w, n, moved) >= cost, "the weighted fit is the weighted least squares plane");
        }
        /* a weight of 2 counts as the point twice */
        for (i = 0, m = 0; i < n; i++){
            intw[i] = (double)(1 + (k + i) % 3);
            for (r = 0; r < (uint32_t)intw[i]; r++) dup[m++] = p[i];
        }
        o = weighted(intw);
        CHECK(rant_points_fit_plane(p, n, &o, &got) == RANT_MATH_OK && rant_points_fit_plane(dup, m, NULL, &again) == RANT_MATH_OK
              && near3(got.position, again.position, 1e-11) && near3(got.normal, again.normal, 1e-10), "an integer weight repeats a point");
    }
    {   /* far from the origin */
        RantDouble3 base = rant_double3(1e6, -2e6, 5e5);
        for (i = 0; i < 50; i++) p[i] = rant_double3_add(base, rant_double3(random_range(-1, 1), random_range(-1, 1), 0.0));
        CHECK(rant_points_fit_plane(p, 50, NULL, &got) == RANT_MATH_OK && fabs(got.normal.z) > 1.0 - 1e-12
              && got.normal.z < 0.0 && near(got.position.z, 5e5, 1e-9), "far from the origin, facing it");
        for (i = 0; i < 50; i++) p[i] = rant_double3(random_range(-1, 1), 0.0, random_range(-1, 1));
        p[0] = rant_double3(0, 0, 0);
        CHECK(rant_points_fit_plane(p, 50, NULL, &got) == RANT_MATH_OK && fabs(got.normal.y) > 1.0 - 1e-12 && got.normal.y > 0.0,
              "a wall through the origin faces +y");
        for (i = 0; i < 50; i++) p[i] = rant_double3(random_range(-1, 1), random_range(-1, 1), 0.0);
        CHECK(rant_points_fit_plane(p, 50, NULL, &got) == RANT_MATH_OK && got.normal.z > 1.0 - 1e-12, "a floor through the origin faces up");
        for (k = 0; k < 50; k++){
            for (i = 0; i < 200; i++) p[i] = rant_double3(random_range(-5, 5), random_range(-5, 5), noise(0.002));
            CHECK(rant_points_fit_plane(p, 200, NULL, &got) == RANT_MATH_OK && got.normal.z > 0.99, "a noisy floor through the origin faces up");
            for (i = 0; i < 200; i++) p[i].z += 0.8;
            CHECK(rant_points_fit_plane(p, 200, NULL, &got) == RANT_MATH_OK && got.normal.z < -0.99, "a table above the origin faces down to it");
            for (i = 0; i < 200; i++) p[i].z -= 2.0;
            CHECK(rant_points_fit_plane(p, 200, NULL, &got) == RANT_MATH_OK && got.normal.z > 0.99, "a floor below it faces up to it");
        }
    }
}

static void line(void){
    RantDouble3 p[MOST], dup[3 * MOST];
    RantDouble2 q[MOST];
    RantSegment got, again;
    RantSegment2D flat;
    RantFitOptions o;
    double w[MOST], intw[MOST], cost;
    uint32_t i, k, n, m, r;
    load3(REF_FIT_LINE_POINTS, p, REF_FIT_COUNT);
    CHECK(rant_points_fit_line(p, REF_FIT_COUNT, NULL, &got) == RANT_MATH_OK, "the line fit runs");
    {   RantDouble3 c = rant_double3(REF_FIT_LINE[0], REF_FIT_LINE[1], REF_FIT_LINE[2]);
        RantDouble3 d = rant_double3_normalize(rant_double3_subtract(got.b, got.a));
        CHECK(fabs(rant_double3_dot(d, rant_double3(REF_FIT_LINE[3], REF_FIT_LINE[4], REF_FIT_LINE[5]))) > 1.0 - 1e-13,
              "along numpy's first singular vector");
        CHECK(rant_double3_length(rant_double3_cross(rant_double3_subtract(c, got.a), d)) < 1e-12, "through numpy's mean");
    }
    for (k = 0; k < 200; k++){
        RantDouble3 dir = rant_double3_normalize(random_double3(1.0)), at = random_double3(5.0), d;
        double lo = HUGE_VAL, hi = -HUGE_VAL;
        n = 2 + (uint32_t)(random_unit() * 60);
        for (i = 0; i < n; i++)
            p[i] = rant_double3_add(at, rant_double3_add(rant_double3_scale(dir, random_range(-3, 3)), k < 100 ? rant_double3(0, 0, 0) : random_double3(0.05)));
        if (rant_points_fit_line(p, n, NULL, &got) != RANT_MATH_OK){ CHECK(0, "a fit of a random line runs"); continue; }
        d = rant_double3_normalize(rant_double3_subtract(got.b, got.a));
        CHECK(rant_double3_dot(d, rant_double3_subtract(p[n - 1], p[0])) >= 0.0, "it runs from the first point toward the last");
        for (i = 0; i < n; i++){
            double t = rant_double3_dot(rant_double3_subtract(p[i], got.a), d);
            lo = fmin(lo, t);
            hi = fmax(hi, t);
        }
        CHECK(near(lo, 0.0, 1e-10) && near(hi, rant_double3_distance(got.a, got.b), 1e-10), "the segment spans the points along it");
        if (k < 100) CHECK(fabs(rant_double3_dot(d, dir)) > 1.0 - 1e-12 && line_cost(p, NULL, n, got.a, d) < 1e-20, "points on a line give that line");
        cost = line_cost(p, NULL, n, got.a, d);
        for (i = 0; i < 6; i++)
            CHECK(line_cost(p, NULL, n, rant_double3_add(got.a, random_double3(1e-3)), rant_double3_add(d, random_double3(1e-3))) >= cost,
                  "the fit is the least squares line");
        for (i = 0; i < n; i++) w[i] = random_range(0.1, 3.0);
        o = weighted(w);
        CHECK(rant_points_fit_line(p, n, &o, &got) == RANT_MATH_OK, "a weighted fit runs");
        d = rant_double3_subtract(got.b, got.a);
        cost = line_cost(p, w, n, got.a, d);
        for (i = 0; i < 6; i++)
            CHECK(line_cost(p, w, n, rant_double3_add(got.a, random_double3(1e-3)), rant_double3_add(rant_double3_normalize(d), random_double3(1e-3))) >= cost,
                  "the weighted fit is the weighted least squares line");
        for (i = 0, m = 0; i < n; i++){
            intw[i] = (double)(1 + (k + i) % 3);
            for (r = 0; r < (uint32_t)intw[i]; r++) dup[m++] = p[i];
        }
        o = weighted(intw);
        CHECK(rant_points_fit_line(p, n, &o, &got) == RANT_MATH_OK && rant_points_fit_line(dup, m, NULL, &again) == RANT_MATH_OK
              && near3(got.a, again.a, 1e-10) && near3(got.b, again.b, 1e-10), "an integer weight repeats a point");
        {   /* moved points give the moved line */
            RantPose t = random_pose(3.0);
            for (i = 0; i < n; i++) dup[i] = rant_pose_apply(t, p[i]);
            CHECK(rant_points_fit_line(p, n, NULL, &got) == RANT_MATH_OK && rant_points_fit_line(dup, n, NULL, &again) == RANT_MATH_OK
                  && near3(rant_pose_apply(t, got.a), again.a, 1e-9) && near3(rant_pose_apply(t, got.b), again.b, 1e-9), "the fit moves with the points");
        }
        /* the 2D fit is the fit of the points on z = 0 */
        for (i = 0; i < n; i++){
            q[i] = rant_double2(p[i].x, p[i].y);
            dup[i] = rant_double3(p[i].x, p[i].y, 0.0);
        }
        if (rant_points_fit_line(dup, n, NULL, &got) == RANT_MATH_OK)
            CHECK(rant_points2d_fit_line(q, n, NULL, &flat) == RANT_MATH_OK && near2(flat.a, rant_double2(got.a.x, got.a.y), 1e-12)
                  && near2(flat.b, rant_double2(got.b.x, got.b.y), 1e-12), "the 2D line is the 3D one on z = 0");
    }
}

/* The center's gradient and the radius's: both zero at the least squares circle or sphere. */
static void ball_stationary(const RantDouble3 *p, const double *w, uint32_t n, RantDouble3 c, double r, double *radial, double *centered){
    RantDouble3 g = rant_double3(0, 0, 0);
    double s = 0.0, total = 0.0;
    uint32_t i;
    for (i = 0; i < n; i++){
        RantDouble3 d = rant_double3_subtract(p[i], c);
        double len = rant_double3_length(d), e = len - r;
        s += weight_of(w, i) * e;
        total += weight_of(w, i);
        g = rant_double3_add(g, rant_double3_scale(d, weight_of(w, i) * e / len));
    }
    *radial = fabs(s) / total;
    *centered = rant_double3_length(g) / total;
}

static void ball(void){
    RantDouble3 p[MOST], dup[3 * MOST];
    RantDouble2 q[MOST];
    RantCircle circle;
    RantSphere sphere, again;
    RantFitOptions o;
    double w[MOST], intw[MOST], radial, centered;
    uint32_t i, k, n, m, r;
    o = weighted(REF_FIT_WEIGHTS);
    load2(REF_FIT_CIRCLE_POINTS, q, REF_FIT_COUNT);
    CHECK(rant_points2d_fit_circle(q, REF_FIT_COUNT, NULL, &circle) == RANT_MATH_OK
          && near2(circle.center, rant_double2(REF_FIT_CIRCLE[0], REF_FIT_CIRCLE[1]), 1e-12) && near(circle.radius, REF_FIT_CIRCLE[2], 1e-12),
          "the circle scipy's least squares finds");
    CHECK(rant_points2d_fit_circle(q, REF_FIT_COUNT, &o, &circle) == RANT_MATH_OK
          && near2(circle.center, rant_double2(REF_FIT_CIRCLE_WEIGHTED[0], REF_FIT_CIRCLE_WEIGHTED[1]), 1e-12)
          && near(circle.radius, REF_FIT_CIRCLE_WEIGHTED[2], 1e-12), "and its weighted circle");
    load3(REF_FIT_SPHERE_POINTS, p, REF_FIT_COUNT);
    CHECK(rant_points_fit_sphere(p, REF_FIT_COUNT, NULL, &sphere) == RANT_MATH_OK
          && near3(sphere.center, rant_double3(REF_FIT_SPHERE[0], REF_FIT_SPHERE[1], REF_FIT_SPHERE[2]), 1e-12) && near(sphere.radius, REF_FIT_SPHERE[3], 1e-12),
          "the sphere scipy's least squares finds");
    CHECK(rant_points_fit_sphere(p, REF_FIT_COUNT, &o, &sphere) == RANT_MATH_OK
          && near3(sphere.center, rant_double3(REF_FIT_SPHERE_WEIGHTED[0], REF_FIT_SPHERE_WEIGHTED[1], REF_FIT_SPHERE_WEIGHTED[2]), 1e-12)
          && near(sphere.radius, REF_FIT_SPHERE_WEIGHTED[3], 1e-12), "and its weighted sphere");
    for (k = 0; k < 300; k++){
        int dim = k % 2 ? 3 : 2, exact = k < 100;
        RantDouble3 c = random_double3(5.0);
        double radius = random_range(0.2, 4.0), arc = random_range(1.0, 2.0 * PI), cap = random_range(0.8, PI);
        n = (dim == 2 ? 3 : 4) + (uint32_t)(random_unit() * 60);
        if (dim == 2) c.z = 0.0;
        for (i = 0; i < n; i++){
            double a = random_range(0.0, arc), e = exact ? 0.0 : noise(0.02 * radius);
            if (dim == 2) p[i] = rant_double3_add(c, rant_double3((radius + e) * cos(a), (radius + e) * sin(a), 0.0));
            else {
                double z = random_range(cos(cap), 1.0), s = sqrt(1.0 - z * z);
                p[i] = rant_double3_add(c, rant_double3_scale(rant_double3(s * cos(a), s * sin(a), z), radius + e));
            }
            q[i] = rant_double2(p[i].x, p[i].y);
            w[i] = random_range(0.1, 3.0);
        }
        if (dim == 2){
            if (rant_points2d_fit_circle(q, n, NULL, &circle) != RANT_MATH_OK){ CHECK(0, "a fit of a random circle runs"); continue; }
            sphere.center = rant_double3(circle.center.x, circle.center.y, 0.0);
            sphere.radius = circle.radius;
        } else if (rant_points_fit_sphere(p, n, NULL, &sphere) != RANT_MATH_OK){ CHECK(0, "a fit of a random sphere runs"); continue; }
        if (exact) CHECK(near3(sphere.center, c, 1e-8 * radius) && near(sphere.radius, radius, 1e-8 * radius), "points on a circle or sphere give it");
        ball_stationary(p, NULL, n, sphere.center, sphere.radius, &radial, &centered);
        CHECK(radial < 1e-13 * radius && centered < 1e-12 * radius, "the least squares circle or sphere: no move of its center or radius helps");
        for (i = 0; i < 6; i++){
            RantDouble3 moved = rant_double3_add(sphere.center, random_double3(1e-4 * radius));
            if (dim == 2) moved.z = 0.0;
            CHECK(ball_cost(p, NULL, n, moved, sphere.radius + noise(1e-4 * radius)) >= ball_cost(p, NULL, n, sphere.center, sphere.radius),
                  "a minimum, not a saddle");
        }
        o = weighted(w);
        if (dim == 2){
            CHECK(rant_points2d_fit_circle(q, n, &o, &circle) == RANT_MATH_OK, "a weighted circle fit runs");
            sphere.center = rant_double3(circle.center.x, circle.center.y, 0.0);
            sphere.radius = circle.radius;
        } else CHECK(rant_points_fit_sphere(p, n, &o, &sphere) == RANT_MATH_OK, "a weighted sphere fit runs");
        ball_stationary(p, w, n, sphere.center, sphere.radius, &radial, &centered);
        CHECK(radial < 1e-13 * radius && centered < 1e-12 * radius, "the weighted least squares circle or sphere");
        if (dim == 3){
            for (i = 0, m = 0; i < n; i++){
                intw[i] = (double)(1 + (k + i) % 3);
                for (r = 0; r < (uint32_t)intw[i]; r++) dup[m++] = p[i];
            }
            o = weighted(intw);
            CHECK(rant_points_fit_sphere(p, n, &o, &sphere) == RANT_MATH_OK && rant_points_fit_sphere(dup, m, NULL, &again) == RANT_MATH_OK
                  && near3(sphere.center, again.center, 1e-11 * radius) && near(sphere.radius, again.radius, 1e-11 * radius), "an integer weight repeats a point");
        }
    }
    for (k = 0; k < 100; k++){
        /* three points give the circle through them, four the sphere */
        for (i = 0; i < 4; i++){
            p[i] = random_double3(3.0);
            q[i] = rant_double2(p[i].x, p[i].y);
        }
        if (rant_points2d_fit_circle(q, 3, NULL, &circle) == RANT_MATH_OK)
            for (i = 0; i < 3; i++) CHECK(near(rant_double2_distance(q[i], circle.center), circle.radius, 1e-9 * (1.0 + circle.radius)), "through all three");
        else CHECK(0, "three random points have a circle");
        if (rant_points_fit_sphere(p, 4, NULL, &sphere) == RANT_MATH_OK)
            for (i = 0; i < 4; i++) CHECK(near(rant_double3_distance(p[i], sphere.center), sphere.radius, 1e-9 * (1.0 + sphere.radius)), "through all four");
        else CHECK(0, "four random points have a sphere");
    }
    {   /* moved points give the moved sphere */
        RantPose t = random_pose(4.0);
        load3(REF_FIT_SPHERE_POINTS, p, REF_FIT_COUNT);
        for (i = 0; i < REF_FIT_COUNT; i++) dup[i] = rant_pose_apply(t, p[i]);
        CHECK(rant_points_fit_sphere(p, REF_FIT_COUNT, NULL, &sphere) == RANT_MATH_OK && rant_points_fit_sphere(dup, REF_FIT_COUNT, NULL, &again) == RANT_MATH_OK
              && near3(rant_pose_apply(t, sphere.center), again.center, 1e-9) && near(sphere.radius, again.radius, 1e-9), "the fit moves with the points");
    }
}

static void pose(void){
    RantDouble3 a[MOST], b[MOST];
    RantDouble2 a2[MOST], b2[MOST];
    RantPose got, want;
    RantPose2D got2, want2;
    RantFitOptions o;
    double w[MOST], cost;
    uint32_t i, k, n;
    load3(REF_FIT_POSE_FROM, a, REF_FIT_COUNT);
    load3(REF_FIT_POSE_TO, b, REF_FIT_COUNT);
    want.position = rant_double3(REF_FIT_POSE[0], REF_FIT_POSE[1], REF_FIT_POSE[2]);
    want.orientation = rant_quaternion(REF_FIT_POSE[3], REF_FIT_POSE[4], REF_FIT_POSE[5], REF_FIT_POSE[6]);
    CHECK(rant_points_fit_pose(a, b, REF_FIT_COUNT, NULL, &got) == RANT_MATH_OK && same_pose(got, want, 1e-12), "the pose scipy's align_vectors finds");
    o = weighted(REF_FIT_WEIGHTS);
    want.position = rant_double3(REF_FIT_POSE_WEIGHTED[0], REF_FIT_POSE_WEIGHTED[1], REF_FIT_POSE_WEIGHTED[2]);
    want.orientation = rant_quaternion(REF_FIT_POSE_WEIGHTED[3], REF_FIT_POSE_WEIGHTED[4], REF_FIT_POSE_WEIGHTED[5], REF_FIT_POSE_WEIGHTED[6]);
    CHECK(rant_points_fit_pose(a, b, REF_FIT_COUNT, &o, &got) == RANT_MATH_OK && same_pose(got, want, 1e-12), "and its weighted pose");
    for (k = 0; k < 300; k++){
        int exact = k < 100;
        want = random_pose(5.0);
        want2.position = random_double2(5.0);
        want2.angle = random_range(-PI, PI);
        n = 3 + (uint32_t)(random_unit() * 60);
        for (i = 0; i < n; i++){
            a[i] = random_double3(2.0);
            b[i] = rant_double3_add(rant_pose_apply(want, a[i]), exact ? rant_double3(0, 0, 0) : random_double3(0.05));
            a2[i] = random_double2(2.0);
            b2[i] = rant_double2_add(rant_pose2d_apply(want2, a2[i]), exact ? rant_double2(0, 0) : random_double2(0.05));
            w[i] = random_range(0.1, 3.0);
        }
        if (k % 10 == 0) for (i = 0; i < n; i++){
            /* points in one plane still fix the pose */
            a[i].z = 0.5;
            b[i] = rant_double3_add(rant_pose_apply(want, a[i]), exact ? rant_double3(0, 0, 0) : random_double3(0.05));
        }
        if (rant_points_fit_pose(a, b, n, NULL, &got) != RANT_MATH_OK || rant_points2d_fit_pose(a2, b2, n, NULL, &got2) != RANT_MATH_OK){
            CHECK(0, "the pose fits run");
            continue;
        }
        CHECK(near(sqrt(got.orientation.x * got.orientation.x + got.orientation.y * got.orientation.y + got.orientation.z * got.orientation.z
                        + got.orientation.w * got.orientation.w), 1.0, 1e-14) && got.orientation.w >= 0.0, "a unit turn with w not negative");
        if (exact){
            CHECK(same_pose(got, want, 1e-12) && near3(got.position, want.position, 1e-10), "matched points give the pose that moved them");
            CHECK(near2(got2.position, want2.position, 1e-10) && near(rant_angle_difference(got2.angle, want2.angle), 0.0, 1e-12),
                  "and in 2D");
        }
        cost = pose_cost(a, b, NULL, n, got);
        for (i = 0; i < 6; i++){
            RantPose moved = got;
            moved.position = rant_double3_add(got.position, random_double3(1e-4));
            moved.orientation = rant_quaternion_compose(rant_quaternion_from_rotation_vector(random_double3(1e-4)), got.orientation);
            CHECK(pose_cost(a, b, NULL, n, moved) >= cost, "the fit is the least squares pose");
        }
        cost = pose2d_cost(a2, b2, NULL, n, got2);
        for (i = 0; i < 6; i++){
            RantPose2D moved = got2;
            moved.position = rant_double2_add(got2.position, random_double2(1e-4));
            moved.angle += noise(1e-4);
            CHECK(pose2d_cost(a2, b2, NULL, n, moved) >= cost, "the 2D fit is the least squares pose");
        }
        o = weighted(w);
        CHECK(rant_points_fit_pose(a, b, n, &o, &got) == RANT_MATH_OK && rant_points2d_fit_pose(a2, b2, n, &o, &got2) == RANT_MATH_OK,
              "the weighted pose fits run");
        cost = pose_cost(a, b, w, n, got);
        for (i = 0; i < 6; i++){
            RantPose moved = got;
            moved.position = rant_double3_add(got.position, random_double3(1e-4));
            moved.orientation = rant_quaternion_compose(rant_quaternion_from_rotation_vector(random_double3(1e-4)), got.orientation);
            CHECK(pose_cost(a, b, w, n, moved) >= cost, "the weighted fit is the weighted least squares pose");
        }
        cost = pose2d_cost(a2, b2, w, n, got2);
        for (i = 0; i < 6; i++){
            RantPose2D moved = got2;
            moved.position = rant_double2_add(got2.position, random_double2(1e-4));
            moved.angle += noise(1e-4);
            CHECK(pose2d_cost(a2, b2, w, n, moved) >= cost, "the weighted 2D fit is the weighted least squares pose");
        }
    }
    {   /* a mirror image still gets a turn, never a reflection */
        for (i = 0; i < 20; i++){
            a[i] = random_double3(2.0);
            b[i] = rant_double3(a[i].x, a[i].y, -a[i].z);
        }
        CHECK(rant_points_fit_pose(a, b, 20, NULL, &got) == RANT_MATH_OK, "a mirrored set fits");
        cost = pose_cost(a, b, NULL, 20, got);
        for (k = 0; k < 200; k++){
            RantPose any;
            any.orientation = random_quaternion();
            any.position = rant_double3(0, 0, 0);
            for (i = 0; i < 20; i++) any.position = rant_double3_add(any.position, rant_double3_scale(rant_double3_subtract(b[i], rant_quaternion_rotate(any.orientation, a[i])), 1.0 / 20));
            CHECK(pose_cost(a, b, NULL, 20, any) >= cost, "no other turn fits a mirror better");
        }
    }
}

static void polynomial(void){
    double x[MOST], y[MOST], c[12], want[12], w[MOST], cost;
    RantFitOptions o;
    uint32_t i, k, n, e;
    CHECK(rant_polynomial_fit(REF_FIT_POLY_X, REF_FIT_POLY_Y, REF_FIT_COUNT, 3, NULL, c) == RANT_MATH_OK, "the polynomial fit runs");
    for (e = 0; e <= 3; e++) CHECK(near(c[e], REF_FIT_POLY[e], 1e-11), "numpy's coefficients");
    o = weighted(REF_FIT_WEIGHTS);
    CHECK(rant_polynomial_fit(REF_FIT_POLY_X, REF_FIT_POLY_Y, REF_FIT_COUNT, 3, &o, c) == RANT_MATH_OK, "the weighted fit runs");
    for (e = 0; e <= 3; e++) CHECK(near(c[e], REF_FIT_POLY_WEIGHTED[e], 1e-11), "numpy's weighted coefficients");
    CHECK(rant_polynomial_fit(REF_FIT_POLY_FAR_X, REF_FIT_POLY_FAR_Y, REF_FIT_COUNT, 2, NULL, c) == RANT_MATH_OK, "x far from 0 fits");
    for (i = 0; i < REF_FIT_COUNT; i++) CHECK(near(rant_polynomial_at(c, 2, REF_FIT_POLY_FAR_X[i]), REF_FIT_POLY_FAR_VALUES[i], 1e-7), "with numpy's values there");
    for (k = 0; k < 300; k++){
        uint32_t degree = k % 7;
        int exact = k < 150;
        double offset = random_range(-2, 2), span = random_range(1.0, 4.0);
        n = degree + 1 + (uint32_t)(random_unit() * 40);
        for (e = 0; e <= degree; e++) want[e] = random_range(-2, 2);
        for (i = 0; i < n; i++){
            x[i] = offset + span * (i + random_unit()) / n;
            y[i] = poly_value(want, degree, x[i]) + (exact ? 0.0 : noise(0.1));
            w[i] = random_range(0.1, 3.0);
        }
        if (rant_polynomial_fit(x, y, n, degree, NULL, c) != RANT_MATH_OK){ CHECK(0, "a random polynomial fits"); continue; }
        for (i = 0; i < n; i++) CHECK(near(rant_polynomial_at(c, degree, x[i]), poly_value(c, degree, x[i]), 1e-9 * (1.0 + fabs(y[i]))), "at sums the powers");
        if (exact)
            for (i = 0; i < n; i++) CHECK(near(rant_polynomial_at(c, degree, x[i]), y[i], 1e-8 * (1.0 + fabs(y[i]))), "exact points give their polynomial");
        cost = poly_cost(x, y, NULL, n, c, degree);
        for (i = 0; i < 6; i++){
            double moved[12];
            for (e = 0; e <= degree; e++) moved[e] = c[e] + noise(1e-5) / pow(1.0 + fabs(offset) + span, (double)e);
            CHECK(poly_cost(x, y, NULL, n, moved, degree) >= cost * (1.0 - 1e-12), "the fit is the least squares polynomial");
        }
        o = weighted(w);
        CHECK(rant_polynomial_fit(x, y, n, degree, &o, c) == RANT_MATH_OK, "a weighted fit runs");
        cost = poly_cost(x, y, w, n, c, degree);
        for (i = 0; i < 6; i++){
            double moved[12];
            for (e = 0; e <= degree; e++) moved[e] = c[e] + noise(1e-5) / pow(1.0 + fabs(offset) + span, (double)e);
            CHECK(poly_cost(x, y, w, n, moved, degree) >= cost * (1.0 - 1e-12), "the weighted fit is the weighted least squares polynomial");
        }
    }
    {   /* degree 0 is the weighted mean, degree 1 the regression line */
        double sw = 0.0, swy = 0.0, mx = 0.0, my = 0.0, sxx = 0.0, sxy = 0.0;
        n = 30;
        for (i = 0; i < n; i++){
            x[i] = random_range(-3, 3);
            y[i] = random_range(-3, 3);
            w[i] = random_range(0.1, 3.0);
            sw += w[i];
            swy += w[i] * y[i];
            mx += x[i] / n;
            my += y[i] / n;
        }
        o = weighted(w);
        CHECK(rant_polynomial_fit(x, y, n, 0, &o, c) == RANT_MATH_OK && near(c[0], swy / sw, 1e-13), "degree 0 is the weighted mean");
        for (i = 0; i < n; i++){
            sxx += (x[i] - mx) * (x[i] - mx);
            sxy += (x[i] - mx) * (y[i] - my);
        }
        CHECK(rant_polynomial_fit(x, y, n, 1, NULL, c) == RANT_MATH_OK && near(c[1], sxy / sxx, 1e-12) && near(c[0], my - c[1] * mx, 1e-12),
              "degree 1 is the regression line");
        for (i = 0; i < n; i++) x[i] = 2.0;
        CHECK(rant_polynomial_fit(x, y, n, 0, NULL, c) == RANT_MATH_OK, "one x fits a constant");
        CHECK(rant_polynomial_fit(x, y, n, 1, NULL, c) == RANT_MATH_ERR_DEGENERATE, "but no line");
        want[0] = 1.0; want[1] = -2.0; want[2] = 0.5;
        CHECK(near(rant_polynomial_at(want, 2, 3.0), 1.0 - 6.0 + 4.5, 1e-15) && rant_polynomial_at(want, 0, 9.0) == 1.0, "at");
    }
}

/* A mask the same as the points within tol of the answer. */
static int mask_is_near(const uint8_t *mask, const double *errors, uint32_t n, double tol){
    uint32_t i;
    for (i = 0; i < n; i++) if (mask[i] != (errors[i] <= tol)) return 0;
    return 1;
}

static void ransac(void){
    RantDouble3 p[MOST], kept[MOST], b[MOST];
    RantDouble2 q[MOST], q2[MOST], kept2[MOST];
    double err[MOST], x[MOST], y[MOST], kx[MOST], ky[MOST], w[MOST], c[4], c2[4];
    uint8_t mask[MOST], truth[MOST];
    RantFitOptions o;
    uint32_t i, k, n, m, trial;
    memset(&o, 0, sizeof o);
    for (trial = 0; trial < 20; trial++){
        RantPlane want = pl_of(random_double3(2.0), random_double3(1.0)), got, again, other;
        RantDouble3 u = rant_double3_normalize(rant_double3_perpendicular(want.normal)), v = rant_double3_cross(want.normal, u);
        n = 500;
        for (i = 0; i < n; i++){
            truth[i] = i % 5 < 3;
            if (truth[i])
                p[i] = rant_double3_add(want.position, rant_double3_add(rant_double3_add(rant_double3_scale(u, random_range(-3, 3)),
                                                                                         rant_double3_scale(v, random_range(-3, 3))),
                                                                        rant_double3_scale(want.normal, noise(0.005))));
            else p[i] = rant_double3_add(want.position, random_double3(3.0));
        }
        o.tolerance = 0.02;
        o.seed = trial;
        o.inliers = mask;
        CHECK(rant_points_fit_plane(p, n, &o, &got) == RANT_MATH_OK, "RANSAC finds a plane among outliers");
        CHECK(fabs(rant_double3_dot(got.normal, want.normal)) > 1.0 - 1e-6 && fabs(rant_plane_distance(want, got.position)) < 2e-3,
              "the plane the inliers lie on");
        for (i = 0; i < n; i++) err[i] = fabs(rant_plane_distance(got, p[i]));
        CHECK(mask_is_near(mask, err, n, 0.02), "inliers are the points within tolerance of the answer");
        for (i = 0; i < n; i++) if (truth[i]) CHECK(mask[i], "every true inlier is kept");
        for (i = 0, m = 0; i < n; i++) if (mask[i]) kept[m++] = p[i];
        CHECK(rant_points_fit_plane(kept, m, NULL, &again) == RANT_MATH_OK && near3(again.position, got.position, 1e-12)
              && near3(again.normal, got.normal, 1e-12), "the answer is the least squares fit of its inliers");
        CHECK(rant_points_fit_plane(p, n, &o, &again) == RANT_MATH_OK && !memcmp(&again, &got, sizeof got), "the same seed, the same answer");
        o.tries = 1;
        CHECK(rant_points_fit_plane(p, n, &o, &other) == RANT_MATH_OK, "a single try still answers");
        o.tries = 0;
        o.inliers = NULL;
        CHECK(rant_points_fit_plane(p, n, &o, &other) == RANT_MATH_OK && !memcmp(&other, &got, sizeof got), "with no mask, the same answer");
        {   /* weighted, the answer is the weighted fit of its inliers */
            RantFitOptions wo = weighted(w);
            double kw[MOST];
            for (i = 0; i < n; i++) w[i] = random_range(0.1, 3.0);
            wo.tolerance = 0.02;
            wo.seed = trial;
            wo.inliers = mask;
            CHECK(rant_points_fit_plane(p, n, &wo, &got) == RANT_MATH_OK, "weighted RANSAC runs");
            for (i = 0, m = 0; i < n; i++) if (mask[i]){ kept[m] = p[i]; kw[m] = w[i]; m++; }
            wo = weighted(kw);
            CHECK(rant_points_fit_plane(kept, m, &wo, &again) == RANT_MATH_OK && near3(again.position, got.position, 1e-12)
                  && near3(again.normal, got.normal, 1e-12), "the weighted answer is the weighted fit of its inliers");
            wo = weighted(w);
            wo.tolerance = HUGE_VAL;
            wo.inliers = mask;
            CHECK(rant_points_fit_plane(p, n, &wo, &got) == RANT_MATH_OK, "an endless tolerance runs");
            wo.tolerance = 0.0;
            CHECK(rant_points_fit_plane(p, n, &wo, &again) == RANT_MATH_OK && near3(again.position, got.position, 1e-12)
                  && near3(again.normal, got.normal, 1e-12), "an endless tolerance is least squares over every point");
        }
    }
    for (trial = 0; trial < 20; trial++){
        /* a wall in a laser scan */
        RantDouble2 from = random_double2(3.0), to = rant_double2_add(from, rant_double2_scale(rant_double2_from_heading(random_range(-PI, PI)), 4.0));
        RantSegment2D got;
        RantDouble2 d = rant_double2_normalize(rant_double2_subtract(to, from));
        n = 300;
        /* clutter off the wall's line: a point on it past an end would join the segment */
        for (i = 0; i < n; i++){
            truth[i] = i < 180;
            if (truth[i]) q[i] = rant_double2_add(rant_double2_lerp(from, to, i / 179.0), rant_double2_scale(rant_double2_perpendicular(d), noise(0.002)));
            else do q[i] = rant_double2_add(from, random_double2(4.0));
                 while (fabs(rant_double2_cross(rant_double2_subtract(q[i], from), d)) < 0.05);
        }
        o.tolerance = 0.01;
        o.seed = 7 + trial;
        o.inliers = mask;
        CHECK(rant_points2d_fit_line(q, n, &o, &got) == RANT_MATH_OK, "RANSAC finds a wall");
        CHECK(fabs(rant_double2_cross(rant_double2_normalize(rant_double2_subtract(got.b, got.a)), d)) < 1e-3
              && rant_double2_distance(got.a, from) < 0.05 && rant_double2_distance(got.b, to) < 0.05, "from end to end, in the scan's order");
        for (i = 0; i < n; i++) if (truth[i]) CHECK(mask[i], "every point of the wall is kept");
        for (i = 0, m = 0; i < n; i++) if (mask[i]) kept2[m++] = q[i];
        {   RantSegment2D again;
            CHECK(rant_points2d_fit_line(kept2, m, NULL, &again) == RANT_MATH_OK && near2(again.a, got.a, 1e-12) && near2(again.b, got.b, 1e-12),
                  "the answer is the least squares fit of its inliers");
        }
    }
    for (trial = 0; trial < 20; trial++){
        RantCircle want, got, again;
        RantSphere wants, gots;
        want.center = random_double2(3.0);
        want.radius = random_range(0.5, 2.0);
        wants.center = random_double3(3.0);
        wants.radius = random_range(0.5, 2.0);
        n = 400;
        for (i = 0; i < n; i++){
            double a = random_range(0, 2 * PI);
            truth[i] = i % 4 < 3;
            q[i] = truth[i] ? rant_double2_add(want.center, rant_double2_scale(rant_double2_from_heading(a), want.radius + noise(0.003)))
                            : rant_double2_add(want.center, random_double2(3.0));
            p[i] = truth[i] ? rant_double3_add(wants.center, rant_double3_scale(rant_double3_normalize(random_double3(1.0)), wants.radius + noise(0.003)))
                            : rant_double3_add(wants.center, random_double3(3.0));
        }
        o.tolerance = 0.015;
        o.seed = 100 + trial;
        o.inliers = mask;
        CHECK(rant_points2d_fit_circle(q, n, &o, &got) == RANT_MATH_OK && near2(got.center, want.center, 2e-3) && near(got.radius, want.radius, 2e-3),
              "RANSAC finds a circle among outliers");
        for (i = 0; i < n; i++) err[i] = fabs(rant_double2_distance(q[i], got.center) - got.radius);
        CHECK(mask_is_near(mask, err, n, 0.015), "inliers are the points within tolerance of the circle");
        for (i = 0, m = 0; i < n; i++) if (mask[i]) kept2[m++] = q[i];
        CHECK(rant_points2d_fit_circle(kept2, m, NULL, &again) == RANT_MATH_OK && near2(again.center, got.center, 1e-11)
              && near(again.radius, got.radius, 1e-11), "the circle is the least squares fit of its inliers");
        CHECK(rant_points_fit_sphere(p, n, &o, &gots) == RANT_MATH_OK && near3(gots.center, wants.center, 2e-3) && near(gots.radius, wants.radius, 2e-3),
              "RANSAC finds a sphere among outliers");
        for (i = 0; i < n; i++) if (truth[i]) CHECK(mask[i], "every point of the sphere is kept");
    }
    for (trial = 0; trial < 20; trial++){
        RantPose want = random_pose(3.0), got;
        RantPose2D want2, got2;
        want2.position = random_double2(3.0);
        want2.angle = random_range(-PI, PI);
        n = 200;
        /* a wrong match lands anywhere but near where its point moves */
        for (i = 0; i < n; i++){
            truth[i] = i % 10 < 7;
            p[i] = random_double3(2.0);
            q[i] = random_double2(2.0);
            if (truth[i]){
                b[i] = rant_double3_add(rant_pose_apply(want, p[i]), random_double3(0.001));
                q2[i] = rant_double2_add(rant_pose2d_apply(want2, q[i]), random_double2(0.001));
                continue;
            }
            do b[i] = random_double3(4.0); while (rant_double3_distance(b[i], rant_pose_apply(want, p[i])) < 0.05);
            do q2[i] = random_double2(4.0); while (rant_double2_distance(q2[i], rant_pose2d_apply(want2, q[i])) < 0.05);
        }
        o.tolerance = 0.01;
        o.seed = 200 + trial;
        o.inliers = mask;
        CHECK(rant_points_fit_pose(p, b, n, &o, &got) == RANT_MATH_OK && same_turn(got.orientation, want.orientation, 1e-6) && near3(got.position, want.position, 1e-3),
              "RANSAC finds the pose despite wrong matches");
        CHECK(!memcmp(mask, truth, n), "and tells the wrong matches apart");
        CHECK(rant_points2d_fit_pose(q, q2, n, &o, &got2) == RANT_MATH_OK && near2(got2.position, want2.position, 1e-3)
              && fabs(rant_angle_difference(got2.angle, want2.angle)) < 1e-3, "and the 2D pose");
        CHECK(!memcmp(mask, truth, n), "with the 2D wrong matches told apart");
    }
    for (trial = 0; trial < 20; trial++){
        double want[3];
        want[0] = random_range(-2, 2); want[1] = random_range(-2, 2); want[2] = random_range(-1, 1);
        n = 200;
        for (i = 0; i < n; i++){
            truth[i] = i % 10 < 7;
            x[i] = random_range(-2, 2);
            y[i] = truth[i] ? poly_value(want, 2, x[i]) + noise(0.01) : random_range(-10, 10);
        }
        o.tolerance = 0.05;
        o.seed = 300 + trial;
        o.inliers = mask;
        CHECK(rant_polynomial_fit(x, y, n, 2, &o, c) == RANT_MATH_OK && near(c[0], want[0], 0.02) && near(c[1], want[1], 0.02) && near(c[2], want[2], 0.02),
              "RANSAC finds the polynomial among outliers");
        for (i = 0, m = 0; i < n; i++) if (mask[i]){ kx[m] = x[i]; ky[m] = y[i]; m++; }
        CHECK(rant_polynomial_fit(kx, ky, m, 2, NULL, c2) == RANT_MATH_OK && near(c2[0], c[0], 1e-11) && near(c2[1], c[1], 1e-11) && near(c2[2], c[2], 1e-11),
              "the answer is the least squares fit of its inliers");
    }
    {   /* every point an inlier stops after one try, and without RANSAC every point counts */
        RantPlane got;
        for (i = 0; i < 100; i++) p[i] = rant_double3(random_range(-1, 1), random_range(-1, 1), 0.0);
        o.tolerance = 0.1;
        o.inliers = mask;
        CHECK(rant_points_fit_plane(p, 100, &o, &got) == RANT_MATH_OK && fabs(got.normal.z) > 1.0 - 1e-12, "a clean plane");
        for (i = 0, k = 0; i < 100; i++) k += mask[i];
        CHECK(k == 100, "keeps every point");
        o.tolerance = 0.0;
        memset(mask, 0, sizeof mask);
        CHECK(rant_points_fit_plane(p, 100, &o, &got) == RANT_MATH_OK, "least squares");
        for (i = 0, k = 0; i < 100; i++) k += mask[i];
        CHECK(k == 100, "marks every point an inlier");
    }
}

static void failures(void){
    RantDouble3 p[8];
    RantDouble2 q[8];
    RantPlane plane, keep;
    RantSegment seg;
    RantSegment2D seg2;
    RantSphere sphere;
    RantCircle circle;
    RantPose pose;
    RantPose2D pose2;
    RantFitOptions o;
    double x[8], y[8], c[4], w[8];
    uint32_t i;
    for (i = 0; i < 8; i++){
        p[i] = rant_double3(i, 2.0 * i, -1.0 * i);
        q[i] = rant_double2(i, 3.0 * i);
        x[i] = 1.0;
        y[i] = i;
        w[i] = 1.0;
    }
    memset(&keep, 0x5A, sizeof keep);
    plane = keep;
    CHECK(rant_points_fit_plane(p, 8, NULL, &plane) == RANT_MATH_ERR_DEGENERATE && !memcmp(&plane, &keep, sizeof plane),
          "points on a line fix no plane, and the answer is left as it was");
    CHECK(rant_points2d_fit_circle(q, 8, NULL, &circle) == RANT_MATH_ERR_DEGENERATE, "nor a circle");
    CHECK(rant_points_fit_pose(p, p, 8, NULL, &pose) == RANT_MATH_ERR_DEGENERATE, "nor a pose");
    CHECK(rant_points_fit_line(p, 8, NULL, &seg) == RANT_MATH_OK && rant_points2d_fit_line(q, 8, NULL, &seg2) == RANT_MATH_OK, "but a line");
    CHECK(rant_polynomial_fit(x, y, 8, 2, NULL, c) == RANT_MATH_ERR_DEGENERATE, "one x fixes no curve");
    for (i = 0; i < 8; i++) p[i] = rant_double3(cos(i), sin(i), 0.25);
    CHECK(rant_points_fit_sphere(p, 8, NULL, &sphere) == RANT_MATH_ERR_DEGENERATE, "points in a plane fix no sphere");
    for (i = 0; i < 8; i++){ p[i] = rant_double3(1, 2, 3); q[i] = rant_double2(1, 2); }
    CHECK(rant_points_fit_line(p, 8, NULL, &seg) == RANT_MATH_ERR_DEGENERATE && rant_points2d_fit_line(q, 8, NULL, &seg2) == RANT_MATH_ERR_DEGENERATE
          && rant_points2d_fit_pose(q, q, 8, NULL, &pose2) == RANT_MATH_ERR_DEGENERATE, "one place fixes no line and no turn");
    for (i = 0; i < 8; i++) p[i] = random_double3(1.0);
    CHECK(rant_points_fit_plane(p, 2, NULL, &plane) == RANT_MATH_ERR_ARGUMENT && rant_points_fit_line(p, 1, NULL, &seg) == RANT_MATH_ERR_ARGUMENT
          && rant_points_fit_sphere(p, 3, NULL, &sphere) == RANT_MATH_ERR_ARGUMENT && rant_points_fit_pose(p, p, 2, NULL, &pose) == RANT_MATH_ERR_ARGUMENT
          && rant_points2d_fit_circle(q, 2, NULL, &circle) == RANT_MATH_ERR_ARGUMENT && rant_points2d_fit_pose(q, q, 1, NULL, &pose2) == RANT_MATH_ERR_ARGUMENT
          && rant_polynomial_fit(x, y, 2, 2, NULL, c) == RANT_MATH_ERR_ARGUMENT, "too few points");
    CHECK(rant_points_fit_plane(NULL, 8, NULL, &plane) == RANT_MATH_ERR_ARGUMENT && rant_points_fit_plane(p, 8, NULL, NULL) == RANT_MATH_ERR_ARGUMENT
          && rant_points_fit_pose(p, NULL, 8, NULL, &pose) == RANT_MATH_ERR_ARGUMENT && rant_polynomial_fit(x, NULL, 8, 1, NULL, c) == RANT_MATH_ERR_ARGUMENT
          && rant_polynomial_fit(x, y, 8, UINT32_MAX, NULL, c) == RANT_MATH_ERR_ARGUMENT, "NULL arguments");
    o = weighted(w);
    w[3] = -1.0;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_ARGUMENT, "a negative weight");
    w[3] = sqrt(-1.0);
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_ARGUMENT, "a weight that is not a number");
    w[3] = HUGE_VAL;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_ARGUMENT, "an endless weight");
    for (i = 0; i < 8; i++) w[i] = 0.0;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_DEGENERATE, "every weight zero");
    w[0] = w[1] = 1.0;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_DEGENERATE, "two points that count fix no plane");
    w[2] = 1.0;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_OK && fabs(rant_plane_distance(plane, p[0])) < 1e-12
          && fabs(rant_plane_distance(plane, p[1])) < 1e-12 && fabs(rant_plane_distance(plane, p[2])) < 1e-12, "three that count fix it");
    memset(&o, 0, sizeof o);
    o.tolerance = -1.0;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_ARGUMENT, "a negative tolerance");
    o.tolerance = sqrt(-1.0);
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_ARGUMENT, "a tolerance that is not a number");
    for (i = 0; i < 8; i++) p[i] = rant_double3(i, 2.0 * i, -1.0 * i);
    o.tolerance = 0.1;
    CHECK(rant_points_fit_plane(p, 8, &o, &plane) == RANT_MATH_ERR_DEGENERATE, "RANSAC draws no plane from points on a line");
    CHECK(rant_polynomial_at(NULL, 3, 1.0) == 0.0, "no coefficients are 0");
}

void test_fit(void){
    plane();
    line();
    ball();
    pose();
    polynomial();
    ransac();
    failures();
}
