#include <stdlib.h>
#include "check.h"

/* The smallest ball by trying every ball through two, three or four of the points. */
static double brute_sphere(const RantDouble3 *p, uint32_t n){
    double best = HUGE_VAL;
    uint32_t i, j, k, l, q;
    RantSphere s;
    for (i = 0; i < n; i++)
        for (j = i; j < n; j++)
            for (k = j; k < n; k++)
                for (l = k; l < n; l++){
                    RantDouble3 pick[4];
                    pick[0] = p[i]; pick[1] = p[j]; pick[2] = p[k]; pick[3] = p[l];
                    if (rant_points_enclosing_sphere(pick, 4, &s) != RANT_MATH_OK) continue;
                    for (q = 0; q < n; q++) if (rant_double3_distance(s.center, p[q]) > s.radius * (1.0 + 1e-9) + 1e-12) break;
                    if (q == n && s.radius < best) best = s.radius;
                }
    return best;
}

static void sets3(void){
    RantDouble3 p[300];
    uint32_t i, k, n;
    for (i = 0; i < 100; i++){
        RantAlignedBox b;
        RantDouble3 mean = rant_double3(0, 0, 0), sum = rant_double3(0, 0, 0);
        RantCovariance3 c;
        RantOrientedBox ob;
        RantSphere s;
        double trace = 0.0;
        int on = 0;
        n = 1 + (uint32_t)(random_unit() * 299);
        for (k = 0; k < n; k++) p[k] = rant_double3_add(rant_double3(100, -50, 7), random_double3(3.0));
        CHECK(rant_points_bounds(p, n, &b) == RANT_MATH_OK && rant_points_mean(p, n, &mean) == RANT_MATH_OK
              && rant_points_covariance(p, n, &c) == RANT_MATH_OK, "the set calls run");
        for (k = 0; k < n; k++){
            sum = rant_double3_add(sum, p[k]);
            CHECK(rant_aligned_box_contains(b, p[k]), "bounds hold every point");
        }
        CHECK(near3(mean, rant_double3_scale(sum, 1.0 / n), 1e-11), "mean");
        for (k = 0; k < n; k++) trace += rant_double3_length_squared(rant_double3_subtract(p[k], mean)) / n;
        CHECK(near(c.m[0][0] + c.m[1][1] + c.m[2][2], trace, 1e-10) && near(c.m[0][1], c.m[1][0], 0.0), "the covariance's trace is the spread");
        CHECK(rant_points_oriented_box(p, n, &ob) == RANT_MATH_OK, "oriented box runs");
        for (k = 0; k < n; k++) CHECK(rant_oriented_box_distance(ob, p[k]) <= 1e-9, "the oriented box holds every point");
        {   RantQuaternion q = ob.pose.orientation;
            CHECK(near(sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w), 1.0, 1e-12)
                  && ob.size.x >= 0.0 && ob.size.y >= 0.0 && ob.size.z >= 0.0, "a unit turn, sizes not negative");
        }
        CHECK(rant_points_enclosing_sphere(p, n, &s) == RANT_MATH_OK, "the enclosing sphere runs");
        for (k = 0; k < n; k++){
            double d = rant_double3_distance(s.center, p[k]);
            CHECK(d <= s.radius * (1.0 + 1e-9) + 1e-12, "the sphere holds every point");
            on += fabs(d - s.radius) <= 1e-9 * (1.0 + s.radius);
        }
        CHECK(n == 1 || on >= 2, "at least two points lie on the smallest sphere");
        if (n <= 6) CHECK(near(s.radius, brute_sphere(p, n), 1e-9), "no ball through some of the points is smaller");
    }
    {   /* points from inside a turned box give back about that box */
        RantOrientedBox want, got;
        want.pose = random_pose(5.0);
        want.size = rant_double3(6, 2, 0.5);
        for (k = 0; k < 300; k++)
            p[k] = rant_pose_apply(want.pose, rant_double3(random_range(-3, 3), random_range(-1, 1), random_range(-0.25, 0.25)));
        rant_points_oriented_box(p, 300, &got);
        CHECK(near(got.size.x, 6.0, 0.1) && near(got.size.y, 2.0, 0.1) && near(got.size.z, 0.5, 0.05), "the principal axes find the box");
        CHECK(fabs(rant_double3_dot(rant_quaternion_rotate(got.pose.orientation, rant_double3(1, 0, 0)),
                                    rant_quaternion_rotate(want.pose.orientation, rant_double3(1, 0, 0)))) > 0.999, "along its long side");
    }
    {   RantSphere s;
        double a = 1.0 / sqrt(3.0);
        p[0] = rant_double3(a, a, a); p[1] = rant_double3(a, -a, -a); p[2] = rant_double3(-a, a, -a); p[3] = rant_double3(-a, -a, a);
        CHECK(rant_points_enclosing_sphere(p, 4, &s) == RANT_MATH_OK && near(s.radius, 1.0, 1e-12) && near3(s.center, rant_double3(0, 0, 0), 1e-12),
              "a regular tetrahedron's sphere goes through its corners");
        for (k = 0; k < 300; k++) p[k] = rant_double3_normalize(random_double3(1.0));
        rant_points_enclosing_sphere(p, 300, &s);
        CHECK(s.radius <= 1.0 + 1e-9 && s.radius > 0.98, "points on a ball are held by about that ball");
        CHECK(rant_points_enclosing_sphere(p, 0, &s) == RANT_MATH_ERR_ARGUMENT && rant_points_mean(NULL, 3, &p[0]) == RANT_MATH_ERR_ARGUMENT,
              "a set of no points fails");
    }
}

static double brute_circle(const RantDouble2 *p, uint32_t n){
    double best = HUGE_VAL;
    uint32_t i, j, k, q;
    RantCircle c;
    for (i = 0; i < n; i++)
        for (j = i; j < n; j++)
            for (k = j; k < n; k++){
                RantDouble2 pick[3];
                pick[0] = p[i]; pick[1] = p[j]; pick[2] = p[k];
                rant_points2d_enclosing_circle(pick, 3, &c);
                for (q = 0; q < n; q++) if (rant_double2_distance(c.center, p[q]) > c.radius * (1.0 + 1e-9) + 1e-12) break;
                if (q == n && c.radius < best) best = c.radius;
            }
    return best;
}

static void sets2(void){
    RantDouble2 p[300], h[600];
    uint32_t i, k, n, m;
    for (i = 0; i < 200; i++){
        RantPolygon2DView hull;
        RantOrientedBox2D ob;
        RantCircle c;
        RantAlignedBox2D b;
        double best = HUGE_VAL;
        int a;
        n = 1 + (uint32_t)(random_unit() * 200);
        for (k = 0; k < n; k++) p[k] = i % 4 == 0 ? rant_double2(floor(random_range(-3, 3)), floor(random_range(-3, 3))) : random_double2(4.0);
        m = 600;
        CHECK(rant_points2d_hull(p, n, h, &m) == RANT_MATH_OK, "hull runs");
        hull.points = h; hull.count = m;
        if (m >= 3){
            CHECK(rant_polygon2d_is_convex(hull) && rant_polygon2d_signed_area(hull) > 0.0, "the hull is convex and counterclockwise");
            for (k = 0; k < m; k++){
                RantDouble2 o = h[(k + m - 1) % m], q = h[(k + 1) % m];
                CHECK(rant_double2_cross(rant_double2_subtract(h[k], o), rant_double2_subtract(q, h[k])) > 0.0, "no hull corner lies on a straight edge");
            }
        }
        for (k = 0; k < n; k++) CHECK(m < 3 || rant_polygon2d_distance(hull, p[k]) <= 1e-12, "the hull holds every point");
        rant_points2d_bounds(p, n, &b);
        CHECK(near(h[0].x, b.min.x, 0.0), "the hull starts at the leftmost point");
        CHECK(rant_points2d_oriented_box(p, n, &ob) == RANT_MATH_OK, "oriented box2d runs");
        CHECK(ob.pose.angle > -PI / 2 - 1e-15 && ob.pose.angle <= PI / 2 + 1e-15, "its angle is in the right half turn");
        for (k = 0; k < n; k++) CHECK(rant_oriented_box2d_distance(ob, p[k]) <= 1e-9, "the oriented box2d holds every point");
        for (a = 0; a < 720; a++){
            double t = PI * a / 720, lo = HUGE_VAL, hi = -HUGE_VAL, lo2 = HUGE_VAL, hi2 = -HUGE_VAL;
            RantDouble2 e = rant_double2(cos(t), sin(t));
            for (k = 0; k < n; k++){
                double u = rant_double2_dot(p[k], e), v = rant_double2_cross(e, p[k]);
                if (u < lo) lo = u;
                if (u > hi) hi = u;
                if (v < lo2) lo2 = v;
                if (v > hi2) hi2 = v;
            }
            if ((hi - lo) * (hi2 - lo2) < best) best = (hi - lo) * (hi2 - lo2);
        }
        CHECK(fabs(ob.size.x * ob.size.y) <= best + 1e-9, "no rectangle at any angle is smaller");
        CHECK(rant_points2d_enclosing_circle(p, n, &c) == RANT_MATH_OK, "the enclosing circle runs");
        for (k = 0; k < n; k++) CHECK(rant_double2_distance(c.center, p[k]) <= c.radius * (1.0 + 1e-9) + 1e-12, "the circle holds every point");
        if (n <= 7) CHECK(near(c.radius, brute_circle(p, n), 1e-9), "no circle through some of the points is smaller");
    }
    {   RantDouble2 line[4];
        RantDouble2 mean;
        line[0] = rant_double2(0, 0); line[1] = rant_double2(1, 1); line[2] = rant_double2(3, 3); line[3] = rant_double2(1, 1);
        m = 8;
        CHECK(rant_points2d_hull(line, 4, h, &m) == RANT_MATH_OK && m == 2 && near2(h[0], line[0], 0.0) && near2(h[1], line[2], 0.0),
              "points in a line give the two ends");
        m = 1;
        CHECK(rant_points2d_hull(line, 4, h, &m) == RANT_MATH_ERR_SPACE && m == 2, "too little room asks for more");
        CHECK(rant_points2d_mean(line, 4, &mean) == RANT_MATH_OK && near2(mean, rant_double2(1.25, 1.25), 0.0), "mean2d");
    }
}

void test_points(void){
    sets3();
    sets2();
}
