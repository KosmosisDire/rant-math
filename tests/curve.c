#include "check.h"

/* A line that runs on along x, so each of its points is its own nearest. */
static uint32_t onward_line(RantDouble3 *out, uint32_t most){
    uint32_t n = 2 + (uint32_t)(random_unit() * (most - 1)), i;
    double x = random_range(-2, 2);
    if (n > most) n = most;
    for (i = 0; i < n; i++){
        out[i] = rant_double3(x, random_range(-0.3, 0.3), random_range(-0.3, 0.3));
        x += random_range(0.5, 2.0);
    }
    return n;
}

static double brute_distance(const RantDouble3 *p, uint32_t n, RantDouble3 q){
    double best = n ? rant_double3_distance(p[0], q) : HUGE_VAL;
    uint32_t i;
    for (i = 0; i + 1 < n; i++){
        RantSegment s;
        double d;
        s.a = p[i]; s.b = p[i + 1];
        d = rant_segment_distance(s, q);
        if (d < best) best = d;
    }
    return best;
}

static void polylines(void){
    RantDouble3 pts[64], out[256];
    RantPolylineView l, kept;
    uint32_t i, k, count;
    for (i = 0; i < 300; i++){
        double len;
        l.points = pts;
        l.count = i % 2 ? onward_line(pts, 40) : (uint32_t)(2 + random_unit() * 30);
        if (i % 2 == 0) for (k = 0; k < l.count; k++) pts[k] = random_double3(3.0);
        if (i % 7 == 0) pts[1] = pts[0];
        len = rant_polyline_length(l);
        for (k = 0; k < 20; k++){
            RantDouble3 q = random_double3(5.0);
            RantCurvePoint c = rant_polyline_closest(l, q);
            CHECK(near(rant_polyline_distance(l, q), brute_distance(pts, l.count, q), 1e-12), "the distance is the nearest segment's");
            CHECK(near(rant_double3_distance(c.point, q), rant_polyline_distance(l, q), 1e-12), "closest sits at that distance");
            CHECK(near3(rant_polyline_at(l, c.distance).point, c.point, 1e-9) || brute_distance(pts, l.count, rant_polyline_at(l, c.distance).point) < 1e-9,
                  "the closest point's distance along leads back to it");
            CHECK(near(rant_double3_length(c.tangent), len > 0.0 ? 1.0 : 0.0, 1e-12), "its tangent is unit, or zero on a line of no length");
            CHECK(c.distance >= 0.0 && c.distance <= len + 1e-12, "its distance along lies on the line");
        }
        for (k = 0; k < 20; k++){
            double d = random_range(-1.0, len + 1.0), e = random_range(-1.0, len + 1.0);
            RantCurvePoint a = rant_polyline_at(l, d), b = rant_polyline_at(l, e);
            CHECK(brute_distance(pts, l.count, a.point) < 1e-12, "at gives a point of the line");
            CHECK(near(a.distance, d < 0.0 ? 0.0 : (d > len ? len : d), 1e-9), "at holds the distance to the line's ends");
            CHECK(rant_double3_distance(a.point, b.point) <= fabs(a.distance - b.distance) + 1e-9, "no chord is longer than its arc");
            if (i % 2) CHECK(near(rant_polyline_closest(l, a.point).distance, a.distance, 1e-9), "on an onward line closest undoes at");
        }
        CHECK(near3(rant_polyline_at(l, 0.0).point, pts[0], 0.0) && near3(rant_polyline_at(l, len).point, pts[l.count - 1], 1e-12),
              "at runs from the first point to the last");
        count = 1 + (uint32_t)(random_unit() * 50);
        CHECK(rant_polyline_resample(l, out, count) == RANT_MATH_OK, "resample runs");
        CHECK(near3(out[0], pts[0], 0.0) && (count == 1 || near3(out[count - 1], pts[l.count - 1], 0.0)), "resample keeps the ends");
        if (i % 2 && count > 1)
            for (k = 0; k < count; k++)
                CHECK(near(rant_polyline_closest(l, out[k]).distance, len * k / (count - 1), 1e-9), "resampled points are evenly spread");
        {   double tol = random_range(0.0, 0.5);
            uint32_t m = 256;
            CHECK(rant_polyline_simplify(l, tol, out, &m) == RANT_MATH_OK && m >= (l.count > 1 ? 2u : 1u) && m <= l.count, "simplify runs");
            kept.points = out; kept.count = m;
            CHECK(near3(out[0], pts[0], 0.0) && near3(out[m - 1], pts[l.count - 1], 0.0), "simplify keeps the ends");
            for (k = 0; k < l.count; k++) CHECK(rant_polyline_distance(kept, pts[k]) <= tol + 1e-12, "every dropped point lies within tolerance");
            {   uint32_t j = 0;
                for (k = 0; k < l.count && j < m; k++) if (near3(pts[k], out[j], 0.0)) j++;
                CHECK(j == m, "the kept points are the line's own, in order");
            }
        }
        {   RantSegment e[64];
            RantAlignedBox b = rant_polyline_bounds(l);
            uint32_t m = 64;
            double sum = 0.0;
            CHECK(rant_polyline_edges(l, e, &m) == RANT_MATH_OK, "edges run");
            for (k = 0; k < m; k++){ sum += rant_segment_length(e[k]); CHECK(rant_segment_length(e[k]) > 0.0, "no edge has no length"); }
            CHECK(near(sum, len, 1e-12), "the edges add to the length");
            for (k = 0; k < l.count; k++)
                CHECK(pts[k].x >= b.min.x && pts[k].x <= b.max.x && pts[k].z >= b.min.z && pts[k].z <= b.max.z, "bounds hold every point");
        }
    }
    {   RantDouble3 line[5];
        uint32_t m = 8;
        for (i = 0; i < 5; i++) line[i] = rant_double3(i, 2.0 * i, 0);
        l.points = line; l.count = 5;
        CHECK(rant_polyline_simplify(l, 0.0, out, &m) == RANT_MATH_OK && m == 2, "a straight line simplifies to its ends");
        CHECK(near(rant_polyline_length(l), 4.0 * sqrt(5.0), 1e-14), "length");
        line[2].z = 1.0;
        m = 1;
        CHECK(rant_polyline_simplify(l, 0.5, out, &m) == RANT_MATH_ERR_SPACE && m == 3, "a bump past tolerance stays, and room is asked for");
        CHECK(rant_polyline_resample(l, out, 0) == RANT_MATH_ERR_ARGUMENT, "resample needs a count");
        l.count = 0;
        CHECK(rant_polyline_distance(l, line[0]) == HUGE_VAL && rant_polyline_resample(l, out, 3) == RANT_MATH_ERR_DEGENERATE,
              "a line of no points is nowhere");
        l.count = 1;
        CHECK(near(rant_polyline_distance(l, rant_double3(0, 0, 3)), 3.0, 0.0) && rant_polyline_length(l) == 0.0
              && rant_double3_length(rant_polyline_at(l, 2.0).tangent) == 0.0, "a line of one point is that point");
    }
}

static void polylines2d(void){
    RantDouble2 pts[8], out[8];
    RantPolyline2DView l;
    RantPlane2D planes[8];
    uint32_t m;
    pts[0] = rant_double2(0, 0); pts[1] = rant_double2(4, 0); pts[2] = pts[1]; pts[3] = rant_double2(4, 3);
    l.points = pts; l.count = 4;
    CHECK(near(rant_polyline2d_length(l), 7.0, 0.0), "polyline2d length");
    m = 8;
    CHECK(rant_polyline2d_planes(l, planes, &m) == RANT_MATH_OK && m == 2, "one plane per step with length");
    CHECK(near2(planes[0].normal, rant_double2(0, -1), 0.0) && near2(planes[1].normal, rant_double2(1, 0), 0.0),
          "each faces the right of the way the line runs");
    {   RantCurvePoint2D c = rant_polyline2d_closest(l, rant_double2(5, 1));
        CHECK(near2(c.point, rant_double2(4, 1), 0.0) && near(c.distance, 5.0, 0.0) && near2(c.tangent, rant_double2(0, 1), 0.0),
              "polyline2d closest");
        c = rant_polyline2d_at(l, 4.0);
        CHECK(near2(c.point, rant_double2(4, 0), 0.0) && near2(c.tangent, rant_double2(0, 1), 0.0), "at a corner the next step's way");
        c = rant_polyline2d_at(l, 99.0);
        CHECK(near2(c.point, rant_double2(4, 3), 0.0) && near2(c.tangent, rant_double2(0, 1), 0.0) && near(c.distance, 7.0, 0.0),
              "past the end, the end and the last step's way");
    }
    CHECK(near(rant_polyline2d_distance(l, rant_double2(2, -2)), 2.0, 0.0), "polyline2d distance");
    CHECK(rant_polyline2d_resample(l, out, 8) == RANT_MATH_OK && near2(out[4], rant_double2(4, 0), 1e-15) && near2(out[7], rant_double2(4, 3), 0.0),
          "polyline2d resample");
    m = 8;
    CHECK(rant_polyline2d_simplify(l, 0.1, out, &m) == RANT_MATH_OK && m == 3, "the repeated corner goes");
    {   RantSegment2D e[8];
        RantAlignedBox2D b = rant_polyline2d_bounds(l);
        m = 8;
        CHECK(rant_polyline2d_edges(l, e, &m) == RANT_MATH_OK && m == 2 && near2(e[1].b, rant_double2(4, 3), 0.0), "polyline2d edges");
        CHECK(near2(b.min, rant_double2(0, 0), 0.0) && near2(b.max, rant_double2(4, 3), 0.0), "polyline2d bounds");
    }
}

static void paths(void){
    RantPose poses[4];
    RantPose2D poses2[3];
    RantPathView p;
    RantPath2DView p2;
    RantQuaternion turn = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2);
    RantPose out[5];
    int i;
    for (i = 0; i < 4; i++) poses[i] = rant_pose_identity();
    poses[1].position = rant_double3(1, 0, 0);
    poses[2].position = rant_double3(1, 0, 0); poses[2].orientation = turn;
    poses[3].position = rant_double3(1, 1, 0); poses[3].orientation = turn;
    p.poses = poses; p.count = 4;
    CHECK(near(rant_path_length(p), 2.0, 0.0), "a path's length is its positions'");
    CHECK(same_turn(rant_path_at(p, 0.999).orientation, rant_quaternion_identity(), 1e-12), "before the corner it has not turned");
    CHECK(same_turn(rant_path_at(p, 1.0).orientation, turn, 1e-12) && near3(rant_path_at(p, 1.0).position, rant_double3(1, 0, 0), 0.0),
          "at the corner the turn in place is done");
    CHECK(same_pose(rant_path_at(p, 5.0), poses[3], 0.0), "past the end, the last pose");
    {   RantCurvePoint c = rant_path_closest(p, rant_double3(1.5, 0.5, 0));
        CHECK(near(c.distance, 1.5, 1e-15) && near(rant_path_distance(p, rant_double3(1.5, 0.5, 0)), 0.5, 1e-15), "path closest");
    }
    CHECK(rant_path_resample(p, out, 5) == RANT_MATH_OK && same_pose(out[4], poses[3], 0.0) && same_pose(out[0], poses[0], 0.0)
          && near3(out[2].position, rant_double3(1, 0, 0), 0.0) && same_turn(out[2].orientation, turn, 1e-12), "path resample");
    poses[1].orientation = turn;
    CHECK(same_turn(rant_path_at(p, 0.5).orientation, rant_quaternion_slerp(poses[0].orientation, turn, 0.5), 1e-12),
          "between poses the turn is slerped");
    {   RantAlignedBox b = rant_path_bounds(p);
        CHECK(near3(b.min, rant_double3(0, 0, 0), 0.0) && near3(b.max, rant_double3(1, 1, 0), 0.0), "path bounds");
    }
    for (i = 0; i < 3; i++){ poses2[i].position = rant_double2(i, 0); poses2[i].angle = 0.0; }
    poses2[1].angle = 3.0; poses2[2].angle = -3.0;
    p2.poses = poses2; p2.count = 3;
    CHECK(near(fabs(rant_path2d_at(p2, 1.5).angle), PI, 1e-12), "a 2D path turns the shorter way through pi");
    CHECK(near(rant_path2d_at(p2, 0.5).angle, 1.5, 1e-12) && near2(rant_path2d_at(p2, 0.5).position, rant_double2(0.5, 0), 0.0), "and lerps its heading");
    CHECK(near(rant_path2d_length(p2), 2.0, 0.0) && near(rant_path2d_distance(p2, rant_double2(1, 2)), 2.0, 0.0), "path2d length and distance");
    {   RantPose2D o2[3];
        RantCurvePoint2D c = rant_path2d_closest(p2, rant_double2(1.25, -1));
        RantAlignedBox2D b = rant_path2d_bounds(p2);
        CHECK(rant_path2d_resample(p2, o2, 3) == RANT_MATH_OK && near(o2[1].angle, 3.0, 0.0) && near(o2[2].angle, -3.0, 0.0), "path2d resample");
        CHECK(near(c.distance, 1.25, 0.0) && near2(c.tangent, rant_double2(1, 0), 0.0), "path2d closest");
        CHECK(near2(b.max, rant_double2(2, 0), 0.0), "path2d bounds");
    }
    p.count = 0;
    CHECK(same_pose(rant_path_at(p, 1.0), rant_pose_identity(), 0.0), "an empty path gives the identity");
}

void test_curve(void){
    polylines();
    polylines2d();
    paths();
}
