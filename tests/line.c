#include "check.h"

static RantPlane random_plane(void){
    RantPlane p;
    p.position = random_double3(5.0);
    p.normal = rant_double3_normalize(random_double3(1.0));
    return p;
}

static RantRay random_ray(void){
    RantRay r;
    r.origin = random_double3(5.0);
    r.direction = rant_double3_normalize(random_double3(1.0));
    return r;
}

static RantSegment random_segment(double r){
    RantSegment s;
    s.a = random_double3(r);
    s.b = random_double3(r);
    return s;
}

/* The least distance between points sampled n per unit of each parameter. */
static double sampled_segment_gap(RantDouble3 p1, RantDouble3 d1, double hi1, RantDouble3 p2, RantDouble3 d2, double hi2, int n){
    double best = HUGE_VAL;
    int i, j;
    for (i = 0; i <= n; i++)
        for (j = 0; j <= n; j++){
            RantDouble3 a = rant_double3_add(p1, rant_double3_scale(d1, hi1 * i / n));
            RantDouble3 b = rant_double3_add(p2, rant_double3_scale(d2, hi2 * j / n));
            double d = rant_double3_distance(a, b);
            if (d < best) best = d;
        }
    return best;
}

static void planes(void){
    int i;
    for (i = 0; i < 500; i++){
        RantDouble3 a = random_double3(5.0), b = random_double3(5.0), c = random_double3(5.0), x = random_double3(8.0);
        RantPlane p, q, moved;
        RantPose pose = random_pose(4.0);
        CHECK(rant_plane_from_points(a, b, c, &p) == RANT_MATH_OK, "from_points runs");
        CHECK(near(rant_plane_distance(p, a), 0.0, 1e-12) && near(rant_plane_distance(p, b), 0.0, 1e-12)
              && near(rant_plane_distance(p, c), 0.0, 1e-12), "the three points lie on it");
        CHECK(near(rant_double3_length(p.normal), 1.0, 1e-15), "its normal is unit");
        CHECK(rant_double3_dot(p.normal, rant_double3_cross(rant_double3_subtract(b, a), rant_double3_subtract(c, a))) > 0.0,
              "the normal follows the right hand rule");
        CHECK(near3(rant_double3_add(rant_plane_closest(p, x), rant_double3_scale(p.normal, rant_plane_distance(p, x))), x, 1e-12),
              "closest plus distance along the normal is the point");
        q = p; q.normal = rant_double3_scale(p.normal, 3.7);
        CHECK(near(rant_plane_distance(q, x), rant_plane_distance(p, x), 1e-12), "a long normal counts as unit");
        moved = rant_plane_transform(p, pose);
        CHECK(near(rant_plane_distance(moved, rant_pose_apply(pose, x)), rant_plane_distance(p, x), 1e-11),
              "transform keeps every distance");
        CHECK(rant_plane_from_points(a, b, rant_double3_lerp(a, b, 2.5), &q) == RANT_MATH_ERR_DEGENERATE,
              "from_points refuses points in a line");
    }
    for (i = 0; i < 500; i++){
        RantPlane p = random_plane();
        RantRay r = random_ray();
        RantRayHit h;
        double d0 = rant_plane_distance(p, r.origin), d1 = rant_plane_distance(p, rant_ray_at(r, 1e6));
        int hit = rant_plane_raycast(p, r, &h);
        CHECK(hit == ((d0 > 0.0) != (d1 > 0.0)), "a ray hits exactly when it crosses to the other side");
        if (hit){
            CHECK(h.distance >= 0.0 && near(rant_plane_distance(p, h.point), 0.0, 1e-10), "the hit lies on the plane");
            CHECK(near3(h.point, rant_ray_at(r, h.distance), 1e-12), "the hit lies along the ray");
            CHECK(near3(h.normal, p.normal, 1e-15), "the hit normal is the plane's from either side");
        }
        r.direction = rant_double3_scale(r.direction, 0.01);
        CHECK(rant_plane_raycast(p, r, NULL) == hit, "a short direction counts as unit");
    }
    {   RantPlane floor, wall;
        RantRay r;
        floor.position = rant_double3(0, 0, 0); floor.normal = rant_double3(0, 0, 1);
        wall.position = rant_double3(2, 0, 0); wall.normal = rant_double3(-1, 0, 0);
        r.origin = rant_double3(0, 0, 0); r.direction = rant_double3(1, 0, 0);
        CHECK(!rant_plane_raycast(floor, r, NULL), "a ray in the plane misses it");
        r.origin = rant_double3(0, 0, -3); r.direction = rant_double3(0, 0, 1);
        {   RantRayHit h;
            CHECK(rant_plane_raycast(floor, r, &h) && near(h.distance, 3.0, 1e-15), "a ray from below meets the floor");
        }
        {   RantRay line;
            RantDouble3 corner;
            RantPlane side;
            CHECK(rant_plane_intersect(floor, wall, &line) == RANT_MATH_OK
                  && near3(line.origin, rant_double3(2, 0, 0), 1e-15) && near3(line.direction, rant_double3(0, -1, 0), 1e-15),
                  "the floor meets the wall along -y through x = 2");
            side.position = rant_double3(0, 5, 0); side.normal = rant_double3(0, 1, 0);
            CHECK(rant_plane_intersect3(floor, wall, side, &corner) == RANT_MATH_OK && near3(corner, rant_double3(2, 5, 0), 1e-15),
                  "three planes meet in the corner");
            side = floor; side.position.z = 1.0;
            CHECK(rant_plane_intersect(floor, side, &line) == RANT_MATH_ERR_DEGENERATE, "parallel planes never meet");
            CHECK(rant_plane_intersect3(floor, wall, side, &corner) == RANT_MATH_ERR_DEGENERATE, "nor do three with two parallel");
        }
    }
    for (i = 0; i < 300; i++){
        RantPlane a = random_plane(), b = random_plane(), c = random_plane();
        RantRay line;
        RantDouble3 x;
        CHECK(rant_plane_intersect(a, b, &line) == RANT_MATH_OK, "two random planes meet");
        CHECK(near(rant_plane_distance(a, line.origin), 0.0, 1e-9) && near(rant_plane_distance(b, line.origin), 0.0, 1e-9)
              && near(rant_plane_distance(a, rant_ray_at(line, 7.0)), 0.0, 1e-9) && near(rant_plane_distance(b, rant_ray_at(line, -7.0)), 0.0, 1e-9),
              "the line lies on both planes");
        CHECK(near(rant_double3_dot(line.origin, line.direction), 0.0, 1e-9), "its origin is the point nearest the frame origin");
        CHECK(near3(line.direction, rant_double3_normalize(rant_double3_cross(a.normal, b.normal)), 1e-12), "it runs along a cross b");
        CHECK(rant_plane_intersect3(a, b, c, &x) == RANT_MATH_OK && near(rant_plane_distance(a, x), 0.0, 1e-8)
              && near(rant_plane_distance(b, x), 0.0, 1e-8) && near(rant_plane_distance(c, x), 0.0, 1e-8),
              "three random planes meet in one point");
    }
}

static void segments_and_rays(void){
    int i;
    for (i = 0; i < 300; i++){
        RantSegment s = random_segment(5.0), t = random_segment(5.0), c;
        RantDouble3 x = random_double3(8.0), p = rant_segment_closest(s, x), d = rant_double3_subtract(s.b, s.a);
        double best = HUGE_VAL, gap;
        int k;
        RantPose pose = random_pose(3.0);
        for (k = 0; k <= 2000; k++){
            double e = rant_double3_distance(rant_double3_lerp(s.a, s.b, k / 2000.0), x);
            if (e < best) best = e;
        }
        CHECK(rant_segment_distance(s, x) <= best + 1e-12 && rant_segment_distance(s, x) >= best - 1e-2,
              "the closest point beats every sampled one");
        CHECK(near(rant_double3_length(rant_double3_cross(rant_double3_subtract(p, s.a), d)), 0.0, 1e-10)
              && rant_double3_dot(rant_double3_subtract(p, s.a), d) >= -1e-12 && rant_double3_dot(rant_double3_subtract(p, s.b), d) <= 1e-12,
              "the closest point lies on the segment");
        c = rant_segment_closest_segment(s, t);
        gap = sampled_segment_gap(s.a, d, 1.0, t.a, rant_double3_subtract(t.b, t.a), 1.0, 200);
        CHECK(rant_double3_distance(c.a, c.b) <= gap + 1e-12, "the joining segment beats every sampled pair");
        CHECK(near(rant_segment_distance(s, c.a), 0.0, 1e-10) && near(rant_segment_distance(t, c.b), 0.0, 1e-10),
              "its ends lie on the two segments");
        CHECK(near(rant_segment_distance(rant_segment_transform(s, pose), rant_pose_apply(pose, x)), rant_segment_distance(s, x), 1e-11),
              "transform keeps distances");
        {   RantAlignedBox b = rant_segment_bounds(s);
            CHECK(near3(b.min, rant_double3_min(s.a, s.b), 0.0) && near3(b.max, rant_double3_max(s.a, s.b), 0.0), "bounds");
        }
    }
    {   RantSegment a, b, c;
        a.a = rant_double3(-1, 0, 0); a.b = rant_double3(1, 0, 0);
        b.a = rant_double3(0, -1, 2); b.b = rant_double3(0, 1, 2);
        c = rant_segment_closest_segment(a, b);
        CHECK(near3(c.a, rant_double3(0, 0, 0), 1e-15) && near3(c.b, rant_double3(0, 0, 2), 1e-15), "crossed segments join at their middles");
        b.a = rant_double3(3, 1, 0); b.b = rant_double3(5, 1, 0);
        c = rant_segment_closest_segment(a, b);
        CHECK(near(rant_double3_distance(c.a, c.b), sqrt(5.0), 1e-15), "parallel segments past each other join end to end");
        b.a = rant_double3(0, 0, 0); b.b = b.a;
        c = rant_segment_closest_segment(b, b);
        CHECK(near3(c.a, b.a, 0.0) && near3(c.b, b.a, 0.0), "two points join themselves");
        CHECK(near(rant_segment_length(a), 2.0, 0.0), "length");
        CHECK(near3(rant_segment_closest(b, rant_double3(4, 4, 4)), b.a, 0.0), "a point segment's closest is the point");
    }
    for (i = 0; i < 300; i++){
        RantRay a = random_ray(), b = random_ray();
        RantSegment c = rant_ray_closest_ray(a, b);
        RantDouble3 x = random_double3(8.0);
        double gap = sampled_segment_gap(a.origin, a.direction, 30.0, b.origin, b.direction, 30.0, 300);
        CHECK(rant_double3_distance(c.a, c.b) <= gap + 1e-12, "the rays' joining segment beats every sampled pair");
        CHECK(near(rant_ray_distance(a, c.a), 0.0, 1e-10) && near(rant_ray_distance(b, c.b), 0.0, 1e-10), "its ends lie on the rays");
        CHECK(rant_ray_distance(a, x) <= rant_double3_distance(a.origin, x) + 1e-12, "a ray is never farther than its origin");
    }
    {   /* two cameras looking at one target */
        RantDouble3 target = rant_double3(2, 1, 0.5);
        RantRay a, b;
        RantSegment c;
        a.origin = rant_double3(0, 0, 1); a.direction = rant_double3_subtract(target, a.origin);
        b.origin = rant_double3(0, 2, 1); b.direction = rant_double3_subtract(target, b.origin);
        c = rant_ray_closest_ray(a, b);
        CHECK(near3(c.a, target, 1e-14) && near3(c.b, target, 1e-14), "two camera rays meet at their target");
        CHECK(near3(rant_ray_at(a, rant_double3_distance(a.origin, target)), target, 1e-14), "at walks a unit direction");
        a.origin = rant_double3(0, 0, 0); a.direction = rant_double3(1, 0, 0);
        b.origin = rant_double3(-1, 1, 0); b.direction = rant_double3(-1, 0, 0);
        c = rant_ray_closest_ray(a, b);
        CHECK(near3(c.a, a.origin, 0.0) && near3(c.b, b.origin, 0.0), "rays pointing apart join at their origins");
    }
    for (i = 0; i < 200; i++){
        RantRay r = random_ray();
        RantPose pose = random_pose(4.0);
        RantDouble3 x = random_double3(8.0);
        CHECK(near(rant_ray_distance(rant_ray_transform(r, pose), rant_pose_apply(pose, x)), rant_ray_distance(r, x), 1e-11),
              "ray transform keeps distances");
    }
}

static RantSegment2D seg2(double ax, double ay, double bx, double by){
    RantSegment2D s;
    s.a = rant_double2(ax, ay); s.b = rant_double2(bx, by);
    return s;
}

static void two_d(void){
    int i;
    {   RantPlane2D p;
        RantDouble2 square[4];
        int k;
        square[0] = rant_double2(0, 0); square[1] = rant_double2(1, 0); square[2] = rant_double2(1, 1); square[3] = rant_double2(0, 1);
        CHECK(rant_plane2d_from_points(rant_double2(0, 0), rant_double2(1, 0), &p) == RANT_MATH_OK
              && near2(p.normal, rant_double2(0, -1), 0.0), "the normal is on the right going from a to b");
        for (k = 0; k < 4; k++){
            RantDouble2 mid = rant_double2_lerp(square[k], square[(k + 1) % 4], 0.5);
            rant_plane2d_from_points(square[k], square[(k + 1) % 4], &p);
            CHECK(rant_double2_dot(p.normal, rant_double2_subtract(mid, rant_double2(0.5, 0.5))) > 0.0
                  && near(rant_plane2d_distance(p, rant_double2(0.5, 0.5)), -0.5, 1e-15),
                  "each edge of a counterclockwise square faces out");
        }
        CHECK(rant_plane2d_from_points(square[1], square[1], &p) == RANT_MATH_ERR_DEGENERATE, "from_points refuses one point");
    }
    for (i = 0; i < 500; i++){
        RantDouble2 a = random_double2(5.0), b = random_double2(5.0), x = random_double2(8.0), cross;
        RantPlane2D p, q;
        RantRay2D r;
        RantRayHit2D h;
        RantPose2D pose;
        pose.position = random_double2(4.0); pose.angle = random_range(-PI, PI);
        rant_plane2d_from_points(a, b, &p);
        rant_plane2d_from_points(random_double2(5.0), random_double2(5.0), &q);
        CHECK(near(rant_plane2d_distance(p, b), 0.0, 1e-12), "the line runs through b");
        CHECK(near2(rant_double2_add(rant_plane2d_closest(p, x), rant_double2_scale(p.normal, rant_plane2d_distance(p, x))), x, 1e-12),
              "closest plus distance along the normal is the point");
        CHECK(near(rant_plane2d_distance(rant_plane2d_transform(p, pose), rant_pose2d_apply(pose, x)), rant_plane2d_distance(p, x), 1e-11),
              "transform keeps distances");
        CHECK(rant_plane2d_intersect(p, q, &cross) == RANT_MATH_OK && near(rant_plane2d_distance(p, cross), 0.0, 1e-8)
              && near(rant_plane2d_distance(q, cross), 0.0, 1e-8), "two lines cross in a point on both");
        r.origin = x; r.direction = random_double2(1.0);
        if (rant_plane2d_raycast(p, r, &h)){
            CHECK(near(rant_plane2d_distance(p, h.point), 0.0, 1e-10) && near2(h.point, rant_ray2d_at(r, h.distance), 1e-12)
                  && h.distance >= 0.0, "a 2D hit lies on the line and along the ray");
        } else {
            CHECK(rant_plane2d_distance(p, x) * rant_plane2d_distance(p, rant_ray2d_at(r, 1e6)) > 0.0, "a miss stays on one side");
        }
    }
    {   RantPlane2D p, q;
        RantDouble2 x;
        rant_plane2d_from_points(rant_double2(0, 0), rant_double2(1, 0), &p);
        rant_plane2d_from_points(rant_double2(0, 1), rant_double2(1, 1), &q);
        CHECK(rant_plane2d_intersect(p, q, &x) == RANT_MATH_ERR_DEGENERATE, "parallel lines never cross");
    }
    for (i = 0; i < 1000; i++){
        RantSegment2D a = seg2(random_range(-3, 3), random_range(-3, 3), random_range(-3, 3), random_range(-3, 3));
        RantSegment2D b = seg2(random_range(-3, 3), random_range(-3, 3), random_range(-3, 3), random_range(-3, 3));
        RantSegment2D c = rant_segment2d_closest_segment(a, b);
        RantDouble2 x;
        double gap = rant_double2_distance(c.a, c.b);
        int meet = rant_segment2d_intersect(a, b, &x);
        CHECK(meet == (gap < 1e-12), "segments cross exactly when the gap between them is nil");
        if (meet) CHECK(rant_segment2d_distance(a, x) < 1e-12 && rant_segment2d_distance(b, x) < 1e-12, "the crossing lies on both");
        {   RantRay2D r;
            RantRayHit2D h;
            r.origin = random_double2(4.0); r.direction = rant_double2_normalize(random_double2(1.0));
            if (rant_segment2d_raycast(a, r, &h)){
                CHECK(rant_segment2d_distance(a, h.point) < 1e-10 && near2(h.point, rant_ray2d_at(r, h.distance), 1e-12)
                      && rant_double2_dot(h.normal, r.direction) <= 0.0 && near(rant_double2_dot(h.normal, rant_double2_subtract(a.b, a.a)), 0.0, 1e-12),
                      "a segment hit lies on it with its normal facing the ray");
            } else {
                CHECK(!rant_segment2d_intersect(a, seg2(r.origin.x, r.origin.y, r.origin.x + r.direction.x * 100,
                                                       r.origin.y + r.direction.y * 100), NULL),
                      "a missed segment never crosses the ray");
            }
        }
    }
    {   RantDouble2 x;
        CHECK(rant_segment2d_intersect(seg2(0, 0, 2, 0), seg2(1, 0, 3, 0), &x) && near2(x, rant_double2(1, 0), 0.0),
              "overlapping segments give the shared point nearest a.a");
        CHECK(rant_segment2d_intersect(seg2(2, 0, 0, 0), seg2(1, 0, 3, 0), &x) && near2(x, rant_double2(2, 0), 0.0),
              "and from the other end");
        CHECK(rant_segment2d_intersect(seg2(0, 0, 1, 0), seg2(1, 0, 1, 1), &x) && near2(x, rant_double2(1, 0), 0.0),
              "segments sharing an end touch");
        CHECK(rant_segment2d_intersect(seg2(0, 0, 1, 1), seg2(0.3, 0.3, 0.3, 0.3), &x) && near2(x, rant_double2(0.3, 0.3), 0.0),
              "a point on a segment meets it");
        CHECK(!rant_segment2d_intersect(seg2(0, 0, 1, 1), seg2(0.3, 0.4, 0.3, 0.4), NULL), "a point off it does not");
        CHECK(!rant_segment2d_intersect(seg2(0, 0, 1, 0), seg2(0, 1, 1, 1), NULL), "parallel segments apart never meet");
        CHECK(!rant_segment2d_intersect(seg2(0, 0, 1, 0), seg2(2, 0, 3, 0), NULL), "nor do segments on one line with a gap");
        CHECK(rant_segment2d_intersect(seg2(-1, 0, 1, 0), seg2(0, -1, 0, 1), &x) && near2(x, rant_double2(0, 0), 0.0), "a cross");
    }
    {   RantRay2D r;
        RantRayHit2D h;
        r.origin = rant_double2(0, 0); r.direction = rant_double2(2, 0);
        CHECK(rant_segment2d_raycast(seg2(3, -1, 3, 1), r, &h) && near(h.distance, 3.0, 0.0) && near2(h.normal, rant_double2(-1, 0), 0.0),
              "a laser meets a wall three meters ahead, the normal facing back");
        CHECK(rant_segment2d_raycast(seg2(3, 1, 3, -1), r, &h) && near2(h.normal, rant_double2(-1, 0), 0.0), "whichever way the wall runs");
        CHECK(!rant_segment2d_raycast(seg2(-3, -1, -3, 1), r, NULL), "a wall behind is missed");
        CHECK(near(rant_ray2d_distance(r, rant_double2(-3, 4)), 5.0, 1e-15) && near(rant_ray2d_distance(r, rant_double2(3, 4)), 4.0, 1e-15),
              "a ray's distance behind its origin is to the origin");
        CHECK(near(rant_segment2d_length(seg2(0, 0, 3, 4)), 5.0, 0.0), "segment2d length");
    }
    for (i = 0; i < 200; i++){
        RantSegment2D s = seg2(random_range(-3, 3), random_range(-3, 3), random_range(-3, 3), random_range(-3, 3));
        RantRay2D r;
        RantPose2D pose;
        RantDouble2 x = random_double2(5.0);
        RantAlignedBox2D b = rant_segment2d_bounds(s);
        pose.position = random_double2(4.0); pose.angle = random_range(-PI, PI);
        r.origin = random_double2(3.0); r.direction = random_double2(1.0);
        CHECK(near(rant_segment2d_distance(rant_segment2d_transform(s, pose), rant_pose2d_apply(pose, x)), rant_segment2d_distance(s, x), 1e-11),
              "segment2d transform keeps distances");
        CHECK(near(rant_ray2d_distance(rant_ray2d_transform(r, pose), rant_pose2d_apply(pose, x)), rant_ray2d_distance(r, x), 1e-11),
              "ray2d transform keeps distances");
        CHECK(near2(b.min, rant_double2_min(s.a, s.b), 0.0) && near2(b.max, rant_double2_max(s.a, s.b), 0.0), "segment2d bounds");
    }
}

void test_line(void){
    planes();
    segments_and_rays();
    two_d();
}
