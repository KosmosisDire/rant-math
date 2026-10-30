#include "check.h"

static RantDouble3 lift(RantDouble2 v){ return rant_double3(v.x, v.y, 0.0); }

/* The 12 triangles of a box, counterclockwise seen from outside. */
static const uint32_t box_triangles[36] = {
    0, 2, 1, 0, 3, 2,   4, 5, 6, 4, 6, 7,   0, 1, 5, 0, 5, 4,
    1, 2, 6, 1, 6, 5,   2, 3, 7, 2, 7, 6,   3, 0, 4, 3, 4, 7
};

static RantMeshView box_mesh(RantDouble3 v[8], RantAlignedBox b){
    RantMeshView m;
    rant_aligned_box_corners(b, v);
    m.vertices = v; m.vertex_count = 8;
    m.triangles = box_triangles; m.triangle_count = 12;
    return m;
}

static void meshes(void){
    RantAlignedBox b;
    RantDouble3 v[8];
    RantMeshView m;
    uint32_t flipped[36], bad[36];
    int i, k;
    b.min = rant_double3(-1, 0, 2); b.max = rant_double3(3, 2, 5);
    m = box_mesh(v, b);
    CHECK(rant_mesh_check(m) == RANT_MATH_OK, "a box mesh checks out");
    CHECK(near(rant_mesh_volume(m), rant_aligned_box_volume(b), 1e-13) && near(rant_mesh_area(m), rant_aligned_box_area(b), 1e-13),
          "a box mesh has the box's volume and area");
    CHECK(near3(rant_mesh_centroid(m), rant_aligned_box_centroid(b), 1e-14), "and its centroid");
    {   RantInertia a = rant_mesh_inertia(m, 5.0), want = rant_aligned_box_inertia(b, 5.0);
        CHECK(near(a.ixx, want.ixx, 1e-12) && near(a.iyy, want.iyy, 1e-12) && near(a.izz, want.izz, 1e-12)
              && near(a.ixy, 0.0, 1e-12) && near(a.ixz, 0.0, 1e-12) && near(a.iyz, 0.0, 1e-12), "and its inertia");
    }
    for (i = 0; i < 2000; i++){
        RantDouble3 q = rant_double3(random_range(-3, 5), random_range(-2, 4), random_range(0, 7));
        double d = rant_aligned_box_distance(b, q);
        RantSurfacePoint s = rant_mesh_closest(m, q);
        RantRay r;
        RantRayHit h, hb;
        CHECK(near(rant_mesh_distance(m, q), fabs(d), 1e-12), "a box mesh's distance is the box's without its sign");
        CHECK(near(rant_double3_distance(s.point, q), fabs(d), 1e-12) && fabs(rant_aligned_box_distance(b, s.point)) < 1e-12,
              "its closest point lies on the box");
        CHECK(near(rant_double3_length(s.normal), 1.0, 1e-12), "with a unit face normal");
        if (fabs(d) > 1e-9) CHECK(rant_mesh_contains(m, q) == (d < 0.0), "contains by winding agrees with the box");
        r.origin = q; r.direction = random_double3(1.0);
        k = rant_aligned_box_raycast(b, r, &hb);
        CHECK(rant_mesh_raycast(m, r, &h) == k, "the mesh meets rays where the box does");
        if (k && rant_mesh_raycast(m, r, &h)) CHECK(near(h.distance, hb.distance, 1e-10), "at the same distance");
    }
    {   RantRay r;
        RantRayHit h;
        r.origin = rant_double3(1, 1, 3.5); r.direction = rant_double3(0, 0, 1);
        CHECK(rant_mesh_raycast(m, r, &h) && near(h.distance, 1.5, 1e-15) && near3(h.normal, rant_double3(0, 0, 1), 1e-15),
              "from inside, the face's own outward normal");
        r.origin = rant_double3(1, 1, 3.5); r.direction = rant_double3(1, 1, 0);
        CHECK(rant_mesh_raycast(m, r, &h) && near(h.distance, sqrt(2.0), 1e-12), "a ray along a diagonal seam still meets");
    }
    for (i = 0; i < 36; i += 3){ flipped[i] = box_triangles[i]; flipped[i + 1] = box_triangles[i + 2]; flipped[i + 2] = box_triangles[i + 1]; }
    {   RantMeshView inside_out = m;
        RantInertia a, want = rant_aligned_box_inertia(b, 5.0);
        inside_out.triangles = flipped;
        a = rant_mesh_inertia(inside_out, 5.0);
        CHECK(near(rant_mesh_volume(inside_out), -rant_aligned_box_volume(b), 1e-13), "an inside out mesh holds a negative volume");
        CHECK(near(a.ixx, want.ixx, 1e-12) && near(a.izz, want.izz, 1e-12), "and the same inertia");
        CHECK(!rant_mesh_contains(inside_out, rant_aligned_box_centroid(b)), "and winds the other way round its middle");
    }
    for (i = 0; i < 100; i++){
        RantPose pose = random_pose(4.0);
        RantDouble3 moved[8];
        RantMeshView mm = m;
        RantOrientedBox ob = rant_aligned_box_transform(b, pose);
        RantInertia a, want = rant_oriented_box_inertia(ob, 2.0);
        RantDouble3 q = rant_double3_add(ob.pose.position, random_double3(4.0));
        rant_pose_apply_points(pose, v, moved, 8);
        mm.vertices = moved;
        a = rant_mesh_inertia(mm, 2.0);
        CHECK(near(a.ixx, want.ixx, 1e-11) && near(a.ixy, want.ixy, 1e-11) && near(a.ixz, want.ixz, 1e-11) && near(a.iyy, want.iyy, 1e-11)
              && near(a.iyz, want.iyz, 1e-11) && near(a.izz, want.izz, 1e-11) && near3(a.center, want.center, 1e-12),
              "a moved box mesh has the oriented box's inertia");
        CHECK(near(rant_mesh_distance(mm, q), fabs(rant_oriented_box_distance(ob, q)), 1e-12), "and its distances");
        CHECK(near(rant_mesh_volume(mm), 24.0, 1e-11), "and its volume");
    }
    memcpy(bad, box_triangles, sizeof bad);
    bad[4] = 8;
    {   RantMeshView broken = m;
        broken.triangles = bad;
        CHECK(rant_mesh_check(broken) == RANT_MATH_ERR_ARGUMENT, "check finds an index past the vertices");
        CHECK(near(rant_mesh_area(broken), rant_aligned_box_area(b) - 4.0, 1e-13), "the others skip that triangle");
        broken.triangles = NULL;
        CHECK(rant_mesh_check(broken) == RANT_MATH_ERR_ARGUMENT, "and triangles that are not there");
        broken.triangle_count = 0;
        CHECK(rant_mesh_check(broken) == RANT_MATH_OK && rant_mesh_distance(broken, v[0]) == HUGE_VAL, "an empty mesh is nowhere");
    }
    {   /* a ball of triangles holds close to the ball's volume */
        RantDouble3 ball[2 + 30 * 60];
        uint32_t tri[3 * (2 * 30 * 60)], nt = 0, a, s, rings = 31, around = 60;
        RantMeshView bm;
        ball[0] = rant_double3(0, 0, 1);
        ball[1] = rant_double3(0, 0, -1);
        for (a = 1; a < rings; a++)
            for (s = 0; s < around; s++){
                double th = PI * a / rings, ph = 2.0 * PI * s / around;
                ball[2 + (a - 1) * around + s] = rant_double3(sin(th) * cos(ph), sin(th) * sin(ph), cos(th));
            }
        for (s = 0; s < around; s++){
            uint32_t s1 = (s + 1) % around, last = 2 + (rings - 2) * around;
            tri[nt * 3] = 0; tri[nt * 3 + 1] = 2 + s; tri[nt * 3 + 2] = 2 + s1; nt++;
            tri[nt * 3] = 1; tri[nt * 3 + 1] = last + s1; tri[nt * 3 + 2] = last + s; nt++;
            for (a = 0; a + 2 < rings; a++){
                uint32_t p = 2 + a * around + s, q = 2 + a * around + s1, r = p + around, t = q + around;
                tri[nt * 3] = p; tri[nt * 3 + 1] = r; tri[nt * 3 + 2] = t; nt++;
                tri[nt * 3] = p; tri[nt * 3 + 1] = t; tri[nt * 3 + 2] = q; nt++;
            }
        }
        bm.vertices = ball; bm.vertex_count = 2 + (rings - 1) * around; bm.triangles = tri; bm.triangle_count = nt;
        CHECK(rant_mesh_check(bm) == RANT_MATH_OK, "the ball mesh checks out");
        CHECK(near(rant_mesh_volume(bm), 4.0 / 3.0 * PI, 0.02) && near(rant_mesh_area(bm), 4.0 * PI, 0.03), "a ball mesh holds a ball");
        CHECK(near(rant_mesh_inertia(bm, 1.0).izz, 0.4, 0.01) && near3(rant_mesh_centroid(bm), rant_double3(0, 0, 0), 1e-12),
              "and turns like one");
        for (i = 0; i < 300; i++){
            RantDouble3 q = random_double3(1.5);
            double r = rant_double3_length(q);
            if (fabs(r - 1.0) > 0.01) CHECK(rant_mesh_contains(bm, q) == (r < 1.0), "winding tells inside a ball mesh");
            CHECK(fabs(rant_mesh_distance(bm, q) - fabs(r - 1.0)) < 0.01, "its distance is near the ball's");
        }
    }
}

static void flat_polygons(void){
    RantDouble2 base[64], flat[64];
    RantDouble3 pts[64];
    RantPolygonView p;
    RantPrismView sheet;
    RantPlane plane;
    RantPose frame;
    uint32_t i, k;
    for (i = 0; i < 200; i++){
        RantPose pose = random_pose(3.0);
        p.points = pts;
        p.count = random_polygon(base, 30);
        for (k = 0; k < p.count; k++) pts[k] = rant_pose_apply(pose, lift(base[k]));
        sheet.pose = pose; sheet.base.points = base; sheet.base.count = p.count; sheet.height = 0.0;
        CHECK(rant_polygon_plane(p, &plane) == RANT_MATH_OK, "a flat polygon has a plane");
        CHECK(near(fabs(rant_double3_dot(plane.normal, rant_quaternion_rotate(pose.orientation, rant_double3(0, 0, 1)))), 1.0, 1e-12),
              "square to the polygon");
        CHECK(rant_double3_dot(plane.normal, rant_quaternion_rotate(pose.orientation, rant_double3(0, 0, 1))) * rant_polygon2d_signed_area(sheet.base) > 0.0,
              "facing by the right hand rule round the corners");
        CHECK(near(rant_polygon_area(p), rant_polygon2d_area(sheet.base), 1e-12) && near(rant_polygon_perimeter(p), rant_polygon2d_perimeter(sheet.base), 1e-12),
              "area and perimeter are the 2D ones");
        CHECK(near3(rant_polygon_centroid(p), rant_pose_apply(pose, lift(rant_polygon2d_centroid(sheet.base))), 1e-12), "centroid");
        CHECK(rant_polygon_flatten(p, &frame, flat) == RANT_MATH_OK, "flatten runs");
        for (k = 0; k < p.count; k++) CHECK(near3(rant_pose_apply(frame, lift(flat[k])), pts[k], 1e-12), "the frame maps flat corners back");
        CHECK(near3(rant_pose_apply(frame, rant_double3(0, 0, 0)), pts[0], 1e-12), "the frame sits on the first corner");
        for (k = 0; k < 30; k++){
            RantDouble3 q = rant_double3_add(pose.position, random_double3(3.0));
            RantSurfacePoint s = rant_polygon_closest(p, q);
            RantRay r;
            RantRayHit a, b;
            CHECK(near(rant_polygon_distance(p, q), rant_prism_distance(sheet, q), 1e-11), "a polygon is a prism of no height");
            CHECK(near(rant_double3_distance(s.point, q), rant_polygon_distance(p, q), 1e-12) && near3(s.normal, plane.normal, 1e-12),
                  "closest lies at the distance with the polygon's normal");
            r.origin = q; r.direction = rant_double3_subtract(rant_pose_apply(pose, lift(base[0])), q);
            r.direction = rant_double3_lerp(r.direction, random_double3(1.0), 0.3);
            if (rant_polygon_raycast(p, r, &a)){
                CHECK(rant_polygon_distance(p, a.point) < 1e-10 && near3(a.normal, plane.normal, 1e-12), "a polygon hit lies on it");
                CHECK(!rant_prism_raycast(sheet, r, &b) || near(a.distance, b.distance, 1e-9), "where the flat prism's does");
            } else {
                CHECK(!rant_prism_raycast(sheet, r, NULL), "a ray that misses the polygon misses the flat prism");
            }
        }
        {   RantSegment e[64];
            uint32_t m = 64;
            CHECK(rant_polygon_edges(p, e, &m) == RANT_MATH_OK && m == p.count, "one edge per side");
        }
    }
    {   RantDouble3 line[3];
        line[0] = rant_double3(0, 0, 0); line[1] = rant_double3(1, 1, 1); line[2] = rant_double3(2, 2, 2);
        p.points = line; p.count = 3;
        CHECK(rant_polygon_plane(p, &plane) == RANT_MATH_ERR_DEGENERATE, "corners in a line make no plane");
        CHECK(near(rant_polygon_distance(p, rant_double3(0, 0, 1)), sqrt(2.0 / 3.0), 1e-15), "and the polygon is its edges");
        {   RantRay r;
            r.origin = rant_double3(0, 0, 0); r.direction = rant_double3(1, 0, 0);
            CHECK(!rant_polygon_raycast(p, r, NULL), "which no ray meets");
        }
    }
}

static void vertex_normals(void){
    static RantDouble3 v[2000], n[2000];
    static uint32_t t[3 * 4000];
    RantSphere ball;
    RantShape s;
    RantMeshView m;
    uint32_t nv = 2000, nt = 4000, i;
    int bad = 0;
    double worst = 0.0;
    ball.center = rant_double3(1, -2, 0.5);
    ball.radius = 2.0;
    s = rant_shape_from_sphere(ball);
    rant_shape_mesh(&s, 48, v, &nv, t, &nt);
    m.vertices = v; m.vertex_count = nv; m.triangles = t; m.triangle_count = nt;
    CHECK(rant_mesh_normals(m, n) == RANT_MATH_OK, "a ball's normals");
    {   /* the sum of each triangle's twice area normal, by its definition */
        static RantDouble3 sum[2000];
        for (i = 0; i < nv; i++) sum[i] = rant_double3(0, 0, 0);
        for (i = 0; i < nt; i++){
            uint32_t a = t[3 * i], b = t[3 * i + 1], c = t[3 * i + 2];
            RantDouble3 f = rant_double3_cross(rant_double3_subtract(v[b], v[a]), rant_double3_subtract(v[c], v[a]));
            sum[a] = rant_double3_add(sum[a], f); sum[b] = rant_double3_add(sum[b], f); sum[c] = rant_double3_add(sum[c], f);
        }
        for (i = 0; i < nv; i++){
            double a = rant_double3_angle(n[i], rant_double3_subtract(v[i], ball.center));
            if (a > worst) worst = a;
            bad += !near3(n[i], rant_double3_normalize(sum[i]), 1e-14) || !(a < 0.03);
        }
    }
    CHECK(bad == 0, "%d differ from the area weighted sum or point far from out, worst %.3g rad", bad, worst);
    {   /* a flat square whose last vertex no triangle uses, and a triangle past the vertices */
        RantDouble3 sq[5], out[5];
        uint32_t tri[9] = { 0, 1, 2, 0, 2, 3, 0, 9, 1 };
        sq[0] = rant_double3(0, 0, 1); sq[1] = rant_double3(1, 0, 1); sq[2] = rant_double3(1, 1, 1); sq[3] = rant_double3(0, 1, 1);
        sq[4] = rant_double3(5, 5, 5);
        m.vertices = sq; m.vertex_count = 5; m.triangles = tri; m.triangle_count = 3;
        CHECK(rant_mesh_normals(m, out) == RANT_MATH_OK, "a square");
        CHECK(near3(out[0], rant_double3(0, 0, 1), 1e-15) && near3(out[3], rant_double3(0, 0, 1), 1e-15), "faces +z, the bad triangle skipped");
        CHECK(out[4].x != out[4].x, "a vertex no triangle uses is NaN");
        CHECK(rant_mesh_normals(m, NULL) == RANT_MATH_ERR_ARGUMENT, "normals need room");
    }
}

void test_surface(void){
    meshes();
    flat_polygons();
    vertex_normals();
}
