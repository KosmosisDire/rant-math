#include "check.h"

static void three_d(void){
    int i;
    for (i = 0; i < 300; i++){
        RantPose a = random_pose(10.0), b = random_pose(10.0), c = random_pose(10.0);
        RantDouble3 v = random_double3(10.0), pts[3], out[3];
        RantPose ab = rant_pose_compose(a, b);
        CHECK(near3(rant_pose_apply(ab, v), rant_pose_apply(a, rant_pose_apply(b, v)), 1e-11),
              "compose(a, b) maps b's frame through a's");
        CHECK(same_pose(rant_pose_compose(ab, c), rant_pose_compose(a, rant_pose_compose(b, c)), 1e-12),
              "compose is associative");
        CHECK(same_pose(rant_pose_compose(a, rant_pose_inverse(a)), rant_pose_identity(), 1e-12),
              "a pose and its inverse cancel");
        CHECK(near3(rant_pose_apply_inverse(a, rant_pose_apply(a, v)), v, 1e-11), "apply_inverse undoes apply");
        CHECK(same_pose(rant_pose_compose(a, rant_pose_between(a, b)), b, 1e-12), "between(a, b) carries a to b");
        pts[0] = v; pts[1] = b.position; pts[2] = rant_double3(0, 0, 0);
        rant_pose_apply_points(a, pts, out, 3);
        CHECK(near3(out[0], rant_pose_apply(a, v), 1e-11) && near3(out[1], rant_pose_apply(a, b.position), 1e-11)
              && near3(out[2], a.position, 1e-12), "apply_points is apply on each");
        rant_pose_apply_points(a, pts, pts, 3);
        CHECK(near3(pts[0], out[0], 0.0), "apply_points in place");
        CHECK(same_pose(rant_pose_interpolate(a, b, 0.0), a, 1e-15) && same_pose(rant_pose_interpolate(a, b, 1.0), b, 1e-12),
              "interpolate ends at its ends");
    }
    {   /* the camera on a robot on the floor */
        RantPose world_robot = rant_pose_identity(), robot_camera = rant_pose_identity(), world_camera;
        world_robot.position = rant_double3(2, 0, 0);
        world_robot.orientation = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2);
        robot_camera.position = rant_double3(0.5, 0, 1);
        world_camera = rant_pose_compose(world_robot, robot_camera);
        CHECK(near3(world_camera.position, rant_double3(2, 0.5, 1), 1e-15), "the camera half a meter ahead of a robot facing +y");
    }
    {   RantPose p;
        RantDouble3 o = rant_double3(1, 1, 0);
        CHECK(rant_pose_from_points(o, rant_double3(1, 3, 0), rant_double3(-4, 2, 0), &p) == RANT_MATH_OK,
              "from_points runs");
        CHECK(near3(rant_pose_apply(p, rant_double3(1, 0, 0)), rant_double3(1, 2, 0), 1e-15)
              && near3(rant_pose_apply(p, rant_double3(0, 1, 0)), rant_double3(0, 1, 0), 1e-15)
              && near3(rant_quaternion_rotate(p.orientation, rant_double3(0, 0, 1)), rant_double3(0, 0, 1), 1e-15),
              "from_points: x toward on_x, y toward in_xy, z up");
        CHECK(rant_pose_from_points(o, rant_double3(2, 2, 0), rant_double3(3, 3, 0), &p) == RANT_MATH_ERR_DEGENERATE,
              "from_points refuses three points in a line");
        CHECK(rant_pose_from_points(o, o, rant_double3(3, 3, 0), &p) == RANT_MATH_ERR_DEGENERATE,
              "from_points refuses on_x at the origin");
        CHECK(rant_pose_look_at(rant_double3(0, 0, 2), rant_double3(3, 4, 2), rant_double3(0, 0, 1), &p) == RANT_MATH_OK
              && near3(rant_quaternion_rotate(p.orientation, rant_double3(1, 0, 0)), rant_double3(0.6, 0.8, 0), 1e-15),
              "look_at points +x at the target");
        CHECK(rant_pose_look_at(o, o, rant_double3(0, 0, 1), &p) == RANT_MATH_ERR_DEGENERATE, "look_at at itself");
    }
    for (i = 0; i < 100; i++){
        RantPose p = random_pose(5.0), back;
        RantMatrix4x4 m = rant_matrix4x4_from_pose(p);
        RantDouble3 v = random_double3(3.0);
        back = rant_pose_from_matrix4x4(m);
        CHECK(same_pose(back, p, 1e-6), "a pose survives a Matrix4x4");
        CHECK(near3(rant_matrix4x4_apply(m, v), rant_pose_apply(p, v), 1e-5), "the matrix moves points as the pose does");
        {   int r, c;
            for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) m.m[r][c] *= 2.5f;
            CHECK(same_turn(rant_pose_from_matrix4x4(m).orientation, p.orientation, 1e-6), "a scaled matrix keeps its turn");
        }
    }
}

static void two_d(void){
    int i;
    for (i = 0; i < 300; i++){
        RantPose2D a, b, ab;
        RantDouble2 v = random_double2(10.0), pts[2], out[2];
        a.position = random_double2(10.0); a.angle = random_range(-PI, PI);
        b.position = random_double2(10.0); b.angle = random_range(-PI, PI);
        ab = rant_pose2d_compose(a, b);
        CHECK(near2(rant_pose2d_apply(ab, v), rant_pose2d_apply(a, rant_pose2d_apply(b, v)), 1e-11), "2D compose chains");
        CHECK(near2(rant_pose2d_apply_inverse(a, rant_pose2d_apply(a, v)), v, 1e-11), "2D apply_inverse");
        CHECK(near2(rant_pose2d_compose(a, rant_pose2d_inverse(a)).position, rant_double2(0, 0), 1e-11)
              && near(rant_pose2d_compose(a, rant_pose2d_inverse(a)).angle, 0.0, 1e-14), "2D inverse");
        {   RantPose2D d = rant_pose2d_compose(a, rant_pose2d_between(a, b));
            CHECK(near2(d.position, b.position, 1e-11) && fabs(rant_angle_difference(d.angle, b.angle)) < 1e-14, "2D between");
        }
        pts[0] = v; pts[1] = b.position;
        rant_pose2d_apply_points(a, pts, out, 2);
        CHECK(near2(out[0], rant_pose2d_apply(a, v), 1e-12) && near2(out[1], rant_pose2d_apply(a, b.position), 1e-12),
              "2D apply_points");
        {   RantPose lifted = rant_pose_from_pose2d(a);
            RantPose2D down = rant_pose2d_from_pose(lifted);
            CHECK(near2(down.position, a.position, 0.0) && fabs(rant_angle_difference(down.angle, a.angle)) < 1e-14,
                  "a 2D pose lifted and flattened comes back");
            CHECK(near3(rant_pose_apply(lifted, rant_double3(v.x, v.y, 0)),
                        rant_double3(rant_pose2d_apply(a, v).x, rant_pose2d_apply(a, v).y, 0), 1e-11),
                  "the lifted pose moves points as the 2D one does");
        }
    }
    {   RantPose2D a, b, m;
        a.position = rant_double2(0, 0); a.angle = 3.0;
        b.position = rant_double2(2, 0); b.angle = -3.0;
        m = rant_pose2d_interpolate(a, b, 0.5);
        CHECK(near(fabs(m.angle), PI, 1e-12) && near2(m.position, rant_double2(1, 0), 0.0),
              "2D interpolate turns the short way across pi (%g)", m.angle);
    }
    {   RantPose p = rant_pose_identity();
        p.orientation = rant_quaternion_from_euler(0.3, -0.2, 1.1, RANT_EULER_FIXED_XYZ);
        CHECK(near(rant_pose2d_from_pose(p).angle, rant_quaternion_yaw(p.orientation), 0.0),
              "flattening keeps the heading of the turned +x");
    }
}

void test_pose(void){
    three_d();
    two_d();
}
