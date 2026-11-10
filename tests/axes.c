#include "check.h"

static const char *const names[] = {
    "unity", "unreal", "godot", "bevy", "gltf", "blender", "opencv camera", "opengl camera", "ned", "frd"
};

static void presets(void){
    RantAxes unity = rant_axes_of(RANT_AXES_UNITY), unreal = rant_axes_of(RANT_AXES_UNREAL);
    RantAxes cv = rant_axes_of(RANT_AXES_OPENCV_CAMERA), gl = rant_axes_of(RANT_AXES_OPENGL_CAMERA);
    RantAxes ned = rant_axes_of(RANT_AXES_NED);
    CHECK(near3(rant_axes_point_to(unity, rant_double3(1, 2, 3)), rant_double3(-2, 3, 1), 0.0),
          "Unity: Rant (1, 2, 3) is (-2, 3, 1)");
    CHECK(near3(rant_axes_direction_to(unity, rant_double3(1, 0, 0)), rant_double3(0, 0, 1), 0.0),
          "Rant forward is Unity forward");
    CHECK(near3(rant_axes_point_to(unreal, rant_double3(1, 2, 3)), rant_double3(100, -200, 300), 0.0),
          "Unreal: centimeters, y to the right");
    CHECK(near3(rant_axes_direction_to(cv, rant_double3(1, 0, 0)), rant_double3(0, 0, 1), 0.0)
          && near3(rant_axes_direction_to(cv, rant_double3(0, 0, 1)), rant_double3(0, -1, 0), 0.0),
          "OpenCV camera: forward is +z, up is -y");
    CHECK(near3(rant_axes_direction_to(gl, rant_double3(1, 0, 0)), rant_double3(0, 0, -1), 0.0),
          "OpenGL camera looks down -z");
    CHECK(near3(rant_axes_point_to(ned, rant_double3(10, 20, 3)), rant_double3(20, 10, -3), 0.0),
          "NED: east north up becomes north east down");
    {   /* a turn of +90 degrees about Rant's up is a left turn: Unity, which is left handed, calls it -90 about y */
        RantQuaternion q = rant_axes_quaternion_to(unity, rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2));
        CHECK(same_turn(q, rant_quaternion_from_axis_angle(rant_double3(0, 1, 0), -PI / 2), 1e-15),
              "a left turn in Unity is a negative turn about y");
    }
    {   RantAxes bad;
        CHECK(rant_axes_make(1, 1, 3, 1.0, &bad) == RANT_MATH_ERR_ARGUMENT, "make refuses a repeated axis");
        CHECK(rant_axes_make(1, 2, 4, 1.0, &bad) == RANT_MATH_ERR_ARGUMENT, "make refuses a fourth axis");
        CHECK(rant_axes_make(1, 2, 3, 0.0, &bad) == RANT_MATH_ERR_ARGUMENT, "make refuses scale 0");
        CHECK(rant_axes_make(-3, 1, -2, 0.001, &bad) == RANT_MATH_OK && bad.x == -3 && bad.scale == 0.001, "make");
    }
}

/* For every preset: values come back, and the converted values still describe the same
 * physics in the tool's own axes. */
static void consistency(void){
    int p, i;
    for (p = 0; p <= (int)RANT_AXES_FRD; p++){
        RantAxes a = rant_axes_of((RantAxesPreset)p);
        double s = a.scale;
        for (i = 0; i < 50; i++){
            RantPose pose = random_pose(4.0), tp;
            RantDouble3 v = random_double3(3.0), x = random_double3(3.0);
            RantTwist t, tt;
            RantWrench w, tw;
            RantCovariance6 c, tc, bc;
            int r, k, ok = 1;
            t.linear = random_double3(2.0); t.angular = random_double3(2.0);
            w.force = random_double3(5.0); w.torque = rant_double3_cross(x, w.force);
            tp = rant_axes_pose_to(a, pose);
            CHECK(same_pose(rant_axes_pose_from(a, tp), pose, 1e-12), "%s: a pose comes back", names[p]);
            CHECK(near3(rant_pose_apply(tp, rant_axes_point_to(a, v)), rant_axes_point_to(a, rant_pose_apply(pose, v)), 1e-9 * s),
                  "%s: the converted pose moves converted points", names[p]);
            tt = rant_axes_twist_to(a, t);
            CHECK(near3(rant_double3_add(tt.linear, rant_double3_cross(tt.angular, rant_axes_point_to(a, x))),
                        rant_axes_point_to(a, rant_double3_add(t.linear, rant_double3_cross(t.angular, x))), 1e-9 * s),
                  "%s: a converted twist moves the converted point the same", names[p]);
            CHECK(near3(rant_axes_twist_from(a, tt).angular, t.angular, 1e-14)
                  && near3(rant_axes_twist_from(a, tt).linear, t.linear, 1e-12), "%s: a twist comes back", names[p]);
            tw = rant_axes_wrench_to(a, w);
            CHECK(near3(tw.torque, rant_double3_cross(rant_axes_point_to(a, x), tw.force), 1e-9 * s * s),
                  "%s: a converted wrench keeps its moment", names[p]);
            CHECK(near3(rant_axes_wrench_from(a, tw).torque, w.torque, 1e-10), "%s: a wrench comes back", names[p]);
            {   RantAccel acc, back;
                acc.linear = random_double3(1.0); acc.angular = random_double3(1.0);
                back = rant_axes_accel_from(a, rant_axes_accel_to(a, acc));
                CHECK(near3(back.linear, acc.linear, 1e-14) && near3(back.angular, acc.angular, 1e-15),
                      "%s: an accel comes back", names[p]);
            }
            for (r = 0; r < 6; r++) for (k = 0; k < 6; k++) c.m[r][k] = random_range(-1, 1);
            rant_axes_covariance6_to(a, &c, &tc);
            rant_axes_covariance6_from(a, &tc, &bc);
            for (r = 0; r < 6; r++) for (k = 0; k < 6; k++) if (!near(bc.m[r][k], c.m[r][k], 1e-13)) ok = 0;
            CHECK(ok, "%s: a covariance comes back", names[p]);
            CHECK(near(tc.m[0][0] + tc.m[1][1] + tc.m[2][2], s * s * (c.m[0][0] + c.m[1][1] + c.m[2][2]), 1e-9 * s * s),
                  "%s: a position variance scales by the square", names[p]);
            {   RantDouble3 pts[2], out[2];
                pts[0] = v; pts[1] = x;
                rant_axes_points_to(a, pts, out, 2);
                CHECK(near3(out[1], rant_axes_point_to(a, x), 0.0), "%s: points_to", names[p]);
                rant_axes_points_from(a, out, out, 2);
                CHECK(near3(out[0], v, 1e-14), "%s: points come back in place", names[p]);
            }
        }
    }
}

void test_axes(void){
    presets();
    consistency();
}
