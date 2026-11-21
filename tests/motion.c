#include "check.h"

static void velocities_and_forces(void){
    int i;
    for (i = 0; i < 300; i++){
        RantPose p = random_pose(5.0);
        RantTwist t, ta;
        RantWrench w, wa;
        RantDouble3 x = random_double3(3.0), r = random_double3(3.0), f = random_double3(10.0), xa, va, want;
        t.linear = random_double3(2.0); t.angular = random_double3(2.0);
        ta = rant_twist_transform(p, t);
        /* any body point moves the same however the twist is written */
        xa = rant_pose_apply(p, x);
        va = rant_double3_add(ta.linear, rant_double3_cross(ta.angular, xa));
        want = rant_quaternion_rotate(p.orientation, rant_double3_add(t.linear, rant_double3_cross(t.angular, x)));
        CHECK(near3(va, want, 1e-10), "a transformed twist moves every body point the same");
        /* a force at a point gives the same torque about the parent's origin */
        w.force = f; w.torque = rant_double3_cross(r, f);
        wa = rant_wrench_transform(p, w);
        want = rant_double3_cross(rant_pose_apply(p, r), rant_quaternion_rotate(p.orientation, f));
        CHECK(near3(wa.torque, want, 1e-9) && near3(wa.force, rant_quaternion_rotate(p.orientation, f), 1e-12),
              "a transformed wrench keeps its moment");
    }
    for (i = 0; i < 300; i++){
        RantPose2D p;
        RantTwist2D t, ta;
        RantWrench2D w, wa;
        RantDouble2 x = random_double2(3.0), r = random_double2(3.0), f = random_double2(5.0), xa, va, want;
        p.position = random_double2(5.0); p.angle = random_range(-PI, PI);
        t.linear = random_double2(2.0); t.angular = random_range(-2, 2);
        ta = rant_twist2d_transform(p, t);
        xa = rant_pose2d_apply(p, x);
        va = rant_double2_add(ta.linear, rant_double2_scale(rant_double2_perpendicular(xa), ta.angular));
        want = rant_double2_rotate(rant_double2_add(t.linear, rant_double2_scale(rant_double2_perpendicular(x), t.angular)), p.angle);
        CHECK(near2(va, want, 1e-10), "a transformed 2D twist moves every body point the same");
        w.force = f; w.torque = rant_double2_cross(r, f);
        wa = rant_wrench2d_transform(p, w);
        CHECK(near(wa.torque, rant_double2_cross(rant_pose2d_apply(p, r), rant_double2_rotate(f, p.angle)), 1e-10),
              "a transformed 2D wrench keeps its moment");
    }
}

static void integration(void){
    int i, k;
    for (i = 0; i < 200; i++){
        RantPose a = random_pose(5.0), b, small;
        RantTwist t, back;
        double dt = random_range(0.01, 2.0);
        t.linear = random_double3(3.0); t.angular = random_double3(1.4);
        if (i == 0) t.angular = rant_double3(0, 0, 0);
        if (i == 1) t.angular = rant_double3(1e-7, 0, 0);
        b = rant_pose_integrate(a, t, dt);
        CHECK(rant_twist_between(a, b, dt, &back) == RANT_MATH_OK, "twist_between runs");
        CHECK(same_pose(rant_pose_integrate(a, back, dt), b, 1e-10), "twist_between reaches b (case %d)", i);
        if (rant_double3_length(t.angular) * dt < PI - 1e-3)
            CHECK(near3(back.linear, t.linear, 1e-9) && near3(back.angular, t.angular, 1e-10),
                  "twist_between undoes integrate under half a turn (case %d)", i);
        else
            CHECK(rant_double3_length(back.angular) * dt <= PI + 1e-12, "twist_between turns at most half a turn");
        /* many tiny steps converge on the same arc */
        small = a;
        for (k = 0; k < 2000; k++){
            RantPose step;
            step.position = rant_double3_scale(t.linear, dt / 2000);
            step.orientation = rant_quaternion_from_rotation_vector(rant_double3_scale(t.angular, dt / 2000));
            small = rant_pose_compose(small, step);
        }
        if (i < 20) CHECK(same_pose(small, b, 1e-2 * dt), "integrate is the limit of small steps");
    }
    {   /* a wheel turning at 1 rad/s moving 1 m/s ahead traces a unit circle */
        RantPose start = rant_pose_identity(), end;
        RantTwist t;
        t.linear = rant_double3(1, 0, 0); t.angular = rant_double3(0, 0, 1);
        end = rant_pose_integrate(start, t, PI);
        CHECK(near3(end.position, rant_double3(0, 2, 0), 1e-14), "half a circle ends 2 m to the left");
    }
    {   RantTwist out;
        RantPose p = rant_pose_identity();
        CHECK(rant_twist_between(p, p, 0.0, &out) == RANT_MATH_ERR_ARGUMENT, "twist_between refuses dt 0");
    }
    for (i = 0; i < 200; i++){
        RantPose2D a, b;
        RantTwist2D t, back;
        double dt = random_range(0.01, 2.0);
        a.position = random_double2(5.0); a.angle = random_range(-PI, PI);
        t.linear = random_double2(3.0); t.angular = random_range(-1.5, 1.5);
        if (i == 0) t.angular = 0.0;
        b = rant_pose2d_integrate(a, t, dt);
        CHECK(rant_twist2d_between(a, b, dt, &back) == RANT_MATH_OK && near2(back.linear, t.linear, 1e-9)
              && near(back.angular, t.angular, 1e-12), "2D twist_between undoes integrate (case %d)", i);
        {   RantPose up = rant_pose_integrate(rant_pose_from_pose2d(a),
                (RantTwist){ { t.linear.x, t.linear.y, 0 }, { 0, 0, t.angular } }, dt);
            RantPose2D flat = rant_pose2d_from_pose(up);
            CHECK(near2(flat.position, b.position, 1e-12) && fabs(rant_angle_difference(flat.angle, b.angle)) < 1e-12,
                  "2D integrate is the 3D one on the floor");
        }
    }
}

static void covariance_and_inertia(void){
    RantCovariance3 c3, r3;
    RantCovariance6 c6, r6, back;
    RantQuaternion q = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2);
    int i, j, ok = 1;
    memset(&c3, 0, sizeof c3);
    c3.m[0][0] = 4.0; c3.m[1][1] = 1.0; c3.m[2][2] = 9.0; c3.m[0][1] = c3.m[1][0] = 0.5;
    rant_covariance3_rotate(q, &c3, &r3);
    CHECK(near(r3.m[0][0], 1.0, 1e-14) && near(r3.m[1][1], 4.0, 1e-14) && near(r3.m[2][2], 9.0, 1e-14)
          && near(r3.m[0][1], -0.5, 1e-14), "a quarter turn swaps the x and y variances");
    rant_covariance3_rotate(q, &c3, &c3);
    CHECK(near(c3.m[0][0], 1.0, 1e-14), "covariance3_rotate in place");
    for (i = 0; i < 6; i++) for (j = 0; j < 6; j++) c6.m[i][j] = (i == j) ? 1.0 + i : 0.1 * (i + j);
    q = random_quaternion();
    rant_covariance6_rotate(q, &c6, &r6);
    rant_covariance6_rotate(rant_quaternion_inverse(q), &r6, &back);
    for (i = 0; i < 6; i++) for (j = 0; j < 6; j++) if (!near(back.m[i][j], c6.m[i][j], 1e-13)) ok = 0;
    CHECK(ok, "covariance6_rotate there and back");
    {   double trace = 0.0, trace_r = 0.0;
        for (i = 0; i < 3; i++){ trace += c6.m[i][i]; trace_r += r6.m[i][i]; }
        CHECK(near(trace, trace_r, 1e-13), "a rotated position covariance keeps its trace");
    }
    {   /* spreads of 9, 4 and 1 turned at random come back as their axes, longest first, at two deviations */
        RantCovariance3 d, turned;
        RantQuaternion t, found;
        RantDouble3 radii;
        memset(&d, 0, sizeof d);
        d.m[0][0] = 1.0; d.m[1][1] = 9.0; d.m[2][2] = 4.0;
        CHECK(rant_covariance3_ellipsoid(&d, 2.0, &t, &radii) == RANT_MATH_OK && near3(radii, rant_double3(6, 4, 2), 1e-12)
              && near3(rant_quaternion_rotate(t, rant_double3(1, 0, 0)), rant_double3(0, 1, 0), 1e-12)
              && near3(rant_quaternion_rotate(t, rant_double3(0, 1, 0)), rant_double3(0, 0, 1), 1e-12)
              && near3(rant_quaternion_rotate(t, rant_double3(0, 0, 1)), rant_double3(1, 0, 0), 1e-12), "a diagonal one");
        for (i = 0, ok = 1; i < 20; i++){
            RantDouble3 x;
            q = random_quaternion();
            memset(&d, 0, sizeof d);
            d.m[0][0] = 9.0; d.m[1][1] = 4.0; d.m[2][2] = 1.0;
            rant_covariance3_rotate(q, &d, &turned);
            if (rant_covariance3_ellipsoid(&turned, 1.0, &found, &radii) != RANT_MATH_OK || !near3(radii, rant_double3(3, 2, 1), 1e-9)) ok = 0;
            for (j = 0; j < 3; j++){
                x = rant_quaternion_rotate(found, rant_double3(j == 0, j == 1, j == 2));
                if (!near(fabs(rant_double3_dot(x, rant_quaternion_rotate(q, rant_double3(j == 0, j == 1, j == 2)))), 1.0, 1e-9)) ok = 0;
            }
        }
        CHECK(ok, "a turned one finds its own axes");
        memset(&d, 0, sizeof d);
        d.m[0][0] = 2.0; d.m[1][1] = 1.0; d.m[0][1] = d.m[1][0] = 0.5;
        CHECK(rant_covariance3_ellipsoid(&d, 1.0, &t, &radii) == RANT_MATH_OK && radii.z == 0.0 &&
              near(fabs(rant_quaternion_rotate(t, rant_double3(0, 0, 1)).z), 1.0, 1e-12), "a flat one keeps its plane, its last radius 0");
        d.m[2][2] = -1.0;
        CHECK(rant_covariance3_ellipsoid(&d, 1.0, &t, &radii) == RANT_MATH_ERR_ARGUMENT, "a spread below 0 is refused");
        d.m[2][2] = NAN;
        CHECK(rant_covariance3_ellipsoid(&d, 1.0, &t, &radii) == RANT_MATH_ERR_ARGUMENT, "as is NaN");
        d.m[2][2] = 1.0;
        CHECK(rant_covariance3_ellipsoid(&d, -1.0, &t, &radii) == RANT_MATH_ERR_ARGUMENT, "and deviations below 0");
    }

    {   /* two unit point masses on x at -1 and +1 */
        RantInertia a, b, s;
        memset(&a, 0, sizeof a); memset(&b, 0, sizeof b);
        a.mass = 1.0; a.center = rant_double3(-1, 0, 0);
        b.mass = 1.0; b.center = rant_double3(1, 0, 0);
        s = rant_inertia_add(a, b);
        CHECK(s.mass == 2.0 && near3(s.center, rant_double3(0, 0, 0), 0.0) && s.ixx == 0.0 && s.iyy == 2.0
              && s.izz == 2.0 && s.ixy == 0.0, "two point masses add up");
    }
    {   RantInertia a, moved;
        RantPose p = rant_pose_identity();
        memset(&a, 0, sizeof a);
        a.mass = 3.0; a.center = rant_double3(1, 0, 0); a.ixx = 1.0; a.iyy = 2.0; a.izz = 3.0;
        p.position = rant_double3(0, 0, 5);
        p.orientation = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2);
        moved = rant_inertia_transform(a, p);
        CHECK(near3(moved.center, rant_double3(0, 1, 5), 1e-15) && near(moved.ixx, 2.0, 1e-15) && near(moved.iyy, 1.0, 1e-15)
              && near(moved.izz, 3.0, 1e-15), "a turned body swaps its x and y moments");
        memset(&moved, 0, sizeof moved);
        CHECK(rant_inertia_add(moved, moved).mass == 0.0, "two massless bodies stay massless");
    }
    {   /* boxes of 0.2, 0.4 and 0.6 placed at random come back as themselves */
        RantOrientedBox box, found;
        RantInertia body;
        for (i = 0, ok = 1; i < 20; i++){
            box.pose = random_pose(2.0);
            box.size = rant_double3(0.2, 0.4, 0.6);
            body = rant_oriented_box_inertia(box, 2.5);
            if (rant_inertia_box(body, &found) != RANT_MATH_OK || !near3(found.size, box.size, 1e-9) ||
                !near3(found.pose.position, box.pose.position, 1e-12)) ok = 0;
            else {
                RantInertia again = rant_oriented_box_inertia(found, 2.5);
                if (!near(again.ixx, body.ixx, 1e-12) || !near(again.ixy, body.ixy, 1e-12) || !near(again.ixz, body.ixz, 1e-12) ||
                    !near(again.iyy, body.iyy, 1e-12) || !near(again.iyz, body.iyz, 1e-12) || !near(again.izz, body.izz, 1e-12)) ok = 0;
            }
        }
        CHECK(ok, "the box of a box's inertia is that box");
        memset(&body, 0, sizeof body);
        body.mass = 1.0; body.iyy = body.izz = 1.0 / 12.0;
        CHECK(rant_inertia_box(body, &found) == RANT_MATH_OK && near3(found.size, rant_double3(0, 0, 1), 1e-12) &&
              near(fabs(rant_quaternion_rotate(found.pose.orientation, rant_double3(0, 0, 1)).x), 1.0, 1e-12),
              "a rod of length 1 along x is one long side, last");
        body.mass = 0.0;
        CHECK(rant_inertia_box(body, &found) == RANT_MATH_ERR_ARGUMENT, "a body with no mass is refused");
    }
}

void test_motion(void){
    velocities_and_forces();
    integration();
    covariance_and_inertia();
}
