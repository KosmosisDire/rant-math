#include "check.h"
#include "reference.h"

static double angle_gap(double a, double b){ return fabs(rant_angle_wrap(a - b)); }

static void euler_against_scipy(void){
    int o, i;
    for (o = 0; o < 24; o++){
        for (i = 0; i < REF_EULER_CASES; i++){
            const double *c = &REF_FROM_EULER[(o * REF_EULER_CASES + i) * 7];
            RantQuaternion want = rant_quaternion(c[3], c[4], c[5], c[6]);
            RantQuaternion got = rant_quaternion_from_euler(c[0], c[1], c[2], (RantEulerOrder)o);
            CHECK(same_turn(got, want, 1e-14), "from_euler order %d case %d matches scipy", o, i);
        }
        for (i = 0; i < REF_EULER_CASES; i++){
            const double *c = &REF_TO_EULER[(o * REF_EULER_CASES + i) * 7];
            double got[3];
            rant_quaternion_to_euler(rant_quaternion(c[0], c[1], c[2], c[3]), (RantEulerOrder)o, got);
            CHECK(angle_gap(got[0], c[4]) < 1e-12 && angle_gap(got[1], c[5]) < 1e-12 && angle_gap(got[2], c[6]) < 1e-12,
                  "to_euler order %d case %d: %g %g %g, scipy %g %g %g", o, i, got[0], got[1], got[2], c[4], c[5], c[6]);
        }
    }
}

static void euler_round_trips_and_locks(void){
    int o, i;
    for (o = 0; o < 24; o++){
        int proper = (o % 12) >= 6;
        for (i = 0; i < 100; i++){
            RantQuaternion q = random_quaternion();
            double a[3];
            rant_quaternion_to_euler(q, (RantEulerOrder)o, a);
            CHECK(same_turn(rant_quaternion_from_euler(a[0], a[1], a[2], (RantEulerOrder)o), q, 1e-13),
                  "euler order %d round trips", o);
            CHECK(a[0] > -PI && a[0] <= PI && a[2] > -PI && a[2] <= PI, "euler outer angles in (-pi, pi]");
            CHECK(proper ? (a[1] >= 0.0 && a[1] <= PI) : (a[1] >= -PI / 2 && a[1] <= PI / 2), "euler middle angle range");
        }
        /* the locked middle angles: +-90 degrees, or 0 and 180 for a repeated axis */
        for (i = 0; i < 2; i++){
            double mid = proper ? (i ? PI : 0.0) : (i ? -PI / 2 : PI / 2), a[3];
            RantQuaternion q = rant_quaternion_from_euler(0.7, mid, -0.4, (RantEulerOrder)o);
            rant_quaternion_to_euler(q, (RantEulerOrder)o, a);
            CHECK(same_turn(rant_quaternion_from_euler(a[0], a[1], a[2], (RantEulerOrder)o), q, 1e-12),
                  "euler order %d at the lock %g still gives the same turn", o, mid);
            CHECK(a[2] == 0.0, "euler order %d at the lock sets the third angle to 0 (%g)", o, a[2]);
        }
    }
    /* roll, pitch, yaw: fixed xyz is the moving zyx with the angles reversed */
    {   RantQuaternion a = rant_quaternion_from_euler(0.1, 0.2, 0.3, RANT_EULER_FIXED_XYZ);
        RantQuaternion b = rant_quaternion_from_euler(0.3, 0.2, 0.1, RANT_EULER_MOVING_ZYX);
        CHECK(same_turn(a, b, 1e-15), "fixed xyz is moving zyx reversed");
        CHECK(near(rant_quaternion_yaw(a), 0.3, 1e-15), "the yaw of roll pitch yaw is the yaw");
    }
}

static void conversions(void){
    int i, r, c;
    for (i = 0; i < REF_ROT_CASES; i++){
        const double *v = &REF_ROTVEC[i * 7], *m = &REF_MATRIX[i * 13];
        RantQuaternion q = rant_quaternion(v[0], v[1], v[2], v[3]);
        RantDouble3 got = rant_quaternion_to_rotation_vector(q);
        RantMatrix3x3 mat = rant_matrix3x3_from_quaternion(q);
        int ok = 1;
        CHECK(near3(got, rant_double3(v[4], v[5], v[6]), 1e-13), "rotation vector case %d matches scipy", i);
        CHECK(same_turn(rant_quaternion_from_rotation_vector(rant_double3(v[4], v[5], v[6])), q, 1e-15),
              "from rotation vector case %d", i);
        for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) if (!near(mat.m[r][c], m[4 + r * 3 + c], 1e-7)) ok = 0;
        CHECK(ok, "matrix case %d matches scipy", i);
        CHECK(same_turn(rant_quaternion_from_matrix3x3(mat), q, 1e-12), "matrix case %d converts back", i);
    }
    {   RantDouble3 tiny = rant_double3(1e-9, -2e-9, 0.5e-9), axis;
        RantDouble3 back = rant_quaternion_to_rotation_vector(rant_quaternion_from_rotation_vector(tiny));
        CHECK(near3(back, tiny, 1e-24), "a tiny rotation vector keeps its digits (%g)", back.x - tiny.x);
        CHECK(near(rant_quaternion_to_axis_angle(rant_quaternion_identity(), &axis), 0.0, 0.0)
              && near3(axis, rant_double3(1, 0, 0), 0.0), "the identity has angle 0 about +x");
        CHECK(near(rant_quaternion_to_axis_angle(rant_quaternion(0, 0, -1, 0), &axis), PI, 1e-15),
              "a half turn has angle pi");
        CHECK(same_turn(rant_quaternion_from_axis_angle(rant_double3(0, 0, 7), PI / 2),
                        rant_quaternion(0, 0, sqrt(0.5), sqrt(0.5)), 1e-15), "axis angle takes any axis length");
        CHECK(same_turn(rant_quaternion_from_axis_angle(rant_double3(0, 0, 0), 1.0), rant_quaternion_identity(), 0.0),
              "a zero axis is the identity");
        CHECK(near(rant_quaternion_to_axis_angle(rant_quaternion(0, 0, 1, -1), &axis), PI / 2, 1e-15)
              && near3(axis, rant_double3(0, 0, -1), 1e-15), "a quaternion with w below 0 gives the short angle");
    }
}

static void composition(void){
    int i;
    for (i = 0; i < 200; i++){
        RantQuaternion a = random_quaternion(), b = random_quaternion(), c = random_quaternion();
        RantDouble3 v = random_double3(5.0);
        RantQuaternion ab = rant_quaternion_compose(a, b);
        CHECK(near3(rant_quaternion_rotate(ab, v), rant_quaternion_rotate(a, rant_quaternion_rotate(b, v)), 1e-12),
              "compose(a, b) turns by b, then by a: the frame chain");
        CHECK(same_turn(rant_quaternion_compose(ab, c), rant_quaternion_compose(a, rant_quaternion_compose(b, c)), 1e-14),
              "compose is associative");
        CHECK(same_turn(rant_quaternion_compose(a, rant_quaternion_inverse(a)), rant_quaternion_identity(), 1e-14),
              "a quaternion and its inverse cancel");
        CHECK(near(rant_double3_length(rant_quaternion_rotate(a, v)), rant_double3_length(v), 1e-12), "rotate keeps length");
        CHECK(near3(rant_quaternion_rotate(rant_quaternion(a.x * 3, a.y * 3, a.z * 3, a.w * 3), v),
                    rant_quaternion_rotate(a, v), 1e-12), "rotate treats a scaled quaternion as its unit");
        CHECK(near(rant_quaternion_angle(a, rant_quaternion_compose(a, rant_quaternion_from_axis_angle(v, 0.3))), 0.3, 1e-12),
              "angle measures the turn between");
    }
    CHECK(same_turn(rant_quaternion_inverse(rant_quaternion(0, 0, 0, 0)), rant_quaternion_identity(), 0.0),
          "the inverse of zero is the identity");
    CHECK(same_turn(rant_quaternion_normalize(rant_quaternion(0, 0, 0, 0)), rant_quaternion_identity(), 0.0),
          "normalizing zero gives the identity");
}

static void between_and_look(void){
    int i;
    for (i = 0; i < 300; i++){
        RantDouble3 a = random_double3(3.0), b = random_double3(3.0);
        RantQuaternion q;
        if (i == 0) b = rant_double3_scale(a, -2.0);
        if (i == 1) b = rant_double3_scale(a, 4.0);
        if (i == 2){ a = rant_double3(0, 0, 1); b = rant_double3(0, 0, -1); }
        q = rant_quaternion_between(a, b);
        CHECK(near3(rant_quaternion_rotate(q, rant_double3_normalize(a)), rant_double3_normalize(b), 1e-12),
              "between turns from onto to (case %d)", i);
        if (i > 2) CHECK(near(rant_quaternion_angle(rant_quaternion_identity(), q), rant_double3_angle(a, b), 1e-12),
                         "between is the shortest turn");
    }
    for (i = 0; i < 300; i++){
        RantDouble3 f = random_double3(2.0), up = random_double3(2.0), x, z;
        RantQuaternion q;
        if (i == 0) up = f;
        q = rant_quaternion_look(f, up);
        x = rant_quaternion_rotate(q, rant_double3(1, 0, 0));
        z = rant_quaternion_rotate(q, rant_double3(0, 0, 1));
        CHECK(near3(x, rant_double3_normalize(f), 1e-12), "look points +x along forward");
        if (i > 0){
            RantDouble3 want = rant_double3_normalize(rant_double3_subtract(up, rant_double3_project(up, f)));
            CHECK(near3(z, want, 1e-10), "look points +z as near up as it can");
        }
    }
    CHECK(same_turn(rant_quaternion_look(rant_double3(0, 0, 0), rant_double3(0, 0, 1)), rant_quaternion_identity(), 0.0),
          "look along nothing is the identity");
}

static void slerp_and_motion(void){
    int i;
    for (i = 0; i < REF_SLERP_CASES; i++){
        const double *c = &REF_SLERP[i * 13];
        RantQuaternion a = rant_quaternion(c[0], c[1], c[2], c[3]), b = rant_quaternion(c[4], c[5], c[6], c[7]);
        CHECK(same_turn(rant_quaternion_slerp(a, b, c[8]), rant_quaternion(c[9], c[10], c[11], c[12]), 1e-14),
              "slerp case %d matches scipy", i);
        CHECK(same_turn(rant_quaternion_slerp(a, b, 0.0), a, 1e-15) && same_turn(rant_quaternion_slerp(a, b, 1.0), b, 1e-15),
              "slerp ends at its ends");
    }
    {   RantQuaternion a = rant_quaternion_identity(), b = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), 1e-9);
        RantQuaternion m = rant_quaternion_slerp(a, b, 0.5);
        CHECK(near(rant_quaternion_angle(a, m), 0.5e-9, 1e-18), "slerp of nearly equal turns");
        b = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), 3.0);
        CHECK(near(rant_quaternion_yaw(rant_quaternion_slerp(a, rant_quaternion(-b.x, -b.y, -b.z, -b.w), 0.5)), 1.5, 1e-12),
              "slerp takes the short way whatever sign b has");
    }
    {   /* turning about the body's own x after a quarter turn about z turns about the world's y */
        RantQuaternion q = rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), PI / 2);
        RantQuaternion r = rant_quaternion_integrate(q, rant_double3(1.0, 0, 0), PI / 2);
        RantDouble3 z = rant_quaternion_rotate(r, rant_double3(0, 0, 1));
        CHECK(near3(z, rant_double3(1, 0, 0), 1e-12), "integrate turns about the body's own axes (%g %g %g)", z.x, z.y, z.z);
    }
    CHECK(near(rant_angle_wrap(PI), PI, 0.0) && near(rant_angle_wrap(-PI), PI, 1e-15), "wrap keeps pi and moves -pi to pi");
    CHECK(near(rant_angle_wrap(3 * PI), PI, 1e-14) && near(rant_angle_wrap(-3 * PI), PI, 1e-14), "wrap of odd half turns");
    CHECK(near(rant_angle_wrap(7.0), 7.0 - 2 * PI, 1e-15) && near(rant_angle_wrap(-0.25), -0.25, 0.0), "wrap");
    CHECK(near(rant_angle_difference(3.0, -3.0), 2 * PI - 6.0, 1e-15), "difference takes the short way across pi");
}

void test_rotation(void){
    euler_against_scipy();
    euler_round_trips_and_locks();
    conversions();
    composition();
    between_and_look();
    slerp_and_motion();
}
