#include "check.h"

void test_vector(void){
    RantDouble2 a2 = rant_double2(3.0, 4.0), b2 = rant_double2(-1.0, 2.0);
    RantDouble3 a3 = rant_double3(1.0, 2.0, 3.0), b3 = rant_double3(-4.0, 0.5, 2.0);
    int i;

    CHECK(near2(rant_double2_add(a2, b2), rant_double2(2.0, 6.0), 0.0), "double2 add");
    CHECK(near2(rant_double2_subtract(a2, b2), rant_double2(4.0, 2.0), 0.0), "double2 subtract");
    CHECK(near2(rant_double2_scale(a2, -2.0), rant_double2(-6.0, -8.0), 0.0), "double2 scale");
    CHECK(near2(rant_double2_multiply(a2, b2), rant_double2(-3.0, 8.0), 0.0), "double2 multiply");
    CHECK(rant_double2_dot(a2, b2) == 5.0, "double2 dot");
    CHECK(rant_double2_cross(rant_double2(1, 0), rant_double2(0, 1)) == 1.0, "double2 cross is +1 for x then y");
    CHECK(rant_double2_length(a2) == 5.0 && rant_double2_length_squared(a2) == 25.0, "double2 length");
    CHECK(near(rant_double2_distance(a2, b2), sqrt(20.0), 1e-15), "double2 distance");
    CHECK(near2(rant_double2_normalize(a2), rant_double2(0.6, 0.8), 1e-15), "double2 normalize");
    CHECK(near2(rant_double2_normalize(rant_double2(0, 0)), rant_double2(0, 0), 0.0), "double2 zero stays zero");
    CHECK(near2(rant_double2_lerp(a2, b2, 0.25), rant_double2(2.0, 3.5), 1e-15), "double2 lerp");
    CHECK(near2(rant_double2_min(a2, b2), rant_double2(-1, 2), 0.0) && near2(rant_double2_max(a2, b2), rant_double2(3, 4), 0.0),
          "double2 min and max");
    CHECK(near(rant_double2_angle(rant_double2(1, 0), rant_double2(0, 1)), PI / 2, 1e-15), "double2 angle counterclockwise positive");
    CHECK(near(rant_double2_angle(rant_double2(0, 1), rant_double2(1, 0)), -PI / 2, 1e-15), "double2 angle clockwise negative");
    CHECK(near(rant_double2_angle(rant_double2(1, 0), rant_double2(-1, 0)), PI, 1e-15), "double2 angle half turn is pi");
    CHECK(near(rant_double2_heading(rant_double2(0, -2)), -PI / 2, 1e-15), "double2 heading");
    CHECK(near2(rant_double2_from_heading(PI / 2), rant_double2(0, 1), 1e-15), "double2 from heading");
    CHECK(near2(rant_double2_rotate(rant_double2(1, 0), PI / 2), rant_double2(0, 1), 1e-15), "double2 rotate counterclockwise");
    CHECK(near2(rant_double2_perpendicular(a2), rant_double2(-4, 3), 0.0), "double2 perpendicular is a quarter turn left");
    CHECK(near2(rant_double2_project(a2, rant_double2(2, 0)), rant_double2(3, 0), 1e-15), "double2 project");
    CHECK(near2(rant_double2_project(a2, rant_double2(0, 0)), rant_double2(0, 0), 0.0), "double2 project on zero");
    CHECK(near2(rant_double2_reflect(rant_double2(1, -1), rant_double2(0, 5)), rant_double2(1, 1), 1e-15),
          "double2 reflect takes any normal length");

    CHECK(near3(rant_double3_add(a3, b3), rant_double3(-3, 2.5, 5), 0.0), "double3 add");
    CHECK(near3(rant_double3_subtract(a3, b3), rant_double3(5, 1.5, 1), 0.0), "double3 subtract");
    CHECK(near3(rant_double3_multiply(a3, b3), rant_double3(-4, 1, 6), 0.0), "double3 multiply");
    CHECK(rant_double3_dot(a3, b3) == 3.0, "double3 dot");
    CHECK(near3(rant_double3_cross(rant_double3(1, 0, 0), rant_double3(0, 1, 0)), rant_double3(0, 0, 1), 0.0),
          "double3 cross is right handed");
    CHECK(near(rant_double3_length(rant_double3(2, 3, 6)), 7.0, 1e-15), "double3 length");
    CHECK(near3(rant_double3_normalize(rant_double3(0, 0, 0)), rant_double3(0, 0, 0), 0.0), "double3 zero stays zero");
    CHECK(near(rant_double3_angle(rant_double3(1, 0, 0), rant_double3(0, 0, 3)), PI / 2, 1e-15), "double3 angle");
    CHECK(near(rant_double3_angle(rant_double3(1, 0, 0), rant_double3(1, 1e-9, 0)), 1e-9, 1e-20),
          "double3 angle keeps its digits near zero");
    CHECK(near3(rant_double3_reflect(rant_double3(1, 2, -3), rant_double3(0, 0, 2)), rant_double3(1, 2, 3), 1e-15),
          "double3 reflect");
    for (i = 0; i < 200; i++){
        RantDouble3 v = random_double3(10.0), p;
        if (i == 0) v = rant_double3(0, 0, 5);
        if (i == 1) v = rant_double3(1e-200, 0, 0);
        p = rant_double3_perpendicular(v);
        CHECK(near(rant_double3_length(p), 1.0, 1e-14) && fabs(rant_double3_dot(p, rant_double3_normalize(v))) < 1e-14,
              "double3 perpendicular is a unit at right angles (%g %g %g)", v.x, v.y, v.z);
        CHECK(near3(rant_double3_add(rant_double3_project(v, a3),
                                     rant_double3_subtract(v, rant_double3_project(v, a3))), v, 1e-12),
              "double3 project and its rest sum back");
        CHECK(fabs(rant_double3_dot(rant_double3_subtract(v, rant_double3_project(v, a3)), a3)) < 1e-11,
              "double3 the rest of a projection is at right angles");
    }
    CHECK(near3(rant_double3_perpendicular(rant_double3(0, 0, 0)), rant_double3(1, 0, 0), 0.0), "double3 perpendicular of zero");

    CHECK(near(rant_double4_length(rant_double4(1, 1, 1, 1)), 2.0, 1e-15), "double4 length");
    CHECK(rant_double4_dot(rant_double4(1, 2, 3, 4), rant_double4(4, 3, 2, 1)) == 20.0, "double4 dot");
    CHECK(near(rant_double4_normalize(rant_double4(0, 0, 0, 2)).w, 1.0, 0.0), "double4 normalize");
    CHECK(near(rant_double4_lerp(rant_double4(0, 0, 0, 0), rant_double4(4, 4, 4, 4), 0.5).z, 2.0, 0.0), "double4 lerp");

    {   RantFloat3 f = rant_float3_from_double3(rant_double3(0.1, -2.5, 1e30));
        RantDouble3 d = rant_double3_from_float3(f);
        CHECK(f.x == 0.1f && f.y == -2.5f && d.y == -2.5, "float3 to and from double3");
        CHECK(rant_double2_from_int2(rant_int2(-3, 7)).x == -3.0, "int2 to double2");
        CHECK(rant_int2_floor(rant_double2(-0.5, 2.999)).x == -1 && rant_int2_floor(rant_double2(-0.5, 2.999)).y == 2,
              "int2 floor rounds down, negatives too");
        CHECK(rant_int3_floor(rant_double3(-1.0, 0.0, 1.5)).x == -1, "int3 floor of a whole number");
    }
}
