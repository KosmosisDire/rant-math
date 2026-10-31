/* Vectors: Double2, Double3 and Double4, and moving Float and Int vectors to and from them.
 * The library computes in double. */
#ifndef RANT_MATH_VECTOR_H
#define RANT_MATH_VECTOR_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

RANT_MATH_API RantDouble2 rant_double2_add(RantDouble2 a, RantDouble2 b);
RANT_MATH_API RantDouble2 rant_double2_subtract(RantDouble2 a, RantDouble2 b);
RANT_MATH_API RantDouble2 rant_double2_scale(RantDouble2 a, double k);
RANT_MATH_API RantDouble2 rant_double2_multiply(RantDouble2 a, RantDouble2 b);   /* component by component */
RANT_MATH_API double      rant_double2_dot(RantDouble2 a, RantDouble2 b);
RANT_MATH_API double      rant_double2_cross(RantDouble2 a, RantDouble2 b);      /* the z of the 3D cross */
RANT_MATH_API double      rant_double2_length(RantDouble2 a);
RANT_MATH_API double      rant_double2_length_squared(RantDouble2 a);
RANT_MATH_API double      rant_double2_distance(RantDouble2 a, RantDouble2 b);
RANT_MATH_API RantDouble2 rant_double2_normalize(RantDouble2 a);                  /* zero stays zero */
RANT_MATH_API RantDouble2 rant_double2_lerp(RantDouble2 a, RantDouble2 b, double t);
RANT_MATH_API RantDouble2 rant_double2_min(RantDouble2 a, RantDouble2 b);
RANT_MATH_API RantDouble2 rant_double2_max(RantDouble2 a, RantDouble2 b);
/* The signed turn from a to b in (-pi, pi], counterclockwise positive. */
RANT_MATH_API double      rant_double2_angle(RantDouble2 a, RantDouble2 b);
/* The angle of a from +x toward +y, and the unit vector at an angle. */
RANT_MATH_API double      rant_double2_heading(RantDouble2 a);
RANT_MATH_API RantDouble2 rant_double2_from_heading(double angle);
RANT_MATH_API RantDouble2 rant_double2_rotate(RantDouble2 a, double angle);
RANT_MATH_API RantDouble2 rant_double2_perpendicular(RantDouble2 a);              /* a quarter turn counterclockwise */
RANT_MATH_API RantDouble2 rant_double2_project(RantDouble2 a, RantDouble2 onto);
RANT_MATH_API RantDouble2 rant_double2_reflect(RantDouble2 a, RantDouble2 normal);

RANT_MATH_API RantDouble3 rant_double3_add(RantDouble3 a, RantDouble3 b);
RANT_MATH_API RantDouble3 rant_double3_subtract(RantDouble3 a, RantDouble3 b);
RANT_MATH_API RantDouble3 rant_double3_scale(RantDouble3 a, double k);
RANT_MATH_API RantDouble3 rant_double3_multiply(RantDouble3 a, RantDouble3 b);   /* component by component */
RANT_MATH_API double      rant_double3_dot(RantDouble3 a, RantDouble3 b);
RANT_MATH_API RantDouble3 rant_double3_cross(RantDouble3 a, RantDouble3 b);
RANT_MATH_API double      rant_double3_length(RantDouble3 a);
RANT_MATH_API double      rant_double3_length_squared(RantDouble3 a);
RANT_MATH_API double      rant_double3_distance(RantDouble3 a, RantDouble3 b);
RANT_MATH_API RantDouble3 rant_double3_normalize(RantDouble3 a);                  /* zero stays zero */
RANT_MATH_API RantDouble3 rant_double3_lerp(RantDouble3 a, RantDouble3 b, double t);
RANT_MATH_API RantDouble3 rant_double3_min(RantDouble3 a, RantDouble3 b);
RANT_MATH_API RantDouble3 rant_double3_max(RantDouble3 a, RantDouble3 b);
RANT_MATH_API double      rant_double3_angle(RantDouble3 a, RantDouble3 b);       /* in [0, pi] */
RANT_MATH_API RantDouble3 rant_double3_project(RantDouble3 a, RantDouble3 onto);
RANT_MATH_API RantDouble3 rant_double3_reflect(RantDouble3 a, RantDouble3 normal);
/* A unit vector at right angles to a, the same one every time for the same a. */
RANT_MATH_API RantDouble3 rant_double3_perpendicular(RantDouble3 a);

RANT_MATH_API RantDouble4 rant_double4_add(RantDouble4 a, RantDouble4 b);
RANT_MATH_API RantDouble4 rant_double4_subtract(RantDouble4 a, RantDouble4 b);
RANT_MATH_API RantDouble4 rant_double4_scale(RantDouble4 a, double k);
RANT_MATH_API double      rant_double4_dot(RantDouble4 a, RantDouble4 b);
RANT_MATH_API double      rant_double4_length(RantDouble4 a);
RANT_MATH_API RantDouble4 rant_double4_normalize(RantDouble4 a);
RANT_MATH_API RantDouble4 rant_double4_lerp(RantDouble4 a, RantDouble4 b, double t);

RANT_MATH_API RantDouble2 rant_double2_from_float2(RantFloat2 a);
RANT_MATH_API RantDouble3 rant_double3_from_float3(RantFloat3 a);
RANT_MATH_API RantDouble4 rant_double4_from_float4(RantFloat4 a);
RANT_MATH_API RantFloat2  rant_float2_from_double2(RantDouble2 a);
RANT_MATH_API RantFloat3  rant_float3_from_double3(RantDouble3 a);
RANT_MATH_API RantFloat4  rant_float4_from_double4(RantDouble4 a);
RANT_MATH_API RantDouble2 rant_double2_from_int2(RantInt2 a);
RANT_MATH_API RantDouble3 rant_double3_from_int3(RantInt3 a);
/* The cell holding a point: each component rounded down. */
RANT_MATH_API RantInt2    rant_int2_floor(RantDouble2 a);
RANT_MATH_API RantInt3    rant_int3_floor(RantDouble3 a);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_VECTOR_H */
