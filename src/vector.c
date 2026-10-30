#include "internal.h"

RantDouble2 rant_double2_add(RantDouble2 a, RantDouble2 b){ return rant_double2(a.x + b.x, a.y + b.y); }
RantDouble2 rant_double2_subtract(RantDouble2 a, RantDouble2 b){ return rant_double2(a.x - b.x, a.y - b.y); }
RantDouble2 rant_double2_scale(RantDouble2 a, double k){ return rant_double2(a.x * k, a.y * k); }
RantDouble2 rant_double2_multiply(RantDouble2 a, RantDouble2 b){ return rant_double2(a.x * b.x, a.y * b.y); }
double rant_double2_dot(RantDouble2 a, RantDouble2 b){ return a.x * b.x + a.y * b.y; }
double rant_double2_cross(RantDouble2 a, RantDouble2 b){ return a.x * b.y - a.y * b.x; }
double rant_double2_length(RantDouble2 a){ return sqrt(a.x * a.x + a.y * a.y); }
double rant_double2_length_squared(RantDouble2 a){ return a.x * a.x + a.y * a.y; }
double rant_double2_distance(RantDouble2 a, RantDouble2 b){ return rant_double2_length(rant_double2_subtract(a, b)); }

RantDouble2 rant_double2_normalize(RantDouble2 a){
    double n = rant_double2_length(a);
    return n > 0.0 ? rant_double2_scale(a, 1.0 / n) : a;
}

RantDouble2 rant_double2_lerp(RantDouble2 a, RantDouble2 b, double t){
    return rant_double2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}
RantDouble2 rant_double2_min(RantDouble2 a, RantDouble2 b){ return rant_double2(i_math_min(a.x, b.x), i_math_min(a.y, b.y)); }
RantDouble2 rant_double2_max(RantDouble2 a, RantDouble2 b){ return rant_double2(i_math_max(a.x, b.x), i_math_max(a.y, b.y)); }

double rant_double2_angle(RantDouble2 a, RantDouble2 b){
    return atan2(rant_double2_cross(a, b), rant_double2_dot(a, b));
}
double rant_double2_heading(RantDouble2 a){ return atan2(a.y, a.x); }
RantDouble2 rant_double2_from_heading(double angle){ return rant_double2(cos(angle), sin(angle)); }

RantDouble2 rant_double2_rotate(RantDouble2 a, double angle){
    double c = cos(angle), s = sin(angle);
    return rant_double2(c * a.x - s * a.y, s * a.x + c * a.y);
}
RantDouble2 rant_double2_perpendicular(RantDouble2 a){ return rant_double2(-a.y, a.x); }

RantDouble2 rant_double2_project(RantDouble2 a, RantDouble2 onto){
    double d = rant_double2_dot(onto, onto);
    return d > 0.0 ? rant_double2_scale(onto, rant_double2_dot(a, onto) / d) : rant_double2(0.0, 0.0);
}

RantDouble2 rant_double2_reflect(RantDouble2 a, RantDouble2 normal){
    RantDouble2 n = rant_double2_normalize(normal);
    return rant_double2_subtract(a, rant_double2_scale(n, 2.0 * rant_double2_dot(a, n)));
}

RantDouble3 rant_double3_add(RantDouble3 a, RantDouble3 b){ return rant_double3(a.x + b.x, a.y + b.y, a.z + b.z); }
RantDouble3 rant_double3_subtract(RantDouble3 a, RantDouble3 b){ return rant_double3(a.x - b.x, a.y - b.y, a.z - b.z); }
RantDouble3 rant_double3_scale(RantDouble3 a, double k){ return rant_double3(a.x * k, a.y * k, a.z * k); }
RantDouble3 rant_double3_multiply(RantDouble3 a, RantDouble3 b){ return rant_double3(a.x * b.x, a.y * b.y, a.z * b.z); }
double rant_double3_dot(RantDouble3 a, RantDouble3 b){ return a.x * b.x + a.y * b.y + a.z * b.z; }

RantDouble3 rant_double3_cross(RantDouble3 a, RantDouble3 b){
    return rant_double3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
double rant_double3_length(RantDouble3 a){ return sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
double rant_double3_length_squared(RantDouble3 a){ return a.x * a.x + a.y * a.y + a.z * a.z; }
double rant_double3_distance(RantDouble3 a, RantDouble3 b){ return rant_double3_length(rant_double3_subtract(a, b)); }

RantDouble3 rant_double3_normalize(RantDouble3 a){
    double n = rant_double3_length(a);
    return n > 0.0 ? rant_double3_scale(a, 1.0 / n) : a;
}

RantDouble3 rant_double3_lerp(RantDouble3 a, RantDouble3 b, double t){
    return rant_double3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}
RantDouble3 rant_double3_min(RantDouble3 a, RantDouble3 b){
    return rant_double3(i_math_min(a.x, b.x), i_math_min(a.y, b.y), i_math_min(a.z, b.z));
}
RantDouble3 rant_double3_max(RantDouble3 a, RantDouble3 b){
    return rant_double3(i_math_max(a.x, b.x), i_math_max(a.y, b.y), i_math_max(a.z, b.z));
}

/* atan2 of the cross and dot stays exact near 0 and pi, where acos of the dot loses half its digits. */
double rant_double3_angle(RantDouble3 a, RantDouble3 b){
    return atan2(rant_double3_length(rant_double3_cross(a, b)), rant_double3_dot(a, b));
}

RantDouble3 rant_double3_project(RantDouble3 a, RantDouble3 onto){
    double d = rant_double3_dot(onto, onto);
    return d > 0.0 ? rant_double3_scale(onto, rant_double3_dot(a, onto) / d) : rant_double3(0.0, 0.0, 0.0);
}

RantDouble3 rant_double3_reflect(RantDouble3 a, RantDouble3 normal){
    RantDouble3 n = rant_double3_normalize(normal);
    return rant_double3_subtract(a, rant_double3_scale(n, 2.0 * rant_double3_dot(a, n)));
}

/* Crossing with the axis a is least along keeps the result well away from zero. */
RantDouble3 rant_double3_perpendicular(RantDouble3 a){
    double ax = fabs(a.x), ay = fabs(a.y), az = fabs(a.z);
    RantDouble3 axis = ax <= ay && ax <= az ? rant_double3(1.0, 0.0, 0.0)
                     : (ay <= az ? rant_double3(0.0, 1.0, 0.0) : rant_double3(0.0, 0.0, 1.0));
    RantDouble3 p = rant_double3_normalize(rant_double3_cross(a, axis));
    return rant_double3_length_squared(p) > 0.0 ? p : rant_double3(1.0, 0.0, 0.0);
}

RantDouble4 rant_double4_add(RantDouble4 a, RantDouble4 b){ return rant_double4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w); }
RantDouble4 rant_double4_subtract(RantDouble4 a, RantDouble4 b){ return rant_double4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w); }
RantDouble4 rant_double4_scale(RantDouble4 a, double k){ return rant_double4(a.x * k, a.y * k, a.z * k, a.w * k); }
double rant_double4_dot(RantDouble4 a, RantDouble4 b){ return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
double rant_double4_length(RantDouble4 a){ return sqrt(rant_double4_dot(a, a)); }

RantDouble4 rant_double4_normalize(RantDouble4 a){
    double n = rant_double4_length(a);
    return n > 0.0 ? rant_double4_scale(a, 1.0 / n) : a;
}

RantDouble4 rant_double4_lerp(RantDouble4 a, RantDouble4 b, double t){
    return rant_double4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                        a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

RantDouble2 rant_double2_from_float2(RantFloat2 a){ return rant_double2(a.x, a.y); }
RantDouble3 rant_double3_from_float3(RantFloat3 a){ return rant_double3(a.x, a.y, a.z); }
RantDouble4 rant_double4_from_float4(RantFloat4 a){ return rant_double4(a.x, a.y, a.z, a.w); }
RantFloat2 rant_float2_from_double2(RantDouble2 a){ return rant_float2((float)a.x, (float)a.y); }
RantFloat3 rant_float3_from_double3(RantDouble3 a){ return rant_float3((float)a.x, (float)a.y, (float)a.z); }
RantFloat4 rant_float4_from_double4(RantDouble4 a){ return rant_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w); }
RantDouble2 rant_double2_from_int2(RantInt2 a){ return rant_double2(a.x, a.y); }
RantDouble3 rant_double3_from_int3(RantInt3 a){ return rant_double3(a.x, a.y, a.z); }
RantInt2 rant_int2_floor(RantDouble2 a){ return rant_int2((int32_t)floor(a.x), (int32_t)floor(a.y)); }
RantInt3 rant_int3_floor(RantDouble3 a){ return rant_int3((int32_t)floor(a.x), (int32_t)floor(a.y), (int32_t)floor(a.z)); }
