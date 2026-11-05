#include "internal.h"

static const RantAxes i_presets[] = {
    { -2,  3,  1,   1.0 },   /* UNITY */
    {  1, -2,  3, 100.0 },   /* UNREAL */
    { -2,  3, -1,   1.0 },   /* GODOT */
    { -2,  3, -1,   1.0 },   /* BEVY */
    {  2,  3,  1,   1.0 },   /* GLTF */
    {  2, -1,  3,   1.0 },   /* BLENDER */
    { -2, -3,  1,   1.0 },   /* OPENCV_CAMERA */
    { -2,  3, -1,   1.0 },   /* OPENGL_CAMERA */
    {  2,  1, -3,   1.0 },   /* NED */
    {  1, -2, -3,   1.0 }    /* FRD */
};

RantAxes rant_axes_of(RantAxesPreset preset){
    int p = (int)preset;
    RantAxes identity = { 1, 2, 3, 1.0 };
    return p >= 0 && p < (int)(sizeof i_presets / sizeof i_presets[0]) ? i_presets[p] : identity;
}

RantMathResult rant_axes_make(int x, int y, int z, double scale, RantAxes *out){
    int ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, az = z < 0 ? -z : z;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (ax < 1 || ax > 3 || ay < 1 || ay > 3 || az < 1 || az > 3) return RANT_MATH_ERR_ARGUMENT;
    if (ax == ay || ax == az || ay == az || !(scale > 0.0)) return RANT_MATH_ERR_ARGUMENT;
    out->x = (int8_t)x; out->y = (int8_t)y; out->z = (int8_t)z; out->scale = scale;
    return RANT_MATH_OK;
}

/* The signed permutation matrix M with tool = M rant, and its determinant, -1 for a tool
 * of the other handedness. A component naming no axis leaves its row zero. */
typedef struct { double m[3][3]; double det; } i_Axes;

static i_Axes i_axes(RantAxes a){
    i_Axes r;
    int code[3], i;
    code[0] = a.x; code[1] = a.y; code[2] = a.z;
    memset(&r, 0, sizeof r);
    for (i = 0; i < 3; i++){
        int c = code[i] < 0 ? -code[i] : code[i];
        if (c >= 1 && c <= 3) r.m[i][c - 1] = code[i] < 0 ? -1.0 : 1.0;
    }
    r.det = r.m[0][0] * (r.m[1][1] * r.m[2][2] - r.m[1][2] * r.m[2][1])
          - r.m[0][1] * (r.m[1][0] * r.m[2][2] - r.m[1][2] * r.m[2][0])
          + r.m[0][2] * (r.m[1][0] * r.m[2][1] - r.m[1][1] * r.m[2][0]);
    return r;
}

static RantDouble3 i_to(const i_Axes *a, RantDouble3 v, double k){
    return rant_double3(k * (a->m[0][0] * v.x + a->m[0][1] * v.y + a->m[0][2] * v.z),
                        k * (a->m[1][0] * v.x + a->m[1][1] * v.y + a->m[1][2] * v.z),
                        k * (a->m[2][0] * v.x + a->m[2][1] * v.y + a->m[2][2] * v.z));
}

static RantDouble3 i_from(const i_Axes *a, RantDouble3 v, double k){
    return rant_double3(k * (a->m[0][0] * v.x + a->m[1][0] * v.y + a->m[2][0] * v.z),
                        k * (a->m[0][1] * v.x + a->m[1][1] * v.y + a->m[2][1] * v.z),
                        k * (a->m[0][2] * v.x + a->m[1][2] * v.y + a->m[2][2] * v.z));
}

static double i_inverse_scale(RantAxes a){ return a.scale > 0.0 ? 1.0 / a.scale : 0.0; }

RantDouble3 rant_axes_point_to(RantAxes axes, RantDouble3 point){
    i_Axes a = i_axes(axes);
    return i_to(&a, point, axes.scale);
}

RantDouble3 rant_axes_point_from(RantAxes axes, RantDouble3 point){
    i_Axes a = i_axes(axes);
    return i_from(&a, point, i_inverse_scale(axes));
}

RantDouble3 rant_axes_direction_to(RantAxes axes, RantDouble3 direction){
    i_Axes a = i_axes(axes);
    return i_to(&a, direction, 1.0);
}

RantDouble3 rant_axes_direction_from(RantAxes axes, RantDouble3 direction){
    i_Axes a = i_axes(axes);
    return i_from(&a, direction, 1.0);
}

/* A quaternion's vector part is a turn axis, so a change of handedness flips it. */
RantQuaternion rant_axes_quaternion_to(RantAxes axes, RantQuaternion q){
    i_Axes a = i_axes(axes);
    RantDouble3 u = i_to(&a, rant_double3(q.x, q.y, q.z), a.det);
    return rant_quaternion(u.x, u.y, u.z, q.w);
}

RantQuaternion rant_axes_quaternion_from(RantAxes axes, RantQuaternion q){
    i_Axes a = i_axes(axes);
    RantDouble3 u = i_from(&a, rant_double3(q.x, q.y, q.z), a.det);
    return rant_quaternion(u.x, u.y, u.z, q.w);
}

RantPose rant_axes_pose_to(RantAxes axes, RantPose p){
    RantPose r;
    r.position = rant_axes_point_to(axes, p.position);
    r.orientation = rant_axes_quaternion_to(axes, p.orientation);
    return r;
}

RantPose rant_axes_pose_from(RantAxes axes, RantPose p){
    RantPose r;
    r.position = rant_axes_point_from(axes, p.position);
    r.orientation = rant_axes_quaternion_from(axes, p.orientation);
    return r;
}

RantTwist rant_axes_twist_to(RantAxes axes, RantTwist t){
    i_Axes a = i_axes(axes);
    RantTwist r;
    r.linear = i_to(&a, t.linear, axes.scale);
    r.angular = i_to(&a, t.angular, a.det);
    return r;
}

RantTwist rant_axes_twist_from(RantAxes axes, RantTwist t){
    i_Axes a = i_axes(axes);
    RantTwist r;
    r.linear = i_from(&a, t.linear, i_inverse_scale(axes));
    r.angular = i_from(&a, t.angular, a.det);
    return r;
}

RantWrench rant_axes_wrench_to(RantAxes axes, RantWrench w){
    i_Axes a = i_axes(axes);
    RantWrench r;
    r.force = i_to(&a, w.force, axes.scale);
    r.torque = i_to(&a, w.torque, a.det * axes.scale * axes.scale);
    return r;
}

RantWrench rant_axes_wrench_from(RantAxes axes, RantWrench w){
    i_Axes a = i_axes(axes);
    double k = i_inverse_scale(axes);
    RantWrench r;
    r.force = i_from(&a, w.force, k);
    r.torque = i_from(&a, w.torque, a.det * k * k);
    return r;
}

RantAccel rant_axes_accel_to(RantAxes axes, RantAccel acc){
    i_Axes a = i_axes(axes);
    RantAccel r;
    r.linear = i_to(&a, acc.linear, axes.scale);
    r.angular = i_to(&a, acc.angular, a.det);
    return r;
}

RantAccel rant_axes_accel_from(RantAxes axes, RantAccel acc){
    i_Axes a = i_axes(axes);
    RantAccel r;
    r.linear = i_from(&a, acc.linear, i_inverse_scale(axes));
    r.angular = i_from(&a, acc.angular, a.det);
    return r;
}

/* C' = J C J^T with J = blockdiag(k M, M). The sign a turn takes cancels in the product. */
static void i_covariance6(const i_Axes *a, int inverse, double k, const RantCovariance6 *in, RantCovariance6 *out){
    double j[6][6], t[6][6], o[6][6];
    int r, c, i;
    memset(j, 0, sizeof j);
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++){
            double m = inverse ? a->m[c][r] : a->m[r][c];
            j[r][c] = k * m;
            j[r + 3][c + 3] = m;
        }
    for (r = 0; r < 6; r++)
        for (c = 0; c < 6; c++){
            double s = 0.0;
            for (i = 0; i < 6; i++) s += j[r][i] * in->m[i][c];
            t[r][c] = s;
        }
    for (r = 0; r < 6; r++)
        for (c = 0; c < 6; c++){
            double s = 0.0;
            for (i = 0; i < 6; i++) s += t[r][i] * j[c][i];
            o[r][c] = s;
        }
    memcpy(out->m, o, sizeof o);
}

void rant_axes_covariance6_to(RantAxes axes, const RantCovariance6 *in, RantCovariance6 *out){
    i_Axes a = i_axes(axes);
    if (in && out) i_covariance6(&a, 0, axes.scale, in, out);
}

void rant_axes_covariance6_from(RantAxes axes, const RantCovariance6 *in, RantCovariance6 *out){
    i_Axes a = i_axes(axes);
    if (in && out) i_covariance6(&a, 1, i_inverse_scale(axes), in, out);
}

void rant_axes_points_to(RantAxes axes, const RantDouble3 *in, RantDouble3 *out, uint32_t count){
    i_Axes a = i_axes(axes);
    uint32_t i;
    if (!in || !out) return;
    for (i = 0; i < count; i++) out[i] = i_to(&a, in[i], axes.scale);
}

void rant_axes_points_from(RantAxes axes, const RantDouble3 *in, RantDouble3 *out, uint32_t count){
    i_Axes a = i_axes(axes);
    double k = i_inverse_scale(axes);
    uint32_t i;
    if (!in || !out) return;
    for (i = 0; i < count; i++) out[i] = i_from(&a, in[i], k);
}
