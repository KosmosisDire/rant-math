#include "internal.h"

RantPose rant_pose_compose(RantPose a, RantPose b){
    RantPose r;
    r.position = rant_double3_add(a.position, rant_quaternion_rotate(a.orientation, b.position));
    r.orientation = rant_quaternion_normalize(rant_quaternion_compose(a.orientation, b.orientation));
    return r;
}

RantPose rant_pose_inverse(RantPose p){
    RantPose r;
    r.orientation = rant_quaternion_inverse(rant_quaternion_normalize(p.orientation));
    r.position = rant_double3_scale(rant_quaternion_rotate(r.orientation, p.position), -1.0);
    return r;
}

RantPose rant_pose_between(RantPose a, RantPose b){
    return rant_pose_compose(rant_pose_inverse(a), b);
}

RantDouble3 rant_pose_apply(RantPose p, RantDouble3 point){
    return rant_double3_add(p.position, rant_quaternion_rotate(p.orientation, point));
}

RantDouble3 rant_pose_apply_inverse(RantPose p, RantDouble3 point){
    return rant_quaternion_rotate(rant_quaternion_inverse(p.orientation), rant_double3_subtract(point, p.position));
}

/* One matrix for every point: the quaternion form costs twice the multiplies per point. */
void rant_pose_apply_points(RantPose p, const RantDouble3 *in, RantDouble3 *out, uint32_t count){
    i_MathMat3 r;
    uint32_t i;
    if (!in || !out) return;
    r = i_math_mat3_from_quaternion(p.orientation);
    for (i = 0; i < count; i++)
        out[i] = rant_double3_add(p.position, i_math_mat3_apply(&r, in[i]));
}

RantPose rant_pose_interpolate(RantPose a, RantPose b, double t){
    RantPose r;
    r.position = rant_double3_lerp(a.position, b.position, t);
    r.orientation = rant_quaternion_slerp(a.orientation, b.orientation, t);
    return r;
}

RantMathResult rant_pose_from_points(RantDouble3 origin, RantDouble3 on_x, RantDouble3 in_xy, RantPose *out){
    RantDouble3 x = rant_double3_subtract(on_x, origin), toward = rant_double3_subtract(in_xy, origin), z;
    double lx = rant_double3_length(x), lz;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    z = rant_double3_cross(x, toward);
    lz = rant_double3_length(z);
    if (!(lx > 0.0) || !(lz > I_MATH_TINY * lx * rant_double3_length(toward))) return RANT_MATH_ERR_DEGENERATE;
    out->position = origin;
    out->orientation = rant_quaternion_look(x, z);
    return RANT_MATH_OK;
}

RantMathResult rant_pose_look_at(RantDouble3 eye, RantDouble3 target, RantDouble3 up, RantPose *out){
    RantDouble3 f = rant_double3_subtract(target, eye);
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if (!(rant_double3_length(f) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    out->position = eye;
    out->orientation = rant_quaternion_look(f, up);
    return RANT_MATH_OK;
}

RantMatrix4x4 rant_matrix4x4_from_pose(RantPose p){
    i_MathMat3 r = i_math_mat3_from_quaternion(p.orientation);
    RantMatrix4x4 m;
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) m.m[i][j] = (float)r.m[i][j];
    m.m[0][3] = (float)p.position.x; m.m[1][3] = (float)p.position.y; m.m[2][3] = (float)p.position.z;
    m.m[3][0] = m.m[3][1] = m.m[3][2] = 0.0f; m.m[3][3] = 1.0f;
    return m;
}

/* The columns are made orthonormal before reading the turn, so a scaled or slightly skewed
 * matrix still gives the rotation nearest to it. */
RantPose rant_pose_from_matrix4x4(RantMatrix4x4 m){
    RantPose p;
    RantDouble3 x = rant_double3(m.m[0][0], m.m[1][0], m.m[2][0]);
    RantDouble3 y = rant_double3(m.m[0][1], m.m[1][1], m.m[2][1]);
    RantDouble3 z;
    i_MathMat3 r;
    x = rant_double3_normalize(x);
    y = rant_double3_normalize(rant_double3_subtract(y, rant_double3_scale(x, rant_double3_dot(x, y))));
    z = rant_double3_cross(x, y);
    if (rant_double3_length_squared(x) == 0.0 || rant_double3_length_squared(y) == 0.0){
        p.orientation = rant_quaternion_identity();
    } else {
        r.m[0][0] = x.x; r.m[0][1] = y.x; r.m[0][2] = z.x;
        r.m[1][0] = x.y; r.m[1][1] = y.y; r.m[1][2] = z.y;
        r.m[2][0] = x.z; r.m[2][1] = y.z; r.m[2][2] = z.z;
        p.orientation = i_math_quaternion_from_mat3(&r);
    }
    p.position = rant_double3(m.m[0][3], m.m[1][3], m.m[2][3]);
    return p;
}

RantPose2D rant_pose2d_from_pose(RantPose p){
    RantPose2D r;
    r.position = rant_double2(p.position.x, p.position.y);
    r.angle = rant_quaternion_yaw(p.orientation);
    return r;
}

RantPose rant_pose_from_pose2d(RantPose2D p){
    RantPose r;
    r.position = rant_double3(p.position.x, p.position.y, 0.0);
    r.orientation = rant_quaternion_from_axis_angle(rant_double3(0.0, 0.0, 1.0), p.angle);
    return r;
}

RantPose2D rant_pose2d_compose(RantPose2D a, RantPose2D b){
    RantPose2D r;
    r.position = rant_double2_add(a.position, rant_double2_rotate(b.position, a.angle));
    r.angle = rant_angle_wrap(a.angle + b.angle);
    return r;
}

RantPose2D rant_pose2d_inverse(RantPose2D p){
    RantPose2D r;
    r.angle = rant_angle_wrap(-p.angle);
    r.position = rant_double2_scale(rant_double2_rotate(p.position, -p.angle), -1.0);
    return r;
}

RantPose2D rant_pose2d_between(RantPose2D a, RantPose2D b){
    return rant_pose2d_compose(rant_pose2d_inverse(a), b);
}

RantDouble2 rant_pose2d_apply(RantPose2D p, RantDouble2 point){
    return rant_double2_add(p.position, rant_double2_rotate(point, p.angle));
}

RantDouble2 rant_pose2d_apply_inverse(RantPose2D p, RantDouble2 point){
    return rant_double2_rotate(rant_double2_subtract(point, p.position), -p.angle);
}

void rant_pose2d_apply_points(RantPose2D p, const RantDouble2 *in, RantDouble2 *out, uint32_t count){
    double c = cos(p.angle), s = sin(p.angle);
    uint32_t i;
    if (!in || !out) return;
    for (i = 0; i < count; i++){
        RantDouble2 v = in[i];
        out[i] = rant_double2(p.position.x + c * v.x - s * v.y, p.position.y + s * v.x + c * v.y);
    }
}

RantPose2D rant_pose2d_interpolate(RantPose2D a, RantPose2D b, double t){
    RantPose2D r;
    r.position = rant_double2_lerp(a.position, b.position, t);
    r.angle = rant_angle_wrap(a.angle + rant_angle_difference(a.angle, b.angle) * t);
    return r;
}
