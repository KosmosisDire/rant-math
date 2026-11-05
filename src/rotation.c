#include "internal.h"

/* The 3 by 3 in double. */

i_MathMat3 i_math_mat3_from_quaternion(RantQuaternion q){
    i_MathMat3 r;
    double x, y, z, w;
    q = rant_quaternion_normalize(q);
    x = q.x; y = q.y; z = q.z; w = q.w;
    r.m[0][0] = 1.0 - 2.0 * (y * y + z * z);
    r.m[0][1] = 2.0 * (x * y - w * z);
    r.m[0][2] = 2.0 * (x * z + w * y);
    r.m[1][0] = 2.0 * (x * y + w * z);
    r.m[1][1] = 1.0 - 2.0 * (x * x + z * z);
    r.m[1][2] = 2.0 * (y * z - w * x);
    r.m[2][0] = 2.0 * (x * z - w * y);
    r.m[2][1] = 2.0 * (y * z + w * x);
    r.m[2][2] = 1.0 - 2.0 * (x * x + y * y);
    return r;
}

/* Shepperd: build from the largest of the trace and the diagonal, so nothing divides by a
 * small number. */
RantQuaternion i_math_quaternion_from_mat3(const i_MathMat3 *r){
    const double (*m)[3] = r->m;
    double t = m[0][0] + m[1][1] + m[2][2], s;
    RantQuaternion q;
    if (t >= m[0][0] && t >= m[1][1] && t >= m[2][2]){
        s = 2.0 * sqrt(1.0 + t);
        q = rant_quaternion((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s);
    } else if (m[0][0] >= m[1][1] && m[0][0] >= m[2][2]){
        s = 2.0 * sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
        q = rant_quaternion(0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s);
    } else if (m[1][1] >= m[2][2]){
        s = 2.0 * sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
        q = rant_quaternion((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s);
    } else {
        s = 2.0 * sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
        q = rant_quaternion((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s);
    }
    if (q.w < 0.0) q = rant_quaternion(-q.x, -q.y, -q.z, -q.w);
    return rant_quaternion_normalize(q);
}

i_MathMat3 i_math_mat3_multiply(const i_MathMat3 *a, const i_MathMat3 *b){
    i_MathMat3 r;
    int i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            r.m[i][j] = a->m[i][0] * b->m[0][j] + a->m[i][1] * b->m[1][j] + a->m[i][2] * b->m[2][j];
    return r;
}

i_MathMat3 i_math_mat3_transpose(const i_MathMat3 *a){
    i_MathMat3 r;
    int i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) r.m[i][j] = a->m[j][i];
    return r;
}

RantDouble3 i_math_mat3_apply(const i_MathMat3 *a, RantDouble3 v){
    return rant_double3(a->m[0][0] * v.x + a->m[0][1] * v.y + a->m[0][2] * v.z,
                        a->m[1][0] * v.x + a->m[1][1] * v.y + a->m[1][2] * v.z,
                        a->m[2][0] * v.x + a->m[2][1] * v.y + a->m[2][2] * v.z);
}

/* Quaternions. */

RantQuaternion rant_quaternion_compose(RantQuaternion a, RantQuaternion b){
    return rant_quaternion(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                           a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                           a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                           a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

RantQuaternion rant_quaternion_inverse(RantQuaternion q){
    double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!(n > 0.0)) return rant_quaternion_identity();
    return rant_quaternion(-q.x / n, -q.y / n, -q.z / n, q.w / n);
}

RantQuaternion rant_quaternion_normalize(RantQuaternion q){
    double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!(n > 0.0)) return rant_quaternion_identity();
    return rant_quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
}

/* v + 2 u x (u x v + w v) for the unit quaternion (u, w). */
RantDouble3 rant_quaternion_rotate(RantQuaternion q, RantDouble3 v){
    RantDouble3 u, t;
    q = rant_quaternion_normalize(q);
    u = rant_double3(q.x, q.y, q.z);
    t = rant_double3_add(rant_double3_cross(u, v), rant_double3_scale(v, q.w));
    return rant_double3_add(v, rant_double3_scale(rant_double3_cross(u, t), 2.0));
}

RantQuaternion rant_quaternion_from_axis_angle(RantDouble3 axis, double angle){
    double n = rant_double3_length(axis), s;
    if (!(n > 0.0)) return rant_quaternion_identity();
    s = sin(0.5 * angle) / n;
    return rant_quaternion(axis.x * s, axis.y * s, axis.z * s, cos(0.5 * angle));
}

double rant_quaternion_to_axis_angle(RantQuaternion q, RantDouble3 *axis){
    double s;
    q = rant_quaternion_normalize(q);
    if (q.w < 0.0) q = rant_quaternion(-q.x, -q.y, -q.z, -q.w);
    s = sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (axis) *axis = s > 0.0 ? rant_double3(q.x / s, q.y / s, q.z / s) : rant_double3(1.0, 0.0, 0.0);
    return 2.0 * atan2(s, q.w);
}

RantQuaternion rant_quaternion_from_rotation_vector(RantDouble3 v){
    double angle = rant_double3_length(v);
    /* sin(angle / 2) / angle without the division, exact to double below 1e-4 */
    double k = angle > 1e-4 ? sin(0.5 * angle) / angle : 0.5 - angle * angle / 48.0;
    return rant_quaternion(v.x * k, v.y * k, v.z * k, cos(0.5 * angle));
}

RantDouble3 rant_quaternion_to_rotation_vector(RantQuaternion q){
    RantDouble3 axis;
    double angle = rant_quaternion_to_axis_angle(q, &axis);
    return rant_double3_scale(axis, angle);
}

RantQuaternion rant_quaternion_from_matrix3x3(RantMatrix3x3 m){
    i_MathMat3 r;
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r.m[i][j] = m.m[i][j];
    return i_math_quaternion_from_mat3(&r);
}

RantMatrix3x3 rant_matrix3x3_from_quaternion(RantQuaternion q){
    i_MathMat3 r = i_math_mat3_from_quaternion(q);
    RantMatrix3x3 m;
    int i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) m.m[i][j] = (float)r.m[i][j];
    return m;
}

/* The half vector of from and to stays well conditioned until they point exactly apart. */
RantQuaternion rant_quaternion_between(RantDouble3 from, RantDouble3 to){
    RantDouble3 a = rant_double3_normalize(from), b = rant_double3_normalize(to), h, c;
    if (rant_double3_length_squared(a) == 0.0 || rant_double3_length_squared(b) == 0.0)
        return rant_quaternion_identity();
    h = rant_double3_add(a, b);
    if (rant_double3_length(h) < 1e-9){
        RantDouble3 p = rant_double3_perpendicular(a);
        return rant_quaternion(p.x, p.y, p.z, 0.0);
    }
    h = rant_double3_normalize(h);
    c = rant_double3_cross(a, h);
    return rant_quaternion_normalize(rant_quaternion(c.x, c.y, c.z, rant_double3_dot(a, h)));
}

RantQuaternion rant_quaternion_look(RantDouble3 forward, RantDouble3 up){
    RantDouble3 x = rant_double3_normalize(forward), y, z;
    i_MathMat3 r;
    if (rant_double3_length_squared(x) == 0.0) return rant_quaternion_identity();
    y = rant_double3_cross(up, x);
    if (rant_double3_length(y) < I_MATH_TINY * (1.0 + rant_double3_length(up)))
        y = rant_double3_perpendicular(x);
    y = rant_double3_normalize(y);
    z = rant_double3_cross(x, y);
    r.m[0][0] = x.x; r.m[0][1] = y.x; r.m[0][2] = z.x;
    r.m[1][0] = x.y; r.m[1][1] = y.y; r.m[1][2] = z.y;
    r.m[2][0] = x.z; r.m[2][1] = y.z; r.m[2][2] = z.z;
    return i_math_quaternion_from_mat3(&r);
}

RantQuaternion rant_quaternion_slerp(RantQuaternion a, RantQuaternion b, double t){
    double d, theta, s, wa, wb;
    a = rant_quaternion_normalize(a);
    b = rant_quaternion_normalize(b);
    d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0){ b = rant_quaternion(-b.x, -b.y, -b.z, -b.w); d = -d; }
    if (d > 0.9999995){
        wa = 1.0 - t; wb = t;
    } else {
        theta = acos(d);
        s = sin(theta);
        wa = sin((1.0 - t) * theta) / s;
        wb = sin(t * theta) / s;
    }
    return rant_quaternion_normalize(rant_quaternion(wa * a.x + wb * b.x, wa * a.y + wb * b.y,
                                                     wa * a.z + wb * b.z, wa * a.w + wb * b.w));
}

double rant_quaternion_angle(RantQuaternion a, RantQuaternion b){
    RantQuaternion d = rant_quaternion_compose(rant_quaternion_inverse(rant_quaternion_normalize(a)),
                                               rant_quaternion_normalize(b));
    double s = sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    return 2.0 * atan2(s, fabs(d.w));
}

RantQuaternion rant_quaternion_integrate(RantQuaternion q, RantDouble3 angular_velocity, double dt){
    return rant_quaternion_normalize(rant_quaternion_compose(
        q, rant_quaternion_from_rotation_vector(rant_double3_scale(angular_velocity, dt))));
}

double rant_quaternion_yaw(RantQuaternion q){
    RantDouble3 x = rant_quaternion_rotate(q, rant_double3(1.0, 0.0, 0.0));
    return atan2(x.y, x.x);
}

/* Euler angles. */

/* The axes of each order as the turns of a MOVING sequence, in the enum's order. */
static const int i_euler_axes[12][3] = {
    { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 },
    { 0, 1, 0 }, { 0, 2, 0 }, { 1, 0, 1 }, { 1, 2, 1 }, { 2, 0, 2 }, { 2, 1, 2 }
};

static RantQuaternion i_euler_axis_turn(int axis, double angle){
    double s = sin(0.5 * angle), c = cos(0.5 * angle);
    return rant_quaternion(axis == 0 ? s : 0.0, axis == 1 ? s : 0.0, axis == 2 ? s : 0.0, c);
}

RantQuaternion rant_quaternion_from_euler(double first, double second, double third, RantEulerOrder order){
    int o = (int)order, moving = o >= 12;
    const int *ax;
    RantQuaternion q1, q2, q3;
    if (o < 0 || o >= 24) return rant_quaternion_identity();
    ax = i_euler_axes[moving ? o - 12 : o];
    q1 = i_euler_axis_turn(ax[0], first);
    q2 = i_euler_axis_turn(ax[1], second);
    q3 = i_euler_axis_turn(ax[2], third);
    /* moving: each turn in the frame the last left, fixed: each turn in the parent's */
    return moving ? rant_quaternion_compose(rant_quaternion_compose(q1, q2), q3)
                  : rant_quaternion_compose(rant_quaternion_compose(q3, q2), q1);
}

/* R = R_i(a) R_j(b) R_c(c) for the moving sequence i, j, c, s 1 when i, j, third run as x y z
 * does, else -1. At the lock one outer angle is free: zero_first sets a to 0, else c. */
static void i_euler_moving(const i_MathMat3 *rm, int i, int j, int third, int zero_first, double out[3]){
    const double (*r)[3] = rm->m;
    double a, b, c, s;
    s = ((j - i + 3) % 3 == 1) ? 1.0 : -1.0;
    if (third != i){
        int k = third;
        double cb = sqrt(r[i][i] * r[i][i] + r[i][j] * r[i][j]);
        b = atan2(s * r[i][k], cb);
        if (cb > 1e-12){
            a = atan2(-s * r[j][k], r[k][k]);
            c = atan2(-s * r[i][j], r[i][i]);
        } else if (zero_first){
            a = 0.0;
            c = atan2(s * r[j][i], r[j][j]);
        } else {
            c = 0.0;
            a = atan2(s * r[k][j], r[j][j]);
        }
    } else {
        int k = 3 - i - j;
        double sb = sqrt(r[i][j] * r[i][j] + r[i][k] * r[i][k]);
        b = atan2(sb, r[i][i]);
        if (sb > 1e-12){
            a = atan2(r[j][i], -s * r[k][i]);
            c = atan2(r[i][j], s * r[i][k]);
        } else if (zero_first){
            a = 0.0;
            c = atan2(-s * r[j][k], r[j][j]);
        } else {
            c = 0.0;
            a = atan2(s * r[k][j], r[j][j]);
        }
    }
    out[0] = rant_angle_wrap(a);
    out[1] = b;
    out[2] = rant_angle_wrap(c);
}

void rant_quaternion_to_euler(RantQuaternion q, RantEulerOrder order, double angles[3]){
    int o = (int)order;
    const int *ax;
    i_MathMat3 r;
    if (!angles) return;
    if (o < 0 || o >= 24){ angles[0] = angles[1] = angles[2] = 0.0; return; }
    r = i_math_mat3_from_quaternion(q);
    if (o >= 12){
        ax = i_euler_axes[o - 12];
        i_euler_moving(&r, ax[0], ax[1], ax[2], 0, angles);
    } else {
        /* fixed a1 a2 a3 is moving a3 a2 a1 with the angles reversed, so its third is a */
        double m[3];
        ax = i_euler_axes[o];
        i_euler_moving(&r, ax[2], ax[1], ax[0], 1, m);
        angles[0] = m[2]; angles[1] = m[1]; angles[2] = m[0];
    }
}

/* 2D angles. */

double rant_angle_wrap(double angle){
    double r = fmod(angle, 2.0 * I_MATH_PI);
    if (r > I_MATH_PI) r -= 2.0 * I_MATH_PI;
    else if (r <= -I_MATH_PI) r += 2.0 * I_MATH_PI;
    return r;
}

double rant_angle_difference(double from, double to){
    return rant_angle_wrap(to - from);
}
