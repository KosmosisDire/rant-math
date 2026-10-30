#include "internal.h"

RantTwist rant_twist_transform(RantPose p, RantTwist t){
    RantTwist r;
    i_MathMat3 m = i_math_mat3_from_quaternion(p.orientation);
    r.angular = i_math_mat3_apply(&m, t.angular);
    r.linear = rant_double3_add(i_math_mat3_apply(&m, t.linear), rant_double3_cross(p.position, r.angular));
    return r;
}

RantWrench rant_wrench_transform(RantPose p, RantWrench w){
    RantWrench r;
    i_MathMat3 m = i_math_mat3_from_quaternion(p.orientation);
    r.force = i_math_mat3_apply(&m, w.force);
    r.torque = rant_double3_add(i_math_mat3_apply(&m, w.torque), rant_double3_cross(p.position, r.force));
    return r;
}

/* The SE(3) exponential: a constant twist rotates by phi = w dt and moves by V rho, rho = v dt,
 * with V = I + b [phi] + c [phi]^2. Below 1e-4 the series keeps b and c exact. */
RantPose rant_pose_integrate(RantPose p, RantTwist body, double dt){
    RantDouble3 phi = rant_double3_scale(body.angular, dt), rho = rant_double3_scale(body.linear, dt);
    double theta = rant_double3_length(phi), t2 = theta * theta, b, c;
    RantDouble3 pr, ppr;
    RantPose step;
    if (theta > 1e-4){
        b = (1.0 - cos(theta)) / t2;
        c = (theta - sin(theta)) / (t2 * theta);
    } else {
        b = 0.5 - t2 / 24.0;
        c = 1.0 / 6.0 - t2 / 120.0;
    }
    pr = rant_double3_cross(phi, rho);
    ppr = rant_double3_cross(phi, pr);
    step.position = rant_double3_add(rho, rant_double3_add(rant_double3_scale(pr, b), rant_double3_scale(ppr, c)));
    step.orientation = rant_quaternion_from_rotation_vector(phi);
    return rant_pose_compose(p, step);
}

/* The SE(3) logarithm: V^-1 = I - [phi] / 2 + d [phi]^2. */
RantMathResult rant_twist_between(RantPose a, RantPose b, double dt, RantTwist *out){
    RantPose step;
    RantDouble3 phi, rho, pr, ppr;
    double theta, t2, d;
    if (!out || !(dt > 0.0)) return RANT_MATH_ERR_ARGUMENT;
    step = rant_pose_between(a, b);
    phi = rant_quaternion_to_rotation_vector(step.orientation);
    theta = rant_double3_length(phi);
    t2 = theta * theta;
    d = theta > 1e-4 ? (1.0 - theta * sin(theta) / (2.0 * (1.0 - cos(theta)))) / t2
                     : 1.0 / 12.0 + t2 / 720.0;
    pr = rant_double3_cross(phi, step.position);
    ppr = rant_double3_cross(phi, pr);
    rho = rant_double3_add(step.position, rant_double3_add(rant_double3_scale(pr, -0.5), rant_double3_scale(ppr, d)));
    out->linear = rant_double3_scale(rho, 1.0 / dt);
    out->angular = rant_double3_scale(phi, 1.0 / dt);
    return RANT_MATH_OK;
}

RantTwist2D rant_twist2d_transform(RantPose2D p, RantTwist2D t){
    RantTwist2D r;
    RantDouble2 v = rant_double2_rotate(t.linear, p.angle);
    r.angular = t.angular;
    r.linear = rant_double2(v.x + p.position.y * t.angular, v.y - p.position.x * t.angular);
    return r;
}

RantWrench2D rant_wrench2d_transform(RantPose2D p, RantWrench2D w){
    RantWrench2D r;
    r.force = rant_double2_rotate(w.force, p.angle);
    r.torque = w.torque + rant_double2_cross(p.position, r.force);
    return r;
}

/* The SE(2) form of the same arc: V = [[s, -c], [c, s]] with s = sin(theta) / theta and
 * c = (1 - cos(theta)) / theta. */
static void i_se2_v(double theta, double *s, double *c){
    double t2 = theta * theta;
    if (fabs(theta) > 1e-4){
        *s = sin(theta) / theta;
        *c = (1.0 - cos(theta)) / theta;
    } else {
        *s = 1.0 - t2 / 6.0;
        *c = theta / 2.0 - theta * t2 / 24.0;
    }
}

RantPose2D rant_pose2d_integrate(RantPose2D p, RantTwist2D body, double dt){
    double theta = body.angular * dt, s, c;
    RantDouble2 rho = rant_double2_scale(body.linear, dt);
    RantPose2D step;
    i_se2_v(theta, &s, &c);
    step.position = rant_double2(s * rho.x - c * rho.y, c * rho.x + s * rho.y);
    step.angle = theta;
    return rant_pose2d_compose(p, step);
}

RantMathResult rant_twist2d_between(RantPose2D a, RantPose2D b, double dt, RantTwist2D *out){
    RantPose2D step;
    double s, c, det;
    if (!out || !(dt > 0.0)) return RANT_MATH_ERR_ARGUMENT;
    step = rant_pose2d_between(a, b);
    i_se2_v(step.angle, &s, &c);
    det = s * s + c * c;
    out->linear = rant_double2((s * step.position.x + c * step.position.y) / det / dt,
                               (-c * step.position.x + s * step.position.y) / det / dt);
    out->angular = step.angle / dt;
    return RANT_MATH_OK;
}

/* r a r^T for the 3 by 3 block of src at (row, col), stride n, into dst. */
static void i_rotate_block(const i_MathMat3 *r, const double *src, double *dst, int n, int row, int col){
    double t[3][3];
    int i, j, k;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++){
            double s = 0.0;
            for (k = 0; k < 3; k++) s += r->m[i][k] * src[(row + k) * n + col + j];
            t[i][j] = s;
        }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++){
            double s = 0.0;
            for (k = 0; k < 3; k++) s += t[i][k] * r->m[j][k];
            dst[(row + i) * n + col + j] = s;
        }
}

void rant_covariance3_rotate(RantQuaternion q, const RantCovariance3 *in, RantCovariance3 *out){
    i_MathMat3 r;
    RantCovariance3 t;
    if (!in || !out) return;
    r = i_math_mat3_from_quaternion(q);
    i_rotate_block(&r, &in->m[0][0], &t.m[0][0], 3, 0, 0);
    *out = t;
}

void rant_covariance6_rotate(RantQuaternion q, const RantCovariance6 *in, RantCovariance6 *out){
    i_MathMat3 r;
    RantCovariance6 t;
    int bi, bj;
    if (!in || !out) return;
    r = i_math_mat3_from_quaternion(q);
    for (bi = 0; bi < 2; bi++)
        for (bj = 0; bj < 2; bj++) i_rotate_block(&r, &in->m[0][0], &t.m[0][0], 6, bi * 3, bj * 3);
    *out = t;
}

static i_MathMat3 i_inertia_tensor(RantInertia i){
    i_MathMat3 t;
    t.m[0][0] = i.ixx; t.m[0][1] = i.ixy; t.m[0][2] = i.ixz;
    t.m[1][0] = i.ixy; t.m[1][1] = i.iyy; t.m[1][2] = i.iyz;
    t.m[2][0] = i.ixz; t.m[2][1] = i.iyz; t.m[2][2] = i.izz;
    return t;
}

static void i_inertia_set_tensor(RantInertia *i, const i_MathMat3 *t){
    i->ixx = t->m[0][0]; i->ixy = t->m[0][1]; i->ixz = t->m[0][2];
    i->iyy = t->m[1][1]; i->iyz = t->m[1][2]; i->izz = t->m[2][2];
}

RantInertia rant_inertia_transform(RantInertia i, RantPose p){
    i_MathMat3 r = i_math_mat3_from_quaternion(p.orientation), t = i_inertia_tensor(i), rt, out;
    RantInertia o = i;
    o.center = rant_pose_apply(p, i.center);
    rt = i_math_mat3_transpose(&r);
    out = i_math_mat3_multiply(&r, &t);
    out = i_math_mat3_multiply(&out, &rt);
    i_inertia_set_tensor(&o, &out);
    return o;
}

/* The parallel axis theorem: a body of mass m whose center sits d from the new center adds
 * m (|d|^2 E - d d^T) to its tensor. */
static void i_parallel_axis(i_MathMat3 *t, double m, RantDouble3 d){
    double dd = rant_double3_dot(d, d), v[3];
    int i, j;
    v[0] = d.x; v[1] = d.y; v[2] = d.z;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) t->m[i][j] += m * ((i == j ? dd : 0.0) - v[i] * v[j]);
}

RantInertia rant_inertia_add(RantInertia a, RantInertia b){
    RantInertia o;
    i_MathMat3 ta = i_inertia_tensor(a), tb = i_inertia_tensor(b);
    int i, j;
    memset(&o, 0, sizeof o);
    o.mass = a.mass + b.mass;
    if (!(o.mass > 0.0)) return o;
    o.center = rant_double3_scale(rant_double3_add(rant_double3_scale(a.center, a.mass),
                                                   rant_double3_scale(b.center, b.mass)), 1.0 / o.mass);
    i_parallel_axis(&ta, a.mass, rant_double3_subtract(a.center, o.center));
    i_parallel_axis(&tb, b.mass, rant_double3_subtract(b.center, o.center));
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) ta.m[i][j] += tb.m[i][j];
    i_inertia_set_tensor(&o, &ta);
    return o;
}
