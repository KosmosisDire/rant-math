/* Rotations: quaternions, Euler angles in every order, axis and angle, rotation vectors,
 * rotation matrices, and 2D angles. Angles are radians. */
#ifndef RANT_MATH_ROTATION_H
#define RANT_MATH_ROTATION_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FIXED turns about the parent's axes, MOVING about the axes as they turn, the angles in the
 * order the name lists the axes. FIXED_XYZ is roll, pitch, yaw, as MOVING_ZYX reversed. */
typedef enum {
    RANT_EULER_FIXED_XYZ, RANT_EULER_FIXED_XZY, RANT_EULER_FIXED_YXZ,
    RANT_EULER_FIXED_YZX, RANT_EULER_FIXED_ZXY, RANT_EULER_FIXED_ZYX,
    RANT_EULER_FIXED_XYX, RANT_EULER_FIXED_XZX, RANT_EULER_FIXED_YXY,
    RANT_EULER_FIXED_YZY, RANT_EULER_FIXED_ZXZ, RANT_EULER_FIXED_ZYZ,
    RANT_EULER_MOVING_XYZ, RANT_EULER_MOVING_XZY, RANT_EULER_MOVING_YXZ,
    RANT_EULER_MOVING_YZX, RANT_EULER_MOVING_ZXY, RANT_EULER_MOVING_ZYX,
    RANT_EULER_MOVING_XYX, RANT_EULER_MOVING_XZX, RANT_EULER_MOVING_YXY,
    RANT_EULER_MOVING_YZY, RANT_EULER_MOVING_ZXZ, RANT_EULER_MOVING_ZYZ
} RantEulerOrder;

RANT_MATH_API RantQuaternion rant_quaternion_from_euler(double first, double second, double third,
                                                        RantEulerOrder order);
/* The middle angle is in [-pi/2, pi/2] for three axes and [0, pi] for a repeated one, the
 * others in (-pi, pi]. Where the middle angle locks the outer two, the third is 0. */
RANT_MATH_API void rant_quaternion_to_euler(RantQuaternion q, RantEulerOrder order, double angles[3]);

/* The axis need not be unit length. A zero axis gives the identity. */
RANT_MATH_API RantQuaternion rant_quaternion_from_axis_angle(RantDouble3 axis, double angle);
/* The angle in [0, pi] and its unit axis, +x when there is no turn. */
RANT_MATH_API double rant_quaternion_to_axis_angle(RantQuaternion q, RantDouble3 *axis);
/* A rotation vector is the axis times the angle, as robot controllers and gyros give it. */
RANT_MATH_API RantQuaternion rant_quaternion_from_rotation_vector(RantDouble3 v);
RANT_MATH_API RantDouble3    rant_quaternion_to_rotation_vector(RantQuaternion q);
RANT_MATH_API RantQuaternion rant_quaternion_from_matrix3x3(RantMatrix3x3 m);
RANT_MATH_API RantMatrix3x3  rant_matrix3x3_from_quaternion(RantQuaternion q);

/* The shortest turn that points from along to. */
RANT_MATH_API RantQuaternion rant_quaternion_between(RantDouble3 from, RantDouble3 to);
/* The turn that points +x along forward and +z as near up as it can. */
RANT_MATH_API RantQuaternion rant_quaternion_look(RantDouble3 forward, RantDouble3 up);

/* b given in a's frame, as rant_pose_compose: the Hamilton product a b. */
RANT_MATH_API RantQuaternion rant_quaternion_compose(RantQuaternion a, RantQuaternion b);
RANT_MATH_API RantQuaternion rant_quaternion_inverse(RantQuaternion q);
/* Unit length. The zero quaternion becomes the identity. */
RANT_MATH_API RantQuaternion rant_quaternion_normalize(RantQuaternion q);
RANT_MATH_API RantDouble3    rant_quaternion_rotate(RantQuaternion q, RantDouble3 v);
/* The shorter way from a at 0 to b at 1, at an even turning speed. */
RANT_MATH_API RantQuaternion rant_quaternion_slerp(RantQuaternion a, RantQuaternion b, double t);
/* The size of the turn from a to b, in [0, pi]. */
RANT_MATH_API double         rant_quaternion_angle(RantQuaternion a, RantQuaternion b);
/* q after turning at angular_velocity, given in its own frame, for dt seconds. */
RANT_MATH_API RantQuaternion rant_quaternion_integrate(RantQuaternion q, RantDouble3 angular_velocity, double dt);
/* The heading of the turned +x seen from above, from +x toward +y. */
RANT_MATH_API double         rant_quaternion_yaw(RantQuaternion q);

/* An angle in (-pi, pi]. */
RANT_MATH_API double rant_angle_wrap(double angle);
/* The shorter turn from one heading to another, in (-pi, pi]. */
RANT_MATH_API double rant_angle_difference(double from, double to);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_ROTATION_H */
