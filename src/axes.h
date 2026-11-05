/* Other tools' axes. _to turns a Rant value into a tool's axes and _from turns it back. A turn,
 * as an angular velocity, a torque or a quaternion's x y z, also flips for a left handed tool. */
#ifndef RANT_MATH_AXES_H
#define RANT_MATH_AXES_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Each of x, y and z is the Rant axis it is: 1 for x, 2 for y, 3 for z, negative when it
 * points the other way. Unity is { -2, 3, 1, 1.0 }: its x is Rant's -y, and so on. */
typedef struct { int8_t x, y, z; double scale; } RantAxes;

typedef enum {
    RANT_AXES_UNITY,          /* left handed, y up, z forward */
    RANT_AXES_UNREAL,         /* left handed, z up, x forward, centimeters */
    RANT_AXES_GODOT,          /* right handed, y up, -z forward */
    RANT_AXES_BEVY,           /* right handed, y up, -z forward */
    RANT_AXES_GLTF,           /* right handed, y up, z forward: three.js and glTF assets */
    RANT_AXES_BLENDER,        /* right handed, z up, -y forward */
    RANT_AXES_OPENCV_CAMERA,  /* x right, y down, z out of the lens */
    RANT_AXES_OPENGL_CAMERA,  /* x right, y up, -z out of the lens */
    RANT_AXES_NED,            /* x north, y east, z down, from a Rant world frame east north up */
    RANT_AXES_FRD             /* x forward, y right, z down, the aircraft body frame */
} RantAxesPreset;

RANT_MATH_API RantAxes rant_axes_of(RantAxesPreset preset);
/* Any other tool's axes. Fails unless x, y and z name each Rant axis once and scale is above 0. */
RANT_MATH_API RantMathResult rant_axes_make(int x, int y, int z, double scale, RantAxes *out);

/* A point or length vector is scaled, a direction or normal is not. */
RANT_MATH_API RantDouble3    rant_axes_point_to(RantAxes axes, RantDouble3 point);
RANT_MATH_API RantDouble3    rant_axes_point_from(RantAxes axes, RantDouble3 point);
RANT_MATH_API RantDouble3    rant_axes_direction_to(RantAxes axes, RantDouble3 direction);
RANT_MATH_API RantDouble3    rant_axes_direction_from(RantAxes axes, RantDouble3 direction);
RANT_MATH_API RantQuaternion rant_axes_quaternion_to(RantAxes axes, RantQuaternion q);
RANT_MATH_API RantQuaternion rant_axes_quaternion_from(RantAxes axes, RantQuaternion q);
RANT_MATH_API RantPose       rant_axes_pose_to(RantAxes axes, RantPose p);
RANT_MATH_API RantPose       rant_axes_pose_from(RantAxes axes, RantPose p);
/* Each quantity is scaled by its power of length: velocity and force once, torque twice. */
RANT_MATH_API RantTwist      rant_axes_twist_to(RantAxes axes, RantTwist t);
RANT_MATH_API RantTwist      rant_axes_twist_from(RantAxes axes, RantTwist t);
RANT_MATH_API RantWrench     rant_axes_wrench_to(RantAxes axes, RantWrench w);
RANT_MATH_API RantWrench     rant_axes_wrench_from(RantAxes axes, RantWrench w);
RANT_MATH_API RantAccel      rant_axes_accel_to(RantAxes axes, RantAccel a);
RANT_MATH_API RantAccel      rant_axes_accel_from(RantAxes axes, RantAccel a);
/* The Covariance6 of a pose or twist. out may be in. */
RANT_MATH_API void rant_axes_covariance6_to(RantAxes axes, const RantCovariance6 *in, RantCovariance6 *out);
RANT_MATH_API void rant_axes_covariance6_from(RantAxes axes, const RantCovariance6 *in, RantCovariance6 *out);
/* rant_axes_point_to over count points. out may be in. */
RANT_MATH_API void rant_axes_points_to(RantAxes axes, const RantDouble3 *in, RantDouble3 *out, uint32_t count);
RANT_MATH_API void rant_axes_points_from(RantAxes axes, const RantDouble3 *in, RantDouble3 *out, uint32_t count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_AXES_H */
