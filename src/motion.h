/* Motion: twists and wrenches moved between frames, poses moved by a twist, covariances, inertia.
 * A twist is its frame origin's velocity and the angular velocity, both in that frame's axes. */
#ifndef RANT_MATH_MOTION_H
#define RANT_MATH_MOTION_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The same motion measured in p's parent, where p is the twist's frame in that parent. The
 * linear part becomes the velocity of the point at the parent's origin. */
RANT_MATH_API RantTwist  rant_twist_transform(RantPose p, RantTwist t);
/* The same force measured in p's parent, its torque taken about the parent's origin. */
RANT_MATH_API RantWrench rant_wrench_transform(RantPose p, RantWrench w);
/* Where p ends after moving dt seconds at a twist given in its own frame, as Odometry gives
 * it. The path is the exact arc a constant twist traces. */
RANT_MATH_API RantPose   rant_pose_integrate(RantPose p, RantTwist body, double dt);
/* The constant twist in a's frame that carries a to b in dt seconds, turning at most half a
 * turn: the inverse of rant_pose_integrate for any shorter turn. dt must be above 0. */
RANT_MATH_API RantMathResult rant_twist_between(RantPose a, RantPose b, double dt, RantTwist *out);

RANT_MATH_API RantTwist2D  rant_twist2d_transform(RantPose2D p, RantTwist2D t);
RANT_MATH_API RantWrench2D rant_wrench2d_transform(RantPose2D p, RantWrench2D w);
RANT_MATH_API RantPose2D   rant_pose2d_integrate(RantPose2D p, RantTwist2D body, double dt);
RANT_MATH_API RantMathResult rant_twist2d_between(RantPose2D a, RantPose2D b, double dt, RantTwist2D *out);

/* A covariance measured in axes turned by q: R C R^T, block by block for the Covariance6 of
 * a pose or twist. out may be in. */
RANT_MATH_API void rant_covariance3_rotate(RantQuaternion q, const RantCovariance3 *in, RantCovariance3 *out);
RANT_MATH_API void rant_covariance6_rotate(RantQuaternion q, const RantCovariance6 *in, RantCovariance6 *out);

/* The body measured in p's parent: its center moved and its tensor turned. */
RANT_MATH_API RantInertia rant_inertia_transform(RantInertia i, RantPose p);
/* Two bodies in one frame joined into one, about their shared center of mass. */
RANT_MATH_API RantInertia rant_inertia_add(RantInertia a, RantInertia b);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_MOTION_H */
