/* Poses in 3D and 2D. A pose maps a point from its own frame into its parent's, so
 * compose(world_robot, robot_camera) is the camera in the world. */
#ifndef RANT_MATH_POSE_H
#define RANT_MATH_POSE_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* b given in a's frame, returned in a's parent. */
RANT_MATH_API RantPose    rant_pose_compose(RantPose a, RantPose b);
RANT_MATH_API RantPose    rant_pose_inverse(RantPose p);
/* b as seen from a: compose(inverse(a), b). */
RANT_MATH_API RantPose    rant_pose_between(RantPose a, RantPose b);
/* A point in p's frame to p's parent, and back. */
RANT_MATH_API RantDouble3 rant_pose_apply(RantPose p, RantDouble3 point);
RANT_MATH_API RantDouble3 rant_pose_apply_inverse(RantPose p, RantDouble3 point);
/* rant_pose_apply over count points. out may be in. */
RANT_MATH_API void        rant_pose_apply_points(RantPose p, const RantDouble3 *in, RantDouble3 *out, uint32_t count);
/* From a at 0 to b at 1: the position along a line, the orientation by slerp. */
RANT_MATH_API RantPose    rant_pose_interpolate(RantPose a, RantPose b, double t);
/* The frame taught by three points: its origin, a point along +x, and a point on the
 * +y side of the xy plane, as a robot user frame is taught. */
RANT_MATH_API RantMathResult rant_pose_from_points(RantDouble3 origin, RantDouble3 on_x, RantDouble3 in_xy, RantPose *out);
/* At eye with +x toward target and +z as near up as it can. */
RANT_MATH_API RantMathResult rant_pose_look_at(RantDouble3 eye, RantDouble3 target, RantDouble3 up, RantPose *out);
RANT_MATH_API RantMatrix4x4 rant_matrix4x4_from_pose(RantPose p);
/* The rotation and translation of m. Any scale or shear is dropped. */
RANT_MATH_API RantPose    rant_pose_from_matrix4x4(RantMatrix4x4 m);

/* The top view of a pose: x, y and the heading. And a 2D pose lifted onto z = 0. */
RANT_MATH_API RantPose2D  rant_pose2d_from_pose(RantPose p);
RANT_MATH_API RantPose    rant_pose_from_pose2d(RantPose2D p);

RANT_MATH_API RantPose2D  rant_pose2d_compose(RantPose2D a, RantPose2D b);
RANT_MATH_API RantPose2D  rant_pose2d_inverse(RantPose2D p);
RANT_MATH_API RantPose2D  rant_pose2d_between(RantPose2D a, RantPose2D b);
RANT_MATH_API RantDouble2 rant_pose2d_apply(RantPose2D p, RantDouble2 point);
RANT_MATH_API RantDouble2 rant_pose2d_apply_inverse(RantPose2D p, RantDouble2 point);
RANT_MATH_API void        rant_pose2d_apply_points(RantPose2D p, const RantDouble2 *in, RantDouble2 *out, uint32_t count);
/* The heading turns the shorter way. */
RANT_MATH_API RantPose2D  rant_pose2d_interpolate(RantPose2D a, RantPose2D b, double t);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_POSE_H */
