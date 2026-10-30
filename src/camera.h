/* A camera's pinhole model and lens, as CameraIntrinsics holds them. A camera looks along +x of
 * its frame, y left and z up, and pixel centers sit on whole numbers from the top left corner. */
#ifndef RANT_MATH_CAMERA_H
#define RANT_MATH_CAMERA_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t width, height;      /* the resolution these values hold for */
    double fx, fy, cx, cy;       /* fx and fy above 0 */
    RantDistortionModel model;
    /* OpenCV's order: k1 k2 p1 p2 k3, then k4 k5 k6 for Rational, or k1 to k4 for Fisheye. 0 past
     * the model's count. */
    double coeffs[8];
} RantCameraIntrinsics;

/* Where a point in the camera's frame lands, maybe off the image. RANT_MATH_ERR_OUTSIDE behind
 * a pinhole camera, or past where the lens model folds back, landing among the points it sees. */
RANT_MATH_API RantMathResult rant_camera_project(const RantCameraIntrinsics *camera, RantDouble3 point, RantDouble2 *pixel);
/* rant_camera_project for count points, NaN for each one it cannot project. */
RANT_MATH_API RantMathResult rant_camera_project_points(const RantCameraIntrinsics *camera, const RantDouble3 *points,
                                                        RantDouble2 *pixels, uint32_t count);
/* The ray through a pixel from the camera's origin. RANT_MATH_ERR_OUTSIDE past its lens's reach. */
RANT_MATH_API RantMathResult rant_camera_ray(const RantCameraIntrinsics *camera, RantDouble2 pixel, RantRay *out);
/* The point a depth image's pixel saw: on its ray, depth ahead of the camera along +x. */
RANT_MATH_API RantMathResult rant_camera_unproject(const RantCameraIntrinsics *camera, RantDouble2 pixel, double depth,
                                                   RantDouble3 *out);

/* The intrinsics of the image resized to width by height, and of the part a crop keeps. */
RANT_MATH_API RantMathResult rant_camera_scale(const RantCameraIntrinsics *camera, uint32_t width, uint32_t height,
                                               RantCameraIntrinsics *out);
RANT_MATH_API RantMathResult rant_camera_crop(const RantCameraIntrinsics *camera, uint32_t x, uint32_t y,
                                              uint32_t width, uint32_t height, RantCameraIntrinsics *out);
/* The angle the image spans edge to edge through the principal point: x across, y down. */
RANT_MATH_API RantMathResult rant_camera_fov(const RantCameraIntrinsics *camera, RantDouble2 *out);
/* A centered pinhole camera with no distortion spanning fov.x across and fov.y down, which 0
 * makes square pixels. */
RANT_MATH_API RantMathResult rant_camera_from_fov(uint32_t width, uint32_t height, RantDouble2 fov, RantCameraIntrinsics *out);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_CAMERA_H */
