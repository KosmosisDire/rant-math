/* Shapes, poses and polynomials fitted to points, by least squares, or by RANSAC when the options
 * give a tolerance. ERR_DEGENERATE means the points fix no single answer. */
#ifndef RANT_MATH_FIT_H
#define RANT_MATH_FIT_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* How a fit treats its points. All zero, or a NULL pointer, fits every point alike. */
typedef struct {
    /* One per point: how many times its squared error counts. NULL counts each once. */
    const double *weights;
    /* Above 0, RANSAC: a point farther than this from a model is an outlier of it. */
    double tolerance;
    /* The most models RANSAC tries, 0 for 1000. It stops once it is 99.9 percent sure. */
    uint32_t tries;
    /* RANSAC's random draws: the same seed gives the same answer. */
    uint64_t seed;
    /* When not NULL, one per point: 1 within tolerance of the answer, else 0. All 1 without RANSAC. */
    uint8_t *inliers;
} RantFitOptions;

/* Through the points' mean, its normal facing the origin, where the sensor that saw them
 * sits, or up when the origin is no farther off the plane than the points are. Needs 3. */
RANT_MATH_API RantMathResult rant_points_fit_plane(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantPlane *out);
/* The line as the segment the points span along it, running the way they go from first
 * to last. Needs 2 points. */
RANT_MATH_API RantMathResult rant_points_fit_line(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantSegment *out);
/* The sphere nearest the points by their distance from its surface. Needs 4 points. */
RANT_MATH_API RantMathResult rant_points_fit_sphere(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantSphere *out);
/* The pose that moves each from point nearest onto its to point, by Horn's method. Needs 3. */
RANT_MATH_API RantMathResult rant_points_fit_pose(const RantDouble3 *from, const RantDouble3 *to, uint32_t count, const RantFitOptions *options, RantPose *out);

RANT_MATH_API RantMathResult rant_points2d_fit_line(const RantDouble2 *points, uint32_t count, const RantFitOptions *options, RantSegment2D *out);
/* Needs 3 points. */
RANT_MATH_API RantMathResult rant_points2d_fit_circle(const RantDouble2 *points, uint32_t count, const RantFitOptions *options, RantCircle *out);
/* Needs 2 points. */
RANT_MATH_API RantMathResult rant_points2d_fit_pose(const RantDouble2 *from, const RantDouble2 *to, uint32_t count, const RantFitOptions *options, RantPose2D *out);

/* y = c[0] + c[1] x + ... + c[degree] x^degree nearest the points by their error in y, into
 * degree + 1 coefficients. Needs degree + 1 points with different x. */
RANT_MATH_API RantMathResult rant_polynomial_fit(const double *x, const double *y, uint32_t count, uint32_t degree,
                                                 const RantFitOptions *options, double *coefficients);
RANT_MATH_API double         rant_polynomial_at(const double *coefficients, uint32_t degree, double x);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_FIT_H */
