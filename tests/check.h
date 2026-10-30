/* The test harness: checks that count failures, a seeded random source, and helpers that
 * compare the standard types within a tolerance. */
#ifndef RANT_MATH_CHECK_H
#define RANT_MATH_CHECK_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rant_math.h"

extern int check_failures;
extern int check_count;

void check_report(const char *file, int line, const char *fmt, ...);

#define CHECK(cond, ...) do { check_count++; \
    if (!(cond)) { check_failures++; check_report(__FILE__, __LINE__, __VA_ARGS__); } } while (0)

/* Within tol of each other, or both the same infinity. NaN never passes. */
int near(double a, double b, double tol);
int near2(RantDouble2 a, RantDouble2 b, double tol);
int near3(RantDouble3 a, RantDouble3 b, double tol);
/* The same rotation: q and -q both pass. */
int same_turn(RantQuaternion a, RantQuaternion b, double tol);
int same_pose(RantPose a, RantPose b, double tol);

/* A seeded xorshift, so a failure replays. */
void   random_seed(uint64_t seed);
double random_unit(void);                      /* [0, 1) */
double random_range(double lo, double hi);
RantDouble2 random_double2(double r);          /* each component in [-r, r) */
RantDouble3 random_double3(double r);
RantQuaternion random_quaternion(void);        /* uniform over rotations */
RantPose random_pose(double r);
/* Corners in order of their angle about the origin, each gap under half a turn, so the polygon
 * never crosses itself and is often concave. Half the time clockwise. Returns 3 to most. */
uint32_t random_polygon(RantDouble2 *out, uint32_t most);

#define PI 3.14159265358979323846

/* Each test file holds one of these per part. */
void test_vector(void);
void test_rotation(void);
void test_matrix(void);
void test_pose(void);
void test_motion(void);
void test_axes(void);
void test_line(void);
void test_region(void);
void test_solid(void);
void test_curve(void);
void test_surface(void);
void test_points(void);
void test_shape(void);
void test_fit(void);
void test_color(void);
void test_camera(void);
void test_image(void);
void test_cloud(void);
void test_neighbors(void);
void test_grid(void);
void test_geo(void);
void test_detection(void);

#endif /* RANT_MATH_CHECK_H */
