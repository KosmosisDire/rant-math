/* Helpers the parts share. Never part of the public API. */
#ifndef RANT_MATH_INTERNAL_H
#define RANT_MATH_INTERNAL_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rant_math.h"

#define I_MATH_PI 3.14159265358979323846
/* Below this a length counts as zero when it decides a degenerate case. */
#define I_MATH_TINY 1e-12

static inline double i_math_sqr(double x){ return x * x; }
static inline double i_math_clamp(double x, double lo, double hi){
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline double i_math_min(double a, double b){ return a < b ? a : b; }
static inline double i_math_max(double a, double b){ return a > b ? a : b; }
/* Neither NaN nor infinite, without the classify macros some C libraries warn in. */
static inline int i_math_finite(double x){ return x - x == 0.0; }
static inline int i_math_finite3(RantDouble3 p){ return i_math_finite(p.x) && i_math_finite(p.y) && i_math_finite(p.z); }

/* Intersection over union from the shared measure and each one's own, 0 for nothing shared. */
static inline double i_math_iou(double both, double a, double b){
    double either = a + b - both;
    return both > 0.0 && either > 0.0 ? i_math_min(1.0, both / either) : 0.0;
}

/* A 2D point on z = 0, and a 3D point seen from above, for 2D work a 3D answer covers. */
static inline RantDouble3 i_math_lift(RantDouble2 v){ return rant_double3(v.x, v.y, 0.0); }
static inline RantDouble2 i_math_flat(RantDouble3 v){ return rant_double2(v.x, v.y); }

/* The points p1 + s d1 and p2 + t d2 nearest each other, s in [0, hi1] and t in [0, hi2],
 * hi HUGE_VAL for a ray. */
void i_math_closest_lines(RantDouble3 p1, RantDouble3 d1, double hi1,
                          RantDouble3 p2, RantDouble3 d2, double hi2, double *s, double *t);

/* The rule of a call that writes a list: *count holds the room and gets the need. */
static inline RantMathResult i_math_room(uint32_t *count, uint32_t need, const void *out){
    uint32_t room = *count;
    *count = need;
    return need > room || (need && !out) ? RANT_MATH_ERR_SPACE : RANT_MATH_OK;
}

/* A box lo to hi in dim axes, 2 or 3, shared by the 2D and 3D boxes. closest returns the
 * signed distance, raycast 1 with the distance along a unit direction. */
double i_math_box_closest(int dim, const double *lo, const double *hi, const double *p, double *point, double *normal);
int    i_math_box_raycast(int dim, const double *lo, const double *hi, const double *o, const double *d,
                          double *t, double *normal);
void   i_math_box_grow(int dim, double *lo, double *hi, double margin);

/* The nearest point of the edges pts[i] to pts[i + 1], closed back to the first, and its distance.
 * normal faces where it grows, given out 1 when the right of each edge is outside and q's inside. */
double i_math_edges_closest(const RantDouble2 *pts, uint32_t n, int closed, RantDouble2 q, double out, int inside,
                            RantDouble2 *point, RantDouble2 *normal);

/* The point of triangle a b c nearest p, as weights on its corners. */
void i_math_triangle_closest(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantDouble3 p, double w[3]);

/* The unit direction from a to b, +z when they meet. And a unit vector square to a unit axis,
 * level where it can be, so a 2D shape lifted to z = 0 keeps its answers in the plane. */
RantDouble3 i_math_axis(RantDouble3 a, RantDouble3 b);
RantDouble3 i_math_side(RantDouble3 axis);

/* A camera checked once for work on many points, in OpenCV's optical frame: x right, y down,
 * z ahead. Out to where each bound, a polynomial in the radius, first reaches 0, its lens is one to one. */
typedef struct {
    const RantCameraIntrinsics *c;
    int fisheye;
    double k[8];
    double bound[3][14];
    int degree[3], bounds;
    double fold;     /* the radius where the first bound reaches 0, below 0 until found */
} i_MathLens;

/* many finds the fold now, which then answers each point at once. */
RantMathResult i_math_lens(const RantCameraIntrinsics *c, int many, i_MathLens *lens);
RantMathResult i_math_lens_project(i_MathLens *lens, RantDouble3 optical, RantDouble2 *pixel);
/* The direction a pixel sees, not of unit length: z is 1 for a pinhole camera. */
RantMathResult i_math_lens_ray(i_MathLens *lens, RantDouble2 pixel, RantDouble3 *optical);

/* A ScalarKind's bytes, 0 for no kind, and one number of it read or written little endian.
 * A write rounds a whole kind to nearest and clamps it, NaN to 0. */
int    i_math_scalar_bytes(RantScalarKind k);
double i_math_scalar_get(RantScalarKind k, const uint8_t *p);
void   i_math_scalar_put(RantScalarKind k, uint8_t *p, double v);

/* Each map after RANT_COLORMAP_GRAY as 256 sRGB steps, from tools/colormaps.py. */
extern const uint8_t i_math_colormaps[5][256][3];

/* A seeded random source: splitmix64 spreads the seed, then xorshift64 star. */
typedef struct { uint64_t s; } i_MathRng;

static inline i_MathRng i_math_rng(uint64_t seed){
    i_MathRng r;
    uint64_t z = seed + 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    r.s = (z ^ (z >> 31)) | 1u;
    return r;
}

/* In [0, 1). */
static inline double i_math_rng_unit(i_MathRng *r){
    r->s ^= r->s >> 12; r->s ^= r->s << 25; r->s ^= r->s >> 27;
    return (double)((r->s * 0x2545F4914F6CDD1Dull) >> 11) / 9007199254740992.0;
}

/* A 3 by 3 in double, row major, for the work the f32 Matrix3x3 would round. */
typedef struct { double m[3][3]; } i_MathMat3;

i_MathMat3  i_math_mat3_from_quaternion(RantQuaternion q);
RantQuaternion i_math_quaternion_from_mat3(const i_MathMat3 *r);
i_MathMat3  i_math_mat3_multiply(const i_MathMat3 *a, const i_MathMat3 *b);
i_MathMat3  i_math_mat3_transpose(const i_MathMat3 *a);
RantDouble3 i_math_mat3_apply(const i_MathMat3 *a, RantDouble3 v);
/* The symmetric eigen problem of a 3 by 3: values ascending, vectors as columns. */
void        i_math_mat3_eigen(const i_MathMat3 *a, double values[3], i_MathMat3 *vectors);

#endif /* RANT_MATH_INTERNAL_H */
