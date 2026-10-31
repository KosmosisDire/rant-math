/* Point sets worked on by which points lie near which: a kd tree that finds neighbors, and the
 * downsampling, outlier removal, normals, clusters and alignment built on it. */
#ifndef RANT_MATH_NEIGHBORS_H
#define RANT_MATH_NEIGHBORS_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* In every call here a point that is not finite takes no part: never found, kept or labeled. */
typedef struct RantKdTree RantKdTree;

/* A tree over a copy of the points, which need not outlive it. Each point keeps its index. */
RANT_MATH_API RantMathResult rant_kdtree_create(const RantDouble3 *points, uint32_t count, RantKdTree **out);
RANT_MATH_API void           rant_kdtree_destroy(RantKdTree *tree);
/* The count points nearest point, nearest first and ties by index, their distances when distances
 * is not NULL. count gets how many it found, fewer when the tree holds fewer. */
RANT_MATH_API RantMathResult rant_kdtree_nearest(const RantKdTree *tree, RantDouble3 point, uint32_t *indices,
                                                 double *distances, uint32_t *count);
/* Every point no farther than radius, nearest first. count holds the room and gets how many there are. */
RANT_MATH_API RantMathResult rant_kdtree_within(const RantKdTree *tree, RantDouble3 point, double radius,
                                                uint32_t *indices, double *distances, uint32_t *count);

/* keep gets 1 for one point of each cube of edge voxel, from the frame origin, that points fall
 * in: the one nearest the mean of that cube's points. */
RANT_MATH_API RantMathResult rant_points_downsample(const RantDouble3 *points, uint32_t count, double voxel, uint8_t *keep);
/* keep gets 0 where a point's mean distance to its neighbors nearest others lies more than deviations
 * standard deviations above the mean of all of them, as PCL's statistical outlier removal does. */
RANT_MATH_API RantMathResult rant_points_remove_outliers(const RantDouble3 *points, uint32_t count, uint32_t neighbors,
                                                         double deviations, uint8_t *keep);
/* keep gets 0 where a point has fewer than neighbors others within radius, else 1. */
RANT_MATH_API RantMathResult rant_points_remove_isolated(const RantDouble3 *points, uint32_t count, double radius,
                                                         uint32_t neighbors, uint8_t *keep);
/* The unit normal of the plane through each point's neighbors nearest points, itself among them,
 * facing viewpoint, where the sensor sat. NaN where they fix no plane. */
RANT_MATH_API RantMathResult rant_points_normals(const RantDouble3 *points, uint32_t count, uint32_t neighbors,
                                                 RantDouble3 viewpoint, RantDouble3 *normals);
/* A point within distance of a cluster's point joins it. labels gets each point's cluster from 1,
 * largest first, or 0 for one under min_points. clusters gets how many have a label. */
RANT_MATH_API RantMathResult rant_points_clusters(const RantDouble3 *points, uint32_t count, double distance,
                                                  uint32_t min_points, uint32_t *labels, uint32_t *clusters);

/* How rant_points_align pairs points. All zero, or a NULL pointer, pairs each with its nearest. */
typedef struct {
    /* Above 0, a source point with no target point this near is left out. */
    double max_distance;
    /* The most rounds, 0 for 50. It stops sooner once the pose no longer moves. */
    uint32_t iterations;
    /* One per target point: when not NULL, each pair counts by its distance along the normal. */
    const RantDouble3 *normals;
    /* When not NULL, one per source point: 1 when the answer pairs it, else 0. */
    uint8_t *inliers;
    /* When not NULL, gets the RMS distance of the pairs of the answer. */
    double *error;
} RantAlignOptions;

/* The pose that moves source onto target by ICP from guess, so target lies about at
 * apply(out, source). ERR_DEGENERATE when fewer than 3 points pair. */
RANT_MATH_API RantMathResult rant_points_align(const RantDouble3 *source, uint32_t source_count, const RantDouble3 *target,
                                               uint32_t target_count, RantPose guess, const RantAlignOptions *options,
                                               RantPose *out);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_NEIGHBORS_H */
