/* Point clouds as PointCloud holds them, packed point after point as PCL packs them, and the
 * scans and depth images points come from. A cloud without the fields a call needs is ERR_FORMAT. */
#ifndef RANT_MATH_CLOUD_H
#define RANT_MATH_CLOUD_H

#include "camera.h"
#include "image.h"

#ifdef __cplusplus
extern "C" {
#endif

/* count numbers of kind, offset bytes into each point. name has name_len bytes and needs no NUL. */
typedef struct {
    const char *name;
    size_t name_len;
    uint32_t offset;
    RantScalarKind kind;
    uint32_t count;
} RantPointFieldView;

/* size / point_step points, little endian, data holding a whole number of them. */
typedef struct {
    const RantPointFieldView *fields;
    uint32_t field_count;
    uint32_t point_step;
    uint8_t *data;
    size_t size;                 /* the bytes data holds */
} RantPointCloudView;

typedef enum {
    RANT_POINT_CLOUD_XYZ,            /* x, y and z as f32 */
    RANT_POINT_CLOUD_XYZ_INTENSITY,  /* then intensity as f32 */
    RANT_POINT_CLOUD_XYZ_RGB,        /* then rgb packed into 4 bytes as PCL packs it */
    RANT_POINT_CLOUD_XYZ_NORMAL      /* then normal_x, normal_y and normal_z as f32 */
} RantPointCloudLayout;

/* The fields and point_step of a common layout, with no data yet. Point data at count * point_step bytes. */
RANT_MATH_API RantPointCloudView rant_point_cloud_layout(RantPointCloudLayout layout);
/* OK when every field fits inside a point and data holds a whole number of points. */
RANT_MATH_API RantMathResult rant_point_cloud_check(const RantPointCloudView *cloud);
/* The field of this name, a NUL ended string, or NULL when the cloud has none. */
RANT_MATH_API const RantPointFieldView *rant_point_cloud_field(const RantPointCloudView *cloud, const char *name);

/* Each point's x, y and z, NaN kept. count holds the room in out and gets the points the cloud holds. */
RANT_MATH_API RantMathResult rant_point_cloud_read_points(const RantPointCloudView *cloud, RantDouble3 *out, uint32_t *count);
/* One point each into x, y and z, count the points the cloud holds. Other fields stay as they are. */
RANT_MATH_API RantMathResult rant_point_cloud_write_points(RantPointCloudView *cloud, const RantDouble3 *points, uint32_t count);
/* A field's numbers, its count of them per point, point after point, by the count rule above. */
RANT_MATH_API RantMathResult rant_point_cloud_read_values(const RantPointCloudView *cloud, const char *field, double *out,
                                                          uint32_t *count);
/* Whole kinds are rounded and clamped, NaN to 0. */
RANT_MATH_API RantMathResult rant_point_cloud_write_values(RantPointCloudView *cloud, const char *field, const double *values,
                                                           uint32_t count);
/* From the packed rgba field, or rgb with alpha 255. */
RANT_MATH_API RantMathResult rant_point_cloud_read_colors(const RantPointCloudView *cloud, RantColor *out, uint32_t *count);
RANT_MATH_API RantMathResult rant_point_cloud_write_colors(RantPointCloudView *cloud, const RantColor *colors, uint32_t count);

/* Moves every point by pose, and turns normal_x, normal_y and normal_z with it when the cloud has them. */
RANT_MATH_API RantMathResult rant_point_cloud_transform(RantPointCloudView *cloud, RantPose pose);
/* The points keep marks with 1, whole and in order, into data at point_step bytes each. count holds
 * the points data has room for and gets how many keep marks. */
RANT_MATH_API RantMathResult rant_point_cloud_select(const RantPointCloudView *cloud, const uint8_t *keep, uint8_t *data,
                                                     uint32_t *count);

/* A LaserScan's beams turning from +x toward +y, each beam's echoes together, echoes 0 read as 1. */
typedef struct {
    double angle_min, angle_step;
    double range_min, range_max;
    uint8_t echoes;
    const float *ranges;
    uint32_t range_count;
} RantLaserScanView;

/* Each range as a point in the scan's frame, in the order of ranges. NaN where a range is not
 * finite or lies outside range_min to range_max. count follows the count rule above. */
RANT_MATH_API RantMathResult rant_laser_scan_points(const RantLaserScanView *scan, RantDouble2 *out, uint32_t *count);

/* Each pixel of depth, Mono16 in millimeters or Monof32 in meters along the view, as a point in
 * camera's frame, row after row. NaN where the depth is 0 or not finite. Sizes must match. */
RANT_MATH_API RantMathResult rant_image_depth_points(const RantImageView *depth, const RantCameraIntrinsics *camera,
                                                     RantDouble3 *out, uint32_t *count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_CLOUD_H */
