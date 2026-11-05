/* Raw pictures as Image holds them. Yuyv and Nv12 are BT.601 limited range, Mono16 and Monof32
 * little endian. A call fails with RANT_MATH_ERR_FORMAT for a view whose data its layout overruns. */
#ifndef RANT_MATH_IMAGE_H
#define RANT_MATH_IMAGE_H

#include "camera.h"
#include "color.h"
#include "shape.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Rows from the top, stride bytes apart or packed for 0. Nv12's chroma rows follow its luma
 * rows at the same stride. */
typedef struct {
    uint32_t width, height, stride;
    RantImageFormat format;
    uint8_t *data;
    size_t size;                 /* the bytes data holds */
} RantImageView;

typedef enum {
    RANT_FILTER_NEAREST,
    RANT_FILTER_LINEAR           /* bilinear, or the mean of the area a pixel covers as it shrinks */
} RantImageFilter;

/* The bytes a packed image of this format and size takes, 0 for a compressed one. */
RANT_MATH_API size_t         rant_image_bytes(RantImageFormat format, uint32_t width, uint32_t height);
/* OK when the view holds raw pixels and its data holds them all. */
RANT_MATH_API RantMathResult rant_image_check(const RantImageView *image);

/* A pixel as a colour. Mono16 and Monof32 hold measurements, which only colorize shows. */
RANT_MATH_API RantMathResult rant_image_color_at(const RantImageView *image, uint32_t x, uint32_t y, RantColor *out);
/* The number in a pixel of Mono8, Mono16 or Monof32. */
RANT_MATH_API RantMathResult rant_image_value_at(const RantImageView *image, uint32_t x, uint32_t y, double *out);

/* Into dst's format at the same size, between Mono8, Rgb8, Rgba8, Bgr8, Yuyv and Nv12, or a
 * copy in any raw format. Mono8 from colour is BT.601 luma, and alpha is dropped or opaque. */
RANT_MATH_API RantMathResult rant_image_convert(const RantImageView *src, RantImageView *dst);
/* Mono8, Mono16 or Monof32 values from low to high through a map, into a format convert writes. */
RANT_MATH_API RantMathResult rant_image_colorize(const RantImageView *src, double low, double high, RantColormap map,
                                                 RantImageView *dst);
/* A view of the part from x, y on, sharing image's data. Not Nv12, and Yuyv keeps whole pairs. */
RANT_MATH_API RantMathResult rant_image_crop(const RantImageView *image, uint32_t x, uint32_t y, uint32_t width,
                                             uint32_t height, RantImageView *out);

/* These take Mono8, Mono16, Monof32, Rgb8, Rgba8 and Bgr8, with dst in src's format and apart from it. */
RANT_MATH_API RantMathResult rant_image_flip_horizontal(const RantImageView *src, RantImageView *dst);
RANT_MATH_API RantMathResult rant_image_flip_vertical(const RantImageView *src, RantImageView *dst);
/* Clockwise as seen on screen, any number of quarter turns. */
RANT_MATH_API RantMathResult rant_image_rotate(const RantImageView *src, int quarter_turns, RantImageView *dst);
/* To dst's size, the image's edges matched. */
RANT_MATH_API RantMathResult rant_image_resize(const RantImageView *src, RantImageFilter filter, RantImageView *dst);
/* src from camera as ideal would see it, NULL for camera without distortion. A pixel no source
 * reaches is 0, NaN in Monof32. Depth wants RANT_FILTER_NEAREST, which mixes no edges. */
RANT_MATH_API RantMathResult rant_image_undistort(const RantImageView *src, const RantCameraIntrinsics *camera,
                                                  const RantCameraIntrinsics *ideal, RantImageFilter filter,
                                                  RantImageView *dst);

/* A shape in pixels blended over Mono8, Rgb8, Rgba8 or Bgr8 with a smooth edge. width 0 fills,
 * above 0 draws the outline that wide, and a curve with no inside is drawn 1 wide for 0. */
RANT_MATH_API RantMathResult rant_image_draw(RantImageView *image, const RantShape2D *shape, RantColor color, double width);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_IMAGE_H */
