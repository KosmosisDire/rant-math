/* Objects found by a detector, as Detection2D and Detection3D hold them. */
#ifndef RANT_MATH_DETECTION_H
#define RANT_MATH_DETECTION_H

#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/* label has label_len bytes and needs no NUL. */
typedef struct {
    RantOrientedBox2D bounds;
    const char *label;
    size_t label_len;
    double score;
} RantDetection2DView;

typedef struct {
    RantOrientedBox bounds;
    const char *label;
    size_t label_len;
    double score;
} RantDetection3DView;

/* Non maximum suppression: keep gets 0 for each detection whose IoU with a kept one of the same
 * label and a higher score, or as high and earlier, is above iou. Labels left empty all match. */
RANT_MATH_API RantMathResult rant_detections2d_suppress(const RantDetection2DView *detections, uint32_t count, double iou,
                                                        uint8_t *keep);
RANT_MATH_API RantMathResult rant_detections3d_suppress(const RantDetection3DView *detections, uint32_t count, double iou,
                                                        uint8_t *keep);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_DETECTION_H */
