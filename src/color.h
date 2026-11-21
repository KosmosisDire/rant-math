/* Colours as Color holds them: sRGB bytes with straight alpha. A mix runs through OKLab, where
 * an even step looks even and a hue holds as it lightens, as the HMI mixes. */
#ifndef RANT_MATH_COLOR_H
#define RANT_MATH_COLOR_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maps from a number to a colour, each as matplotlib gives it in 256 steps. coolwarm runs out
 * both ways from its middle, and twilight and hsv come round to their start, for angles. */
typedef enum {
    RANT_COLORMAP_GRAY, RANT_COLORMAP_VIRIDIS, RANT_COLORMAP_PLASMA, RANT_COLORMAP_INFERNO,
    RANT_COLORMAP_MAGMA, RANT_COLORMAP_TURBO, RANT_COLORMAP_CIVIDIS, RANT_COLORMAP_JET, RANT_COLORMAP_HOT,
    RANT_COLORMAP_BONE, RANT_COLORMAP_COOLWARM, RANT_COLORMAP_TWILIGHT, RANT_COLORMAP_HSV
} RantColormap;

/* From a toward b by t, 0 to 1, alpha along with it. rant_color_mix(c, white, t) lightens. */
RANT_MATH_API RantColor  rant_color_mix(RantColor a, RantColor b, double t);
/* Red, green and blue in linear light, as lighting and blending take them, and alpha, each 0 to 1. */
RANT_MATH_API RantFloat4 rant_color_to_linear(RantColor c);
/* Each value clamped to 0 to 1 and rounded. */
RANT_MATH_API RantColor  rant_color_from_linear(RantFloat4 linear);
/* The colour of a map at t, 0 to 1, picked from its 256 steps as matplotlib picks it. Below 0
 * and above 1 take the ends, and NaN takes transparent black. An unknown map gives that too. */
RANT_MATH_API RantColor  rant_colormap_at(RantColormap map, double t);
/* A map as count colours from its start to its end, a ramp for colorize or a table to draw with. */
RANT_MATH_API RantMathResult rant_colormap_ramp(RantColormap map, RantColor *out, uint32_t count);
/* count colours through stops set evenly from the first to the last, mixed between them as
 * rant_color_mix mixes. At least two of each. */
RANT_MATH_API RantMathResult rant_color_ramp(const RantColor *stops, uint32_t stop_count, RantColor *out, uint32_t count);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_COLOR_H */
