#include "internal.h"

/* 0 to 1 as a byte the way the HMI writes one, with NaN as 0. */
static uint8_t i_byte(double v){
    if (!(v > 0.0)) return 0;
    if (v >= 1.0) return 255;
    return (uint8_t)floor(v * 255.0 + 0.5);
}

static double i_linear(double v){ return v <= 0.04045 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4); }
static double i_gamma(double v){ return v <= 0.0031308 ? 12.92 * i_math_max(v, 0.0) : 1.055 * pow(v, 1.0 / 2.4) - 0.055; }

/* Ottosson's OKLab with the HMI's constants, so both mix to the same bytes. */
typedef struct { double l, a, b, alpha; } i_Lab;

static i_Lab i_lab(RantColor c){
    double r = i_linear(c.r / 255.0), g = i_linear(c.g / 255.0), b = i_linear(c.b / 255.0);
    double l = cbrt(0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b);
    double m = cbrt(0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b);
    double s = cbrt(0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b);
    i_Lab o;
    o.l = 0.2104542553 * l + 0.793617785 * m - 0.0040720468 * s;
    o.a = 1.9779984951 * l - 2.428592205 * m + 0.4505937099 * s;
    o.b = 0.0259040371 * l + 0.7827717662 * m - 0.808675766 * s;
    o.alpha = c.a / 255.0;
    return o;
}

static RantColor i_srgb(i_Lab c){
    double l = c.l + 0.3963377774 * c.a + 0.2158037573 * c.b;
    double m = c.l - 0.1055613458 * c.a - 0.0638541728 * c.b;
    double s = c.l - 0.0894841775 * c.a - 1.291485548 * c.b;
    l = l * l * l; m = m * m * m; s = s * s * s;
    return rant_color(i_byte(i_gamma(4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s)),
                      i_byte(i_gamma(-1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s)),
                      i_byte(i_gamma(-0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s)),
                      i_byte(c.alpha));
}

RantColor rant_color_mix(RantColor a, RantColor b, double t){
    i_Lab p = i_lab(a), q = i_lab(b), o;
    o.l = p.l + (q.l - p.l) * t;
    o.a = p.a + (q.a - p.a) * t;
    o.b = p.b + (q.b - p.b) * t;
    o.alpha = p.alpha + (q.alpha - p.alpha) * t;
    return i_srgb(o);
}

RantFloat4 rant_color_to_linear(RantColor c){
    return rant_float4((float)i_linear(c.r / 255.0), (float)i_linear(c.g / 255.0),
                       (float)i_linear(c.b / 255.0), (float)(c.a / 255.0));
}

RantColor rant_color_from_linear(RantFloat4 v){
    return rant_color(i_byte(i_gamma(v.x)), i_byte(i_gamma(v.y)), i_byte(i_gamma(v.z)), i_byte(v.w));
}

RantColor rant_colormap_at(RantColormap map, double t){
    const uint8_t *p;
    int i;
    if (t != t || (int)map < 0 || (int)map > (int)RANT_COLORMAP_TURBO) return rant_color(0, 0, 0, 0);
    /* matplotlib's rule: step floor(256 t), with t = 1 the last step */
    i = t <= 0.0 ? 0 : (t >= 1.0 ? 255 : (int)(t * 256.0));
    if (i > 255) i = 255;
    if (map == RANT_COLORMAP_GRAY) return rant_color((uint8_t)i, (uint8_t)i, (uint8_t)i, 255);
    p = i_math_colormaps[(int)map - 1][i];
    return rant_color(p[0], p[1], p[2], 255);
}
