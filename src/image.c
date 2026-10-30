#include "internal.h"

/* BT.601's luma weights. */
#define I_KR 0.299
#define I_KG 0.587
#define I_KB 0.114
/* Rows go through in runs this many pixels long, even so a Yuyv pair never splits. */
#define I_RUN 256

/* Rounded half up and clamped, NaN to 0. A cast truncates, which for v + 0.5 above 0 is floor. */
static uint8_t i_u8(double v){ return !(v > 0.0) ? 0 : (v >= 255.0 ? 255 : (uint8_t)(v + 0.5)); }
static uint16_t i_u16(double v){ return !(v > 0.0) ? 0 : (v >= 65535.0 ? 65535 : (uint16_t)(v + 0.5)); }

/* The bytes of one row's pixels, 0 for a format with no raw pixels. Nv12's chroma rows are the
 * longer ones when the width is odd. */
static uint64_t i_line(RantImageFormat f, uint32_t w){
    switch (f){
    case RANT_IMAGE_MONO8: return w;
    case RANT_IMAGE_MONO16: return 2ull * w;
    case RANT_IMAGE_RGB8: case RANT_IMAGE_BGR8: return 3ull * w;
    case RANT_IMAGE_RGBA8: case RANT_IMAGE_MONOF32: return 4ull * w;
    case RANT_IMAGE_YUYV: return 4ull * ((w + 1ull) / 2);
    case RANT_IMAGE_NV12: return 2ull * ((w + 1ull) / 2);
    default: return 0;
    }
}
static uint64_t i_rows(RantImageFormat f, uint32_t h){ return f == RANT_IMAGE_NV12 ? h + (h + 1ull) / 2 : h; }

size_t rant_image_bytes(RantImageFormat format, uint32_t width, uint32_t height){
    uint64_t line = i_line(format, width), rows = i_rows(format, height);
    if (!line || !rows || line > UINT64_MAX / rows || line * rows > SIZE_MAX) return 0;
    return (size_t)(line * rows);
}

/* What every view must pass, and its stride with 0 read as packed rows. */
static RantMathResult i_view(const RantImageView *v, size_t *stride){
    uint64_t line, rows, s;
    if (!v) return RANT_MATH_ERR_ARGUMENT;
    line = i_line(v->format, v->width);
    rows = i_rows(v->format, v->height);
    if (!v->data || !line || !rows) return RANT_MATH_ERR_FORMAT;
    s = v->stride ? v->stride : line;
    /* the last row needs its pixels, not its padding */
    if (s < line || rows - 1 > (UINT64_MAX - line) / s || s * (rows - 1) + line > v->size) return RANT_MATH_ERR_FORMAT;
    *stride = (size_t)s;
    return RANT_MATH_OK;
}

RantMathResult rant_image_check(const RantImageView *image){
    size_t s;
    return i_view(image, &s);
}

static int i_apart(const RantImageView *a, const RantImageView *b){
    uintptr_t p = (uintptr_t)a->data, q = (uintptr_t)b->data;
    return p + a->size <= q || q + b->size <= p;
}

static int i_colour(RantImageFormat f){
    return f == RANT_IMAGE_MONO8 || f == RANT_IMAGE_RGB8 || f == RANT_IMAGE_RGBA8 || f == RANT_IMAGE_BGR8
        || f == RANT_IMAGE_YUYV || f == RANT_IMAGE_NV12;
}

/* A plain format's channels and the bytes of each. */
static int i_plain(RantImageFormat f, int *channels, int *bytes){
    switch (f){
    case RANT_IMAGE_MONO8: *channels = 1; *bytes = 1; return 1;
    case RANT_IMAGE_MONO16: *channels = 1; *bytes = 2; return 1;
    case RANT_IMAGE_MONOF32: *channels = 1; *bytes = 4; return 1;
    case RANT_IMAGE_RGB8: case RANT_IMAGE_BGR8: *channels = 3; *bytes = 1; return 1;
    case RANT_IMAGE_RGBA8: *channels = 4; *bytes = 1; return 1;
    default: return 0;
    }
}

/* One channel as a number and back, whole kinds rounded and clamped. */
static double i_get(int bytes, const uint8_t *p){
    uint32_t u;
    float f;
    if (bytes == 1) return p[0];
    if (bytes == 2) return (double)(p[0] | p[1] << 8);
    u = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
    memcpy(&f, &u, sizeof f);
    return f;
}
static void i_put(int bytes, uint8_t *p, double v){
    uint32_t u;
    float f;
    if (bytes == 1){ p[0] = i_u8(v); return; }
    if (bytes == 2){ u = i_u16(v); p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); return; }
    f = (float)v;
    memcpy(&u, &f, sizeof u);
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}

static void i_to_yuv(RantColor c, double *y, double *u, double *v){
    double l = I_KR * c.r + I_KG * c.g + I_KB * c.b;
    *y = 16.0 + l * (219.0 / 255.0);
    *u = 128.0 + (c.b - l) * (224.0 / 255.0) / (2.0 * (1.0 - I_KB));
    *v = 128.0 + (c.r - l) * (224.0 / 255.0) / (2.0 * (1.0 - I_KR));
}
static uint8_t i_luma(RantColor c){ return i_u8(I_KR * c.r + I_KG * c.g + I_KB * c.b); }

/* n pixels of row y from x0 on, in a colour format. */
static void i_read(const RantImageView *v, size_t stride, uint32_t y, uint32_t x0, uint32_t n, RantColor *out){
    const uint8_t *row = v->data + (size_t)y * stride, *p;
    const uint8_t *chroma = v->data + ((size_t)v->height + y / 2) * stride;
    double rv = 0.0, gu = 0.0, gv = 0.0, bu = 0.0, l;
    uint32_t i, x;
    switch (v->format){
    case RANT_IMAGE_MONO8:
        for (i = 0; i < n; i++) out[i] = rant_color(row[x0 + i], row[x0 + i], row[x0 + i], 255);
        break;
    case RANT_IMAGE_RGB8:
        for (i = 0, p = row + 3 * (size_t)x0; i < n; i++, p += 3) out[i] = rant_color(p[0], p[1], p[2], 255);
        break;
    case RANT_IMAGE_BGR8:
        for (i = 0, p = row + 3 * (size_t)x0; i < n; i++, p += 3) out[i] = rant_color(p[2], p[1], p[0], 255);
        break;
    case RANT_IMAGE_RGBA8:
        for (i = 0, p = row + 4 * (size_t)x0; i < n; i++, p += 4) out[i] = rant_color(p[0], p[1], p[2], p[3]);
        break;
    case RANT_IMAGE_YUYV: case RANT_IMAGE_NV12:
        for (i = 0; i < n; i++){
            x = x0 + i;
            /* each pair of pixels shares its chroma, so its terms are made once */
            if (i == 0 || !(x & 1)){
                const uint8_t *c = v->format == RANT_IMAGE_YUYV ? row + 4 * (size_t)(x / 2) + 1 : chroma + 2 * (size_t)(x / 2);
                double pb = (c[0] - 128) * (255.0 / 224.0), pr = (c[v->format == RANT_IMAGE_YUYV ? 2 : 1] - 128) * (255.0 / 224.0);
                rv = 2.0 * (1.0 - I_KR) * pr;
                gu = 2.0 * I_KB * (1.0 - I_KB) / I_KG * pb;
                gv = 2.0 * I_KR * (1.0 - I_KR) / I_KG * pr;
                bu = 2.0 * (1.0 - I_KB) * pb;
            }
            l = ((v->format == RANT_IMAGE_YUYV ? row[4 * (size_t)(x / 2) + 2 * (x & 1)] : row[x]) - 16) * (255.0 / 219.0);
            out[i] = rant_color(i_u8(l + rv), i_u8(l - gu - gv), i_u8(l + bu), 255);
        }
        break;
    default: break;
    }
}

/* n pixels of row y from x0 on from top, and for Nv12 of row y + 1 from bottom, NULL past the
 * last row. A chroma sample takes the mean of the pixels that share it. */
static void i_write(RantImageView *v, size_t stride, uint32_t y, uint32_t x0, uint32_t n,
                    const RantColor *top, const RantColor *bottom){
    uint8_t *row = v->data + (size_t)y * stride, *p;
    double yy[4], uu[4], vv[4];
    uint32_t i, x;
    int b, a, k;
    if (v->format == RANT_IMAGE_YUYV || v->format == RANT_IMAGE_NV12){
        for (i = 0; i < n; i += 2){
            double su = 0.0, sv = 0.0;
            k = 0;
            for (b = 0; b < (v->format == RANT_IMAGE_NV12 && bottom ? 2 : 1); b++)
                for (a = 0; a < 2 && i + a < n; a++, k++){
                    i_to_yuv((b ? bottom : top)[i + a], &yy[k], &uu[k], &vv[k]);
                    su += uu[k];
                    sv += vv[k];
                }
            x = x0 + i;
            if (v->format == RANT_IMAGE_YUYV){
                p = row + 2 * (size_t)x;
                p[0] = i_u8(yy[0]);
                p[1] = i_u8(su / k);
                p[2] = i_u8(i + 1 < n ? yy[1] : yy[0]);   /* an odd width's last pair repeats its pixel */
                p[3] = i_u8(sv / k);
            } else {
                k = 0;
                for (b = 0; b < (bottom ? 2 : 1); b++)
                    for (a = 0; a < 2 && i + a < n; a++, k++) row[(size_t)b * stride + x + a] = i_u8(yy[k]);
                p = v->data + ((size_t)v->height + y / 2) * stride + x;
                p[0] = i_u8(su / k);
                p[1] = i_u8(sv / k);
            }
        }
        return;
    }
    for (i = 0; i < n; i++){
        RantColor c = top[i];
        x = x0 + i;
        switch (v->format){
        case RANT_IMAGE_MONO8: row[x] = i_luma(c); break;
        case RANT_IMAGE_RGB8: p = row + 3 * (size_t)x; p[0] = c.r; p[1] = c.g; p[2] = c.b; break;
        case RANT_IMAGE_BGR8: p = row + 3 * (size_t)x; p[0] = c.b; p[1] = c.g; p[2] = c.r; break;
        case RANT_IMAGE_RGBA8: p = row + 4 * (size_t)x; p[0] = c.r; p[1] = c.g; p[2] = c.b; p[3] = c.a; break;
        default: break;
        }
    }
}

/* Fills dst in a colour format with what read gives for each run of a row. */
typedef void (*i_Source)(const void *from, uint32_t y, uint32_t x0, uint32_t n, RantColor *out);

static void i_paint(RantImageView *dst, size_t stride, i_Source read, const void *from){
    RantColor top[I_RUN], bottom[I_RUN];
    uint64_t y, x0, step = dst->format == RANT_IMAGE_NV12 ? 2 : 1;
    for (y = 0; y < dst->height; y += step)
        for (x0 = 0; x0 < dst->width; x0 += I_RUN){
            uint32_t n = (uint32_t)(dst->width - x0 < I_RUN ? dst->width - x0 : I_RUN);
            int two = step == 2 && y + 1 < dst->height;
            read(from, (uint32_t)y, (uint32_t)x0, n, top);
            if (two) read(from, (uint32_t)y + 1, (uint32_t)x0, n, bottom);
            i_write(dst, stride, (uint32_t)y, (uint32_t)x0, n, top, two ? bottom : NULL);
        }
}

typedef struct { const RantImageView *v; size_t stride; double low, span; RantColormap map; int bytes; } i_From;

static void i_pixels(const void *from, uint32_t y, uint32_t x0, uint32_t n, RantColor *out){
    const i_From *f = (const i_From *)from;
    i_read(f->v, f->stride, y, x0, n, out);
}
static void i_values(const void *from, uint32_t y, uint32_t x0, uint32_t n, RantColor *out){
    const i_From *f = (const i_From *)from;
    const uint8_t *row = f->v->data + (size_t)y * f->stride;
    uint32_t i;
    for (i = 0; i < n; i++)
        out[i] = rant_colormap_at(f->map, (i_get(f->bytes, row + (size_t)(x0 + i) * f->bytes) - f->low) / f->span);
}

RantMathResult rant_image_color_at(const RantImageView *image, uint32_t x, uint32_t y, RantColor *out){
    RantMathResult r;
    size_t s;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_view(image, &s)) != RANT_MATH_OK) return r;
    if (!i_colour(image->format)) return RANT_MATH_ERR_FORMAT;
    if (x >= image->width || y >= image->height) return RANT_MATH_ERR_ARGUMENT;
    i_read(image, s, y, x, 1, out);
    return RANT_MATH_OK;
}

RantMathResult rant_image_value_at(const RantImageView *image, uint32_t x, uint32_t y, double *out){
    RantMathResult r;
    size_t s;
    int channels, bytes;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_view(image, &s)) != RANT_MATH_OK) return r;
    if (!i_plain(image->format, &channels, &bytes) || channels != 1) return RANT_MATH_ERR_FORMAT;
    if (x >= image->width || y >= image->height) return RANT_MATH_ERR_ARGUMENT;
    *out = i_get(bytes, image->data + (size_t)y * s + (size_t)x * bytes);
    return RANT_MATH_OK;
}

/* Checks a source and its destination of the given size. */
static RantMathResult i_two_images(const RantImageView *src, size_t *ss, const RantImageView *dst, size_t *ds,
                                   uint32_t width, uint32_t height){
    RantMathResult r;
    if ((r = i_view(src, ss)) != RANT_MATH_OK || (r = i_view(dst, ds)) != RANT_MATH_OK) return r;
    if (dst->width != width || dst->height != height || !i_apart(src, dst)) return RANT_MATH_ERR_ARGUMENT;
    return RANT_MATH_OK;
}

RantMathResult rant_image_convert(const RantImageView *src, RantImageView *dst){
    RantMathResult r;
    size_t ss, ds;
    i_From from;
    if ((r = i_two_images(src, &ss, dst, &ds, src ? src->width : 0, src ? src->height : 0)) != RANT_MATH_OK) return r;
    if (src->format == dst->format){
        uint64_t y, rows = i_rows(src->format, src->height);
        /* only the pixels, so dst's padding stays as it was */
        for (y = 0; y < rows; y++)
            memcpy(dst->data + (size_t)y * ds, src->data + (size_t)y * ss,
                   (size_t)(src->format == RANT_IMAGE_NV12 && y < src->height ? src->width : i_line(src->format, src->width)));
        return RANT_MATH_OK;
    }
    if (!i_colour(src->format) || !i_colour(dst->format)) return RANT_MATH_ERR_FORMAT;
    from.v = src;
    from.stride = ss;
    i_paint(dst, ds, i_pixels, &from);
    return RANT_MATH_OK;
}

RantMathResult rant_image_colorize(const RantImageView *src, double low, double high, RantColormap map, RantImageView *dst){
    RantMathResult r;
    size_t ss, ds;
    int channels;
    i_From from;
    if ((r = i_two_images(src, &ss, dst, &ds, src ? src->width : 0, src ? src->height : 0)) != RANT_MATH_OK) return r;
    if (!i_plain(src->format, &channels, &from.bytes) || channels != 1 || !i_colour(dst->format)) return RANT_MATH_ERR_FORMAT;
    if (!i_math_finite(low) || !i_math_finite(high) || low == high || (int)map < 0 || (int)map > (int)RANT_COLORMAP_TURBO)
        return RANT_MATH_ERR_ARGUMENT;
    from.v = src;
    from.stride = ss;
    from.low = low;
    from.span = high - low;
    from.map = map;
    i_paint(dst, ds, i_values, &from);
    return RANT_MATH_OK;
}

RantMathResult rant_image_crop(const RantImageView *image, uint32_t x, uint32_t y, uint32_t width, uint32_t height,
                               RantImageView *out){
    RantMathResult r;
    RantImageView o;
    size_t s, at;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_view(image, &s)) != RANT_MATH_OK) return r;
    if (image->format == RANT_IMAGE_NV12) return RANT_MATH_ERR_FORMAT;
    if (!width || !height || (uint64_t)x + width > image->width || (uint64_t)y + height > image->height
        || (image->format == RANT_IMAGE_YUYV && ((x & 1) || ((width & 1) && x + width != image->width))))
        return RANT_MATH_ERR_ARGUMENT;
    if (s > UINT32_MAX) return RANT_MATH_ERR_FORMAT;
    at = (size_t)y * s + (size_t)i_line(image->format, x);
    o = *image;
    o.width = width;
    o.height = height;
    o.stride = (uint32_t)s;
    o.data = image->data + at;
    o.size = image->size - at;
    *out = o;
    return RANT_MATH_OK;
}

/* dst from src pixel by pixel: mirror reverses src's x, then turns clockwise quarter turns. */
static RantMathResult i_turn(const RantImageView *src, int turns, int mirror, RantImageView *dst){
    RantMathResult r;
    size_t ss, ds, bpp;
    uint32_t w, h, dx, dy, sx = 0, sy = 0;
    int channels, bytes;
    turns = ((turns % 4) + 4) % 4;
    w = src ? src->width : 0;
    h = src ? src->height : 0;
    if ((r = i_two_images(src, &ss, dst, &ds, turns & 1 ? h : w, turns & 1 ? w : h)) != RANT_MATH_OK) return r;
    if (!i_plain(src->format, &channels, &bytes)) return RANT_MATH_ERR_FORMAT;
    if (dst->format != src->format) return RANT_MATH_ERR_ARGUMENT;
    bpp = (size_t)channels * bytes;
    for (dy = 0; dy < dst->height; dy++){
        uint8_t *row = dst->data + (size_t)dy * ds;
        /* a row that lands whole and unreversed moves at once */
        if ((turns == 0 && !mirror) || (turns == 2 && mirror)){
            memcpy(row, src->data + (size_t)(turns ? h - 1 - dy : dy) * ss, bpp * w);
            continue;
        }
        for (dx = 0; dx < dst->width; dx++){
            switch (turns){
            case 0: sx = dx; sy = dy; break;
            case 1: sx = dy; sy = h - 1 - dx; break;
            case 2: sx = w - 1 - dx; sy = h - 1 - dy; break;
            default: sx = w - 1 - dy; sy = dx; break;
            }
            if (mirror) sx = w - 1 - sx;
            memcpy(row + dx * bpp, src->data + (size_t)sy * ss + sx * bpp, bpp);
        }
    }
    return RANT_MATH_OK;
}

RantMathResult rant_image_flip_horizontal(const RantImageView *src, RantImageView *dst){ return i_turn(src, 0, 1, dst); }
RantMathResult rant_image_flip_vertical(const RantImageView *src, RantImageView *dst){ return i_turn(src, 2, 1, dst); }
RantMathResult rant_image_rotate(const RantImageView *src, int quarter_turns, RantImageView *dst){
    return i_turn(src, quarter_turns, 0, dst);
}

/* How one axis of a resize reads its source: the nearest pixel, two by their distance, or every
 * pixel under the area a shrinking pixel covers, each by how much of it lies there. */
typedef struct { double scale; uint32_t size; int mode; } i_Axis;

static i_Axis i_axis(uint32_t from, uint32_t to, RantImageFilter f){
    i_Axis a;
    a.size = from;
    a.scale = (double)from / to;
    a.mode = f == RANT_FILTER_NEAREST ? 0 : (from > to ? 2 : 1);
    return a;
}
static uint32_t i_clamp_index(double i, uint32_t size){ return i <= 0.0 ? 0 : (i >= size - 1.0 ? size - 1 : (uint32_t)i); }

/* The source pixels lo to hi that pixel d reads. */
static void i_span(const i_Axis *a, uint32_t d, uint32_t *lo, uint32_t *hi){
    double c;
    if (a->mode == 0){
        *lo = *hi = i_clamp_index(floor((d + 0.5) * a->scale), a->size);
    } else if (a->mode == 1){
        c = floor((d + 0.5) * a->scale - 0.5);
        *lo = i_clamp_index(c, a->size);
        *hi = i_clamp_index(c + 1.0, a->size);
    } else {
        *lo = i_clamp_index(floor(d * a->scale), a->size);
        *hi = i_clamp_index(ceil((d + 1.0) * a->scale) - 1.0, a->size);
    }
}
static double i_tap(const i_Axis *a, uint32_t d, uint32_t i){
    double c, f, w = 0.0;
    if (a->mode == 0) return 1.0;
    if (a->mode == 1){
        c = (d + 0.5) * a->scale - 0.5;
        f = c - floor(c);
        if (i_clamp_index(floor(c), a->size) == i) w += 1.0 - f;
        if (i_clamp_index(floor(c) + 1.0, a->size) == i) w += f;
        return w;
    }
    return i_math_max(0.0, i_math_min((d + 1.0) * a->scale, i + 1.0) - i_math_max(d * a->scale, (double)i));
}

RantMathResult rant_image_resize(const RantImageView *src, RantImageFilter filter, RantImageView *dst){
    RantMathResult r;
    size_t ss, ds, bpp;
    uint32_t dx, dy, xlo, xhi, ylo, yhi, sx, sy;
    int channels, bytes, c;
    i_Axis ax, ay;
    if (!src || !dst) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_two_images(src, &ss, dst, &ds, dst->width, dst->height)) != RANT_MATH_OK) return r;
    if (!i_plain(src->format, &channels, &bytes)) return RANT_MATH_ERR_FORMAT;
    if (dst->format != src->format || (filter != RANT_FILTER_NEAREST && filter != RANT_FILTER_LINEAR)) return RANT_MATH_ERR_ARGUMENT;
    bpp = (size_t)channels * bytes;
    ax = i_axis(src->width, dst->width, filter);
    ay = i_axis(src->height, dst->height, filter);
    for (dy = 0; dy < dst->height; dy++){
        i_span(&ay, dy, &ylo, &yhi);
        for (dx = 0; dx < dst->width; dx++){
            double sum[4] = { 0.0, 0.0, 0.0, 0.0 }, total = 0.0;
            i_span(&ax, dx, &xlo, &xhi);
            for (sy = ylo; sy <= yhi; sy++){
                double wy = i_tap(&ay, dy, sy);
                if (wy == 0.0) continue;
                for (sx = xlo; sx <= xhi; sx++){
                    double wt = wy * i_tap(&ax, dx, sx);
                    const uint8_t *p = src->data + (size_t)sy * ss + sx * bpp;
                    if (wt == 0.0) continue;
                    for (c = 0; c < channels; c++) sum[c] += wt * i_get(bytes, p + (size_t)c * bytes);
                    total += wt;
                }
            }
            for (c = 0; c < channels; c++) i_put(bytes, dst->data + (size_t)dy * ds + dx * bpp + (size_t)c * bytes, sum[c] / total);
        }
    }
    return RANT_MATH_OK;
}

RantMathResult rant_image_undistort(const RantImageView *src, const RantCameraIntrinsics *camera,
                                    const RantCameraIntrinsics *ideal, RantImageFilter filter, RantImageView *dst){
    RantMathResult r;
    RantCameraIntrinsics pinhole;
    i_MathLens from, to;
    size_t ss, ds, bpp;
    uint32_t dx, dy;
    int channels, bytes, c;
    if (!src || !dst) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_two_images(src, &ss, dst, &ds, dst->width, dst->height)) != RANT_MATH_OK) return r;
    if (!i_plain(src->format, &channels, &bytes)) return RANT_MATH_ERR_FORMAT;
    if ((r = i_math_lens(camera, 1, &from)) != RANT_MATH_OK) return r;
    if (!ideal){
        pinhole = *camera;
        pinhole.model = RANT_DISTORTION_NONE;
        memset(pinhole.coeffs, 0, sizeof pinhole.coeffs);
        ideal = &pinhole;
    }
    if ((r = i_math_lens(ideal, 1, &to)) != RANT_MATH_OK) return r;
    if (dst->format != src->format || (filter != RANT_FILTER_NEAREST && filter != RANT_FILTER_LINEAR)
        || src->width != camera->width || src->height != camera->height
        || dst->width != ideal->width || dst->height != ideal->height) return RANT_MATH_ERR_ARGUMENT;
    bpp = (size_t)channels * bytes;
    for (dy = 0; dy < dst->height; dy++)
        for (dx = 0; dx < dst->width; dx++){
            uint8_t *out = dst->data + (size_t)dy * ds + dx * bpp;
            RantDouble3 o;
            RantDouble2 p;
            if (i_math_lens_ray(&to, rant_double2(dx, dy), &o) != RANT_MATH_OK
                || i_math_lens_project(&from, o, &p) != RANT_MATH_OK
                || !(p.x >= -0.5 && p.x < src->width - 0.5 && p.y >= -0.5 && p.y < src->height - 0.5)){
                for (c = 0; c < channels; c++) i_put(bytes, out + (size_t)c * bytes, bytes == 4 ? NAN : 0.0);
                continue;
            }
            if (filter == RANT_FILTER_NEAREST){
                const uint8_t *q = src->data + (size_t)i_clamp_index(floor(p.y + 0.5), src->height) * ss
                                 + i_clamp_index(floor(p.x + 0.5), src->width) * bpp;
                memcpy(out, q, bpp);
            } else {
                double x0 = floor(p.x), y0 = floor(p.y), fx = p.x - x0, fy = p.y - y0, sum[4] = { 0.0, 0.0, 0.0, 0.0 };
                double w[4];
                uint32_t xs[2], ys[2];
                int k;
                xs[0] = i_clamp_index(x0, src->width); xs[1] = i_clamp_index(x0 + 1.0, src->width);
                ys[0] = i_clamp_index(y0, src->height); ys[1] = i_clamp_index(y0 + 1.0, src->height);
                w[0] = (1.0 - fx) * (1.0 - fy); w[1] = fx * (1.0 - fy); w[2] = (1.0 - fx) * fy; w[3] = fx * fy;
                for (k = 0; k < 4; k++){
                    const uint8_t *q = src->data + (size_t)ys[k / 2] * ss + xs[k % 2] * bpp;
                    if (w[k] == 0.0) continue;
                    for (c = 0; c < channels; c++) sum[c] += w[k] * i_get(bytes, q + (size_t)c * bytes);
                }
                for (c = 0; c < channels; c++) i_put(bytes, out + (size_t)c * bytes, sum[c]);
            }
        }
    return RANT_MATH_OK;
}

/* color over d by a, straight alpha. */
static uint8_t i_over(uint8_t c, uint8_t d, double a){ return i_u8(c * a + d * (1.0 - a)); }

RantMathResult rant_image_draw(RantImageView *image, const RantShape2D *shape, RantColor color, double width){
    RantMathResult r;
    RantAlignedBox2D b;
    size_t s;
    double half, x0, x1, y0, y1, x, y;
    int fill;
    if ((r = i_view(image, &s)) != RANT_MATH_OK) return r;
    if (image->format != RANT_IMAGE_MONO8 && image->format != RANT_IMAGE_RGB8 && image->format != RANT_IMAGE_RGBA8
        && image->format != RANT_IMAGE_BGR8) return RANT_MATH_ERR_FORMAT;
    if (!shape || !(width >= 0.0) || !i_math_finite(width)) return RANT_MATH_ERR_ARGUMENT;
    if (width == 0.0 && (shape->kind == RANT_SHAPE2D_SEGMENT || shape->kind == RANT_SHAPE2D_POLYLINE)) width = 1.0;
    fill = width == 0.0;
    half = 0.5 * width;
    b = rant_shape2d_bounds(shape);
    /* the pixels whose centers lie within reach of the shape's edge */
    x0 = i_math_max(ceil(b.min.x - half - 1.0), 0.0);
    y0 = i_math_max(ceil(b.min.y - half - 1.0), 0.0);
    x1 = i_math_min(floor(b.max.x + half + 1.0), image->width - 1.0);
    y1 = i_math_min(floor(b.max.y + half + 1.0), image->height - 1.0);
    for (y = y0; y <= y1; y++)
        for (x = x0; x <= x1; x++){
            double d = rant_shape2d_distance(shape, rant_double2(x, y));
            double a = i_math_clamp(fill ? 0.5 - d : half + 0.5 - fabs(d), 0.0, 1.0) * (color.a / 255.0);
            uint8_t *p = image->data + (size_t)y * s + (size_t)x * (size_t)i_line(image->format, 1);
            if (!(a > 0.0)) continue;
            switch (image->format){
            case RANT_IMAGE_MONO8: p[0] = i_over(i_luma(color), p[0], a); break;
            case RANT_IMAGE_RGB8: p[0] = i_over(color.r, p[0], a); p[1] = i_over(color.g, p[1], a); p[2] = i_over(color.b, p[2], a); break;
            case RANT_IMAGE_BGR8: p[0] = i_over(color.b, p[0], a); p[1] = i_over(color.g, p[1], a); p[2] = i_over(color.r, p[2], a); break;
            default: {
                double under = p[3] / 255.0 * (1.0 - a), total = a + under;
                p[0] = i_u8((color.r * a + p[0] * under) / total);
                p[1] = i_u8((color.g * a + p[1] * under) / total);
                p[2] = i_u8((color.b * a + p[2] * under) / total);
                p[3] = i_u8(total * 255.0);
            }
            }
        }
    return RANT_MATH_OK;
}
