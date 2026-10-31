#include <stdlib.h>
#include "check.h"
#include "reference.h"

static uint8_t pool_a[1 << 18], pool_b[1 << 18], pool_c[1 << 18];

static RantImageView view(RantImageFormat f, uint32_t w, uint32_t h, uint32_t stride, uint8_t *data){
    RantImageView v;
    v.width = w; v.height = h; v.stride = stride; v.format = f; v.data = data;
    v.size = stride ? (size_t)stride * (f == RANT_IMAGE_FORMAT_NV12 ? h + (h + 1) / 2 : h) : rant_image_bytes(f, w, h);
    return v;
}
static int same_color(RantColor a, RantColor b){ return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a; }
static void fill_random(uint8_t *p, size_t n){
    size_t i;
    for (i = 0; i < n; i++) p[i] = (uint8_t)(random_unit() * 256.0);
}
static void put_f32(uint8_t *p, float f){ memcpy(p, &f, 4); }
static float get_f32(const uint8_t *p){ float f; memcpy(&f, p, 4); return f; }
/* Every pixel of two images in one plain format the same. */
static int same_pixels(const RantImageView *a, const RantImageView *b){
    uint32_t y;
    size_t line = rant_image_bytes(a->format, a->width, 1);
    size_t sa = a->stride ? a->stride : line, sb = b->stride ? b->stride : line;
    if (a->width != b->width || a->height != b->height || a->format != b->format) return 0;
    for (y = 0; y < a->height; y++) if (memcmp(a->data + y * sa, b->data + y * sb, line)) return 0;
    return 1;
}

static void layout(void){
    RantImageView v;
    CHECK(rant_image_bytes(RANT_IMAGE_FORMAT_MONO8, 7, 5) == 35 && rant_image_bytes(RANT_IMAGE_FORMAT_MONO16, 7, 5) == 70
          && rant_image_bytes(RANT_IMAGE_FORMAT_RGB8, 7, 5) == 105 && rant_image_bytes(RANT_IMAGE_FORMAT_RGBA8, 7, 5) == 140
          && rant_image_bytes(RANT_IMAGE_FORMAT_BGR8, 7, 5) == 105 && rant_image_bytes(RANT_IMAGE_FORMAT_MONOF32, 7, 5) == 140, "the plain sizes");
    /* an odd width's last Yuyv pair is whole, and Nv12's chroma rows round up both ways */
    CHECK(rant_image_bytes(RANT_IMAGE_FORMAT_YUYV, 7, 5) == 80 && rant_image_bytes(RANT_IMAGE_FORMAT_NV12, 7, 5) == 64
          && rant_image_bytes(RANT_IMAGE_FORMAT_NV12, 8, 6) == 72, "the YUV sizes");
    CHECK(rant_image_bytes(RANT_IMAGE_FORMAT_JPEG, 7, 5) == 0 && rant_image_bytes((RantImageFormat)99, 7, 5) == 0, "no raw pixels");
    CHECK(rant_image_bytes(RANT_IMAGE_FORMAT_RGB8, 0, 5) == 0 && rant_image_bytes(RANT_IMAGE_FORMAT_RGB8, 5, 0) == 0, "no pixels at all");
    CHECK(rant_image_bytes(RANT_IMAGE_FORMAT_MONOF32, 0xFFFFFFFFu, 0xFFFFFFFFu) == 0, "more than memory holds");

    v = view(RANT_IMAGE_FORMAT_RGB8, 7, 5, 0, pool_a);
    CHECK(rant_image_check(&v) == RANT_MATH_OK, "packed");
    v.size--;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "a byte short");
    v = view(RANT_IMAGE_FORMAT_RGB8, 7, 5, 24, pool_a);
    v.size = 24 * 4 + 21;
    CHECK(rant_image_check(&v) == RANT_MATH_OK, "the last row needs no padding");
    v.size--;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "but all its pixels");
    v.stride = 20;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "a stride shorter than a row");
    v = view(RANT_IMAGE_FORMAT_NV12, 7, 5, 10, pool_a);
    v.size = 10 * 7 + 8;
    CHECK(rant_image_check(&v) == RANT_MATH_OK, "Nv12 with padding: 8 rows, the last 8 bytes long");
    v.size--;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "Nv12 a byte short");
    v = view(RANT_IMAGE_FORMAT_RGB8, 7, 5, 0, NULL);
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "no data");
    v = view(RANT_IMAGE_FORMAT_RGB8, 0, 5, 0, pool_a);
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "no width");
    v = view(RANT_IMAGE_FORMAT_JPEG, 7, 5, 0, pool_a);
    v.size = 100;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "a compressed file");
    v.format = (RantImageFormat)99;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "an unknown format");
    CHECK(rant_image_check(NULL) == RANT_MATH_ERR_ARGUMENT, "no view");
    v = view(RANT_IMAGE_FORMAT_MONO8, 0xFFFFFFFFu, 3, 0xFFFFFFFFu, pool_a);
    v.size = 1000;
    CHECK(rant_image_check(&v) == RANT_MATH_ERR_FORMAT, "a layout far past its data");
}

static void pixels(void){
    RantImageView v;
    RantColor c;
    double n;
    int i, bad = 0;
    pool_a[0] = 0x34; pool_a[1] = 0x12;
    v = view(RANT_IMAGE_FORMAT_MONO16, 1, 1, 0, pool_a);
    CHECK(rant_image_value_at(&v, 0, 0, &n) == RANT_MATH_OK && n == 0x1234, "Mono16 is little endian");
    CHECK(rant_image_color_at(&v, 0, 0, &c) == RANT_MATH_ERR_FORMAT, "a measurement has no colour");
    pool_a[0] = 0x00; pool_a[1] = 0x00; pool_a[2] = 0xC0; pool_a[3] = 0x3F;
    v = view(RANT_IMAGE_FORMAT_MONOF32, 1, 1, 0, pool_a);
    CHECK(rant_image_value_at(&v, 0, 0, &n) == RANT_MATH_OK && n == 1.5, "Monof32 is little endian");
    pool_a[0] = 10; pool_a[1] = 20; pool_a[2] = 30; pool_a[3] = 40; pool_a[4] = 50; pool_a[5] = 60;
    v = view(RANT_IMAGE_FORMAT_RGB8, 2, 1, 0, pool_a);
    CHECK(rant_image_color_at(&v, 1, 0, &c) == RANT_MATH_OK && same_color(c, rant_color(40, 50, 60, 255)), "Rgb8");
    CHECK(rant_image_value_at(&v, 1, 0, &n) == RANT_MATH_ERR_FORMAT, "a colour has no single value");
    v.format = RANT_IMAGE_FORMAT_BGR8;
    CHECK(rant_image_color_at(&v, 1, 0, &c) == RANT_MATH_OK && same_color(c, rant_color(60, 50, 40, 255)), "Bgr8");
    v = view(RANT_IMAGE_FORMAT_RGBA8, 1, 1, 0, pool_a);
    CHECK(rant_image_color_at(&v, 0, 0, &c) == RANT_MATH_OK && same_color(c, rant_color(10, 20, 30, 40)), "Rgba8");
    v = view(RANT_IMAGE_FORMAT_MONO8, 6, 1, 0, pool_a);
    CHECK(rant_image_color_at(&v, 5, 0, &c) == RANT_MATH_OK && same_color(c, rant_color(60, 60, 60, 255))
          && rant_image_value_at(&v, 5, 0, &n) == RANT_MATH_OK && n == 60, "Mono8 is both");
    CHECK(rant_image_color_at(&v, 6, 0, &c) == RANT_MATH_ERR_ARGUMENT && rant_image_value_at(&v, 0, 1, &n) == RANT_MATH_ERR_ARGUMENT,
          "past the edge");
    CHECK(rant_image_color_at(&v, 0, 0, NULL) == RANT_MATH_ERR_ARGUMENT, "no output");
    memcpy(pool_a, REF_YUYV, sizeof REF_YUYV);
    v = view(RANT_IMAGE_FORMAT_YUYV, 8, 4, 0, pool_a);
    for (i = 0; i < 32; i++)
        if (rant_image_color_at(&v, (uint32_t)i % 8, (uint32_t)i / 8, &c) != RANT_MATH_OK
            || !same_color(c, rant_color(REF_YUYV_RGB[3 * i], REF_YUYV_RGB[3 * i + 1], REF_YUYV_RGB[3 * i + 2], 255))) bad++;
    memcpy(pool_a, REF_NV12, sizeof REF_NV12);
    v = view(RANT_IMAGE_FORMAT_NV12, 8, 6, 0, pool_a);
    for (i = 0; i < 48; i++)
        if (rant_image_color_at(&v, (uint32_t)i % 8, (uint32_t)i / 8, &c) != RANT_MATH_OK
            || !same_color(c, rant_color(REF_NV12_RGB[3 * i], REF_NV12_RGB[3 * i + 1], REF_NV12_RGB[3 * i + 2], 255))) bad++;
    CHECK(bad == 0, "%d YUV pixels differ from BT.601 limited range", bad);
}

/* Within the limited range BT.601 means, where OpenCV and the formula agree. */
static int legal(int y, int u, int v){ return y >= 16 && y <= 235 && u >= 16 && u <= 240 && v >= 16 && v <= 240; }
/* The bytes of a row's pixels, Nv12's chroma rows being the longer. */
static uint32_t line_of(RantImageFormat f, uint32_t w){
    return f == RANT_IMAGE_FORMAT_NV12 ? 2 * ((w + 1) / 2) : (uint32_t)rant_image_bytes(f, w, 1);
}

static int most_apart(const uint8_t *a, const uint8_t *b, size_t n){
    size_t i;
    int most = 0;
    for (i = 0; i < n; i++) if (abs(a[i] - b[i]) > most) most = abs(a[i] - b[i]);
    return most;
}

static const RantImageFormat COLOURS[6] = {
    RANT_IMAGE_FORMAT_MONO8, RANT_IMAGE_FORMAT_RGB8, RANT_IMAGE_FORMAT_RGBA8, RANT_IMAGE_FORMAT_BGR8, RANT_IMAGE_FORMAT_YUYV, RANT_IMAGE_FORMAT_NV12
};

static void convert(void){
    RantImageView s, d, m, e;
    int i, j, k, bad = 0;
    uint32_t n;
    /* against the formula written out, and within one of OpenCV's fixed point */
    memcpy(pool_a, REF_YUYV, sizeof REF_YUYV);
    s = view(RANT_IMAGE_FORMAT_YUYV, 8, 4, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 8, 4, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && !memcmp(pool_b, REF_YUYV_RGB, sizeof REF_YUYV_RGB), "Yuyv to Rgb8");
    for (i = 0; i < 32; i++){
        const uint8_t *pair = REF_YUYV + 4 * (i / 2);
        if (legal(pair[(i & 1) * 2], pair[1], pair[3]) && most_apart(pool_b + 3 * i, REF_YUYV_CV + 3 * i, 3) > 1) bad++;
    }
    CHECK(bad == 0, "%d legal Yuyv pixels more than one from OpenCV", bad);
    memcpy(pool_a, REF_NV12, sizeof REF_NV12);
    s = view(RANT_IMAGE_FORMAT_NV12, 8, 6, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 8, 6, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && !memcmp(pool_b, REF_NV12_RGB, sizeof REF_NV12_RGB), "Nv12 to Rgb8");
    for (i = 0; i < 48; i++){
        const uint8_t *uv = REF_NV12 + 48 + 8 * (i / 16) + 2 * ((i % 8) / 2);
        if (legal(REF_NV12[i], uv[0], uv[1]) && most_apart(pool_b + 3 * i, REF_NV12_CV + 3 * i, 3) > 1) bad++;
    }
    CHECK(bad == 0, "%d legal Nv12 pixels more than one from OpenCV", bad);
    memcpy(pool_a, REF_RGB, sizeof REF_RGB);
    s = view(RANT_IMAGE_FORMAT_RGB8, 7, 5, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_MONO8, 7, 5, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && !memcmp(pool_b, REF_RGB_GRAY, sizeof REF_RGB_GRAY), "Rgb8 to Mono8");
    CHECK(most_apart(pool_b, REF_RGB_GRAY_CV, sizeof REF_RGB_GRAY_CV) <= 1, "luma within one of OpenCV");
    d = view(RANT_IMAGE_FORMAT_YUYV, 7, 5, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && !memcmp(pool_b, REF_RGB_YUYV, sizeof REF_RGB_YUYV), "Rgb8 to Yuyv, odd width");
    d = view(RANT_IMAGE_FORMAT_NV12, 7, 5, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK, "Rgb8 to Nv12, odd sizes");
    /* an odd width's luma rows sit 8 apart, as the chroma rows are 8 long */
    for (i = 0; i < 5; i++) if (memcmp(pool_b + 8 * i, REF_RGB_NV12_Y + 7 * i, 7)) bad++;
    CHECK(bad == 0 && !memcmp(pool_b + 40, REF_RGB_NV12_UV, sizeof REF_RGB_NV12_UV), "Nv12 matches the means of its blocks");

    /* any two formats meet as they would through Rgba8, at odd sizes and padded strides */
    for (i = 0; i < 6; i++)
        for (j = 0; j < 6; j++){
            uint32_t stride_s = line_of(COLOURS[i], 9) + 3, stride_d = line_of(COLOURS[j], 9) + 5;
            RantImageView rgb = view(RANT_IMAGE_FORMAT_RGB8, 9, 7, 0, pool_c);
            fill_random(pool_c, 9 * 7 * 3);
            s = view(COLOURS[i], 9, 7, stride_s, pool_a);
            if (rant_image_convert(&rgb, &s) != RANT_MATH_OK){ bad++; continue; }
            d = view(COLOURS[j], 9, 7, stride_d, pool_b);
            memset(pool_b, 0xAB, d.size);
            m = view(RANT_IMAGE_FORMAT_RGBA8, 9, 7, 0, pool_c);
            e = view(COLOURS[j], 9, 7, stride_d, pool_c + 4096);
            memset(e.data, 0xAB, e.size);
            /* the same format copies, where Rgba8 would lose what YUV rounds */
            if (i == j){
                memset(e.data, 0xAB, e.size);
                for (k = 0; k < (int)(COLOURS[j] == RANT_IMAGE_FORMAT_NV12 ? 11 : 7); k++)
                    memcpy(e.data + k * stride_d, s.data + k * stride_s, COLOURS[j] == RANT_IMAGE_FORMAT_NV12 && k < 7 ? 9 : line_of(COLOURS[j], 9));
            } else if (rant_image_convert(&s, &m) != RANT_MATH_OK || rant_image_convert(&m, &e) != RANT_MATH_OK) bad++;
            if (rant_image_convert(&s, &d) != RANT_MATH_OK || memcmp(d.data, e.data, d.size)) bad++;
            /* the padding past each row is left as it was */
            for (k = 0; k < (int)(COLOURS[j] == RANT_IMAGE_FORMAT_NV12 ? 11 : 7); k++)
                for (n = COLOURS[j] == RANT_IMAGE_FORMAT_NV12 && k < 7 ? 9 : line_of(COLOURS[j], 9); n < stride_d; n++)
                    if (pool_b[k * stride_d + n] != 0xAB) bad++;
        }
    CHECK(bad == 0, "%d conversions disagree or touch padding", bad);
    bad = 0;

    pool_a[0] = 10; pool_a[1] = 20; pool_a[2] = 30; pool_a[3] = 40;
    s = view(RANT_IMAGE_FORMAT_RGBA8, 1, 1, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 1, 1, 0, pool_b);
    m = view(RANT_IMAGE_FORMAT_RGBA8, 1, 1, 0, pool_c);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && pool_b[0] == 10 && pool_b[1] == 20 && pool_b[2] == 30
          && rant_image_convert(&d, &m) == RANT_MATH_OK && pool_c[0] == 10 && pool_c[3] == 255, "alpha is dropped, then opaque");

    for (i = 0; i < 256; i++) pool_a[i] = (uint8_t)i;
    s = view(RANT_IMAGE_FORMAT_MONO8, 256, 1, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 256, 1, 0, pool_b);
    m = view(RANT_IMAGE_FORMAT_MONO8, 256, 1, 0, pool_c);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && rant_image_convert(&d, &m) == RANT_MATH_OK && !memcmp(pool_a, pool_c, 256),
          "gray through Rgb8 and back is exact");
    d = view(RANT_IMAGE_FORMAT_YUYV, 256, 1, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && rant_image_convert(&d, &m) == RANT_MATH_OK && most_apart(pool_a, pool_c, 256) <= 1,
          "gray through limited range is within one");
    /* a flat 2 by 2 block keeps its colour through Nv12, within the rounding of three limited range steps */
    bad = 0;
    for (i = 0; i < 200; i++){
        fill_random(pool_a, 3);
        for (k = 1; k < 4; k++) memcpy(pool_a + 3 * k, pool_a, 3);
        s = view(RANT_IMAGE_FORMAT_RGB8, 2, 2, 0, pool_a);
        d = view(RANT_IMAGE_FORMAT_NV12, 2, 2, 0, pool_b);
        m = view(RANT_IMAGE_FORMAT_RGB8, 2, 2, 0, pool_c);
        if (rant_image_convert(&s, &d) != RANT_MATH_OK || rant_image_convert(&d, &m) != RANT_MATH_OK || most_apart(pool_a, pool_c, 12) > 2) bad++;
    }
    CHECK(bad == 0, "%d flat blocks change colour through Nv12", bad);

    s = view(RANT_IMAGE_FORMAT_MONO16, 9, 7, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 9, 7, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_ERR_FORMAT, "a measurement takes colorize");
    s = view(RANT_IMAGE_FORMAT_RGB8, 9, 7, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_MONOF32, 9, 7, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_ERR_FORMAT, "nor does colour make one");
    d = view(RANT_IMAGE_FORMAT_RGB8, 8, 7, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_ERR_ARGUMENT, "another size");
    d = view(RANT_IMAGE_FORMAT_RGBA8, 9, 7, 0, pool_a + 10);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_ERR_ARGUMENT, "overlapping");
    fill_random(pool_a, 4096);
    s = view(RANT_IMAGE_FORMAT_MONO16, 9, 7, 21, pool_a);
    d = view(RANT_IMAGE_FORMAT_MONO16, 9, 7, 0, pool_b);
    CHECK(rant_image_convert(&s, &d) == RANT_MATH_OK && same_pixels(&s, &d), "the same format copies between strides");
}

static void colorize(void){
    RantImageView s, d, m, e;
    const float values[12] = { -1.0f, 0.0f, 0.5f, 1.0f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f, 0.25f, 0.0f, 0.0f };
    int i, bad = 0;
    for (i = 0; i < 12; i++) put_f32(pool_a + 4 * i, values[i]);
    put_f32(pool_a + 40, (float)NAN);
    put_f32(pool_a + 44, (float)HUGE_VAL);
    s = view(RANT_IMAGE_FORMAT_MONOF32, 4, 3, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGBA8, 4, 3, 0, pool_b);
    CHECK(rant_image_colorize(&s, 1.0, 4.0, RANT_COLORMAP_INFERNO, &d) == RANT_MATH_OK, "colorize");
    for (i = 0; i < 12; i++){
        RantColor want = rant_colormap_at(RANT_COLORMAP_INFERNO, (get_f32(pool_a + 4 * i) - 1.0) / 3.0);
        if (!same_color(rant_color(pool_b[4 * i], pool_b[4 * i + 1], pool_b[4 * i + 2], pool_b[4 * i + 3]), want)) bad++;
    }
    CHECK(bad == 0, "%d pixels are not their map's colour", bad);
    CHECK(pool_b[43] == 0 && pool_b[47] == 255, "NaN is transparent and infinity takes the top");
    /* any colour format writes what Rgba8 holds */
    m = view(RANT_IMAGE_FORMAT_NV12, 4, 3, 0, pool_c);
    e = view(RANT_IMAGE_FORMAT_NV12, 4, 3, 0, pool_c + 1024);
    CHECK(rant_image_colorize(&s, 1.0, 4.0, RANT_COLORMAP_INFERNO, &m) == RANT_MATH_OK && rant_image_convert(&d, &e) == RANT_MATH_OK
          && !memcmp(m.data, e.data, m.size), "into Nv12 as through Rgba8");
    for (i = 0; i < 4; i++){ pool_a[2 * i] = (uint8_t)(i * 85); pool_a[2 * i + 1] = (uint8_t)(i * 85); }
    s = view(RANT_IMAGE_FORMAT_MONO16, 4, 1, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_MONO8, 4, 1, 0, pool_b);
    CHECK(rant_image_colorize(&s, 0.0, 65535.0, RANT_COLORMAP_GRAY, &d) == RANT_MATH_OK
          && pool_b[0] == 0 && pool_b[1] == 85 && pool_b[2] == 170 && pool_b[3] == 255, "Mono16 to gray, a step to each 257");
    CHECK(rant_image_colorize(&s, 65535.0, 0.0, RANT_COLORMAP_GRAY, &d) == RANT_MATH_OK && pool_b[0] == 255 && pool_b[3] == 0,
          "low above high turns the map around");
    CHECK(rant_image_colorize(&s, 3.0, 3.0, RANT_COLORMAP_GRAY, &d) == RANT_MATH_ERR_ARGUMENT, "no span");
    CHECK(rant_image_colorize(&s, NAN, 3.0, RANT_COLORMAP_GRAY, &d) == RANT_MATH_ERR_ARGUMENT, "a NaN end");
    CHECK(rant_image_colorize(&s, 0.0, 3.0, (RantColormap)17, &d) == RANT_MATH_ERR_ARGUMENT, "an unknown map");
    s.format = RANT_IMAGE_FORMAT_RGBA8;
    CHECK(rant_image_colorize(&s, 0.0, 3.0, RANT_COLORMAP_GRAY, &d) == RANT_MATH_ERR_FORMAT, "colour has no values to map");
}

static void crop(void){
    RantImageView s, c, cc, d;
    RantColor a, b;
    uint32_t x, y;
    int bad = 0;
    fill_random(pool_a, 9 * 7 * 3);
    s = view(RANT_IMAGE_FORMAT_RGB8, 9, 7, 0, pool_a);
    CHECK(rant_image_crop(&s, 2, 1, 5, 4, &c) == RANT_MATH_OK && c.width == 5 && c.height == 4 && c.stride == 27
          && c.data == pool_a + 27 + 6 && rant_image_check(&c) == RANT_MATH_OK, "a view inside");
    CHECK(rant_image_crop(&c, 1, 1, 4, 3, &cc) == RANT_MATH_OK && rant_image_check(&cc) == RANT_MATH_OK, "a view of a view");
    for (y = 0; y < 3; y++) for (x = 0; x < 4; x++){
        rant_image_color_at(&cc, x, y, &a);
        rant_image_color_at(&s, x + 3, y + 2, &b);
        if (!same_color(a, b)) bad++;
    }
    CHECK(bad == 0, "%d pixels of the view are not the image's", bad);
    CHECK(rant_image_crop(&s, 4, 3, 5, 4, &c) == RANT_MATH_OK && rant_image_check(&c) == RANT_MATH_OK, "to the far corner exactly");
    CHECK(rant_image_crop(&s, 5, 3, 5, 4, &c) == RANT_MATH_ERR_ARGUMENT, "past the right edge");
    CHECK(rant_image_crop(&s, 0, 0, 0, 4, &c) == RANT_MATH_ERR_ARGUMENT, "nothing to keep");
    d = view(RANT_IMAGE_FORMAT_RGB8, 5, 4, 0, pool_b);
    CHECK(rant_image_crop(&s, 2, 1, 5, 4, &c) == RANT_MATH_OK && rant_image_convert(&c, &d) == RANT_MATH_OK, "a view copies out packed");
    fill_random(pool_a, 1024);
    s = view(RANT_IMAGE_FORMAT_YUYV, 9, 3, 0, pool_a);
    CHECK(rant_image_crop(&s, 3, 0, 4, 2, &c) == RANT_MATH_ERR_ARGUMENT, "Yuyv splits no pair at its start");
    CHECK(rant_image_crop(&s, 2, 0, 3, 2, &c) == RANT_MATH_ERR_ARGUMENT, "or at its end, where a write would reach past it");
    CHECK(rant_image_crop(&s, 4, 1, 5, 2, &c) == RANT_MATH_OK && rant_image_check(&c) == RANT_MATH_OK
          && rant_image_color_at(&c, 4, 1, &a) == RANT_MATH_OK && rant_image_color_at(&s, 8, 2, &b) == RANT_MATH_OK
          && same_color(a, b), "Yuyv to its odd edge");
    s = view(RANT_IMAGE_FORMAT_NV12, 8, 4, 0, pool_a);
    CHECK(rant_image_crop(&s, 0, 0, 4, 2, &c) == RANT_MATH_ERR_FORMAT, "Nv12's chroma plane would not follow");
}

static const RantImageFormat PLAIN[6] = {
    RANT_IMAGE_FORMAT_MONO8, RANT_IMAGE_FORMAT_MONO16, RANT_IMAGE_FORMAT_MONOF32, RANT_IMAGE_FORMAT_RGB8, RANT_IMAGE_FORMAT_RGBA8, RANT_IMAGE_FORMAT_BGR8
};

static void turns(void){
    const uint8_t abc[18] = { 'a','a','a', 'b','b','b', 'c','c','c', 'd','d','d', 'e','e','e', 'f','f','f' };
    const uint8_t turned[18] = { 'd','d','d', 'a','a','a', 'e','e','e', 'b','b','b', 'f','f','f', 'c','c','c' };
    RantImageView s, d, e, f;
    int i, k, bad = 0;
    memcpy(pool_a, abc, sizeof abc);
    s = view(RANT_IMAGE_FORMAT_RGB8, 3, 2, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 2, 3, 0, pool_b);
    /* the top row a b c becomes the right column */
    CHECK(rant_image_rotate(&s, 1, &d) == RANT_MATH_OK && !memcmp(pool_b, turned, sizeof turned), "a quarter turn clockwise");
    for (i = 0; i < 6; i++){
        s = view(PLAIN[i], 5, 3, 0, pool_a);
        fill_random(pool_a, s.size);
        d = view(PLAIN[i], 5, 3, 0, pool_b);
        e = view(PLAIN[i], 5, 3, 0, pool_c);
        f = view(PLAIN[i], 5, 3, 0, pool_c + 4096);
        if (rant_image_rotate(&s, 4, &d) != RANT_MATH_OK || !same_pixels(&s, &d)) bad++;
        if (rant_image_rotate(&s, 2, &d) != RANT_MATH_OK || rant_image_flip_horizontal(&s, &e) != RANT_MATH_OK
            || rant_image_flip_vertical(&e, &f) != RANT_MATH_OK || !same_pixels(&d, &f)) bad++;
        if (rant_image_flip_vertical(&s, &e) != RANT_MATH_OK || rant_image_flip_vertical(&e, &f) != RANT_MATH_OK || !same_pixels(&s, &f)) bad++;
        if (rant_image_flip_horizontal(&s, &e) != RANT_MATH_OK || rant_image_flip_horizontal(&e, &f) != RANT_MATH_OK || !same_pixels(&s, &f)) bad++;
        d = view(PLAIN[i], 3, 5, 0, pool_b);
        e = view(PLAIN[i], 3, 5, 0, pool_c);
        f = view(PLAIN[i], 5, 3, 0, pool_c + 4096);
        if (rant_image_rotate(&s, -1, &d) != RANT_MATH_OK || rant_image_rotate(&s, 3, &e) != RANT_MATH_OK || !same_pixels(&d, &e)) bad++;
        if (rant_image_rotate(&s, 1, &d) != RANT_MATH_OK || rant_image_rotate(&d, 1, &f) != RANT_MATH_OK) bad++;
        e = view(PLAIN[i], 5, 3, 0, pool_c);
        if (rant_image_rotate(&s, 2, &e) != RANT_MATH_OK || !same_pixels(&e, &f)) bad++;
        /* each pixel of a quarter turn from where it should be */
        d = view(PLAIN[i], 3, 5, 0, pool_b);
        rant_image_rotate(&s, 1, &d);
        for (k = 0; k < 15; k++){
            size_t bpp = rant_image_bytes(PLAIN[i], 1, 1);
            uint32_t dx = (uint32_t)k % 3, dy = (uint32_t)k / 3;
            if (memcmp(d.data + (dy * 3 + dx) * bpp, s.data + ((2 - dx) * 5 + dy) * bpp, bpp)) bad++;
        }
    }
    CHECK(bad == 0, "%d turns or flips land wrong", bad);
    s = view(RANT_IMAGE_FORMAT_RGB8, 5, 3, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 5, 3, 0, pool_b);
    CHECK(rant_image_rotate(&s, 1, &d) == RANT_MATH_ERR_ARGUMENT, "a quarter turn swaps the sides");
    d = view(RANT_IMAGE_FORMAT_BGR8, 5, 3, 0, pool_b);
    CHECK(rant_image_flip_vertical(&s, &d) == RANT_MATH_ERR_ARGUMENT, "another format");
    CHECK(rant_image_flip_vertical(&s, &s) == RANT_MATH_ERR_ARGUMENT, "onto itself");
    s = view(RANT_IMAGE_FORMAT_YUYV, 4, 3, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_YUYV, 4, 3, 0, pool_b);
    CHECK(rant_image_flip_horizontal(&s, &d) == RANT_MATH_ERR_FORMAT, "Yuyv pairs would split");
}

static void resize(void){
    RantImageView s, d;
    int i, k, bad = 0, far = 0;
    size_t at = 0;
    const double *refs[2] = { REF_RESIZE_NEAREST, REF_RESIZE_LINEAR };
    memcpy(pool_a, REF_RESIZE_SRC, sizeof REF_RESIZE_SRC);
    s = view(RANT_IMAGE_FORMAT_RGB8, 9, 7, 0, pool_a);
    for (i = 0; i < REF_RESIZE_SIZES; i++){
        uint32_t w = REF_RESIZE_DIMS[2 * i], h = REF_RESIZE_DIMS[2 * i + 1];
        int f;
        d = view(RANT_IMAGE_FORMAT_RGB8, w, h, 0, pool_b);
        for (f = 0; f < 2; f++){
            CHECK(rant_image_resize(&s, (RantImageFilter)f, &d) == RANT_MATH_OK, "to %u by %u", w, h);
            for (k = 0; k < (int)(w * h * 3); k++) if (fabs(pool_b[k] - refs[f][at + k]) > 0.5 + 1e-9) bad++;
            if (f == 0 && memcmp(pool_b, REF_RESIZE_CV_EXACT + at, w * h * 3)) far++;
        }
        at += w * h * 3;
    }
    CHECK(bad == 0, "%d pixels differ from the weights written out", bad);
    CHECK(far == 0, "%d nearest resizes differ from OpenCV's exact nearest", far);
    memcpy(pool_a, REF_AREA_SRC, sizeof REF_AREA_SRC);
    s = view(RANT_IMAGE_FORMAT_RGB8, 12, 8, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_RGB8, 4, 4, 0, pool_b);
    CHECK(rant_image_resize(&s, RANT_FILTER_LINEAR, &d) == RANT_MATH_OK && most_apart(pool_b, REF_AREA_CV, sizeof REF_AREA_CV) <= 1,
          "shrinking is OpenCV's area mean");
    d = view(RANT_IMAGE_FORMAT_RGB8, 19, 13, 0, pool_b);
    CHECK(rant_image_resize(&s, RANT_FILTER_LINEAR, &d) == RANT_MATH_OK && most_apart(pool_b, REF_LINEAR_CV, sizeof REF_LINEAR_CV) <= 1,
          "growing is OpenCV's bilinear");
    d = view(RANT_IMAGE_FORMAT_RGB8, 12, 8, 0, pool_b);
    CHECK(rant_image_resize(&s, RANT_FILTER_LINEAR, &d) == RANT_MATH_OK && same_pixels(&s, &d), "the same size is a copy");

    /* a flat image stays flat, and halving keeps the mean, in every number kind */
    for (i = 0; i < 3; i++){
        RantImageFormat f = PLAIN[i];
        size_t bpp = rant_image_bytes(f, 1, 1);
        double sum_s = 0.0, sum_d = 0.0, v;
        uint32_t x, y;
        s = view(f, 10, 6, 0, pool_a);
        for (k = 0; k < 60; k++){
            if (f == RANT_IMAGE_FORMAT_MONO8) pool_a[k] = 77;
            else if (f == RANT_IMAGE_FORMAT_MONO16){ pool_a[2 * k] = 0x39; pool_a[2 * k + 1] = 0x30; }
            else put_f32(pool_a + 4 * k, 1.0f / 3.0f);
        }
        d = view(f, 7, 13, 0, pool_b);
        for (k = 0; k < 2; k++){
            rant_image_resize(&s, (RantImageFilter)k, &d);
            if (memcmp(pool_b, pool_a, bpp) || memcmp(pool_b + (7 * 13 - 1) * bpp, pool_a, bpp)) bad++;
        }
        for (k = 0; k < 60; k++){
            if (f == RANT_IMAGE_FORMAT_MONO8) pool_a[k] = (uint8_t)(4 * k);
            else if (f == RANT_IMAGE_FORMAT_MONO16){ pool_a[2 * k] = (uint8_t)k; pool_a[2 * k + 1] = (uint8_t)(k * 3); }
            else put_f32(pool_a + 4 * k, (float)(k * 0.25 - 3.0));
        }
        d = view(f, 5, 3, 0, pool_b);
        rant_image_resize(&s, RANT_FILTER_LINEAR, &d);
        for (y = 0; y < 6; y++) for (x = 0; x < 10; x++){ rant_image_value_at(&s, x, y, &v); sum_s += v; }
        for (y = 0; y < 3; y++) for (x = 0; x < 5; x++){ rant_image_value_at(&d, x, y, &v); sum_d += 4.0 * v; }
        if (!near(sum_s, sum_d, f == RANT_IMAGE_FORMAT_MONOF32 ? 1e-4 : 30.0)) bad++;
    }
    CHECK(bad == 0, "%d flat or halved images change", bad);
    d = view(RANT_IMAGE_FORMAT_MONO16, 5, 3, 0, pool_b);
    CHECK(rant_image_resize(&s, RANT_FILTER_LINEAR, &d) == RANT_MATH_ERR_ARGUMENT, "another format");
    d = view(RANT_IMAGE_FORMAT_MONOF32, 5, 3, 0, pool_b);
    CHECK(rant_image_resize(&s, (RantImageFilter)2, &d) == RANT_MATH_ERR_ARGUMENT, "an unknown filter");
    s = view(RANT_IMAGE_FORMAT_NV12, 10, 6, 0, pool_a);
    d = view(RANT_IMAGE_FORMAT_NV12, 5, 3, 0, pool_b);
    CHECK(rant_image_resize(&s, RANT_FILTER_LINEAR, &d) == RANT_MATH_ERR_FORMAT, "Nv12 converts first");
}

/* A smooth scene, the value seen along a direction in the camera's frame. */
static double scene(RantDouble3 d){ return 128.0 + 100.0 * sin(3.0 * d.y / d.x) * cos(2.0 * d.z / d.x); }

static void undistort(void){
    RantCameraIntrinsics c, ideal, plain;
    RantImageView s, d, e;
    RantRay ray;
    RantDouble2 p;
    uint32_t x, y;
    int i, bad = 0, none = 0, seen = 0;
    memset(&plain, 0, sizeof plain);
    plain.width = 40; plain.height = 30; plain.fx = plain.fy = 30.0; plain.cx = 19.5; plain.cy = 14.5;
    for (i = 0; i < 3; i++){
        RantImageFormat f = i == 0 ? RANT_IMAGE_FORMAT_MONO8 : (i == 1 ? RANT_IMAGE_FORMAT_RGB8 : RANT_IMAGE_FORMAT_MONOF32);
        s = view(f, 40, 30, 0, pool_a);
        fill_random(pool_a, s.size);
        if (f == RANT_IMAGE_FORMAT_MONOF32) for (x = 0; x < 1200; x++) put_f32(pool_a + 4 * x, (float)random_unit());
        d = view(f, 40, 30, 0, pool_b);
        CHECK(rant_image_undistort(&s, &plain, NULL, RANT_FILTER_LINEAR, &d) == RANT_MATH_OK && same_pixels(&s, &d), "no lens is a copy");
        CHECK(rant_image_undistort(&s, &plain, &plain, RANT_FILTER_NEAREST, &d) == RANT_MATH_OK && same_pixels(&s, &d), "nearest too");
    }

    /* a scene seen through a lens and undistorted matches the scene seen through the ideal camera */
    memset(&c, 0, sizeof c);
    c.width = 160; c.height = 120; c.fx = 120.0; c.fy = 121.0; c.cx = 80.3; c.cy = 59.6;
    c.model = RANT_DISTORTION_MODEL_BROWN_CONRADY;
    c.coeffs[0] = -0.25; c.coeffs[1] = 0.05; c.coeffs[2] = 1e-3; c.coeffs[3] = -5e-4;
    s = view(RANT_IMAGE_FORMAT_MONOF32, 160, 120, 0, pool_a);
    for (y = 0; y < 120; y++) for (x = 0; x < 160; x++){
        rant_camera_ray(&c, rant_double2(x, y), &ray);
        put_f32(pool_a + 4 * (y * 160 + x), (float)scene(ray.direction));
    }
    ideal = c;
    ideal.model = RANT_DISTORTION_MODEL_NONE;
    ideal.fx = 75.0; ideal.fy = 75.0;
    memset(ideal.coeffs, 0, sizeof ideal.coeffs);
    d = view(RANT_IMAGE_FORMAT_MONOF32, 160, 120, 0, pool_b);
    CHECK(rant_image_undistort(&s, &c, &ideal, RANT_FILTER_LINEAR, &d) == RANT_MATH_OK, "undistort");
    for (y = 0; y < 120; y++) for (x = 0; x < 160; x++){
        float v = get_f32(pool_b + 4 * (y * 160 + x));
        int inside;
        rant_camera_ray(&ideal, rant_double2(x, y), &ray);
        inside = rant_camera_project(&c, ray.direction, &p) == RANT_MATH_OK && p.x >= -0.5 && p.x < 159.5 && p.y >= -0.5 && p.y < 119.5;
        if (v != v){ none++; if (inside) bad++; continue; }
        seen++;
        /* past the outer pixel centers bilinear holds the edge pixel, so only the rest matches */
        if (!inside || (p.x >= 0.0 && p.x <= 159.0 && p.y >= 0.0 && p.y <= 119.0 && fabs(v - scene(ray.direction)) > 0.05)) bad++;
    }
    CHECK(bad == 0, "%d undistorted pixels are not the scene", bad);
    CHECK(none > 1000 && seen > 10000, "the wider view runs off the lens's image: %d none, %d seen", none, seen);
    /* nearest takes the source pixel each ray lands nearest */
    s = view(RANT_IMAGE_FORMAT_RGB8, 160, 120, 0, pool_c);
    fill_random(pool_c, s.size);
    e = view(RANT_IMAGE_FORMAT_RGB8, 160, 120, 0, pool_b);
    CHECK(rant_image_undistort(&s, &c, &ideal, RANT_FILTER_NEAREST, &e) == RANT_MATH_OK, "nearest");
    bad = 0;
    for (y = 0; y < 120; y += 7) for (x = 0; x < 160; x += 5){
        RantColor got, want = rant_color(0, 0, 0, 255);
        rant_camera_ray(&ideal, rant_double2(x, y), &ray);
        if (rant_camera_project(&c, ray.direction, &p) == RANT_MATH_OK && p.x >= -0.5 && p.x < 159.5 && p.y >= -0.5 && p.y < 119.5)
            rant_image_color_at(&s, (uint32_t)floor(p.x + 0.5), (uint32_t)floor(p.y + 0.5), &want);
        rant_image_color_at(&e, x, y, &got);
        if (!same_color(got, want)) bad++;
    }
    CHECK(bad == 0, "%d nearest pixels came from elsewhere", bad);
    d = view(RANT_IMAGE_FORMAT_RGB8, 160, 120, 0, pool_b);
    s = view(RANT_IMAGE_FORMAT_RGB8, 159, 120, 0, pool_c);
    CHECK(rant_image_undistort(&s, &c, NULL, RANT_FILTER_LINEAR, &d) == RANT_MATH_ERR_ARGUMENT, "an image of another size than its camera");
    s = view(RANT_IMAGE_FORMAT_YUYV, 160, 120, 0, pool_c);
    d = view(RANT_IMAGE_FORMAT_YUYV, 160, 120, 0, pool_b);
    CHECK(rant_image_undistort(&s, &c, NULL, RANT_FILTER_LINEAR, &d) == RANT_MATH_ERR_FORMAT, "Yuyv converts first");
}

static double total(const RantImageView *v){
    double t = 0.0;
    size_t i;
    for (i = 0; i < (size_t)v->width * v->height; i++) t += v->data[i] / 255.0;
    return t;
}

static void draw(void){
    RantImageView v;
    RantShape2D sh;
    RantColor red = rant_color(255, 0, 0, 255);
    RantDouble2 tri[3];
    uint32_t x, y;
    int bad = 0;
    /* edges on pixel edges cover whole pixels and nothing past them */
    v = view(RANT_IMAGE_FORMAT_RGB8, 10, 8, 0, pool_a);
    memset(pool_a, 255, v.size);
    sh = rant_shape2d_from_aligned_box2d((RantAlignedBox2D){ { 1.5, 1.5 }, { 4.5, 3.5 } });
    CHECK(rant_image_draw(&v, &sh, red, 0.0) == RANT_MATH_OK, "a box");
    for (y = 0; y < 8; y++) for (x = 0; x < 10; x++){
        RantColor c;
        int in = x >= 2 && x <= 4 && y >= 2 && y <= 3;
        rant_image_color_at(&v, x, y, &c);
        if (!same_color(c, in ? red : rant_color(255, 255, 255, 255))) bad++;
    }
    CHECK(bad == 0, "%d pixels of a crisp box are off", bad);

    v = view(RANT_IMAGE_FORMAT_MONO8, 80, 80, 0, pool_a);
    memset(pool_a, 0, v.size);
    sh = rant_shape2d_from_circle((RantCircle){ { 40.3, 39.7 }, 20.0 });
    rant_image_draw(&v, &sh, rant_color(255, 255, 255, 255), 0.0);
    CHECK(near(total(&v), PI * 400.0, 1.0), "a disc covers its area: %f", total(&v));
    memset(pool_a, 0, v.size);
    rant_image_draw(&v, &sh, rant_color(255, 255, 255, 255), 3.0);
    CHECK(near(total(&v), 2.0 * PI * 20.0 * 3.0, 1.0), "a ring covers its area: %f", total(&v));
    memset(pool_a, 0, v.size);
    sh = rant_shape2d_from_segment2d((RantSegment2D){ { 5.0, 10.0 }, { 45.0, 10.0 } });
    rant_image_draw(&v, &sh, rant_color(255, 255, 255, 255), 0.0);
    CHECK(near(total(&v), 40.0 + PI * 0.25, 0.3), "a line drawn 1 wide with round ends: %f", total(&v));
    memset(pool_a, 0, v.size);
    tri[0] = rant_double2(10.2, 10.1); tri[1] = rant_double2(60.7, 20.4); tri[2] = rant_double2(25.3, 70.9);
    sh = rant_shape2d_from_polygon2d((RantPolygon2DView){ tri, 3 });
    rant_image_draw(&v, &sh, rant_color(255, 255, 255, 255), 0.0);
    CHECK(near(total(&v), 0.5 * fabs((60.7 - 10.2) * (70.9 - 10.1) - (25.3 - 10.2) * (20.4 - 10.1)), 1.0), "a triangle covers its area");
    bad = 0;
    for (y = 0; y < 80; y++) for (x = 0; x < 80; x++) if ((x < 9 || x > 62 || y < 9 || y > 72) && pool_a[y * 80 + x]) bad++;
    CHECK(bad == 0, "%d pixels away from the shape changed", bad);
    memset(pool_a, 0, v.size);
    sh = rant_shape2d_from_circle((RantCircle){ { 40.0, 40.0 }, 10.0 });
    rant_image_draw(&v, &sh, red, 0.0);
    CHECK(pool_a[40 * 80 + 40] == 76, "Mono8 takes the colour's luma");

    v = view(RANT_IMAGE_FORMAT_RGBA8, 3, 1, 0, pool_a);
    memset(pool_a, 0, 12);
    pool_a[10] = 255; pool_a[11] = 255;
    sh = rant_shape2d_from_aligned_box2d((RantAlignedBox2D){ { -0.5, -0.5 }, { 2.5, 0.5 } });
    rant_image_draw(&v, &sh, rant_color(255, 0, 0, 128), 0.0);
    CHECK(pool_a[0] == 255 && pool_a[1] == 0 && pool_a[3] == 128, "over transparent, the colour itself at its alpha");
    CHECK(pool_a[8] == 128 && pool_a[10] == 127 && pool_a[11] == 255, "over opaque blue, half way");
    v = view(RANT_IMAGE_FORMAT_BGR8, 1, 1, 0, pool_a);
    sh = rant_shape2d_from_circle((RantCircle){ { 0.0, 0.0 }, 2.0 });
    rant_image_draw(&v, &sh, red, 0.0);
    CHECK(pool_a[0] == 0 && pool_a[2] == 255, "Bgr8 puts red last");
    CHECK(rant_image_draw(&v, &sh, red, -1.0) == RANT_MATH_ERR_ARGUMENT && rant_image_draw(&v, &sh, red, NAN) == RANT_MATH_ERR_ARGUMENT,
          "a width below 0 or NaN");
    CHECK(rant_image_draw(&v, NULL, red, 0.0) == RANT_MATH_ERR_ARGUMENT, "no shape");
    v = view(RANT_IMAGE_FORMAT_YUYV, 2, 1, 0, pool_a);
    CHECK(rant_image_draw(&v, &sh, red, 0.0) == RANT_MATH_ERR_FORMAT, "Yuyv converts first");
    v = view(RANT_IMAGE_FORMAT_MONO16, 2, 1, 0, pool_a);
    CHECK(rant_image_draw(&v, &sh, red, 0.0) == RANT_MATH_ERR_FORMAT, "a measurement has no colour to draw in");
}

void test_image(void){
    layout();
    pixels();
    convert();
    colorize();
    crop();
    turns();
    resize();
    undistort();
    draw();
}
