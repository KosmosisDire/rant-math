#include "check.h"
#include "reference.h"

static int same_color(RantColor a, RantColor b){ return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a; }
static RantColor hex(uint32_t rgba){ return rant_color_from_hex(rgba); }

/* The HMI's own test of its colour methods: lighten and darken mix toward white or black at the
 * colour's alpha, and mix defaults to half way. */
static void hmi(void){
    RantColor c = hex(0x224493ff);
    RantColor white = rant_color(255, 255, 255, 255), black = rant_color(0, 0, 0, 255);
    CHECK(same_color(rant_color_mix(c, white, 0.0), hex(0x224493ff)), "lighten 0");
    CHECK(same_color(rant_color_mix(c, white, 0.2), hex(0x4a69aaff)), "lighten 0.2 is the HMI's #4a69aa");
    CHECK(same_color(rant_color_mix(c, white, 1.0), hex(0xffffffff)), "lighten 1");
    CHECK(same_color(rant_color_mix(c, black, 1.0), hex(0x000000ff)), "darken 1");
    CHECK(same_color(rant_color_mix(black, white, 1.0), hex(0xffffffff)), "mix black to white at 1");
    CHECK(same_color(rant_color_mix(hex(0xff0000ff), hex(0x0000ffff), 0.5), hex(0x8c53a2ff)), "red and blue meet at the HMI's #8c53a2");
}

static void mix(void){
    int i, bad = 0;
    uint32_t r, g, b;
    for (i = 0; i < REF_MIX_CASES; i++){
        const double *m = REF_MIX + 13 * i;
        RantColor a = rant_color((uint8_t)m[0], (uint8_t)m[1], (uint8_t)m[2], (uint8_t)m[3]);
        RantColor c = rant_color((uint8_t)m[4], (uint8_t)m[5], (uint8_t)m[6], (uint8_t)m[7]);
        RantColor want = rant_color((uint8_t)m[9], (uint8_t)m[10], (uint8_t)m[11], (uint8_t)m[12]);
        RantColor got = rant_color_mix(a, c, m[8]);
        if (!same_color(got, want)) bad++;
        CHECK(same_color(rant_color_mix(a, c, 0.0), a) && same_color(rant_color_mix(a, c, 1.0), c), "case %d: the ends are the colours", i);
    }
    CHECK(bad == 0, "%d of %d mixes differ from the reference", bad, REF_MIX_CASES);
    /* OKLab there and back loses nothing a byte holds */
    bad = 0;
    for (r = 0; r < 256; r += 3) for (g = 0; g < 256; g += 3) for (b = 0; b < 256; b += 3){
        RantColor c = rant_color((uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)(r ^ g));
        if (!same_color(rant_color_mix(c, c, 0.37), c)) bad++;
    }
    CHECK(bad == 0, "%d colours change when mixed with themselves", bad);
    CHECK(same_color(rant_color_mix(hex(0x10203040), hex(0x50607080), NAN), rant_color(0, 0, 0, 0)), "NaN mixes to nothing, as the HMI writes it");
}

static void linear(void){
    int i, bad = 0;
    float last = -1.0f;
    for (i = 0; i < 256; i++){
        RantColor c = rant_color((uint8_t)i, (uint8_t)(255 - i), (uint8_t)(i / 2), (uint8_t)i);
        RantFloat4 v = rant_color_to_linear(c);
        if (!near(v.x, REF_LINEAR[i], 1e-7 * (1.0 + REF_LINEAR[i])) || !near(v.y, REF_LINEAR[255 - i], 1e-7)
            || !near(v.w, i / 255.0, 1e-7)) bad++;
        CHECK(same_color(rant_color_from_linear(v), c), "%d: to linear light and back", i);
        CHECK(v.x > last, "%d: linear light grows with the byte", i);
        last = v.x;
    }
    CHECK(bad == 0, "%d values differ from the sRGB curve", bad);
    CHECK(same_color(rant_color_from_linear(rant_float4(-1.0f, 2.0f, NAN, 0.5f)), rant_color(0, 255, 0, 128)), "out of range clamps");
}

static void colormaps(void){
    int m, i, bad = 0;
    for (m = 0; m < 6; m++)
        for (i = 0; i < REF_MAP_CASES; i++){
            const double *want = REF_MAP + 3 * (m * REF_MAP_CASES + i);
            RantColor got = rant_colormap_at((RantColormap)m, REF_MAP_T[i]);
            if (got.r != want[0] || got.g != want[1] || got.b != want[2] || got.a != 255) bad++;
        }
    CHECK(bad == 0, "%d colormap steps differ from matplotlib", bad);
    CHECK(same_color(rant_colormap_at(RANT_COLORMAP_VIRIDIS, NAN), rant_color(0, 0, 0, 0)), "NaN is transparent");
    CHECK(same_color(rant_colormap_at(RANT_COLORMAP_TURBO, HUGE_VAL), rant_colormap_at(RANT_COLORMAP_TURBO, 1.0))
          && same_color(rant_colormap_at(RANT_COLORMAP_TURBO, -HUGE_VAL), rant_colormap_at(RANT_COLORMAP_TURBO, 0.0)),
          "infinities take the ends, as matplotlib's over and under do");
    CHECK(same_color(rant_colormap_at((RantColormap)6, 0.5), rant_color(0, 0, 0, 0)), "an unknown map is transparent");
    CHECK(same_color(rant_colormap_at((RantColormap)-1, 0.5), rant_color(0, 0, 0, 0)), "a negative map is transparent");
    for (i = 0; i < 256; i++)
        CHECK(rant_colormap_at(RANT_COLORMAP_GRAY, i / 255.0).r == i, "gray step %d is its own byte", i);
}

void test_color(void){
    hmi();
    mix();
    linear();
    colormaps();
}
