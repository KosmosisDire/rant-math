#include <stdlib.h>
#include "check.h"

static RantPointFieldView field(const char *name, uint32_t offset, RantScalarKind kind, uint32_t count){
    RantPointFieldView f;
    f.name = name;
    f.name_len = strlen(name);
    f.offset = offset;
    f.kind = kind;
    f.count = count;
    return f;
}

static RantPointCloudView view(const RantPointFieldView *fields, uint32_t field_count, uint32_t step, uint8_t *data,
                               uint32_t points){
    RantPointCloudView v;
    v.fields = fields;
    v.field_count = field_count;
    v.point_step = step;
    v.data = data;
    v.size = (size_t)step * points;
    return v;
}

static void layouts(void){
    static const char *names[4][6] = {
        { "x", "y", "z" }, { "x", "y", "z", "intensity" }, { "x", "y", "z", "rgb" },
        { "x", "y", "z", "normal_x", "normal_y", "normal_z" } };
    static const uint32_t fields[4] = { 3, 4, 4, 6 };
    uint8_t data[24 * 5];
    int l;
    uint32_t k;
    for (l = 0; l < 4; l++){
        RantPointCloudView v = rant_point_cloud_layout((RantPointCloudLayout)l);
        int ok = v.field_count == fields[l] && v.point_step == 4 * fields[l] && !v.data && !v.size;
        for (k = 0; ok && k < v.field_count; k++)
            ok = v.fields[k].offset == 4 * k && v.fields[k].kind == RANT_SCALAR_F32 && v.fields[k].count == 1
                && v.fields[k].name_len == strlen(names[l][k]) && !memcmp(v.fields[k].name, names[l][k], v.fields[k].name_len);
        CHECK(ok, "layout %d: its fields, packed as f32", l);
        v.data = data;
        v.size = 5 * (size_t)v.point_step;
        CHECK(rant_point_cloud_check(&v) == RANT_MATH_OK, "layout %d: a cloud of 5", l);
        CHECK(rant_point_cloud_field(&v, names[l][fields[l] - 1]) == &v.fields[fields[l] - 1], "layout %d: its last field by name", l);
    }
    CHECK(rant_point_cloud_layout((RantPointCloudLayout)9).field_count == 0, "no layout 9");
}

/* Each kind written and read back: rounded to nearest with halves away from 0, held to its range. */
static void kinds(void){
    static const struct { RantScalarKind kind; double lo, hi; } k[] = {
        { RANT_SCALAR_I8, -128, 127 }, { RANT_SCALAR_U8, 0, 255 }, { RANT_SCALAR_I16, -32768, 32767 },
        { RANT_SCALAR_U16, 0, 65535 }, { RANT_SCALAR_I32, -2147483648.0, 2147483647.0 }, { RANT_SCALAR_U32, 0, 4294967295.0 } };
    static const double in[] = { 2.5, -2.5, 2.4999, -0.5, 0.49, 1e30, -1e30, NAN, 7.0 };
    uint8_t data[8 * 9];
    double back[9];
    int i, j, bad = 0;
    for (i = 0; i < 6; i++){
        RantPointFieldView f = field("v", 0, k[i].kind, 1);
        RantPointCloudView v = view(&f, 1, 8, data, 9);
        uint32_t n = 9;
        CHECK(rant_point_cloud_write_values(&v, "v", in, 9) == RANT_MATH_OK, "kind %d: writes", (int)k[i].kind);
        CHECK(rant_point_cloud_read_values(&v, "v", back, &n) == RANT_MATH_OK && n == 9, "kind %d: reads", (int)k[i].kind);
        for (j = 0; j < 9; j++){
            double want = in[j] != in[j] ? 0.0 : round(in[j]);
            want = want < k[i].lo ? k[i].lo : (want > k[i].hi ? k[i].hi : want);
            if (back[j] != want){ bad++; fprintf(stderr, "kind %d: %g read back as %g, not %g\n", (int)k[i].kind, in[j], back[j], want); }
        }
    }
    CHECK(bad == 0, "whole kinds round and clamp");
    {   /* the 64 bit kinds hold their ends and every double below 2^53 */
        static const double big[] = { 9007199254740993.0, -9223372036854775808.0, 1e19, -1e19, 18446744073709551616.0, 3.0 };
        RantPointFieldView f[2];
        RantPointCloudView v;
        uint8_t raw[16 * 6];
        int64_t s;
        uint64_t u;
        f[0] = field("s", 0, RANT_SCALAR_I64, 1);
        f[1] = field("u", 8, RANT_SCALAR_U64, 1);
        v = view(f, 2, 16, raw, 6);
        CHECK(rant_point_cloud_write_values(&v, "s", big, 6) == RANT_MATH_OK && rant_point_cloud_write_values(&v, "u", big, 6) == RANT_MATH_OK, "64 bit kinds write");
        memcpy(&s, raw + 16 * 1, 8); CHECK(s == INT64_MIN, "i64 low end");
        memcpy(&s, raw + 16 * 2, 8); CHECK(s == INT64_MAX, "i64 past the top holds the top");
        memcpy(&s, raw + 16 * 3, 8); CHECK(s == INT64_MIN, "i64 past the bottom");
        memcpy(&u, raw + 16 * 2 + 8, 8); CHECK(u == 10000000000000000000ull, "u64 holds 1e19");
        memcpy(&u, raw + 16 * 4 + 8, 8); CHECK(u == UINT64_MAX, "u64 2^64 holds the top");
        memcpy(&u, raw + 16 * 3 + 8, 8); CHECK(u == 0, "u64 below 0");
        memcpy(&s, raw + 16 * 5, 8); CHECK(s == 3, "i64 3");
    }
    {   /* little endian on the wire, read byte by byte */
        uint8_t raw[8] = { 0x34, 0x12, 0xFE, 0xFF, 0, 0, 0xC0, 0x3F };
        RantPointFieldView f[3];
        RantPointCloudView v;
        double a, b, c;
        uint32_t n = 1;
        f[0] = field("a", 0, RANT_SCALAR_U16, 1);
        f[1] = field("b", 2, RANT_SCALAR_I16, 1);
        f[2] = field("c", 4, RANT_SCALAR_F32, 1);
        v = view(f, 3, 8, raw, 1);
        rant_point_cloud_read_values(&v, "a", &a, &n);
        rant_point_cloud_read_values(&v, "b", &b, &n);
        rant_point_cloud_read_values(&v, "c", &c, &n);
        CHECK(a == 0x1234 && b == -2 && c == 1.5, "u16 %g, i16 %g and f32 %g as their little endian bytes say", a, b, c);
    }
}

/* x as f64, y as i16 and z as f32 at odd places, with bytes between that must stay. */
static void points(void){
    RantPointFieldView f[4];
    RantPointCloudView v;
    RantDouble3 in[7], out[7];
    uint8_t data[23 * 7];
    double ring[14];
    uint32_t n, i;
    int same = 1, kept = 1;
    f[0] = field("ring", 0, RANT_SCALAR_U8, 2);
    f[1] = field("x", 2, RANT_SCALAR_F64, 1);
    f[2] = field("z", 11, RANT_SCALAR_F32, 1);
    f[3] = field("y", 17, RANT_SCALAR_I16, 1);
    v = view(f, 4, 23, data, 7);
    for (i = 0; i < sizeof data; i++) data[i] = (uint8_t)(i * 37 + 11);
    for (i = 0; i < 7; i++) in[i] = rant_double3(random_range(-50, 50), random_range(-300, 300), random_range(-2, 2));
    CHECK(rant_point_cloud_write_points(&v, in, 7) == RANT_MATH_OK, "mixed kinds take points");
    n = 7;
    CHECK(rant_point_cloud_read_points(&v, out, &n) == RANT_MATH_OK && n == 7, "and give them back");
    for (i = 0; i < 7; i++){
        same &= out[i].x == in[i].x && out[i].y == round(in[i].y) && out[i].z == (double)(float)in[i].z;
        kept &= data[23 * i + 10] == (uint8_t)((23 * i + 10) * 37 + 11) && data[23 * i + 15] == (uint8_t)((23 * i + 15) * 37 + 11)
             && data[23 * i + 16] == (uint8_t)((23 * i + 16) * 37 + 11) && data[23 * i + 19] == (uint8_t)((23 * i + 19) * 37 + 11);
    }
    CHECK(same, "x exact, y rounded, z as an f32");
    CHECK(kept, "the bytes between fields untouched");
    n = 14;
    CHECK(rant_point_cloud_read_values(&v, "ring", ring, &n) == RANT_MATH_OK && n == 14, "a field of 2 gives 2 per point");
    CHECK(ring[0] == data[0] && ring[1] == data[1] && ring[13] == data[23 * 6 + 1], "point after point, in order");
    n = 3;
    CHECK(rant_point_cloud_read_points(&v, out, &n) == RANT_MATH_ERR_SPACE && n == 7, "too little room gets the need");
    n = 0;
    CHECK(rant_point_cloud_read_points(&v, NULL, &n) == RANT_MATH_ERR_SPACE && n == 7, "asking with no room");
    CHECK(rant_point_cloud_write_points(&v, in, 6) == RANT_MATH_ERR_ARGUMENT, "one point per point to write");
    n = 14;
    CHECK(rant_point_cloud_write_values(&v, "ring", ring, 13) == RANT_MATH_ERR_ARGUMENT, "a field of 2 takes 14");
    v.size = 0;
    n = 5;
    CHECK(rant_point_cloud_read_points(&v, out, &n) == RANT_MATH_OK && n == 0, "a cloud of no points");
    v.data = NULL;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_OK, "which needs no data");
}

static void layout_errors(void){
    RantPointFieldView f[3], bad;
    RantPointCloudView v;
    uint8_t data[64];
    RantDouble3 p[4];
    uint32_t n = 4;
    f[0] = field("x", 0, RANT_SCALAR_F32, 1);
    f[1] = field("y", 4, RANT_SCALAR_F32, 1);
    f[2] = field("z", 8, RANT_SCALAR_F32, 1);
    v = view(f, 3, 12, data, 4);
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_OK, "a plain cloud");
    v.size = 47;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "data not a whole number of points");
    v.size = 48; v.point_step = 0;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "a point step of 0");
    v.point_step = 11;
    v.size = 44;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "z runs past the point");
    v.point_step = 12; v.size = 48;
    f[2].kind = (RantScalarKind)11;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "no such kind");
    f[2].kind = RANT_SCALAR_F32; f[2].count = 0;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "a field of no numbers");
    f[2].count = 1;
    v.data = NULL;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "bytes with no data");
    v.data = data;
    f[2].offset = 0xFFFFFFFFu;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "an offset that would wrap");
    f[2].offset = 8;
    CHECK(rant_point_cloud_check(NULL) == RANT_MATH_ERR_ARGUMENT, "no view");
    CHECK(rant_point_cloud_read_points(&v, p, NULL) == RANT_MATH_ERR_ARGUMENT, "no count");
    v.field_count = 2;
    CHECK(rant_point_cloud_read_points(&v, p, &n) == RANT_MATH_ERR_FORMAT && n == 4, "no z is a format error, count untouched");
    CHECK(rant_point_cloud_read_values(&v, "intensity", NULL, &n) == RANT_MATH_ERR_FORMAT, "no such field");
    CHECK(rant_point_cloud_read_values(&v, NULL, NULL, &n) == RANT_MATH_ERR_ARGUMENT, "no field name");
    v.field_count = 3;
    /* a name is its bytes, not up to a NUL */
    bad = field("xyz", 0, RANT_SCALAR_F32, 1);
    bad.name_len = 1;
    v.fields = &bad; v.field_count = 1;
    CHECK(rant_point_cloud_field(&v, "x") == &bad && !rant_point_cloud_field(&v, "xyz"), "x is the first byte of xyz");
    bad.name_len = 0;
    bad.name = NULL;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_OK && rant_point_cloud_field(&v, "") == &bad, "an empty name may have no bytes");
    bad.name_len = 2;
    CHECK(rant_point_cloud_check(&v) == RANT_MATH_ERR_FORMAT, "but a name of 2 needs them");
}

static void colors(void){
    RantPointCloudView v = rant_point_cloud_layout(RANT_POINT_CLOUD_XYZ_RGB);
    RantPointFieldView f[4];
    RantColor in[3], out[3];
    uint8_t data[16 * 3];
    uint32_t n = 3, u;
    float packed;
    v.data = data;
    v.size = sizeof data;
    in[0] = rant_color(255, 0, 0, 10); in[1] = rant_color(1, 2, 3, 4); in[2] = rant_color(0, 128, 255, 255);
    CHECK(rant_point_cloud_write_colors(&v, in, 3) == RANT_MATH_OK, "rgb takes colours");
    memcpy(&packed, data + 16 + 12, 4);
    memcpy(&u, &packed, 4);
    CHECK(u == 0xFF010203u, "PCL's packing: 0x%08X as a float, alpha 255", u);
    CHECK(rant_point_cloud_read_colors(&v, out, &n) == RANT_MATH_OK && n == 3, "and gives them back");
    CHECK(out[0].r == 255 && out[0].g == 0 && out[0].a == 255 && out[1].b == 3 && out[2].g == 128, "with alpha 255 from rgb");
    memcpy(f, v.fields, sizeof f);
    f[3] = field("rgba", 12, RANT_SCALAR_U32, 1);
    v.fields = f;
    CHECK(rant_point_cloud_write_colors(&v, in, 3) == RANT_MATH_OK && rant_point_cloud_read_colors(&v, out, &n) == RANT_MATH_OK
          && out[0].a == 10 && out[1].a == 4 && data[16 + 12] == 3 && data[16 + 15] == 4, "rgba keeps alpha, blue first");
    f[3] = field("rgb", 12, RANT_SCALAR_U16, 1);
    CHECK(rant_point_cloud_read_colors(&v, out, &n) == RANT_MATH_ERR_FORMAT, "an rgb of 2 bytes");
    f[3] = field("color", 12, RANT_SCALAR_F32, 1);
    CHECK(rant_point_cloud_read_colors(&v, out, &n) == RANT_MATH_ERR_FORMAT, "no colour field");
}

static void transform(void){
    RantPointCloudView v = rant_point_cloud_layout(RANT_POINT_CLOUD_XYZ_NORMAL);
    RantPointFieldView f[6];
    RantDouble3 p[50], n[50], moved[50], back[50];
    double raw[300];
    uint8_t data[24 * 50];
    RantPose pose = random_pose(3.0);
    uint32_t i, k = 50;
    int ok = 1;
    for (i = 0; i < 50; i++){ p[i] = random_double3(5.0); n[i] = rant_double3_normalize(random_double3(1.0)); }
    /* f64 fields, so the move can be compared exactly */
    for (i = 0; i < 6; i++){
        f[i] = v.fields[i];
        f[i].kind = RANT_SCALAR_F64;
        f[i].offset = 8 * i;
    }
    {   RantPointCloudView w = view(f, 6, 48, (uint8_t *)raw, 50);
        for (i = 0; i < 50; i++){
            raw[6 * i] = p[i].x; raw[6 * i + 1] = p[i].y; raw[6 * i + 2] = p[i].z;
            raw[6 * i + 3] = n[i].x; raw[6 * i + 4] = n[i].y; raw[6 * i + 5] = n[i].z;
        }
        CHECK(rant_point_cloud_transform(&w, pose) == RANT_MATH_OK, "a cloud with normals moves");
        rant_pose_apply_points(pose, p, moved, 50);
        for (i = 0; i < 50; i++){
            RantDouble3 got = rant_double3(raw[6 * i], raw[6 * i + 1], raw[6 * i + 2]);
            RantDouble3 gn = rant_double3(raw[6 * i + 3], raw[6 * i + 4], raw[6 * i + 5]);
            ok &= near3(got, moved[i], 1e-12) && near3(gn, rant_quaternion_rotate(pose.orientation, n[i]), 1e-14);
        }
        CHECK(ok, "points as rant_pose_apply_points moves them, normals turned only");
    }
    v.data = data;
    v.size = sizeof data;
    rant_point_cloud_write_points(&v, p, 50);
    CHECK(rant_point_cloud_transform(&v, pose) == RANT_MATH_OK && rant_point_cloud_transform(&v, rant_pose_inverse(pose)) == RANT_MATH_OK,
          "there and back in f32");
    rant_point_cloud_read_points(&v, back, &k);
    for (i = 0, ok = 1; i < 50; i++) ok &= near3(back[i], p[i], 1e-5);
    CHECK(ok, "within f32's rounding");
}

static void select_points(void){
    RantPointCloudView v = rant_point_cloud_layout(RANT_POINT_CLOUD_XYZ_INTENSITY);
    uint8_t data[16 * 40], out[16 * 40], keep[40];
    uint32_t i, n, want = 0, k;
    int ok = 1;
    for (i = 0; i < sizeof data; i++) data[i] = (uint8_t)(i * 13 + 5);
    v.data = data;
    v.size = sizeof data;
    for (i = 0; i < 40; i++){ keep[i] = random_unit() < 0.4 ? (uint8_t)(1 + i % 3) : 0; want += keep[i] != 0; }
    n = 40;
    CHECK(rant_point_cloud_select(&v, keep, out, &n) == RANT_MATH_OK && n == want, "keeps %u of 40", want);
    for (i = 0, k = 0; i < 40; i++) if (keep[i]) ok &= !memcmp(out + 16 * k++, data + 16 * i, 16);
    CHECK(ok, "whole points in order, any mark but 0 keeps");
    n = want - 1;
    CHECK(rant_point_cloud_select(&v, keep, out, &n) == RANT_MATH_ERR_SPACE && n == want, "too little room");
    n = 40;
    CHECK(rant_point_cloud_select(&v, keep, data, &n) == RANT_MATH_OK && !memcmp(data, out, 16 * (size_t)want), "into its own data");
    CHECK(rant_point_cloud_select(&v, NULL, out, &n) == RANT_MATH_ERR_ARGUMENT, "keep is needed");
}

static void scans(void){
    RantLaserScanView s;
    float r[12] = { 1.0f, 2.0f, 0.05f, 30.5f, 3.0f, 0.1f, 30.0f, 0, 0, 0, 0, 0 };
    RantDouble2 p[12];
    uint32_t n = 12, i;
    int ok = 1;
    r[7] = (float)HUGE_VAL;
    r[8] = (float)NAN;
    r[9] = (float)-HUGE_VAL;
    r[10] = 4.0f;
    r[11] = 5.0f;
    s.angle_min = -1.2; s.angle_step = 0.2; s.range_min = 0.1; s.range_max = 30.0; s.echoes = 0;
    s.ranges = r; s.range_count = 12;
    CHECK(rant_laser_scan_points(&s, p, &n) == RANT_MATH_OK && n == 12, "a scan of 12 beams");
    for (i = 0; i < 12; i++){
        double a = -1.2 + 0.2 * i, d = r[i];
        int valid = d - d == 0.0 && d >= 0.1 && d <= 30.0;
        ok &= valid ? near2(p[i], rant_double2(d * cos(a), d * sin(a)), 1e-12) : p[i].x != p[i].x && p[i].y != p[i].y;
    }
    CHECK(ok, "from +x toward +y, NaN past the limits, at infinity or NaN");
    CHECK(p[5].x == p[5].x && p[6].x == p[6].x, "both limits count as valid");
    s.echoes = 3;
    n = 12;
    rant_laser_scan_points(&s, p, &n);
    CHECK(near2(p[4], rant_double2(3.0 * cos(-1.0), 3.0 * sin(-1.0)), 1e-12) && near2(p[11], rant_double2(5.0 * cos(-0.6), 5.0 * sin(-0.6)), 1e-12),
          "3 echoes a beam share its angle");
    n = 11;
    CHECK(rant_laser_scan_points(&s, p, &n) == RANT_MATH_ERR_SPACE && n == 12, "room for every range");
    s.angle_step = NAN;
    CHECK(rant_laser_scan_points(&s, p, &n) == RANT_MATH_ERR_ARGUMENT, "a NaN step");
    s.angle_step = 0.2; s.ranges = NULL;
    CHECK(rant_laser_scan_points(&s, p, &n) == RANT_MATH_ERR_ARGUMENT, "ranges without data");
}

static void depth(void){
    RantCameraIntrinsics c;
    RantImageView im;
    uint16_t mm[6 * 5 + 6];
    float m[8 * 5];
    RantDouble3 p[40];
    uint32_t n, x, y;
    int ok = 1, k;
    memset(&c, 0, sizeof c);
    c.width = 6; c.height = 5; c.fx = 5.0; c.fy = 5.5; c.cx = 2.4; c.cy = 2.1;
    for (k = 0; k < 36; k++) mm[k] = (uint16_t)(400 + 97 * k);
    mm[7] = 0;
    im.width = 6; im.height = 5; im.stride = 12; im.format = RANT_IMAGE_MONO16; im.data = (uint8_t *)mm; im.size = sizeof mm;
    n = 30;
    CHECK(rant_image_depth_points(&im, &c, p, &n) == RANT_MATH_OK && n == 30, "Mono16 depth");
    for (y = 0; y < 5; y++)
        for (x = 0; x < 6; x++){
            RantDouble3 want;
            uint16_t d = mm[6 * y + x];
            if (!d){ ok &= p[6 * y + x].x != p[6 * y + x].x; continue; }
            rant_camera_unproject(&c, rant_double2(x, y), d / 1000.0, &want);
            ok &= near3(p[6 * y + x], want, 1e-12);
        }
    CHECK(ok, "millimeters along +x as unproject places them, NaN for 0");
    /* a distorted lens, every point seen back at its pixel */
    c.model = RANT_DISTORTION_BROWN_CONRADY;
    c.coeffs[0] = -0.2; c.coeffs[1] = 0.05; c.coeffs[2] = 0.001; c.coeffs[3] = -0.002;
    c.width = 8;
    for (k = 0; k < 40; k++) m[k] = (float)random_range(0.3, 9.0);
    m[3] = (float)NAN; m[9] = 0.0f; m[10] = -1.0f; m[11] = (float)HUGE_VAL;
    im.width = 8; im.stride = 0; im.format = RANT_IMAGE_MONOF32; im.data = (uint8_t *)m; im.size = sizeof m;
    n = 40;
    CHECK(rant_image_depth_points(&im, &c, p, &n) == RANT_MATH_OK, "Monof32 depth through a lens");
    for (k = 0, ok = 1; k < 40; k++){
        RantDouble2 px;
        if (k == 3 || (k >= 9 && k <= 11)){ ok &= p[k].x != p[k].x; continue; }
        ok &= p[k].x == (double)m[k] && rant_camera_project(&c, p[k], &px) == RANT_MATH_OK
              && near2(px, rant_double2(k % 8, k / 8), 1e-9);
    }
    CHECK(ok, "meters, each point back at its pixel, NaN for NaN, 0, below 0 and infinity");
    c.width = 7;
    CHECK(rant_image_depth_points(&im, &c, p, &n) == RANT_MATH_ERR_ARGUMENT, "the sizes must match");
    c.width = 8;
    im.format = RANT_IMAGE_MONO8;
    CHECK(rant_image_depth_points(&im, &c, p, &n) == RANT_MATH_ERR_FORMAT, "not a depth format");
    im.format = RANT_IMAGE_MONOF32;
    n = 39;
    CHECK(rant_image_depth_points(&im, &c, p, &n) == RANT_MATH_ERR_SPACE && n == 40, "room for every pixel");
}

void test_cloud(void){
    layouts();
    kinds();
    points();
    layout_errors();
    colors();
    transform();
    select_points();
    scans();
    depth();
}
