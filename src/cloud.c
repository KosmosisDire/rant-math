#include "internal.h"

static const RantPointFieldView I_XYZ[] = {
    { "x", 1, 0, RANT_SCALAR_F32, 1 }, { "y", 1, 4, RANT_SCALAR_F32, 1 }, { "z", 1, 8, RANT_SCALAR_F32, 1 },
    { "intensity", 9, 12, RANT_SCALAR_F32, 1 }
};
static const RantPointFieldView I_RGB[] = {
    { "x", 1, 0, RANT_SCALAR_F32, 1 }, { "y", 1, 4, RANT_SCALAR_F32, 1 }, { "z", 1, 8, RANT_SCALAR_F32, 1 },
    { "rgb", 3, 12, RANT_SCALAR_F32, 1 }
};
static const RantPointFieldView I_NORMAL[] = {
    { "x", 1, 0, RANT_SCALAR_F32, 1 }, { "y", 1, 4, RANT_SCALAR_F32, 1 }, { "z", 1, 8, RANT_SCALAR_F32, 1 },
    { "normal_x", 8, 12, RANT_SCALAR_F32, 1 }, { "normal_y", 8, 16, RANT_SCALAR_F32, 1 },
    { "normal_z", 8, 20, RANT_SCALAR_F32, 1 }
};

RantPointCloudView rant_point_cloud_layout(RantPointCloudLayout layout){
    RantPointCloudView v;
    memset(&v, 0, sizeof v);
    switch (layout){
    case RANT_POINT_CLOUD_XYZ: v.fields = I_XYZ; v.field_count = 3; break;
    case RANT_POINT_CLOUD_XYZ_INTENSITY: v.fields = I_XYZ; v.field_count = 4; break;
    case RANT_POINT_CLOUD_XYZ_RGB: v.fields = I_RGB; v.field_count = 4; break;
    case RANT_POINT_CLOUD_XYZ_NORMAL: v.fields = I_NORMAL; v.field_count = 6; break;
    default: return v;
    }
    v.point_step = 4 * v.field_count;
    return v;
}

int i_math_scalar_bytes(RantScalarKind k){
    switch (k){
    case RANT_SCALAR_I8: case RANT_SCALAR_U8: return 1;
    case RANT_SCALAR_I16: case RANT_SCALAR_U16: return 2;
    case RANT_SCALAR_I32: case RANT_SCALAR_U32: case RANT_SCALAR_F32: return 4;
    case RANT_SCALAR_F64: case RANT_SCALAR_I64: case RANT_SCALAR_U64: return 8;
    default: return 0;
    }
}

/* Little endian on any host. */
static uint64_t i_load(const uint8_t *p, int n){
    uint64_t u = 0;
    int i;
    for (i = n - 1; i >= 0; i--) u = u << 8 | p[i];
    return u;
}
static void i_store(uint8_t *p, int n, uint64_t u){
    int i;
    for (i = 0; i < n; i++){ p[i] = (uint8_t)u; u >>= 8; }
}

double i_math_scalar_get(RantScalarKind k, const uint8_t *p){
    uint64_t u = i_load(p, i_math_scalar_bytes(k));
    float f;
    double d;
    switch (k){
    case RANT_SCALAR_I8: return (int8_t)(uint8_t)u;
    case RANT_SCALAR_U8: return (double)u;
    case RANT_SCALAR_I16: return (int16_t)(uint16_t)u;
    case RANT_SCALAR_U16: return (double)u;
    case RANT_SCALAR_I32: return (int32_t)(uint32_t)u;
    case RANT_SCALAR_U32: return (double)u;
    case RANT_SCALAR_F32: { uint32_t w = (uint32_t)u; memcpy(&f, &w, sizeof f); return f; }
    case RANT_SCALAR_F64: memcpy(&d, &u, sizeof d); return d;
    case RANT_SCALAR_I64: return (double)(int64_t)u;
    case RANT_SCALAR_U64: return (double)u;
    default: return 0.0;
    }
}

/* Nearest, halves away from 0, held to lo and hi, which a double holds exactly. */
static double i_whole(double v, double lo, double hi){
    if (!(v == v)) return 0.0;
    v = v < 0.0 ? -floor(0.5 - v) : floor(v + 0.5);
    return v < lo ? lo : (v > hi ? hi : v);
}

void i_math_scalar_put(RantScalarKind k, uint8_t *p, double v){
    float f;
    uint32_t w;
    uint64_t u;
    switch (k){
    case RANT_SCALAR_I8: p[0] = (uint8_t)(int8_t)i_whole(v, -128.0, 127.0); return;
    case RANT_SCALAR_U8: p[0] = (uint8_t)i_whole(v, 0.0, 255.0); return;
    case RANT_SCALAR_I16: i_store(p, 2, (uint16_t)(int16_t)i_whole(v, -32768.0, 32767.0)); return;
    case RANT_SCALAR_U16: i_store(p, 2, (uint16_t)i_whole(v, 0.0, 65535.0)); return;
    case RANT_SCALAR_I32: i_store(p, 4, (uint32_t)(int32_t)i_whole(v, -2147483648.0, 2147483647.0)); return;
    case RANT_SCALAR_U32: i_store(p, 4, (uint32_t)i_whole(v, 0.0, 4294967295.0)); return;
    case RANT_SCALAR_F32: f = (float)v; memcpy(&w, &f, sizeof w); i_store(p, 4, w); return;
    case RANT_SCALAR_F64: memcpy(&u, &v, sizeof u); i_store(p, 8, u); return;
    /* 2^63 and 2^64 are past the top, so they stand for it */
    case RANT_SCALAR_I64:
        v = i_whole(v, -9223372036854775808.0, 9223372036854775808.0);
        i_store(p, 8, v >= 9223372036854775808.0 ? (uint64_t)INT64_MAX : (uint64_t)(int64_t)v);
        return;
    case RANT_SCALAR_U64:
        v = i_whole(v, 0.0, 18446744073709551616.0);
        i_store(p, 8, v >= 18446744073709551616.0 ? UINT64_MAX : (uint64_t)v);
        return;
    default: return;
    }
}

/* What every view must pass, and how many points it holds. */
static RantMathResult i_cloud(const RantPointCloudView *c, uint32_t *n){
    uint32_t i;
    if (!c) return RANT_MATH_ERR_ARGUMENT;
    if (!c->point_step || (c->size && !c->data) || (c->field_count && !c->fields)) return RANT_MATH_ERR_FORMAT;
    if (c->size % c->point_step || c->size / c->point_step > UINT32_MAX) return RANT_MATH_ERR_FORMAT;
    for (i = 0; i < c->field_count; i++){
        const RantPointFieldView *f = &c->fields[i];
        int b = i_math_scalar_bytes(f->kind);
        if (!b || !f->count || (f->name_len && !f->name)) return RANT_MATH_ERR_FORMAT;
        if ((uint64_t)f->offset + (uint64_t)b * f->count > c->point_step) return RANT_MATH_ERR_FORMAT;
    }
    *n = (uint32_t)(c->size / c->point_step);
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_check(const RantPointCloudView *cloud){
    uint32_t n;
    return i_cloud(cloud, &n);
}

/* After i_cloud has passed. */
static const RantPointFieldView *i_field(const RantPointCloudView *c, const char *name){
    size_t len;
    uint32_t i;
    if (!name) return NULL;
    len = strlen(name);
    for (i = 0; i < c->field_count; i++)
        if (c->fields[i].name_len == len && (!len || memcmp(c->fields[i].name, name, len) == 0)) return &c->fields[i];
    return NULL;
}

const RantPointFieldView *rant_point_cloud_field(const RantPointCloudView *cloud, const char *name){
    uint32_t n;
    return i_cloud(cloud, &n) == RANT_MATH_OK ? i_field(cloud, name) : NULL;
}

/* A view checked, with its x, y and z, and the room rule applied to count when it is not NULL. */
static RantMathResult i_xyz(const RantPointCloudView *c, const RantPointFieldView **xyz, uint32_t *n){
    RantMathResult r = i_cloud(c, n);
    if (r != RANT_MATH_OK) return r;
    xyz[0] = i_field(c, "x");
    xyz[1] = i_field(c, "y");
    xyz[2] = i_field(c, "z");
    return xyz[0] && xyz[1] && xyz[2] ? RANT_MATH_OK : RANT_MATH_ERR_FORMAT;
}

static RantDouble3 i_point(const uint8_t *p, const RantPointFieldView **xyz){
    return rant_double3(i_math_scalar_get(xyz[0]->kind, p + xyz[0]->offset),
                        i_math_scalar_get(xyz[1]->kind, p + xyz[1]->offset),
                        i_math_scalar_get(xyz[2]->kind, p + xyz[2]->offset));
}
static void i_set_point(uint8_t *p, const RantPointFieldView **xyz, RantDouble3 v){
    i_math_scalar_put(xyz[0]->kind, p + xyz[0]->offset, v.x);
    i_math_scalar_put(xyz[1]->kind, p + xyz[1]->offset, v.y);
    i_math_scalar_put(xyz[2]->kind, p + xyz[2]->offset, v.z);
}

RantMathResult rant_point_cloud_read_points(const RantPointCloudView *cloud, RantDouble3 *out, uint32_t *count){
    const RantPointFieldView *xyz[3];
    uint32_t n, i;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_xyz(cloud, xyz, &n)) != RANT_MATH_OK) return r;
    if ((r = i_math_room(count, n, out)) != RANT_MATH_OK) return r;
    for (i = 0; i < n; i++) out[i] = i_point(cloud->data + (size_t)i * cloud->point_step, xyz);
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_write_points(RantPointCloudView *cloud, const RantDouble3 *points, uint32_t count){
    const RantPointFieldView *xyz[3];
    uint32_t n, i;
    RantMathResult r;
    if ((r = i_xyz(cloud, xyz, &n)) != RANT_MATH_OK) return r;
    if (count != n || (n && !points)) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < n; i++) i_set_point(cloud->data + (size_t)i * cloud->point_step, xyz, points[i]);
    return RANT_MATH_OK;
}

static RantMathResult i_named(const RantPointCloudView *c, const char *name, const RantPointFieldView **f, uint32_t *n){
    RantMathResult r;
    if (!name) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_cloud(c, n)) != RANT_MATH_OK) return r;
    *f = i_field(c, name);
    return *f ? RANT_MATH_OK : RANT_MATH_ERR_FORMAT;
}

RantMathResult rant_point_cloud_read_values(const RantPointCloudView *cloud, const char *field, double *out, uint32_t *count){
    const RantPointFieldView *f;
    uint32_t n, i, j, b;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_named(cloud, field, &f, &n)) != RANT_MATH_OK) return r;
    if ((uint64_t)n * f->count > UINT32_MAX) return RANT_MATH_ERR_FORMAT;
    if ((r = i_math_room(count, n * f->count, out)) != RANT_MATH_OK) return r;
    b = (uint32_t)i_math_scalar_bytes(f->kind);
    for (i = 0; i < n; i++){
        const uint8_t *p = cloud->data + (size_t)i * cloud->point_step + f->offset;
        for (j = 0; j < f->count; j++) *out++ = i_math_scalar_get(f->kind, p + j * b);
    }
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_write_values(RantPointCloudView *cloud, const char *field, const double *values,
                                             uint32_t count){
    const RantPointFieldView *f;
    uint32_t n, i, j, b;
    RantMathResult r;
    if ((r = i_named(cloud, field, &f, &n)) != RANT_MATH_OK) return r;
    if ((uint64_t)n * f->count != count || (count && !values)) return RANT_MATH_ERR_ARGUMENT;
    b = (uint32_t)i_math_scalar_bytes(f->kind);
    for (i = 0; i < n; i++){
        uint8_t *p = cloud->data + (size_t)i * cloud->point_step + f->offset;
        for (j = 0; j < f->count; j++) i_math_scalar_put(f->kind, p + j * b, *values++);
    }
    return RANT_MATH_OK;
}

/* The packed colour field, 4 bytes blue, green, red and alpha, and whether it holds alpha. */
static RantMathResult i_packed(const RantPointCloudView *c, const RantPointFieldView **f, int *alpha, uint32_t *n){
    RantMathResult r = i_cloud(c, n);
    if (r != RANT_MATH_OK) return r;
    *alpha = 1;
    if (!(*f = i_field(c, "rgba"))){ *alpha = 0; *f = i_field(c, "rgb"); }
    return *f && i_math_scalar_bytes((*f)->kind) == 4 ? RANT_MATH_OK : RANT_MATH_ERR_FORMAT;
}

RantMathResult rant_point_cloud_read_colors(const RantPointCloudView *cloud, RantColor *out, uint32_t *count){
    const RantPointFieldView *f;
    uint32_t n, i;
    int alpha;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_packed(cloud, &f, &alpha, &n)) != RANT_MATH_OK) return r;
    if ((r = i_math_room(count, n, out)) != RANT_MATH_OK) return r;
    for (i = 0; i < n; i++){
        const uint8_t *p = cloud->data + (size_t)i * cloud->point_step + f->offset;
        out[i] = rant_color(p[2], p[1], p[0], alpha ? p[3] : 255);
    }
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_write_colors(RantPointCloudView *cloud, const RantColor *colors, uint32_t count){
    const RantPointFieldView *f;
    uint32_t n, i;
    int alpha;
    RantMathResult r;
    if ((r = i_packed(cloud, &f, &alpha, &n)) != RANT_MATH_OK) return r;
    if (count != n || (n && !colors)) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < n; i++){
        uint8_t *p = cloud->data + (size_t)i * cloud->point_step + f->offset;
        p[0] = colors[i].b; p[1] = colors[i].g; p[2] = colors[i].r; p[3] = alpha ? colors[i].a : 255;
    }
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_transform(RantPointCloudView *cloud, RantPose pose){
    const RantPointFieldView *xyz[3], *nxyz[3];
    i_MathMat3 m;
    uint32_t n, i;
    int normals;
    RantMathResult r;
    if ((r = i_xyz(cloud, xyz, &n)) != RANT_MATH_OK) return r;
    nxyz[0] = i_field(cloud, "normal_x");
    nxyz[1] = i_field(cloud, "normal_y");
    nxyz[2] = i_field(cloud, "normal_z");
    normals = nxyz[0] && nxyz[1] && nxyz[2];
    m = i_math_mat3_from_quaternion(pose.orientation);
    for (i = 0; i < n; i++){
        uint8_t *p = cloud->data + (size_t)i * cloud->point_step;
        i_set_point(p, xyz, rant_double3_add(pose.position, i_math_mat3_apply(&m, i_point(p, xyz))));
        if (normals) i_set_point(p, nxyz, i_math_mat3_apply(&m, i_point(p, nxyz)));
    }
    return RANT_MATH_OK;
}

RantMathResult rant_point_cloud_select(const RantPointCloudView *cloud, const uint8_t *keep, uint8_t *data, uint32_t *count){
    uint32_t n, i, need = 0, k = 0;
    RantMathResult r;
    if (!count || !keep) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_cloud(cloud, &n)) != RANT_MATH_OK) return r;
    for (i = 0; i < n; i++) need += keep[i] != 0;
    if ((r = i_math_room(count, need, data)) != RANT_MATH_OK) return r;
    /* each point lands at or before its own place, so data may be the cloud's own */
    for (i = 0; i < n; i++)
        if (keep[i]) memmove(data + (size_t)k++ * cloud->point_step, cloud->data + (size_t)i * cloud->point_step,
                             cloud->point_step);
    return RANT_MATH_OK;
}

RantMathResult rant_laser_scan_points(const RantLaserScanView *scan, RantDouble2 *out, uint32_t *count){
    uint32_t i, e;
    RantMathResult r;
    if (!scan || !count || (scan->range_count && !scan->ranges)) return RANT_MATH_ERR_ARGUMENT;
    if (!i_math_finite(scan->angle_min) || !i_math_finite(scan->angle_step)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, scan->range_count, out)) != RANT_MATH_OK) return r;
    e = scan->echoes ? scan->echoes : 1;
    for (i = 0; i < scan->range_count; i++){
        double d = scan->ranges[i], a = scan->angle_min + (double)(i / e) * scan->angle_step;
        if (i_math_finite(d) && d >= scan->range_min && d <= scan->range_max) out[i] = rant_double2(d * cos(a), d * sin(a));
        else out[i] = rant_double2(NAN, NAN);
    }
    return RANT_MATH_OK;
}

RantMathResult rant_image_depth_points(const RantImageView *depth, const RantCameraIntrinsics *camera, RantDouble3 *out,
                                       uint32_t *count){
    i_MathLens L;
    RantMathResult r;
    uint32_t x, y;
    size_t stride;
    int wide;
    if (!count || !camera) return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_image_check(depth)) != RANT_MATH_OK) return r;
    if (depth->format != RANT_IMAGE_MONO16 && depth->format != RANT_IMAGE_MONOF32) return RANT_MATH_ERR_FORMAT;
    if (depth->width != camera->width || depth->height != camera->height) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_lens(camera, 1, &L)) != RANT_MATH_OK) return r;
    if ((uint64_t)depth->width * depth->height > UINT32_MAX) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, depth->width * depth->height, out)) != RANT_MATH_OK) return r;
    wide = depth->format == RANT_IMAGE_MONOF32;
    stride = depth->stride ? depth->stride : (size_t)depth->width * (wide ? 4 : 2);
    for (y = 0; y < depth->height; y++){
        const uint8_t *row = depth->data + y * stride;
        for (x = 0; x < depth->width; x++){
            double d = wide ? i_math_scalar_get(RANT_SCALAR_F32, row + 4 * x)
                            : 0.001 * i_math_scalar_get(RANT_SCALAR_U16, row + 2 * x);
            RantDouble3 o;
            if (d > 0.0 && i_math_finite(d) && i_math_lens_ray(&L, rant_double2(x, y), &o) == RANT_MATH_OK && o.z > 0.0)
                *out++ = rant_double3(d, -o.x * (d / o.z), -o.y * (d / o.z));
            else
                *out++ = rant_double3(NAN, NAN, NAN);
        }
    }
    return RANT_MATH_OK;
}
