#include "internal.h"

static RantMathResult i_grid(const RantGridView *g){
    if (!g) return RANT_MATH_ERR_ARGUMENT;
    if (!(g->resolution > 0.0) || !i_math_finite(g->resolution) || !g->cells_x || !g->cells_y || !g->cells_z)
        return RANT_MATH_ERR_FORMAT;
    if (!i_math_finite3(g->origin.position) || (g->field_count && !g->fields)) return RANT_MATH_ERR_FORMAT;
    return RANT_MATH_OK;
}

RantMathResult rant_grid_cell(const RantGridView *grid, RantDouble3 point, RantInt3 *out){
    RantMathResult r;
    RantDouble3 local;
    double c[3];
    uint32_t n[3];
    int k;
    if (!out || !i_math_finite3(point)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_grid(grid)) != RANT_MATH_OK) return r;
    local = rant_pose_apply_inverse(grid->origin, point);
    c[0] = floor(local.x / grid->resolution);
    c[1] = floor(local.y / grid->resolution);
    c[2] = grid->cells_z == 1 ? 0.0 : floor(local.z / grid->resolution);
    n[0] = grid->cells_x; n[1] = grid->cells_y; n[2] = grid->cells_z;
    for (k = 0; k < 3; k++) if (c[k] < 0.0 || c[k] >= n[k] || c[k] > 2147483647.0) return RANT_MATH_ERR_OUTSIDE;
    *out = rant_int3((int32_t)c[0], (int32_t)c[1], (int32_t)c[2]);
    return RANT_MATH_OK;
}

RantMathResult rant_grid_center(const RantGridView *grid, RantInt3 cell, RantDouble3 *out){
    RantMathResult r;
    double z;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_grid(grid)) != RANT_MATH_OK) return r;
    z = grid->cells_z == 1 ? 0.0 : (cell.z + 0.5) * grid->resolution;
    *out = rant_pose_apply(grid->origin, rant_double3((cell.x + 0.5) * grid->resolution, (cell.y + 0.5) * grid->resolution, z));
    return RANT_MATH_OK;
}

/* The checked field of this name and the byte offset of a cell in it. */
static RantMathResult i_slot(const RantGridView *g, const char *name, RantInt3 cell, const RantGridFieldView **f,
                             size_t *at){
    RantMathResult r;
    uint64_t cells, bytes;
    size_t len;
    uint32_t i;
    int b;
    if (!name) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_grid(g)) != RANT_MATH_OK) return r;
    len = strlen(name);
    *f = NULL;
    for (i = 0; i < g->field_count && !*f; i++)
        if (g->fields[i].name_len == len && (!len || (g->fields[i].name && !memcmp(g->fields[i].name, name, len))))
            *f = &g->fields[i];
    if (!*f) return RANT_MATH_ERR_FORMAT;
    b = i_math_scalar_bytes((*f)->kind);
    cells = (uint64_t)g->cells_x * g->cells_y * g->cells_z;
    /* a grid whose cell count overflows 64 bits holds more than any data can */
    if (!b || !(*f)->data || (uint64_t)g->cells_y * g->cells_z > UINT64_MAX / g->cells_x || cells > UINT64_MAX / 8
        || cells * b > (*f)->size) return RANT_MATH_ERR_FORMAT;
    if (cell.x < 0 || cell.y < 0 || cell.z < 0 || (uint32_t)cell.x >= g->cells_x || (uint32_t)cell.y >= g->cells_y
        || (uint32_t)cell.z >= g->cells_z) return RANT_MATH_ERR_OUTSIDE;
    bytes = ((uint64_t)cell.x + (uint64_t)g->cells_x * ((uint64_t)cell.y + (uint64_t)g->cells_y * (uint64_t)cell.z)) * b;
    *at = (size_t)bytes;
    return RANT_MATH_OK;
}

RantMathResult rant_grid_read_value(const RantGridView *grid, const char *field, RantInt3 cell, double *out){
    const RantGridFieldView *f;
    size_t at;
    RantMathResult r;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_slot(grid, field, cell, &f, &at)) != RANT_MATH_OK) return r;
    *out = i_math_scalar_get(f->kind, f->data + at);
    return RANT_MATH_OK;
}

RantMathResult rant_grid_write_value(RantGridView *grid, const char *field, RantInt3 cell, double value){
    const RantGridFieldView *f;
    size_t at;
    RantMathResult r;
    if ((r = i_slot(grid, field, cell, &f, &at)) != RANT_MATH_OK) return r;
    i_math_scalar_put(f->kind, f->data + at, value);
    return RANT_MATH_OK;
}
