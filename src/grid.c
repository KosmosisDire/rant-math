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

/* The checked field of this name, holding a number for each of the grid's cells. */
static RantMathResult i_grid_field(const RantGridView *g, const char *name, const RantGridFieldView **f, uint64_t *cells){
    RantMathResult r;
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
    *cells = (uint64_t)g->cells_x * g->cells_y * g->cells_z;
    /* a grid whose cell count overflows 64 bits holds more than any data can */
    if (!b || !(*f)->data || (uint64_t)g->cells_y * g->cells_z > UINT64_MAX / g->cells_x || *cells > UINT64_MAX / 8
        || *cells * b > (*f)->size) return RANT_MATH_ERR_FORMAT;
    return RANT_MATH_OK;
}

/* The checked field of this name and the byte offset of a cell in it. */
static RantMathResult i_slot(const RantGridView *g, const char *name, RantInt3 cell, const RantGridFieldView **f,
                             size_t *at){
    RantMathResult r;
    uint64_t cells, bytes;
    int b;
    if ((r = i_grid_field(g, name, f, &cells)) != RANT_MATH_OK) return r;
    b = i_math_scalar_bytes((*f)->kind);
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

RantMathResult rant_grid_read_values(const RantGridView *grid, const char *field, double *out, uint32_t *count){
    const RantGridFieldView *f;
    uint64_t cells;
    uint32_t i;
    RantMathResult r;
    int b;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_grid_field(grid, field, &f, &cells)) != RANT_MATH_OK) return r;
    if (cells > UINT32_MAX) return RANT_MATH_ERR_FORMAT;
    if ((r = i_math_room(count, (uint32_t)cells, out)) != RANT_MATH_OK) return r;
    b = i_math_scalar_bytes(f->kind);
    for (i = 0; i < (uint32_t)cells; i++) out[i] = i_math_scalar_get(f->kind, f->data + (size_t)i * b);
    return RANT_MATH_OK;
}

RantMathResult rant_grid_read_colors(const RantGridView *grid, RantColor *out, uint32_t *count){
    const RantGridFieldView *f = NULL;
    uint64_t cells;
    uint32_t i;
    int alpha = 1;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_grid_field(grid, "rgba", &f, &cells)) == RANT_MATH_ERR_FORMAT && !f){
        alpha = 0;
        r = i_grid_field(grid, "rgb", &f, &cells);
    }
    if (r != RANT_MATH_OK) return r;
    if (i_math_scalar_bytes(f->kind) != 4 || cells > UINT32_MAX) return RANT_MATH_ERR_FORMAT;
    if ((r = i_math_room(count, (uint32_t)cells, out)) != RANT_MATH_OK) return r;
    for (i = 0; i < (uint32_t)cells; i++){
        const uint8_t *p = f->data + (size_t)i * 4;
        out[i] = rant_color(p[2], p[1], p[0], alpha ? p[3] : 255);
    }
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
