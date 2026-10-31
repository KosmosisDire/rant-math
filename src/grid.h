/* Grids of cells as Grid holds them, such as an occupancy or a height map. Cell 0 sits at the
 * origin and the cells run along its +x, +y and +z. A grid one cell high is flat. */
#ifndef RANT_MATH_GRID_H
#define RANT_MATH_GRID_H

#include "base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One number of kind per cell, x fastest, then y, then z, little endian. name needs no NUL. */
typedef struct {
    const char *name;
    size_t name_len;
    RantScalarKind kind;
    uint8_t *data;
    size_t size;                 /* the bytes data holds */
} RantGridFieldView;

typedef struct {
    RantPose origin;             /* the outer corner of cell 0 in the grid's parent frame */
    double resolution;           /* the edge of a cell */
    uint32_t cells_x, cells_y, cells_z;
    const RantGridFieldView *fields;
    uint32_t field_count;
} RantGridView;

/* The cell holding point, given in the parent frame, ERR_OUTSIDE when the grid holds none. A flat
 * grid takes any z. */
RANT_MATH_API RantMathResult rant_grid_cell(const RantGridView *grid, RantDouble3 point, RantInt3 *out);
/* The middle of a cell in the parent frame, on the xy plane of a flat grid. */
RANT_MATH_API RantMathResult rant_grid_center(const RantGridView *grid, RantInt3 cell, RantDouble3 *out);
/* A cell's number in the field of this name, a NUL ended string. ERR_OUTSIDE for a cell past the grid. */
RANT_MATH_API RantMathResult rant_grid_read_value(const RantGridView *grid, const char *field, RantInt3 cell, double *out);
/* Whole kinds are rounded and clamped, NaN to 0. */
RANT_MATH_API RantMathResult rant_grid_write_value(RantGridView *grid, const char *field, RantInt3 cell, double value);

#ifdef __cplusplus
}
#endif
#endif /* RANT_MATH_GRID_H */
