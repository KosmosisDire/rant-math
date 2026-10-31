#include "check.h"

static RantGridFieldView field(const char *name, RantScalarKind kind, uint8_t *data, size_t size){
    RantGridFieldView f;
    f.name = name;
    f.name_len = strlen(name);
    f.kind = kind;
    f.data = data;
    f.size = size;
    return f;
}

static RantGridView grid(RantPose origin, double resolution, uint32_t x, uint32_t y, uint32_t z){
    RantGridView g;
    memset(&g, 0, sizeof g);
    g.origin = origin;
    g.resolution = resolution;
    g.cells_x = x; g.cells_y = y; g.cells_z = z;
    return g;
}

/* Every cell's center maps back to it, and a point maps to the cell whose middle is within half a cell. */
static void cells(void){
    int flat, x, y, z, bad = 0, far = 0, k;
    for (flat = 0; flat < 2; flat++){
        RantGridView g = grid(random_pose(4.0), 0.25, 5, 4, flat ? 1 : 3);
        for (z = 0; z < (int)g.cells_z; z++)
            for (y = 0; y < 4; y++)
                for (x = 0; x < 5; x++){
                    RantDouble3 c, local;
                    RantInt3 back;
                    if (rant_grid_center(&g, rant_int3(x, y, z), &c) != RANT_MATH_OK
                        || rant_grid_cell(&g, c, &back) != RANT_MATH_OK || back.x != x || back.y != y || back.z != z) bad++;
                    local = rant_pose_apply_inverse(g.origin, c);
                    if (!near3(local, rant_double3(0.25 * (x + 0.5), 0.25 * (y + 0.5), flat ? 0.0 : 0.25 * (z + 0.5)), 1e-12)) bad++;
                }
        for (k = 0; k < 300; k++){
            RantDouble3 local = rant_double3(random_range(0, 1.25), random_range(0, 1.0), flat ? random_range(-9, 9) : random_range(0, 0.75));
            RantDouble3 p = rant_pose_apply(g.origin, local), c;
            RantInt3 cell;
            if (rant_grid_cell(&g, p, &cell) != RANT_MATH_OK){ bad++; continue; }
            rant_grid_center(&g, cell, &c);
            c = rant_pose_apply_inverse(g.origin, c);
            if (fabs(c.x - local.x) > 0.125 + 1e-12 || fabs(c.y - local.y) > 0.125 + 1e-12 || (!flat && fabs(c.z - local.z) > 0.125 + 1e-12)) far++;
        }
    }
    CHECK(bad == 0, "%d cells do not map back from their centers", bad);
    CHECK(far == 0, "%d points landed in a cell whose middle is not within half a cell", far);
}

static void edges(void){
    RantGridView g = grid(rant_pose_identity(), 0.5, 4, 2, 2);
    RantInt3 c = rant_int3(7, 7, 7);
    CHECK(rant_grid_cell(&g, rant_double3(0, 0, 0), &c) == RANT_MATH_OK && c.x == 0 && c.y == 0 && c.z == 0, "the outer corner is in cell 0");
    CHECK(rant_grid_cell(&g, rant_double3(1.999999, 0.999999, 0.999999), &c) == RANT_MATH_OK && c.x == 3 && c.y == 1 && c.z == 1, "the far corner's cell");
    c = rant_int3(7, 7, 7);
    CHECK(rant_grid_cell(&g, rant_double3(2.0, 0.5, 0.5), &c) == RANT_MATH_ERR_OUTSIDE && c.x == 7, "the far face is outside, out untouched");
    CHECK(rant_grid_cell(&g, rant_double3(-1e-12, 0.5, 0.5), &c) == RANT_MATH_ERR_OUTSIDE, "just before cell 0");
    CHECK(rant_grid_cell(&g, rant_double3(0.5, 0.5, 1.5), &c) == RANT_MATH_ERR_OUTSIDE, "above a 3D grid");
    g.cells_z = 1;
    CHECK(rant_grid_cell(&g, rant_double3(0.5, 0.5, 1e9), &c) == RANT_MATH_OK && c.z == 0, "a flat grid takes any z");
    CHECK(rant_grid_cell(&g, rant_double3(NAN, 0.5, 0.0), &c) == RANT_MATH_ERR_ARGUMENT, "a NaN point");
    g.resolution = 0.0;
    CHECK(rant_grid_cell(&g, rant_double3(0.5, 0.5, 0.0), &c) == RANT_MATH_ERR_FORMAT, "a resolution of 0");
    g.resolution = 0.5; g.cells_x = 0;
    CHECK(rant_grid_cell(&g, rant_double3(0.5, 0.5, 0.0), &c) == RANT_MATH_ERR_FORMAT, "no cells along x");
    CHECK(rant_grid_cell(NULL, rant_double3(0.5, 0.5, 0.0), &c) == RANT_MATH_ERR_ARGUMENT, "no grid");
}

static void values(void){
    static uint8_t occupancy[60], height[240], cost[120];
    RantGridFieldView f[3];
    RantGridView g = grid(rant_pose_identity(), 0.1, 5, 4, 3);
    double v;
    int16_t s;
    float h;
    f[0] = field("occupancy", RANT_SCALAR_KIND_U8, occupancy, sizeof occupancy);
    f[1] = field("height", RANT_SCALAR_KIND_F32, height, sizeof height);
    f[2] = field("cost", RANT_SCALAR_KIND_I16, cost, sizeof cost);
    g.fields = f;
    g.field_count = 3;
    CHECK(rant_grid_write_value(&g, "occupancy", rant_int3(2, 3, 1), 99.6) == RANT_MATH_OK && occupancy[37] == 100,
          "x fastest, then y, then z: cell (2, 3, 1) is number 37, rounded");
    CHECK(rant_grid_write_value(&g, "occupancy", rant_int3(0, 0, 0), -5.0) == RANT_MATH_OK && occupancy[0] == 0, "clamped at 0");
    CHECK(rant_grid_write_value(&g, "height", rant_int3(4, 3, 2), 1.375) == RANT_MATH_OK, "a float");
    memcpy(&h, height + 4 * 59, 4);
    CHECK(h == 1.375f, "in the last cell");
    CHECK(rant_grid_write_value(&g, "cost", rant_int3(1, 0, 0), -300.0) == RANT_MATH_OK, "a signed cost");
    memcpy(&s, cost + 2, 2);
    CHECK(s == -300, "little endian");
    CHECK(rant_grid_read_value(&g, "occupancy", rant_int3(2, 3, 1), &v) == RANT_MATH_OK && v == 100.0, "read back");
    CHECK(rant_grid_read_value(&g, "cost", rant_int3(1, 0, 0), &v) == RANT_MATH_OK && v == -300.0, "read back signed");
    v = 5.0;
    CHECK(rant_grid_read_value(&g, "occupancy", rant_int3(5, 0, 0), &v) == RANT_MATH_ERR_OUTSIDE && v == 5.0, "past x");
    CHECK(rant_grid_read_value(&g, "occupancy", rant_int3(0, -1, 0), &v) == RANT_MATH_ERR_OUTSIDE, "before y");
    CHECK(rant_grid_read_value(&g, "occupancy", rant_int3(0, 0, 3), &v) == RANT_MATH_ERR_OUTSIDE, "past z");
    CHECK(rant_grid_read_value(&g, "free", rant_int3(0, 0, 0), &v) == RANT_MATH_ERR_FORMAT, "no such field");
    CHECK(rant_grid_read_value(&g, NULL, rant_int3(0, 0, 0), &v) == RANT_MATH_ERR_ARGUMENT, "no field name");
    f[1].size = 239;
    CHECK(rant_grid_read_value(&g, "height", rant_int3(0, 0, 0), &v) == RANT_MATH_ERR_FORMAT, "a field short of one cell");
    f[1].size = 240;
    f[2].kind = (RantScalarKind)0;
    CHECK(rant_grid_read_value(&g, "cost", rant_int3(0, 0, 0), &v) == RANT_MATH_ERR_FORMAT, "a field of no kind");
    f[2].kind = RANT_SCALAR_KIND_I16;
    f[0].name_len = 3;
    CHECK(rant_grid_read_value(&g, "occ", rant_int3(2, 3, 1), &v) == RANT_MATH_OK && v == 100.0, "a name is its bytes, no NUL needed");
    g.cells_x = 0xFFFFFFFFu; g.cells_y = 0xFFFFFFFFu; g.cells_z = 0xFFFFFFFFu;
    CHECK(rant_grid_read_value(&g, "occ", rant_int3(0, 0, 0), &v) == RANT_MATH_ERR_FORMAT, "a grid too large for any data");
}

void test_grid(void){
    cells();
    edges();
    values();
}
