#include "internal.h"

RantMathResult rant_points_bounds(const RantDouble3 *points, uint32_t count, RantAlignedBox *out){
    uint32_t i;
    RantAlignedBox b;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    b.min = b.max = points[0];
    for (i = 1; i < count; i++){
        b.min = rant_double3_min(b.min, points[i]);
        b.max = rant_double3_max(b.max, points[i]);
    }
    *out = b;
    return RANT_MATH_OK;
}

/* Summed from the first point, which keeps the digits of a set far from the origin. */
RantMathResult rant_points_mean(const RantDouble3 *points, uint32_t count, RantDouble3 *out){
    RantDouble3 sum = rant_double3(0, 0, 0);
    uint32_t i;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    for (i = 1; i < count; i++) sum = rant_double3_add(sum, rant_double3_subtract(points[i], points[0]));
    *out = rant_double3_add(points[0], rant_double3_scale(sum, 1.0 / count));
    return RANT_MATH_OK;
}

RantMathResult rant_points_covariance(const RantDouble3 *points, uint32_t count, RantCovariance3 *out){
    RantCovariance3 c;
    RantDouble3 mean;
    uint32_t i;
    int r, k;
    if (!out || rant_points_mean(points, count, &mean) != RANT_MATH_OK) return RANT_MATH_ERR_ARGUMENT;
    memset(&c, 0, sizeof c);
    for (i = 0; i < count; i++){
        RantDouble3 d = rant_double3_subtract(points[i], mean);
        double v[3];
        v[0] = d.x; v[1] = d.y; v[2] = d.z;
        for (r = 0; r < 3; r++) for (k = 0; k < 3; k++) c.m[r][k] += v[r] * v[k];
    }
    for (r = 0; r < 3; r++) for (k = 0; k < 3; k++) c.m[r][k] /= count;
    *out = c;
    return RANT_MATH_OK;
}

RantMathResult rant_points_oriented_box(const RantDouble3 *points, uint32_t count, RantOrientedBox *out){
    RantCovariance3 cov;
    RantDouble3 mean, lo = rant_double3(HUGE_VAL, HUGE_VAL, HUGE_VAL), hi = rant_double3(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL), x, y, z;
    i_MathMat3 a, v, r;
    double values[3];
    uint32_t i;
    int k;
    if (!out || rant_points_covariance(points, count, &cov) != RANT_MATH_OK ||
        rant_points_mean(points, count, &mean) != RANT_MATH_OK) return RANT_MATH_ERR_ARGUMENT;
    memcpy(a.m, cov.m, sizeof a.m);
    i_math_mat3_eigen(&a, values, &v);
    x = rant_double3_normalize(rant_double3(v.m[0][2], v.m[1][2], v.m[2][2]));
    y = rant_double3_normalize(rant_double3(v.m[0][1], v.m[1][1], v.m[2][1]));
    z = rant_double3_cross(x, y);
    for (k = 0; k < 3; k++){
        RantDouble3 axis = k == 0 ? x : (k == 1 ? y : z);
        r.m[0][k] = axis.x; r.m[1][k] = axis.y; r.m[2][k] = axis.z;
    }
    for (i = 0; i < count; i++){
        RantDouble3 d = rant_double3_subtract(points[i], mean);
        RantDouble3 local = rant_double3(rant_double3_dot(d, x), rant_double3_dot(d, y), rant_double3_dot(d, z));
        lo = rant_double3_min(lo, local);
        hi = rant_double3_max(hi, local);
    }
    out->pose.position = rant_double3_add(mean, i_math_mat3_apply(&r, rant_double3_lerp(lo, hi, 0.5)));
    out->pose.orientation = i_math_quaternion_from_mat3(&r);
    out->size = rant_double3_subtract(hi, lo);
    return RANT_MATH_OK;
}

/* Welzl's smallest ball, run as nested loops over the points in a shuffled order. A ball is
 * set by the points on its boundary, up to four. */
typedef struct { RantDouble3 c; double r; } i_Ball;

static int outside(i_Ball b, RantDouble3 p){ return rant_double3_distance(b.c, p) > b.r * (1.0 + 1e-12); }

static i_Ball ball2(RantDouble3 a, RantDouble3 b){
    i_Ball r;
    r.c = rant_double3_lerp(a, b, 0.5);
    r.r = 0.5 * rant_double3_distance(a, b);
    return r;
}

static int holds(i_Ball b, RantDouble3 p){ return !outside(b, p); }

/* Through three points with its center in their plane, or for three in a line the ball on
 * the two farthest apart. */
static i_Ball ball3(RantDouble3 a, RantDouble3 b, RantDouble3 c){
    RantDouble3 ab = rant_double3_subtract(b, a), ac = rant_double3_subtract(c, a), n = rant_double3_cross(ab, ac);
    double nn = rant_double3_dot(n, n);
    i_Ball r;
    if (nn > I_MATH_TINY * rant_double3_dot(ab, ab) * rant_double3_dot(ac, ac)){
        RantDouble3 off = rant_double3_scale(rant_double3_add(rant_double3_scale(rant_double3_cross(n, ab), rant_double3_dot(ac, ac)),
                                                              rant_double3_scale(rant_double3_cross(ac, n), rant_double3_dot(ab, ab))),
                                             0.5 / nn);
        r.c = rant_double3_add(a, off);
        r.r = rant_double3_length(off);
        return r;
    }
    r = ball2(a, b);
    if (ball2(a, c).r > r.r) r = ball2(a, c);
    if (ball2(b, c).r > r.r) r = ball2(b, c);
    return r;
}

/* Through four points, or for four in a plane the smallest ball of three that holds the fourth. */
static i_Ball ball4(RantDouble3 a, RantDouble3 b, RantDouble3 c, RantDouble3 d){
    RantDouble3 ab = rant_double3_subtract(b, a), ac = rant_double3_subtract(c, a), ad = rant_double3_subtract(d, a);
    RantDouble3 c12 = rant_double3_cross(ac, ad), c20 = rant_double3_cross(ad, ab), c01 = rant_double3_cross(ab, ac);
    double det = rant_double3_dot(ab, c12), scale = rant_double3_length(ab) * rant_double3_length(ac) * rant_double3_length(ad);
    i_Ball r, best;
    RantDouble3 p[4];
    int i, found = 0;
    if (fabs(det) > 1e-9 * scale){
        RantDouble3 off = rant_double3_scale(rant_double3_add(rant_double3_add(rant_double3_scale(c12, rant_double3_dot(ab, ab)),
                                                                               rant_double3_scale(c20, rant_double3_dot(ac, ac))),
                                                              rant_double3_scale(c01, rant_double3_dot(ad, ad))), 0.5 / det);
        r.c = rant_double3_add(a, off);
        r.r = rant_double3_length(off);
        return r;
    }
    p[0] = a; p[1] = b; p[2] = c; p[3] = d;
    best = ball3(a, b, c);
    for (i = 0; i < 4; i++){
        i_Ball t = ball3(p[i == 0 ? 1 : 0], p[i <= 1 ? 2 : 1], p[i <= 2 ? 3 : 2]);
        if (holds(t, p[i]) && (!found || t.r < best.r)){ best = t; found = 1; }
        else if (!found && t.r > best.r) best = t;
    }
    return best;
}

static i_Ball enclose(RantDouble3 *p, uint32_t n){
    uint64_t s = 0x9E3779B97F4A7C15ull;
    uint32_t i, j, k, l;
    i_Ball b;
    for (i = n; i > 1; i--){
        uint32_t pick;
        RantDouble3 t;
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        pick = (uint32_t)(s % i);
        t = p[i - 1]; p[i - 1] = p[pick]; p[pick] = t;
    }
    b.c = p[0];
    b.r = 0.0;
    for (i = 1; i < n; i++){
        if (!outside(b, p[i])) continue;
        b.c = p[i];
        b.r = 0.0;
        for (j = 0; j < i; j++){
            if (!outside(b, p[j])) continue;
            b = ball2(p[i], p[j]);
            for (k = 0; k < j; k++){
                if (!outside(b, p[k])) continue;
                b = ball3(p[i], p[j], p[k]);
                for (l = 0; l < k; l++)
                    if (outside(b, p[l])) b = ball4(p[i], p[j], p[k], p[l]);
            }
        }
    }
    return b;
}

RantMathResult rant_points_enclosing_sphere(const RantDouble3 *points, uint32_t count, RantSphere *out){
    RantDouble3 *p;
    i_Ball b;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    p = (RantDouble3 *)malloc(sizeof(RantDouble3) * count);
    if (!p) return RANT_MATH_ERR_MEMORY;
    memcpy(p, points, sizeof(RantDouble3) * count);
    b = enclose(p, count);
    free(p);
    out->center = b.c;
    out->radius = b.r;
    return RANT_MATH_OK;
}

RantMathResult rant_points2d_bounds(const RantDouble2 *points, uint32_t count, RantAlignedBox2D *out){
    uint32_t i;
    RantAlignedBox2D b;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    b.min = b.max = points[0];
    for (i = 1; i < count; i++){
        b.min = rant_double2_min(b.min, points[i]);
        b.max = rant_double2_max(b.max, points[i]);
    }
    *out = b;
    return RANT_MATH_OK;
}

RantMathResult rant_points2d_mean(const RantDouble2 *points, uint32_t count, RantDouble2 *out){
    RantDouble2 sum = rant_double2(0, 0);
    uint32_t i;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    for (i = 1; i < count; i++) sum = rant_double2_add(sum, rant_double2_subtract(points[i], points[0]));
    *out = rant_double2_add(points[0], rant_double2_scale(sum, 1.0 / count));
    return RANT_MATH_OK;
}

static int by_x_then_y(const void *a, const void *b){
    const RantDouble2 *p = (const RantDouble2 *)a, *q = (const RantDouble2 *)b;
    if (p->x != q->x) return p->x < q->x ? -1 : 1;
    if (p->y != q->y) return p->y < q->y ? -1 : 1;
    return 0;
}

static double turn(RantDouble2 o, RantDouble2 a, RantDouble2 b){
    return rant_double2_cross(rant_double2_subtract(a, o), rant_double2_subtract(b, o));
}

/* Andrew's monotone chain into scratch of 2 count + 1 points. Returns how many it holds. */
static uint32_t hull_of(const RantDouble2 *points, uint32_t count, RantDouble2 *sorted, RantDouble2 *h){
    uint32_t n = 0, i, k = 0, lower;
    memcpy(sorted, points, sizeof(RantDouble2) * count);
    qsort(sorted, count, sizeof(RantDouble2), by_x_then_y);
    for (i = 0; i < count; i++) if (!n || by_x_then_y(&sorted[n - 1], &sorted[i]) != 0) sorted[n++] = sorted[i];
    if (n < 3){
        memcpy(h, sorted, sizeof(RantDouble2) * n);
        return n;
    }
    for (i = 0; i < n; i++){
        while (k >= 2 && turn(h[k - 2], h[k - 1], sorted[i]) <= 0.0) k--;
        h[k++] = sorted[i];
    }
    lower = k + 1;
    for (i = n - 1; i-- > 0;){
        while (k >= lower && turn(h[k - 2], h[k - 1], sorted[i]) <= 0.0) k--;
        h[k++] = sorted[i];
    }
    return k - 1;
}

RantMathResult rant_points2d_hull(const RantDouble2 *points, uint32_t count, RantDouble2 *out, uint32_t *out_count){
    RantDouble2 *scratch;
    uint32_t n;
    RantMathResult r;
    if (!points || !count || !out_count) return RANT_MATH_ERR_ARGUMENT;
    scratch = (RantDouble2 *)malloc(sizeof(RantDouble2) * (3 * (size_t)count + 1));
    if (!scratch) return RANT_MATH_ERR_MEMORY;
    n = hull_of(points, count, scratch, scratch + count);
    if ((r = i_math_room(out_count, n, out)) == RANT_MATH_OK) memcpy(out, scratch + count, sizeof(RantDouble2) * n);
    free(scratch);
    return r;
}

/* The smallest rectangle lies along an edge of the hull, so each edge is tried. */
RantMathResult rant_points2d_oriented_box(const RantDouble2 *points, uint32_t count, RantOrientedBox2D *out){
    RantDouble2 *scratch, *h;
    uint32_t n, i, k;
    double best = HUGE_VAL;
    RantOrientedBox2D r;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    scratch = (RantDouble2 *)malloc(sizeof(RantDouble2) * (3 * (size_t)count + 1));
    if (!scratch) return RANT_MATH_ERR_MEMORY;
    h = scratch + count;
    n = hull_of(points, count, scratch, h);
    r.pose.position = h[0];
    r.pose.angle = 0.0;
    r.size = rant_double2(0, 0);
    for (i = 0; i < n && n > 1; i++){
        RantDouble2 e = rant_double2_normalize(rant_double2_subtract(h[(i + 1) % n], h[i])), side = rant_double2_perpendicular(e);
        double lo = HUGE_VAL, hi = -HUGE_VAL, lo2 = HUGE_VAL, hi2 = -HUGE_VAL, area;
        for (k = 0; k < n; k++){
            RantDouble2 d = rant_double2_subtract(h[k], h[i]);
            double u = rant_double2_dot(d, e), v = rant_double2_dot(d, side);
            lo = i_math_min(lo, u); hi = i_math_max(hi, u);
            lo2 = i_math_min(lo2, v); hi2 = i_math_max(hi2, v);
        }
        area = (hi - lo) * (hi2 - lo2);
        if (area < best){
            double angle = rant_double2_heading(e);
            best = area;
            r.pose.position = rant_double2_add(h[i], rant_double2_add(rant_double2_scale(e, 0.5 * (lo + hi)),
                                                                      rant_double2_scale(side, 0.5 * (lo2 + hi2))));
            r.size = rant_double2(hi - lo, hi2 - lo2);
            if (angle > 0.5 * I_MATH_PI) angle -= I_MATH_PI;
            else if (angle <= -0.5 * I_MATH_PI) angle += I_MATH_PI;
            r.pose.angle = angle;
        }
    }
    free(scratch);
    *out = r;
    return RANT_MATH_OK;
}

/* Points on a plane have their smallest ball centered in it, so the circle is that ball's cut. */
RantMathResult rant_points2d_enclosing_circle(const RantDouble2 *points, uint32_t count, RantCircle *out){
    RantDouble3 *p;
    uint32_t i;
    i_Ball b;
    if (!points || !count || !out) return RANT_MATH_ERR_ARGUMENT;
    p = (RantDouble3 *)malloc(sizeof(RantDouble3) * count);
    if (!p) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i < count; i++) p[i] = i_math_lift(points[i]);
    b = enclose(p, count);
    free(p);
    out->center = i_math_flat(b.c);
    out->radius = b.r;
    return RANT_MATH_OK;
}
