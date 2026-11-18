#include "internal.h"

/* The points a fit reads, 2D ones on z = 0, and their weights. A fit takes the count points
 * that index names, or the first count when index is NULL. */
typedef struct {
    const RantDouble3 *p3;
    const RantDouble2 *p2;
    const double *w;
} i_Points;

static i_Points i_points(const RantDouble3 *p3, const RantDouble2 *p2, const RantFitOptions *o){
    i_Points s;
    s.p3 = p3;
    s.p2 = p2;
    s.w = o ? o->weights : NULL;
    return s;
}

static uint32_t i_ix(const uint32_t *index, uint32_t k){ return index ? index[k] : k; }
static RantDouble3 i_at(const i_Points *s, uint32_t i){ return s->p3 ? s->p3[i] : i_math_lift(s->p2[i]); }
static double i_weight(const double *w, uint32_t i, int weighted){ return weighted && w ? w[i] : 1.0; }
static RantDouble3 i_d3(const double *m){ return rant_double3(m[0], m[1], m[2]); }
static void i_put3(double *m, RantDouble3 v){ m[0] = v.x; m[1] = v.y; m[2] = v.z; }
static RantDouble3 i_column(const i_MathMat3 *v, int c){ return rant_double3(v->m[0][c], v->m[1][c], v->m[2][c]); }
static RantDouble3 i_negate(RantDouble3 v){ return rant_double3_scale(v, -1.0); }

/* The weighted mean, summed from the first point, which keeps the digits of a set far from
 * the origin. Returns the total weight. */
static double i_mean(const i_Points *s, const uint32_t *index, uint32_t k, int weighted, RantDouble3 *mean){
    RantDouble3 o, sum = rant_double3(0, 0, 0);
    double total = 0.0;
    uint32_t j;
    if (!k) return 0.0;
    o = i_at(s, i_ix(index, 0));
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double w = i_weight(s->w, i, weighted);
        sum = rant_double3_add(sum, rant_double3_scale(rant_double3_subtract(i_at(s, i), o), w));
        total += w;
    }
    if (!(total > 0.0)) return 0.0;
    *mean = rant_double3_add(o, rant_double3_scale(sum, 1.0 / total));
    return total;
}

/* The mean and the scatter about it, the sum of w d d^T. Returns the total weight. */
static double i_scatter(const i_Points *s, const uint32_t *index, uint32_t k, int weighted, RantDouble3 *mean, i_MathMat3 *scatter){
    double total = i_mean(s, index, k, weighted, mean);
    uint32_t j;
    int r, c;
    if (!(total > 0.0)) return 0.0;
    memset(scatter, 0, sizeof *scatter);
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double w = i_weight(s->w, i, weighted), d[3];
        RantDouble3 v = rant_double3_subtract(i_at(s, i), *mean);
        d[0] = v.x; d[1] = v.y; d[2] = v.z;
        for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) scatter->m[r][c] += w * d[r] * d[c];
    }
    return total;
}

/* A kind of model: the points that make one, the doubles that hold it, its least squares fit,
 * unweighted for a RANSAC draw and writing only on success, and a point's distance from it. */
typedef struct {
    uint32_t sample, size;
    RantMathResult (*fit)(const void *data, const uint32_t *index, uint32_t count, int weighted, double *model);
    double (*error)(const void *data, const double *model, uint32_t i);
} i_Model;

/* RANSAC stops once it is this sure some draw held only inliers. */
#define I_FIT_SURE 0.999

static uint32_t i_tries_needed(uint32_t nearby, uint32_t n, uint32_t sample, uint32_t cap){
    double all = pow((double)nearby / n, (double)sample), t;
    if (all >= 1.0) return 1;
    t = log(1.0 - I_FIT_SURE) / log1p(-all);
    return t < (double)cap ? (uint32_t)ceil(t) : cap;
}

static uint32_t i_near(const i_Model *m, const void *data, uint32_t n, double tolerance, const double *model, uint32_t *set){
    uint32_t i, k = 0;
    for (i = 0; i < n; i++) if (m->error(data, model, i) <= tolerance) set[k++] = i;
    return k;
}

/* MSAC scoring: an inlier costs its squared error and an outlier the tolerance squared. The
 * best model is then fitted again to its inliers until they stop changing. */
static RantMathResult i_ransac(const i_Model *m, const void *data, uint32_t n, const RantFitOptions *o, double *model){
    uint32_t cap = o->tries ? o->tries : 1000, need = cap, tries, kept, round, i, j, k;
    double tol = o->tolerance, best_score = HUGE_VAL, *cand, *best;
    uint32_t *pick, *set, *next;
    i_MathRng rng = i_math_rng(o->seed);
    void *mem = malloc(sizeof(double) * 2 * m->size + sizeof(uint32_t) * (m->sample + 2 * (size_t)n));
    int found = 0;
    RantMathResult r = RANT_MATH_OK;
    if (!mem) return RANT_MATH_ERR_MEMORY;
    cand = (double *)mem;
    best = cand + m->size;
    pick = (uint32_t *)(best + m->size);
    set = pick + m->sample;
    next = set + n;
    for (tries = 0; tries < need; tries++){
        double score = 0.0;
        uint32_t nearby = 0;
        for (k = 0; k < m->sample; k++)
            do {
                pick[k] = (uint32_t)(i_math_rng_unit(&rng) * n);
                for (j = 0; j < k && pick[j] != pick[k]; j++){}
            } while (j < k);
        if (m->fit(data, pick, m->sample, 0, cand) != RANT_MATH_OK) continue;
        for (i = 0; i < n && score < best_score; i++){
            double e = m->error(data, cand, i);
            if (e <= tol){ nearby++; score += e * e; }
            else score += tol * tol;
        }
        if (score < best_score){
            best_score = score;
            memcpy(best, cand, sizeof(double) * m->size);
            found = 1;
            need = i_tries_needed(nearby, n, m->sample, cap);
        }
    }
    if (!found){
        free(mem);
        return RANT_MATH_ERR_DEGENERATE;
    }
    kept = i_near(m, data, n, tol, best, set);
    for (round = 0; round < 16; round++){
        uint32_t again, *t;
        if (m->fit(data, set, kept, 1, cand) != RANT_MATH_OK){
            if (round == 0) r = RANT_MATH_ERR_DEGENERATE;
            break;
        }
        memcpy(best, cand, sizeof(double) * m->size);
        again = i_near(m, data, n, tol, best, next);
        t = set; set = next; next = t;
        if (again == kept && !memcmp(set, next, sizeof(uint32_t) * kept)) break;
        kept = again;
    }
    if (r == RANT_MATH_OK){
        memcpy(model, best, sizeof(double) * m->size);
        if (o->inliers){
            memset(o->inliers, 0, n);
            for (k = 0; k < kept; k++) o->inliers[set[k]] = 1;
        }
    }
    free(mem);
    return r;
}

static RantMathResult i_fit(const i_Model *m, const void *data, uint32_t n, const RantFitOptions *o, double *model){
    static const RantFitOptions plain = { NULL, 0.0, 0, 0, NULL };
    RantMathResult r;
    uint32_t i;
    if (!o) o = &plain;
    if (n < m->sample || !(o->tolerance >= 0.0)) return RANT_MATH_ERR_ARGUMENT;
    if (o->weights)
        for (i = 0; i < n; i++) if (!(o->weights[i] >= 0.0 && o->weights[i] < HUGE_VAL)) return RANT_MATH_ERR_ARGUMENT;
    if (o->tolerance > 0.0) return i_ransac(m, data, n, o, model);
    r = m->fit(data, NULL, n, 1, model);
    if (r == RANT_MATH_OK && o->inliers) memset(o->inliers, 1, n);
    return r;
}

/* The plane: its point, then its normal. Points on one line leave the middle spread at
 * rounding noise. */
static RantMathResult plane_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    RantDouble3 m, n;
    i_MathMat3 scatter, v;
    double values[3], total, s;
    if (!((total = i_scatter((const i_Points *)data, index, k, weighted, &m, &scatter)) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    i_math_mat3_eigen(&scatter, values, &v);
    if (!(values[1] > 1e-12 * values[2])) return RANT_MATH_ERR_DEGENERATE;
    n = rant_double3_normalize(i_column(&v, 0));
    s = rant_double3_dot(n, m);
    /* an origin within the points' own scatter off the plane lies on no side of it */
    if (fabs(s) <= i_math_max(sqrt(i_math_max(values[0], 0.0) / total), 1e-12 * rant_double3_length(m))){
        double up = fabs(n.z) > 1e-9 ? n.z : (fabs(n.y) > 1e-9 ? n.y : n.x);
        if (up < 0.0) n = i_negate(n);
    } else if (s > 0.0) n = i_negate(n);
    i_put3(model, m);
    i_put3(model + 3, n);
    return RANT_MATH_OK;
}

static double plane_error(const void *data, const double *model, uint32_t i){
    return fabs(rant_double3_dot(rant_double3_subtract(i_at((const i_Points *)data, i), i_d3(model)), i_d3(model + 3)));
}

/* The line: the segment's a, then its b, spanning the points that count. */
static RantMathResult line_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    const i_Points *s = (const i_Points *)data;
    RantDouble3 m, d;
    i_MathMat3 scatter, v;
    double values[3], lo = HUGE_VAL, hi = -HUGE_VAL;
    uint32_t j;
    if (!(i_scatter(s, index, k, weighted, &m, &scatter) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    i_math_mat3_eigen(&scatter, values, &v);
    if (!(values[2] > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    d = rant_double3_normalize(i_column(&v, 2));
    if (rant_double3_dot(rant_double3_subtract(i_at(s, i_ix(index, k - 1)), i_at(s, i_ix(index, 0))), d) < 0.0) d = i_negate(d);
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double t;
        if (!(i_weight(s->w, i, weighted) > 0.0)) continue;
        t = rant_double3_dot(rant_double3_subtract(i_at(s, i), m), d);
        lo = i_math_min(lo, t);
        hi = i_math_max(hi, t);
    }
    i_put3(model, rant_double3_add(m, rant_double3_scale(d, lo)));
    i_put3(model + 3, rant_double3_add(m, rant_double3_scale(d, hi)));
    return RANT_MATH_OK;
}

static double line_error(const void *data, const double *model, uint32_t i){
    RantDouble3 a = i_d3(model), u = rant_double3_normalize(rant_double3_subtract(i_d3(model + 3), a));
    RantDouble3 p = rant_double3_subtract(i_at((const i_Points *)data, i), a);
    return rant_double3_length(rant_double3_subtract(p, rant_double3_scale(u, rant_double3_dot(p, u))));
}

/* A circle or a sphere fitted in the points' own scale about their mean. Its center c moves
 * and its radius stays the mean distance from c, which is best for that c. */
typedef struct {
    const i_Points *s;
    const uint32_t *index;
    uint32_t k;
    int weighted;
    RantDouble3 mean;
    double scale;
} i_SphereFit;

static RantDouble3 ball_point(const i_SphereFit *b, uint32_t j){
    return rant_double3_scale(rant_double3_subtract(i_at(b->s, i_ix(b->index, j)), b->mean), 1.0 / b->scale);
}

/* The weighted sum of squared errors with the center at c, and its radius. With jtj, also the
 * Gauss Newton system jtj step = rhs there. */
static double ball_cost(const i_SphereFit *b, RantDouble3 c, double *radius, i_MathMat3 *jtj, RantDouble3 *rhs){
    double total = 0.0, sum = 0.0, cost = 0.0, r, u[3], su[3] = { 0, 0, 0 };
    uint32_t j;
    int p, q;
    for (j = 0; j < b->k; j++){
        double w = i_weight(b->s->w, i_ix(b->index, j), b->weighted);
        total += w;
        sum += w * rant_double3_distance(ball_point(b, j), c);
    }
    r = sum / total;
    if (jtj){
        memset(jtj, 0, sizeof *jtj);
        *rhs = rant_double3(0, 0, 0);
    }
    for (j = 0; j < b->k; j++){
        double w = i_weight(b->s->w, i_ix(b->index, j), b->weighted);
        RantDouble3 d = rant_double3_subtract(ball_point(b, j), c);
        double len = rant_double3_length(d), e = len - r;
        cost += w * e * e;
        if (!jtj) continue;
        if (len > 0.0) d = rant_double3_scale(d, 1.0 / len);
        else d = rant_double3(0, 0, 0);
        u[0] = d.x; u[1] = d.y; u[2] = d.z;
        for (p = 0; p < 3; p++){
            su[p] += w * u[p];
            for (q = 0; q < 3; q++) jtj->m[p][q] += w * u[p] * u[q];
        }
        *rhs = rant_double3_add(*rhs, rant_double3_scale(d, w * e));
    }
    if (jtj)
        for (p = 0; p < 3; p++) for (q = 0; q < 3; q++) jtj->m[p][q] -= su[p] * su[q] / total;
    *radius = r;
    return cost;
}

static int ball_step(const i_MathMat3 *jtj, RantDouble3 rhs, double damp, int dim, RantDouble3 *step){
    double a[9], b[3], x[3];
    RantMatrixView av, bv, xv;
    int r, c;
    av.data = a; av.rows = av.cols = (uint32_t)dim;
    bv.data = b; bv.rows = (uint32_t)dim; bv.cols = 1;
    xv.data = x; xv.rows = (uint32_t)dim; xv.cols = 1;
    for (r = 0; r < dim; r++) for (c = 0; c < dim; c++) a[r * dim + c] = jtj->m[r][c] + (r == c ? damp : 0.0);
    b[0] = rhs.x; b[1] = rhs.y; b[2] = rhs.z;
    if (rant_matrix_solve(av, bv, xv) != RANT_MATH_OK) return 0;
    *step = rant_double3(x[0], x[1], dim == 3 ? x[2] : 0.0);
    return 1;
}

/* The center and then the radius. Kasa's algebraic fit starts Levenberg Marquardt on the
 * distances from the surface. dim 2 keeps it on z = 0. */
static RantMathResult ball_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model, int dim){
    i_SphereFit b;
    i_MathMat3 scatter, v, jtj;
    RantDouble3 c = rant_double3(0, 0, 0), rhs, lift = rant_double3(0, 0, 0);
    double values[3], total, cost, radius, lambda = 1e-3;
    uint32_t j;
    int e, it;
    b.s = (const i_Points *)data;
    b.index = index;
    b.k = k;
    b.weighted = weighted;
    total = i_scatter(b.s, index, k, weighted, &b.mean, &scatter);
    if (!(total > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    i_math_mat3_eigen(&scatter, values, &v);
    /* a sphere's points in one plane, or a circle's on one line */
    if (!(values[3 - dim] > 1e-12 * values[2])) return RANT_MATH_ERR_DEGENERATE;
    b.scale = sqrt((values[0] + values[1] + values[2]) / total);
    /* about the mean, Kasa's center is half the inverse scatter times the sum of w |q|^2 q */
    for (j = 0; j < k; j++){
        RantDouble3 q = ball_point(&b, j);
        lift = rant_double3_add(lift, rant_double3_scale(q, i_weight(b.s->w, i_ix(index, j), weighted) * rant_double3_length_squared(q)));
    }
    for (e = 3 - dim; e < 3; e++){
        RantDouble3 axis = i_column(&v, e);
        c = rant_double3_add(c, rant_double3_scale(axis, 0.5 * rant_double3_dot(axis, lift) * b.scale * b.scale / values[e]));
    }
    cost = ball_cost(&b, c, &radius, &jtj, &rhs);
    for (it = 0; it < 200 && cost > 0.0 && lambda < 1e12; it++){
        RantDouble3 step, next;
        double next_radius, next_cost, gain, mu = (jtj.m[0][0] + jtj.m[1][1] + jtj.m[2][2]) / dim;
        if (!ball_step(&jtj, rhs, lambda * (mu > 0.0 ? mu : 1.0), dim, &step)){
            lambda *= 10.0;
            continue;
        }
        if (rant_double3_length(step) <= 1e-14 * (1.0 + rant_double3_length(c))) break;
        next = rant_double3_add(c, step);
        next_cost = ball_cost(&b, next, &next_radius, NULL, NULL);
        gain = 2.0 * rant_double3_dot(step, rhs) - rant_double3_dot(step, i_math_mat3_apply(&jtj, step));
        /* near the answer a step gains less than the cost's rounding, so the model is trusted */
        if (next_cost < cost || gain <= 1e-14 * cost){
            c = next;
            cost = ball_cost(&b, c, &radius, &jtj, &rhs);
            lambda = i_math_max(lambda * 0.1, 1e-12);
        } else lambda *= 10.0;
    }
    i_put3(model, rant_double3_add(b.mean, rant_double3_scale(c, b.scale)));
    model[3] = radius * b.scale;
    return RANT_MATH_OK;
}

static RantMathResult circle_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    return ball_fit(data, index, k, weighted, model, 2);
}
static RantMathResult sphere_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    return ball_fit(data, index, k, weighted, model, 3);
}
static double ball_error(const void *data, const double *model, uint32_t i){
    return fabs(rant_double3_distance(i_at((const i_Points *)data, i), i_d3(model)) - model[3]);
}

typedef struct { i_Points from, to; } i_Matches;

/* The pose: its position, then its quaternion. Horn's quaternion is the top eigenvector of
 * a 4 by 4 built from the cross covariance s. */
static RantMathResult pose_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    const i_Matches *p = (const i_Matches *)data;
    RantDouble3 ma, mb;
    RantQuaternion q;
    double s[3][3], n[16], vectors[16], values[4];
    RantMatrixView nv, vv;
    uint32_t j;
    int r, c;
    nv.data = n; vv.data = vectors;
    nv.rows = nv.cols = vv.rows = vv.cols = 4;
    if (!(i_mean(&p->from, index, k, weighted, &ma) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    i_mean(&p->to, index, k, weighted, &mb);
    memset(s, 0, sizeof s);
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double w = i_weight(p->from.w, i, weighted), a[3], b[3];
        RantDouble3 da = rant_double3_subtract(i_at(&p->from, i), ma), db = rant_double3_subtract(i_at(&p->to, i), mb);
        a[0] = da.x; a[1] = da.y; a[2] = da.z;
        b[0] = db.x; b[1] = db.y; b[2] = db.z;
        for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) s[r][c] += w * a[r] * b[c];
    }
    n[0] = s[0][0] + s[1][1] + s[2][2]; n[1] = s[1][2] - s[2][1]; n[2] = s[2][0] - s[0][2]; n[3] = s[0][1] - s[1][0];
    n[5] = s[0][0] - s[1][1] - s[2][2]; n[6] = s[0][1] + s[1][0]; n[7] = s[2][0] + s[0][2];
    n[10] = s[1][1] - s[0][0] - s[2][2]; n[11] = s[1][2] + s[2][1];
    n[15] = s[2][2] - s[0][0] - s[1][1];
    n[4] = n[1]; n[8] = n[2]; n[12] = n[3]; n[9] = n[6]; n[13] = n[7]; n[14] = n[11];
    if (rant_matrix_eigen(nv, values, vv) != RANT_MATH_OK) return RANT_MATH_ERR_DEGENERATE;
    /* points on one line leave the top two equal: any turn about that line fits as well */
    if (!(values[3] - values[2] > 1e-12 * (values[3] - values[0]))) return RANT_MATH_ERR_DEGENERATE;
    q = rant_quaternion_normalize(rant_quaternion(vectors[7], vectors[11], vectors[15], vectors[3]));
    if (q.w < 0.0) q = rant_quaternion(-q.x, -q.y, -q.z, -q.w);
    i_put3(model, rant_double3_subtract(mb, rant_quaternion_rotate(q, ma)));
    model[3] = q.x; model[4] = q.y; model[5] = q.z; model[6] = q.w;
    return RANT_MATH_OK;
}

static double pose_error(const void *data, const double *model, uint32_t i){
    const i_Matches *p = (const i_Matches *)data;
    RantPose t;
    t.position = i_d3(model);
    t.orientation = rant_quaternion(model[3], model[4], model[5], model[6]);
    return rant_double3_distance(rant_pose_apply(t, i_at(&p->from, i)), i_at(&p->to, i));
}

/* The 2D pose: x, y, then its angle, which turns the from points' spread most onto the to's. */
static RantMathResult pose2d_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    const i_Matches *p = (const i_Matches *)data;
    RantDouble3 ma, mb;
    RantDouble2 t;
    double dot = 0.0, cross = 0.0, aa = 0.0, bb = 0.0, angle;
    uint32_t j;
    if (!(i_mean(&p->from, index, k, weighted, &ma) > 0.0)) return RANT_MATH_ERR_DEGENERATE;
    i_mean(&p->to, index, k, weighted, &mb);
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double w = i_weight(p->from.w, i, weighted);
        RantDouble3 a = rant_double3_subtract(i_at(&p->from, i), ma), b = rant_double3_subtract(i_at(&p->to, i), mb);
        dot += w * (a.x * b.x + a.y * b.y);
        cross += w * (a.x * b.y - a.y * b.x);
        aa += w * (a.x * a.x + a.y * a.y);
        bb += w * (b.x * b.x + b.y * b.y);
    }
    /* every from point in one place, or every to point, fits any turn */
    if (!(hypot(dot, cross) > 1e-12 * sqrt(aa * bb))) return RANT_MATH_ERR_DEGENERATE;
    angle = atan2(cross, dot);
    t = rant_double2_subtract(i_math_flat(mb), rant_double2_rotate(i_math_flat(ma), angle));
    model[0] = t.x; model[1] = t.y; model[2] = angle;
    return RANT_MATH_OK;
}

static double pose2d_error(const void *data, const double *model, uint32_t i){
    const i_Matches *p = (const i_Matches *)data;
    RantPose2D t;
    t.position = rant_double2(model[0], model[1]);
    t.angle = model[2];
    return rant_double2_distance(rant_pose2d_apply(t, i_math_flat(i_at(&p->from, i))), i_math_flat(i_at(&p->to, i)));
}

typedef struct { const double *x, *y, *w; uint32_t degree; } i_Poly;

/* Solved in t = (x - mid) / half, on [-1, 1], where the columns of powers stay apart, then
 * turned back into powers of x. */
static RantMathResult poly_fit(const void *data, const uint32_t *index, uint32_t k, int weighted, double *model){
    const i_Poly *p = (const i_Poly *)data;
    uint32_t n = p->degree + 1, j, e, q;
    double lo = HUGE_VAL, hi = -HUGE_VAL, mid, half, *a, *b, *x, *row;
    RantMatrixView av, bv, xv;
    RantMathResult r;
    if (k < n) return RANT_MATH_ERR_DEGENERATE;
    for (j = 0; j < k; j++){
        lo = i_math_min(lo, p->x[i_ix(index, j)]);
        hi = i_math_max(hi, p->x[i_ix(index, j)]);
    }
    mid = 0.5 * (lo + hi);
    half = 0.5 * (hi - lo);
    if (!(half > 0.0)){
        if (p->degree) return RANT_MATH_ERR_DEGENERATE;
        half = 1.0;
    }
    a = (double *)malloc(sizeof(double) * ((size_t)k * n + k + 2 * (size_t)n));
    if (!a) return RANT_MATH_ERR_MEMORY;
    b = a + (size_t)k * n;
    x = b + k;
    row = x + n;
    for (j = 0; j < k; j++){
        uint32_t i = i_ix(index, j);
        double root = sqrt(i_weight(p->w, i, weighted)), t = (p->x[i] - mid) / half, power = root;
        for (e = 0; e < n; e++){
            a[(size_t)j * n + e] = power;
            power *= t;
        }
        b[j] = root * p->y[i];
    }
    av.data = a; av.rows = k; av.cols = n;
    bv.data = b; bv.rows = k; bv.cols = 1;
    xv.data = x; xv.rows = n; xv.cols = 1;
    r = rant_matrix_solve(av, bv, xv);
    if (r == RANT_MATH_OK){
        /* row holds the powers of x in ((x - mid) / half)^e */
        memset(model, 0, sizeof(double) * n);
        memset(row, 0, sizeof(double) * n);
        row[0] = 1.0;
        for (e = 0; e < n; e++){
            for (q = 0; q <= e; q++) model[q] += x[e] * row[q];
            if (e + 1 == n) break;
            for (q = e + 1; q > 0; q--) row[q] = (row[q - 1] - mid * row[q]) / half;
            row[0] = -mid * row[0] / half;
        }
    }
    free(a);
    return r;
}

static double poly_error(const void *data, const double *model, uint32_t i){
    const i_Poly *p = (const i_Poly *)data;
    return fabs(p->y[i] - rant_polynomial_at(model, p->degree, p->x[i]));
}

static const i_Model I_PLANE = { 3, 6, plane_fit, plane_error };
static const i_Model I_LINE = { 2, 6, line_fit, line_error };
static const i_Model I_CIRCLE = { 3, 4, circle_fit, ball_error };
static const i_Model I_SPHERE = { 4, 4, sphere_fit, ball_error };
static const i_Model I_POSE = { 3, 7, pose_fit, pose_error };
static const i_Model I_POSE2D = { 2, 3, pose2d_fit, pose2d_error };

RantMathResult rant_points_fit_plane(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantPlane *out){
    i_Points s = i_points(points, NULL, options);
    double m[6];
    RantMathResult r;
    if (!points || !out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_fit(&I_PLANE, &s, count, options, m)) != RANT_MATH_OK) return r;
    out->position = i_d3(m);
    out->normal = i_d3(m + 3);
    return RANT_MATH_OK;
}

RantMathResult rant_points_fit_line(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantSegment *out){
    i_Points s = i_points(points, NULL, options);
    double m[6];
    RantMathResult r;
    if (!points || !out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_fit(&I_LINE, &s, count, options, m)) != RANT_MATH_OK) return r;
    out->a = i_d3(m);
    out->b = i_d3(m + 3);
    return RANT_MATH_OK;
}

RantMathResult rant_points_fit_sphere(const RantDouble3 *points, uint32_t count, const RantFitOptions *options, RantSphere *out){
    i_Points s = i_points(points, NULL, options);
    double m[4];
    RantMathResult r;
    if (!points || !out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_fit(&I_SPHERE, &s, count, options, m)) != RANT_MATH_OK) return r;
    out->center = i_d3(m);
    out->radius = m[3];
    return RANT_MATH_OK;
}

RantMathResult rant_points_fit_pose(const RantDouble3 *from, const RantDouble3 *to, uint32_t count, const RantFitOptions *options, RantPose *out){
    i_Matches p;
    double m[7];
    RantMathResult r;
    if (!from || !to || !out) return RANT_MATH_ERR_ARGUMENT;
    p.from = i_points(from, NULL, options);
    p.to = i_points(to, NULL, options);
    if ((r = i_fit(&I_POSE, &p, count, options, m)) != RANT_MATH_OK) return r;
    out->position = i_d3(m);
    out->orientation = rant_quaternion(m[3], m[4], m[5], m[6]);
    return RANT_MATH_OK;
}

RantMathResult rant_points2d_fit_line(const RantDouble2 *points, uint32_t count, const RantFitOptions *options, RantSegment2D *out){
    i_Points s = i_points(NULL, points, options);
    double m[6];
    RantMathResult r;
    if (!points || !out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_fit(&I_LINE, &s, count, options, m)) != RANT_MATH_OK) return r;
    out->a = rant_double2(m[0], m[1]);
    out->b = rant_double2(m[3], m[4]);
    return RANT_MATH_OK;
}

RantMathResult rant_points2d_fit_circle(const RantDouble2 *points, uint32_t count, const RantFitOptions *options, RantCircle *out){
    i_Points s = i_points(NULL, points, options);
    double m[4];
    RantMathResult r;
    if (!points || !out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_fit(&I_CIRCLE, &s, count, options, m)) != RANT_MATH_OK) return r;
    out->center = rant_double2(m[0], m[1]);
    out->radius = m[3];
    return RANT_MATH_OK;
}

RantMathResult rant_points2d_fit_pose(const RantDouble2 *from, const RantDouble2 *to, uint32_t count, const RantFitOptions *options, RantPose2D *out){
    i_Matches p;
    double m[3];
    RantMathResult r;
    if (!from || !to || !out) return RANT_MATH_ERR_ARGUMENT;
    p.from = i_points(NULL, from, options);
    p.to = i_points(NULL, to, options);
    if ((r = i_fit(&I_POSE2D, &p, count, options, m)) != RANT_MATH_OK) return r;
    out->position = rant_double2(m[0], m[1]);
    out->angle = m[2];
    return RANT_MATH_OK;
}

RantMathResult rant_polynomial_fit(const double *x, const double *y, uint32_t count, uint32_t degree,
                                   const RantFitOptions *options, double *coefficients){
    i_Poly p;
    i_Model m;
    if (!x || !y || !coefficients || degree == UINT32_MAX) return RANT_MATH_ERR_ARGUMENT;
    p.x = x;
    p.y = y;
    p.w = options ? options->weights : NULL;
    p.degree = degree;
    m.sample = m.size = degree + 1;
    m.fit = poly_fit;
    m.error = poly_error;
    return i_fit(&m, &p, count, options, coefficients);
}

double rant_polynomial_at(const double *coefficients, uint32_t degree, double x){
    double v;
    uint32_t e = degree;
    if (!coefficients) return 0.0;
    v = coefficients[degree];
    while (e-- > 0) v = v * x + coefficients[e];
    return v;
}
