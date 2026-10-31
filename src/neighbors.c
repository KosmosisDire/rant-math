#include "internal.h"

/* A node with no more points than this is a leaf. */
#define I_LEAF 8

typedef struct {
    double split;
    uint32_t lo, hi;         /* its points, in tree order */
    uint32_t left, right;    /* 0 for a leaf, since the root is no one's child */
    int axis;
} i_Node;

struct RantKdTree {
    RantDouble3 *points;     /* in tree order */
    uint32_t *index;         /* each one's index in the points it was made from */
    i_Node *nodes;
    uint32_t count, nodes_used;
};

/* A point found, by the caller's index: a comes before b nearer, or as near with a lower index. */
typedef struct { double d2; uint32_t i; } i_Hit;
static int i_before(i_Hit a, i_Hit b){ return a.d2 < b.d2 || (a.d2 == b.d2 && a.i < b.i); }

static double i_on(RantDouble3 p, int axis){ return axis == 0 ? p.x : (axis == 1 ? p.y : p.z); }
static double i_d2(RantDouble3 a, RantDouble3 b){
    double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return x * x + y * y + z * z;
}

static void i_swap(RantKdTree *t, uint32_t a, uint32_t b){
    RantDouble3 p = t->points[a];
    uint32_t i = t->index[a];
    t->points[a] = t->points[b]; t->index[a] = t->index[b];
    t->points[b] = p; t->index[b] = i;
}

/* Puts the k th smallest along axis at k, none larger before it and none smaller after. Equal
 * values gather in the middle, so many equal points cost no more than distinct ones. */
static void i_select(RantKdTree *t, uint32_t lo, uint32_t hi, uint32_t k, int axis){
    while (hi - lo > 1){
        double a = i_on(t->points[lo], axis), b = i_on(t->points[lo + (hi - lo) / 2], axis);
        double c = i_on(t->points[hi - 1], axis);
        double pivot = a < b ? (b < c ? b : (a < c ? c : a)) : (a < c ? a : (b < c ? c : b));
        uint32_t lt = lo, i = lo, gt = hi;
        while (i < gt){
            double v = i_on(t->points[i], axis);
            if (v < pivot) i_swap(t, lt++, i++);
            else if (v > pivot) i_swap(t, i, --gt);
            else i++;
        }
        if (k < lt) hi = lt;
        else if (k >= gt) lo = gt;
        else return;
    }
}

/* Splits along the widest axis at the median: the left holds none above the split, the right none below. */
static uint32_t i_build(RantKdTree *t, uint32_t lo, uint32_t hi){
    uint32_t me = t->nodes_used++, i, mid, left;
    double a[3] = { HUGE_VAL, HUGE_VAL, HUGE_VAL }, b[3] = { -HUGE_VAL, -HUGE_VAL, -HUGE_VAL }, wide = 0.0;
    int axis = 0, k;
    t->nodes[me].lo = lo; t->nodes[me].hi = hi;
    t->nodes[me].left = t->nodes[me].right = 0;
    t->nodes[me].axis = 0; t->nodes[me].split = 0.0;
    if (hi - lo <= I_LEAF) return me;
    for (i = lo; i < hi; i++)
        for (k = 0; k < 3; k++){
            double v = i_on(t->points[i], k);
            if (v < a[k]) a[k] = v;
            if (v > b[k]) b[k] = v;
        }
    for (k = 0; k < 3; k++) if (b[k] - a[k] > wide){ wide = b[k] - a[k]; axis = k; }
    if (!(wide > 0.0)) return me;    /* all one point: a larger leaf */
    mid = lo + (hi - lo) / 2;
    i_select(t, lo, hi, mid, axis);
    t->nodes[me].axis = axis;
    t->nodes[me].split = i_on(t->points[mid], axis);
    left = i_build(t, lo, mid);
    t->nodes[me].right = i_build(t, mid, hi);
    t->nodes[me].left = left;
    return me;
}

RantMathResult rant_kdtree_create(const RantDouble3 *points, uint32_t count, RantKdTree **out){
    RantKdTree *t;
    uint32_t i, n = 0;
    if (!out || (count && !points)) return RANT_MATH_ERR_ARGUMENT;
    for (i = 0; i < count; i++) n += i_math_finite3(points[i]);
    t = (RantKdTree *)calloc(1, sizeof *t);
    if (!t) return RANT_MATH_ERR_MEMORY;
    /* each leaf holds at least I_LEAF / 2 points, so a tree of n points needs at most n / 2 + 1 nodes */
    t->points = (RantDouble3 *)malloc(sizeof(RantDouble3) * (n ? n : 1));
    t->index = (uint32_t *)malloc(sizeof(uint32_t) * (n ? n : 1));
    t->nodes = (i_Node *)malloc(sizeof(i_Node) * (n / 2 + 1));
    if (!t->points || !t->index || !t->nodes){ rant_kdtree_destroy(t); return RANT_MATH_ERR_MEMORY; }
    for (i = 0; i < count; i++)
        if (i_math_finite3(points[i])){ t->points[t->count] = points[i]; t->index[t->count++] = i; }
    i_build(t, 0, t->count);
    *out = t;
    return RANT_MATH_OK;
}

void rant_kdtree_destroy(RantKdTree *tree){
    if (!tree) return;
    free(tree->points);
    free(tree->index);
    free(tree->nodes);
    free(tree);
}

/* A heap of the k best so far, the worst on top. */
typedef struct { i_Hit *h; uint32_t k, n; } i_Best;

/* x put in at the top of a heap of n, where it sinks to its place. */
static void i_sink(i_Hit *h, uint32_t n, i_Hit x){
    uint32_t i = 0, c;
    for (;;){
        c = 2 * i + 1;
        if (c >= n) break;
        if (c + 1 < n && i_before(h[c], h[c + 1])) c++;
        if (!i_before(x, h[c])) break;
        h[i] = h[c];
        i = c;
    }
    h[i] = x;
}

static void i_offer(i_Best *b, i_Hit x){
    uint32_t i;
    if (b->n < b->k){
        i = b->n++;
        while (i && i_before(b->h[(i - 1) / 2], x)){ b->h[i] = b->h[(i - 1) / 2]; i = (i - 1) / 2; }
        b->h[i] = x;
        return;
    }
    if (b->k && i_before(x, b->h[0])) i_sink(b->h, b->n, x);
}

/* gap holds how far q lies outside the node's cell along each axis, and reach the sum of their
 * squares, nearer than which no point of the cell can be. */
static void i_nearest(const RantKdTree *t, uint32_t node, RantDouble3 q, double reach, double *gap, i_Best *b){
    const i_Node *nd = &t->nodes[node];
    double diff, old, farther;
    uint32_t i;
    if (!nd->left){
        for (i = nd->lo; i < nd->hi; i++){
            i_Hit x;
            x.d2 = i_d2(q, t->points[i]);
            x.i = t->index[i];
            i_offer(b, x);
        }
        return;
    }
    diff = i_on(q, nd->axis) - nd->split;
    i_nearest(t, diff < 0.0 ? nd->left : nd->right, q, reach, gap, b);
    old = gap[nd->axis];
    farther = reach - old * old + diff * diff;
    /* the slack covers rounding in reach, and a far point at exactly the worst distance may still win on its index */
    if (b->n < b->k || farther <= b->h[0].d2 * (1.0 + 1e-12)){
        gap[nd->axis] = diff;
        i_nearest(t, diff < 0.0 ? nd->right : nd->left, q, farther, gap, b);
        gap[nd->axis] = old;
    }
}

/* visit returns 1 to stop the search. */
typedef int (*i_Visit)(void *ctx, i_Hit x);

static int i_within(const RantKdTree *t, uint32_t node, RantDouble3 q, double r2, i_Visit visit, void *ctx){
    const i_Node *nd = &t->nodes[node];
    double diff;
    uint32_t i;
    if (!nd->left){
        for (i = nd->lo; i < nd->hi; i++){
            i_Hit x;
            x.d2 = i_d2(q, t->points[i]);
            x.i = t->index[i];
            if (x.d2 <= r2 && visit(ctx, x)) return 1;
        }
        return 0;
    }
    diff = i_on(q, nd->axis) - nd->split;
    if (i_within(t, diff < 0.0 ? nd->left : nd->right, q, r2, visit, ctx)) return 1;
    return diff * diff <= r2 && i_within(t, diff < 0.0 ? nd->right : nd->left, q, r2, visit, ctx);
}

static int i_compare_hits(const void *a, const void *b){
    i_Hit x = *(const i_Hit *)a, y = *(const i_Hit *)b;
    return i_before(x, y) ? -1 : (i_before(y, x) ? 1 : 0);
}

/* The k nearest into h, sorted, and how many. */
static uint32_t i_knn(const RantKdTree *t, RantDouble3 q, uint32_t k, i_Hit *h){
    i_Best b;
    double gap[3] = { 0.0, 0.0, 0.0 };
    uint32_t n;
    b.h = h; b.k = k < t->count ? k : t->count; b.n = 0;
    if (b.k) i_nearest(t, 0, q, 0.0, gap, &b);
    /* the worst taken off the top to the back, each time, leaves the list in order */
    for (n = b.n; n > 1; n--){
        i_Hit top = h[0];
        i_sink(h, n - 1, h[n - 1]);
        h[n - 1] = top;
    }
    return b.n;
}

RantMathResult rant_kdtree_nearest(const RantKdTree *tree, RantDouble3 point, uint32_t *indices, double *distances,
                                   uint32_t *count){
    i_Hit *h;
    uint32_t n, i;
    if (!tree || !count || (*count && !indices) || !i_math_finite3(point)) return RANT_MATH_ERR_ARGUMENT;
    n = *count < tree->count ? *count : tree->count;
    h = (i_Hit *)malloc(sizeof(i_Hit) * (n ? n : 1));
    if (!h) return RANT_MATH_ERR_MEMORY;
    n = i_knn(tree, point, n, h);
    for (i = 0; i < n; i++){
        indices[i] = h[i].i;
        if (distances) distances[i] = sqrt(h[i].d2);
    }
    free(h);
    *count = n;
    return RANT_MATH_OK;
}

/* A list that grows as hits come. */
typedef struct { i_Hit *h; uint32_t n, room; int failed; } i_List;

static int i_gather(void *ctx, i_Hit x){
    i_List *l = (i_List *)ctx;
    if (l->n == l->room){
        uint32_t room = l->room ? 2 * l->room : 64;
        i_Hit *h = (i_Hit *)realloc(l->h, sizeof(i_Hit) * room);
        if (!h){ l->failed = 1; return 1; }
        l->h = h; l->room = room;
    }
    l->h[l->n++] = x;
    return 0;
}

RantMathResult rant_kdtree_within(const RantKdTree *tree, RantDouble3 point, double radius, uint32_t *indices,
                                  double *distances, uint32_t *count){
    i_List l = { NULL, 0, 0, 0 };
    RantMathResult r;
    uint32_t i;
    if (!tree || !count || !i_math_finite3(point) || !(radius >= 0.0) || !i_math_finite(radius)) return RANT_MATH_ERR_ARGUMENT;
    if (tree->count) i_within(tree, 0, point, radius * radius, i_gather, &l);
    if (l.failed){ free(l.h); return RANT_MATH_ERR_MEMORY; }
    if ((r = i_math_room(count, l.n, indices)) == RANT_MATH_OK && l.n){
        qsort(l.h, l.n, sizeof *l.h, i_compare_hits);
        for (i = 0; i < l.n; i++){
            indices[i] = l.h[i].i;
            if (distances) distances[i] = sqrt(l.h[i].d2);
        }
    }
    free(l.h);
    return r;
}

/* A point's cube and index, ordered cube by cube. */
typedef struct { double key[3]; uint32_t i; } i_Cell;

static int i_compare_cells(const void *a, const void *b){
    const i_Cell *x = (const i_Cell *)a, *y = (const i_Cell *)b;
    int k;
    for (k = 0; k < 3; k++) if (x->key[k] != y->key[k]) return x->key[k] < y->key[k] ? -1 : 1;
    return x->i < y->i ? -1 : (x->i > y->i);
}

RantMathResult rant_points_downsample(const RantDouble3 *points, uint32_t count, double voxel, uint8_t *keep){
    i_Cell *c;
    uint32_t i, n = 0, lo, hi, j;
    if ((count && (!points || !keep)) || !(voxel > 0.0) || !i_math_finite(voxel)) return RANT_MATH_ERR_ARGUMENT;
    c = (i_Cell *)malloc(sizeof(i_Cell) * (count ? count : 1));
    if (!c) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i < count; i++){
        keep[i] = 0;
        if (!i_math_finite3(points[i])) continue;
        c[n].key[0] = floor(points[i].x / voxel);
        c[n].key[1] = floor(points[i].y / voxel);
        c[n].key[2] = floor(points[i].z / voxel);
        c[n++].i = i;
    }
    qsort(c, n, sizeof *c, i_compare_cells);
    for (lo = 0; lo < n; lo = hi){
        RantDouble3 first = points[c[lo].i], sum = rant_double3(0.0, 0.0, 0.0), mean;
        double best = HUGE_VAL;
        uint32_t pick = c[lo].i;
        for (hi = lo; hi < n && !memcmp(c[hi].key, c[lo].key, sizeof c[lo].key); hi++)
            sum = rant_double3_add(sum, rant_double3_subtract(points[c[hi].i], first));
        /* summed from the first point, so a cloud far from the origin keeps its digits */
        mean = rant_double3_add(first, rant_double3_scale(sum, 1.0 / (hi - lo)));
        for (j = lo; j < hi; j++){
            double d = i_d2(points[c[j].i], mean);
            if (d < best){ best = d; pick = c[j].i; }
        }
        keep[pick] = 1;
    }
    free(c);
    return RANT_MATH_OK;
}

RantMathResult rant_points_remove_outliers(const RantDouble3 *points, uint32_t count, uint32_t neighbors,
                                           double deviations, uint8_t *keep){
    RantKdTree *t;
    i_Hit *h;
    double *mean, sum = 0.0, sq = 0.0, mu, sigma, limit;
    uint32_t i, j, m, valid = 0;
    RantMathResult r;
    if ((count && (!points || !keep)) || !neighbors || neighbors == UINT32_MAX || !i_math_finite(deviations))
        return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_kdtree_create(points, count, &t)) != RANT_MATH_OK) return r;
    h = (i_Hit *)malloc(sizeof(i_Hit) * (neighbors + 1));
    mean = (double *)malloc(sizeof(double) * (count ? count : 1));
    if (!h || !mean){ free(h); free(mean); rant_kdtree_destroy(t); return RANT_MATH_ERR_MEMORY; }
    for (i = 0; i < count; i++){
        double d = 0.0;
        if (!i_math_finite3(points[i])) continue;
        /* the nearest is the point itself, or a twin of it at distance 0 */
        m = i_knn(t, points[i], neighbors + 1, h);
        for (j = 1; j < m; j++) d += sqrt(h[j].d2);
        mean[i] = m > 1 ? d / (m - 1) : 0.0;
        sum += mean[i];
        valid++;
    }
    mu = valid ? sum / valid : 0.0;
    for (i = 0; i < count; i++) if (i_math_finite3(points[i])) sq += i_math_sqr(mean[i] - mu);
    sigma = valid > 1 ? sqrt(sq / (valid - 1)) : 0.0;
    limit = mu + deviations * sigma;
    for (i = 0; i < count; i++) keep[i] = i_math_finite3(points[i]) && mean[i] <= limit;
    free(h);
    free(mean);
    rant_kdtree_destroy(t);
    return RANT_MATH_OK;
}

/* Counts the others near a point, stopping once there are enough. */
typedef struct { uint32_t self, found, enough; } i_Count;

static int i_tally(void *ctx, i_Hit x){
    i_Count *c = (i_Count *)ctx;
    if (x.i != c->self) c->found++;
    return c->found >= c->enough;
}

RantMathResult rant_points_remove_isolated(const RantDouble3 *points, uint32_t count, double radius, uint32_t neighbors,
                                           uint8_t *keep){
    RantKdTree *t;
    uint32_t i;
    RantMathResult r;
    if ((count && (!points || !keep)) || !(radius >= 0.0) || !i_math_finite(radius)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_kdtree_create(points, count, &t)) != RANT_MATH_OK) return r;
    for (i = 0; i < count; i++){
        i_Count c;
        c.self = i; c.found = 0; c.enough = neighbors;
        keep[i] = 0;
        if (!i_math_finite3(points[i])) continue;
        if (neighbors) i_within(t, 0, points[i], radius * radius, i_tally, &c);
        keep[i] = c.found >= neighbors;
    }
    rant_kdtree_destroy(t);
    return RANT_MATH_OK;
}

RantMathResult rant_points_normals(const RantDouble3 *points, uint32_t count, uint32_t neighbors, RantDouble3 viewpoint,
                                   RantDouble3 *normals){
    RantKdTree *t;
    i_Hit *h;
    uint32_t i, j, m;
    RantMathResult r;
    if ((count && (!points || !normals)) || neighbors < 3 || !i_math_finite3(viewpoint)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_kdtree_create(points, count, &t)) != RANT_MATH_OK) return r;
    h = (i_Hit *)malloc(sizeof(i_Hit) * neighbors);
    if (!h){ rant_kdtree_destroy(t); return RANT_MATH_ERR_MEMORY; }
    for (i = 0; i < count; i++){
        RantDouble3 c = rant_double3(0.0, 0.0, 0.0), n;
        i_MathMat3 s, v;
        double e[3];
        normals[i] = rant_double3(NAN, NAN, NAN);
        if (!i_math_finite3(points[i])) continue;
        m = i_knn(t, points[i], neighbors, h);
        if (m < 3) continue;
        for (j = 0; j < m; j++) c = rant_double3_add(c, rant_double3_subtract(points[h[j].i], points[i]));
        c = rant_double3_scale(c, 1.0 / m);
        memset(&s, 0, sizeof s);
        for (j = 0; j < m; j++){
            RantDouble3 d = rant_double3_subtract(rant_double3_subtract(points[h[j].i], points[i]), c);
            double a[3];
            int x, y;
            a[0] = d.x; a[1] = d.y; a[2] = d.z;
            for (x = 0; x < 3; x++) for (y = 0; y < 3; y++) s.m[x][y] += a[x] * a[y];
        }
        i_math_mat3_eigen(&s, e, &v);
        /* points on a line or at one place leave the plane's turn about them open */
        if (!(e[1] > 1e-12 * e[2])) continue;
        n = rant_double3_normalize(rant_double3(v.m[0][0], v.m[1][0], v.m[2][0]));
        if (rant_double3_dot(n, rant_double3_subtract(viewpoint, points[i])) < 0.0) n = rant_double3_scale(n, -1.0);
        normals[i] = n;
    }
    free(h);
    rant_kdtree_destroy(t);
    return RANT_MATH_OK;
}

/* Floods a cluster: each point it reaches not yet in one joins it and waits its turn. */
typedef struct { uint32_t *group, *stack, top, id; } i_Flood;

static int i_spread(void *ctx, i_Hit x){
    i_Flood *f = (i_Flood *)ctx;
    if (f->group[x.i] == UINT32_MAX){ f->group[x.i] = f->id; f->stack[f->top++] = x.i; }
    return 0;
}

typedef struct { uint32_t size, id; } i_Group;

static int i_compare_groups(const void *a, const void *b){
    const i_Group *x = (const i_Group *)a, *y = (const i_Group *)b;
    if (x->size != y->size) return x->size > y->size ? -1 : 1;
    return x->id < y->id ? -1 : (x->id > y->id);
}

RantMathResult rant_points_clusters(const RantDouble3 *points, uint32_t count, double distance, uint32_t min_points,
                                    uint32_t *labels, uint32_t *clusters){
    RantKdTree *t;
    i_Flood f;
    i_Group *g;
    uint32_t i, groups = 0, *rank;
    RantMathResult r;
    if ((count && (!points || !labels)) || !clusters || !(distance >= 0.0) || !i_math_finite(distance))
        return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_kdtree_create(points, count, &t)) != RANT_MATH_OK) return r;
    f.group = (uint32_t *)malloc(sizeof(uint32_t) * (count ? count : 1));
    f.stack = (uint32_t *)malloc(sizeof(uint32_t) * (count ? count : 1));
    g = (i_Group *)malloc(sizeof(i_Group) * (count ? count : 1));
    rank = (uint32_t *)malloc(sizeof(uint32_t) * (count ? count : 1));
    if (!f.group || !f.stack || !g || !rank){
        free(f.group); free(f.stack); free(g); free(rank); rant_kdtree_destroy(t);
        return RANT_MATH_ERR_MEMORY;
    }
    for (i = 0; i < count; i++) f.group[i] = i_math_finite3(points[i]) ? UINT32_MAX : UINT32_MAX - 1;
    for (i = 0; i < count; i++){
        if (f.group[i] != UINT32_MAX) continue;
        f.id = groups;
        g[groups].id = groups;
        g[groups].size = 0;
        f.group[i] = groups;
        f.top = 0;
        f.stack[f.top++] = i;
        while (f.top){
            uint32_t j = f.stack[--f.top];
            g[groups].size++;
            i_within(t, 0, points[j], distance * distance, i_spread, &f);
        }
        groups++;
    }
    /* groups were opened in the order of their first point, which breaks ties in size */
    qsort(g, groups, sizeof *g, i_compare_groups);
    *clusters = 0;
    for (i = 0; i < groups; i++){
        rank[g[i].id] = g[i].size >= min_points ? i + 1 : 0;
        if (g[i].size >= min_points) (*clusters)++;
    }
    for (i = 0; i < count; i++) labels[i] = f.group[i] < UINT32_MAX - 1 ? rank[f.group[i]] : 0;
    free(f.group); free(f.stack); free(g); free(rank);
    rant_kdtree_destroy(t);
    return RANT_MATH_OK;
}

/* The pairs of an alignment: each source point moved by pose, and its nearest target point. */
typedef struct {
    const RantDouble3 *source, *target, *normals;
    uint32_t source_count;
    const RantKdTree *tree;
    double limit2;           /* the squared max distance, HUGE_VAL for none */
    RantDouble3 *from, *to, *moved, *normal;
} i_Pairs;

static uint32_t i_pair(i_Pairs *p, RantPose pose, uint8_t *inliers, double *sum2){
    i_MathMat3 m = i_math_mat3_from_quaternion(pose.orientation);
    uint32_t i, k = 0;
    *sum2 = 0.0;
    for (i = 0; i < p->source_count; i++){
        RantDouble3 q;
        i_Hit h;
        if (inliers) inliers[i] = 0;
        if (!i_math_finite3(p->source[i])) continue;
        q = rant_double3_add(pose.position, i_math_mat3_apply(&m, p->source[i]));
        if (i_knn(p->tree, q, 1, &h) != 1 || h.d2 > p->limit2) continue;
        if (p->normals && !i_math_finite3(p->normals[h.i])) continue;
        p->from[k] = p->source[i];
        p->moved[k] = q;
        p->to[k] = p->target[h.i];
        if (p->normals) p->normal[k] = p->normals[h.i];
        if (inliers) inliers[i] = 1;
        *sum2 += h.d2;
        k++;
    }
    return k;
}

/* One Gauss Newton step of the distances along the normals, a turn about the pairs' mean and a
 * shift. A direction the pairs do not hold, such as along a single plane, stays where it is. */
static RantMathResult i_plane_step(const i_Pairs *p, uint32_t k, RantPose pose, RantPose *next){
    double a[36], b[6], x[6], arm2 = 0.0, len, damp = 0.0;
    RantMatrixView av, bv, xv;
    RantDouble3 c = rant_double3(0.0, 0.0, 0.0), w;
    RantQuaternion turn;
    uint32_t i;
    int r, s;
    memset(a, 0, sizeof a);
    memset(b, 0, sizeof b);
    for (i = 0; i < k; i++) c = rant_double3_add(c, p->moved[i]);
    c = rant_double3_scale(c, 1.0 / k);
    for (i = 0; i < k; i++) arm2 += rant_double3_length_squared(rant_double3_subtract(p->moved[i], c));
    /* the turn is solved per unit of the pairs' spread, so both halves of the system weigh alike */
    len = arm2 > 0.0 ? sqrt(arm2 / k) : 1.0;
    for (i = 0; i < k; i++){
        RantDouble3 n = p->normal[i];
        RantDouble3 arm = rant_double3_scale(rant_double3_cross(rant_double3_subtract(p->moved[i], c), n), 1.0 / len);
        double j[6], e = rant_double3_dot(n, rant_double3_subtract(p->moved[i], p->to[i]));
        j[0] = arm.x; j[1] = arm.y; j[2] = arm.z; j[3] = n.x; j[4] = n.y; j[5] = n.z;
        for (r = 0; r < 6; r++){
            b[r] -= j[r] * e;
            for (s = 0; s < 6; s++) a[6 * r + s] += j[r] * j[s];
        }
    }
    for (r = 0; r < 6; r++) damp += a[7 * r];
    for (r = 0; r < 6; r++) a[7 * r] += 1e-9 * damp / 6.0;
    av.data = a; av.rows = 6; av.cols = 6;
    bv.data = b; bv.rows = 6; bv.cols = 1;
    xv.data = x; xv.rows = 6; xv.cols = 1;
    if (rant_matrix_solve(av, bv, xv) != RANT_MATH_OK) return RANT_MATH_ERR_DEGENERATE;
    w = rant_double3(x[0] / len, x[1] / len, x[2] / len);
    turn = rant_quaternion_from_rotation_vector(w);
    next->orientation = rant_quaternion_normalize(rant_quaternion_compose(turn, pose.orientation));
    next->position = rant_double3_add(rant_double3_add(c, rant_quaternion_rotate(turn, rant_double3_subtract(pose.position, c))),
                                      rant_double3(x[3], x[4], x[5]));
    return RANT_MATH_OK;
}

RantMathResult rant_points_align(const RantDouble3 *source, uint32_t source_count, const RantDouble3 *target,
                                 uint32_t target_count, RantPose guess, const RantAlignOptions *options, RantPose *out){
    RantAlignOptions o;
    RantKdTree *t;
    i_Pairs p;
    RantPose pose;
    RantDouble3 mid = rant_double3(0.0, 0.0, 0.0);
    double reach = 0.0, sum2;
    uint32_t i, k, round, valid = 0;
    RantMathResult r = RANT_MATH_OK;
    if (!source || !target || !source_count || !target_count || !out) return RANT_MATH_ERR_ARGUMENT;
    if (options) o = *options;
    else memset(&o, 0, sizeof o);
    if (!(o.max_distance >= 0.0)) return RANT_MATH_ERR_ARGUMENT;
    if (!i_math_finite3(guess.position) || rant_double4_length(rant_double4(guess.orientation.x, guess.orientation.y,
            guess.orientation.z, guess.orientation.w)) < I_MATH_TINY) return RANT_MATH_ERR_ARGUMENT;
    if ((r = rant_kdtree_create(target, target_count, &t)) != RANT_MATH_OK) return r;
    p.source = source; p.target = target; p.normals = o.normals; p.source_count = source_count; p.tree = t;
    p.limit2 = o.max_distance > 0.0 ? o.max_distance * o.max_distance : HUGE_VAL;
    p.from = (RantDouble3 *)malloc(sizeof(RantDouble3) * (source_count ? source_count : 1));
    p.to = (RantDouble3 *)malloc(sizeof(RantDouble3) * (source_count ? source_count : 1));
    p.moved = (RantDouble3 *)malloc(sizeof(RantDouble3) * (source_count ? source_count : 1));
    p.normal = (RantDouble3 *)malloc(sizeof(RantDouble3) * (source_count ? source_count : 1));
    if (!p.from || !p.to || !p.moved || !p.normal){ r = RANT_MATH_ERR_MEMORY; goto done; }
    /* how far the source reaches from its middle, the length a turn moves it by */
    for (i = 0; i < source_count; i++)
        if (i_math_finite3(source[i])){ mid = rant_double3_add(mid, source[i]); valid++; }
    if (valid) mid = rant_double3_scale(mid, 1.0 / valid);
    for (i = 0; i < source_count; i++)
        if (i_math_finite3(source[i])) reach = i_math_max(reach, rant_double3_distance(source[i], mid));
    pose = guess;
    pose.orientation = rant_quaternion_normalize(pose.orientation);
    for (round = 0; round < (o.iterations ? o.iterations : 50u); round++){
        RantPose next;
        double moved;
        if ((k = i_pair(&p, pose, NULL, &sum2)) < 3){ r = RANT_MATH_ERR_DEGENERATE; goto done; }
        r = o.normals ? i_plane_step(&p, k, pose, &next) : rant_points_fit_pose(p.from, p.to, k, NULL, &next);
        if (r != RANT_MATH_OK) goto done;
        moved = rant_double3_distance(next.position, pose.position) + reach * rant_quaternion_angle(next.orientation, pose.orientation);
        pose = next;
        if (moved <= 1e-12 * (reach + rant_double3_length(pose.position))) break;
    }
    if ((k = i_pair(&p, pose, o.inliers, &sum2)) < 3){ r = RANT_MATH_ERR_DEGENERATE; goto done; }
    if (o.error) *o.error = sqrt(sum2 / k);
    *out = pose;
done:
    free(p.from); free(p.to); free(p.moved); free(p.normal);
    rant_kdtree_destroy(t);
    return r;
}
