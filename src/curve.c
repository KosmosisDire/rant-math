#include "internal.h"

/* Every curve is points read with a stride, a 2D one on z = 0, so one walk serves all four.
 * A pose's position is its first member, so a path reads as its positions. */
typedef struct { const unsigned char *base; size_t stride; uint32_t count; int flat; } i_Line;

static i_Line line_of(const void *points, size_t stride, uint32_t count, int flat){
    i_Line l;
    l.base = (const unsigned char *)points;
    l.stride = stride;
    l.count = points ? count : 0;
    l.flat = flat;
    return l;
}

static i_Line polyline(RantPolylineView v){ return line_of(v.points, sizeof(RantDouble3), v.count, 0); }
static i_Line polyline2d(RantPolyline2DView v){ return line_of(v.points, sizeof(RantDouble2), v.count, 1); }
static i_Line path(RantPathView v){ return line_of(v.poses, sizeof(RantPose), v.count, 0); }
static i_Line path2d(RantPath2DView v){ return line_of(v.poses, sizeof(RantPose2D), v.count, 1); }

static RantDouble3 point_of(i_Line l, uint32_t i){
    const void *p = l.base + l.stride * i;
    return l.flat ? i_math_lift(*(const RantDouble2 *)p) : *(const RantDouble3 *)p;
}

static RantDouble3 step_of(i_Line l, uint32_t i){
    return rant_double3_subtract(point_of(l, i + 1), point_of(l, i));
}

static double length_of(i_Line l){
    double s = 0.0;
    uint32_t i;
    for (i = 0; i + 1 < l.count; i++) s += rant_double3_length(step_of(l, i));
    return s;
}

/* The way the line runs from point i: its first step of nonzero length from there, else its
 * last one before, else zero. */
static RantDouble3 tangent_of(i_Line l, uint32_t i){
    uint32_t k;
    for (k = i; k + 1 < l.count; k++)
        if (rant_double3_length_squared(step_of(l, k)) > 0.0) return rant_double3_normalize(step_of(l, k));
    for (k = l.count < 2 ? 0 : l.count - 1; k-- > 0;)
        if (rant_double3_length_squared(step_of(l, k)) > 0.0) return rant_double3_normalize(step_of(l, k));
    return rant_double3(0, 0, 0);
}

/* Moves from point *i, *along from the start, to the step holding distance d, and returns how
 * far into that step it lies. A distance at a point moves past every step of no length there. */
static double locate(i_Line l, double d, uint32_t *i, double *along){
    while (*i + 1 < l.count){
        double len = rant_double3_length(step_of(l, *i));
        if (d < *along + len) return i_math_max(0.0, (d - *along) / len);
        *along += len;
        (*i)++;
    }
    return 0.0;
}

static RantDouble3 place(i_Line l, uint32_t i, double t){
    return t > 0.0 ? rant_double3_lerp(point_of(l, i), point_of(l, i + 1), t) : point_of(l, i);
}

static RantCurvePoint at_of(i_Line l, double distance, uint32_t *seg, double *frac){
    RantCurvePoint r;
    uint32_t i = 0;
    double along = 0.0, t;
    memset(&r, 0, sizeof r);
    if (!l.count){ if (seg) *seg = 0; if (frac) *frac = 0.0; return r; }
    t = locate(l, i_math_max(distance, 0.0), &i, &along);
    r.point = place(l, i, t);
    r.tangent = tangent_of(l, i);
    r.distance = along + t * (i + 1 < l.count ? rant_double3_length(step_of(l, i)) : 0.0);
    if (seg) *seg = i;
    if (frac) *frac = t;
    return r;
}

static RantCurvePoint closest_of(i_Line l, RantDouble3 q){
    RantCurvePoint r;
    double best, along = 0.0;
    uint32_t i;
    memset(&r, 0, sizeof r);
    if (!l.count){ r.point = q; return r; }
    r.point = point_of(l, 0);
    best = rant_double3_distance(r.point, q);
    for (i = 0; i + 1 < l.count; i++){
        RantDouble3 a = point_of(l, i), e = step_of(l, i), c;
        double ee = rant_double3_dot(e, e), len = sqrt(ee), t, d;
        if (!(ee > 0.0)) continue;
        t = i_math_clamp(rant_double3_dot(rant_double3_subtract(q, a), e) / ee, 0.0, 1.0);
        c = rant_double3_add(a, rant_double3_scale(e, t));
        d = rant_double3_distance(c, q);
        if (d < best || (d == best && rant_double3_length_squared(r.tangent) == 0.0)){
            best = d;
            r.point = c;
            r.tangent = rant_double3_scale(e, 1.0 / len);
            r.distance = along + t * len;
        }
        along += len;
    }
    return r;
}

static double distance_of(i_Line l, RantDouble3 q){
    return l.count ? rant_double3_distance(closest_of(l, q).point, q) : HUGE_VAL;
}

static RantAlignedBox bounds_of(i_Line l){
    RantAlignedBox b;
    uint32_t i;
    b.min = rant_double3(HUGE_VAL, HUGE_VAL, HUGE_VAL);
    b.max = rant_double3(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL);
    for (i = 0; i < l.count; i++){
        b.min = rant_double3_min(b.min, point_of(l, i));
        b.max = rant_double3_max(b.max, point_of(l, i));
    }
    return b;
}

static uint32_t steps_of(i_Line l){
    uint32_t i, k = 0;
    for (i = 0; i + 1 < l.count; i++) k += (uint32_t)(rant_double3_length_squared(step_of(l, i)) > 0.0);
    return k;
}

/* Calls back with where each of count evenly spread distances falls, walking the line once. */
typedef void (*i_SpacedVisit)(void *out, uint32_t k, i_Line l, uint32_t i, double t);

static RantMathResult resample_of(i_Line l, void *out, uint32_t count, i_SpacedVisit visit){
    double total = length_of(l), along = 0.0;
    uint32_t k, i = 0;
    if (!out || !count) return RANT_MATH_ERR_ARGUMENT;
    if (!l.count) return RANT_MATH_ERR_DEGENERATE;
    for (k = 0; k < count; k++){
        double d = count > 1 ? total * k / (count - 1) : 0.0, t;
        if (k + 1 == count && count > 1){
            i = l.count - 1;
            t = 0.0;
        } else {
            t = locate(l, d, &i, &along);
        }
        visit(out, k, l, i, t);
    }
    return RANT_MATH_OK;
}

/* Douglas and Peucker: keep the point farthest from the chord while it is beyond tolerance,
 * then split there. The stack holds the spans still to look at. */
static RantMathResult simplify_of(i_Line l, double tolerance, unsigned char **kept, uint32_t *need){
    uint32_t n = l.count, *stack, top = 0, i, k = 0;
    unsigned char *keep;
    *kept = NULL;
    *need = 0;
    if (!n) return RANT_MATH_OK;
    keep = (unsigned char *)calloc(n, 1);
    stack = (uint32_t *)malloc(sizeof(uint32_t) * 2 * n);
    if (!keep || !stack){ free(keep); free(stack); return RANT_MATH_ERR_MEMORY; }
    keep[0] = keep[n - 1] = 1;
    if (n > 2){ stack[top++] = 0; stack[top++] = n - 1; }
    while (top){
        uint32_t hi = stack[--top], lo = stack[--top], far = lo;
        RantSegment chord;
        double worst = -1.0;
        chord.a = point_of(l, lo);
        chord.b = point_of(l, hi);
        for (i = lo + 1; i < hi; i++){
            double d = rant_segment_distance(chord, point_of(l, i));
            if (d > worst){ worst = d; far = i; }
        }
        if (worst > tolerance){
            keep[far] = 1;
            if (far - lo > 1){ stack[top++] = lo; stack[top++] = far; }
            if (hi - far > 1){ stack[top++] = far; stack[top++] = hi; }
        }
    }
    free(stack);
    for (i = 0; i < n; i++) k += keep[i];
    *kept = keep;
    *need = k;
    return RANT_MATH_OK;
}

static void visit_point3(void *out, uint32_t k, i_Line l, uint32_t i, double t){ ((RantDouble3 *)out)[k] = place(l, i, t); }
static void visit_point2(void *out, uint32_t k, i_Line l, uint32_t i, double t){ ((RantDouble2 *)out)[k] = i_math_flat(place(l, i, t)); }

static RantPose pose_at(RantPathView p, uint32_t i, double t){
    return t > 0.0 && i + 1 < p.count ? rant_pose_interpolate(p.poses[i], p.poses[i + 1], t) : p.poses[i];
}

static RantPose2D pose2d_at(RantPath2DView p, uint32_t i, double t){
    return t > 0.0 && i + 1 < p.count ? rant_pose2d_interpolate(p.poses[i], p.poses[i + 1], t) : p.poses[i];
}

/* The resample of a path hands its poses to this through the line's base. */
static void visit_pose(void *out, uint32_t k, i_Line l, uint32_t i, double t){
    RantPathView p;
    p.poses = (const RantPose *)(const void *)l.base;
    p.count = l.count;
    ((RantPose *)out)[k] = pose_at(p, i, t);
}

static void visit_pose2d(void *out, uint32_t k, i_Line l, uint32_t i, double t){
    RantPath2DView p;
    p.poses = (const RantPose2D *)(const void *)l.base;
    p.count = l.count;
    ((RantPose2D *)out)[k] = pose2d_at(p, i, t);
}

static RantCurvePoint2D flat_curve(RantCurvePoint c){
    RantCurvePoint2D r;
    r.point = i_math_flat(c.point);
    r.tangent = i_math_flat(c.tangent);
    r.distance = c.distance;
    return r;
}

static RantAlignedBox2D flat_box(RantAlignedBox b){
    RantAlignedBox2D r;
    r.min = i_math_flat(b.min);
    r.max = i_math_flat(b.max);
    return r;
}

double rant_polyline_length(RantPolylineView l){ return length_of(polyline(l)); }
double rant_polyline_distance(RantPolylineView l, RantDouble3 point){ return distance_of(polyline(l), point); }
RantCurvePoint rant_polyline_closest(RantPolylineView l, RantDouble3 point){ return closest_of(polyline(l), point); }
RantCurvePoint rant_polyline_at(RantPolylineView l, double distance){ return at_of(polyline(l), distance, NULL, NULL); }
RantAlignedBox rant_polyline_bounds(RantPolylineView l){ return bounds_of(polyline(l)); }

RantMathResult rant_polyline_edges(RantPolylineView l, RantSegment *out, uint32_t *count){
    i_Line line = polyline(l);
    uint32_t i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, steps_of(line), out)) != RANT_MATH_OK) return r;
    for (i = 0; i + 1 < line.count; i++){
        if (!(rant_double3_length_squared(step_of(line, i)) > 0.0)) continue;
        out[k].a = point_of(line, i);
        out[k++].b = point_of(line, i + 1);
    }
    return RANT_MATH_OK;
}

RantMathResult rant_polyline_resample(RantPolylineView l, RantDouble3 *out, uint32_t count){
    return resample_of(polyline(l), out, count, visit_point3);
}

RantMathResult rant_polyline_simplify(RantPolylineView l, double tolerance, RantDouble3 *out, uint32_t *count){
    i_Line line = polyline(l);
    unsigned char *keep;
    uint32_t need, i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = simplify_of(line, tolerance, &keep, &need)) != RANT_MATH_OK) return r;
    if ((r = i_math_room(count, need, out)) == RANT_MATH_OK)
        for (i = 0; i < line.count; i++) if (keep[i]) out[k++] = point_of(line, i);
    free(keep);
    return r;
}

double rant_polyline2d_length(RantPolyline2DView l){ return length_of(polyline2d(l)); }
double rant_polyline2d_distance(RantPolyline2DView l, RantDouble2 point){ return distance_of(polyline2d(l), i_math_lift(point)); }
RantCurvePoint2D rant_polyline2d_closest(RantPolyline2DView l, RantDouble2 point){
    return flat_curve(closest_of(polyline2d(l), i_math_lift(point)));
}
RantCurvePoint2D rant_polyline2d_at(RantPolyline2DView l, double distance){
    return flat_curve(at_of(polyline2d(l), distance, NULL, NULL));
}
RantAlignedBox2D rant_polyline2d_bounds(RantPolyline2DView l){ return flat_box(bounds_of(polyline2d(l))); }

RantMathResult rant_polyline2d_edges(RantPolyline2DView l, RantSegment2D *out, uint32_t *count){
    i_Line line = polyline2d(l);
    uint32_t i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, steps_of(line), out)) != RANT_MATH_OK) return r;
    for (i = 0; i + 1 < line.count; i++){
        if (!(rant_double3_length_squared(step_of(line, i)) > 0.0)) continue;
        out[k].a = l.points[i];
        out[k++].b = l.points[i + 1];
    }
    return RANT_MATH_OK;
}

RantMathResult rant_polyline2d_planes(RantPolyline2DView l, RantPlane2D *out, uint32_t *count){
    i_Line line = polyline2d(l);
    uint32_t i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_room(count, steps_of(line), out)) != RANT_MATH_OK) return r;
    for (i = 0; i + 1 < line.count; i++)
        if (rant_double3_length_squared(step_of(line, i)) > 0.0) rant_plane2d_from_points(l.points[i], l.points[i + 1], &out[k++]);
    return RANT_MATH_OK;
}

RantMathResult rant_polyline2d_resample(RantPolyline2DView l, RantDouble2 *out, uint32_t count){
    return resample_of(polyline2d(l), out, count, visit_point2);
}

RantMathResult rant_polyline2d_simplify(RantPolyline2DView l, double tolerance, RantDouble2 *out, uint32_t *count){
    i_Line line = polyline2d(l);
    unsigned char *keep;
    uint32_t need, i, k = 0;
    RantMathResult r;
    if (!count) return RANT_MATH_ERR_ARGUMENT;
    if ((r = simplify_of(line, tolerance, &keep, &need)) != RANT_MATH_OK) return r;
    if ((r = i_math_room(count, need, out)) == RANT_MATH_OK)
        for (i = 0; i < line.count; i++) if (keep[i]) out[k++] = l.points[i];
    free(keep);
    return r;
}

double rant_path_length(RantPathView p){ return length_of(path(p)); }
double rant_path_distance(RantPathView p, RantDouble3 point){ return distance_of(path(p), point); }
RantCurvePoint rant_path_closest(RantPathView p, RantDouble3 point){ return closest_of(path(p), point); }
RantAlignedBox rant_path_bounds(RantPathView p){ return bounds_of(path(p)); }

RantPose rant_path_at(RantPathView p, double distance){
    uint32_t i;
    double t;
    if (!path(p).count) return rant_pose_identity();
    at_of(path(p), distance, &i, &t);
    return pose_at(p, i, t);
}

RantMathResult rant_path_resample(RantPathView p, RantPose *out, uint32_t count){
    return resample_of(path(p), out, count, visit_pose);
}

double rant_path2d_length(RantPath2DView p){ return length_of(path2d(p)); }
double rant_path2d_distance(RantPath2DView p, RantDouble2 point){ return distance_of(path2d(p), i_math_lift(point)); }
RantCurvePoint2D rant_path2d_closest(RantPath2DView p, RantDouble2 point){
    return flat_curve(closest_of(path2d(p), i_math_lift(point)));
}
RantAlignedBox2D rant_path2d_bounds(RantPath2DView p){ return flat_box(bounds_of(path2d(p))); }

RantPose2D rant_path2d_at(RantPath2DView p, double distance){
    uint32_t i;
    double t;
    if (!path2d(p).count){
        RantPose2D z;
        memset(&z, 0, sizeof z);
        return z;
    }
    at_of(path2d(p), distance, &i, &t);
    return pose2d_at(p, i, t);
}

RantMathResult rant_path2d_resample(RantPath2DView p, RantPose2D *out, uint32_t count){
    return resample_of(path2d(p), out, count, visit_pose2d);
}
