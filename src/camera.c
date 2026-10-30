#include "internal.h"

static void i_poly_mul(const double *a, int na, const double *b, int nb, double *out){
    int i, j;
    for (i = 0; i <= na + nb; i++) out[i] = 0.0;
    for (i = 0; i <= na; i++) for (j = 0; j <= nb; j++) out[i + j] += a[i] * b[j];
}

/* How far from 0 the polynomial p stays above 0 toward end, end when it holds all the way. No root
 * is closer than a step that keeps each negative term of p's expansion about s under its share of p(s). */
static double i_reach(const double *p, int n, double end){
    double s = 0.0, a[14] = { 0.0 };
    int step, i, k;
    for (step = 0; step < 400; step++){
        double left = end - s, lose = 0.0, h = HUGE_VAL;
        int neg = 0;
        for (i = 0; i <= n; i++) a[i] = p[i];
        for (i = 0; i < n; i++) for (k = n - 1; k >= i; k--) a[k] += s * a[k + 1];
        if (!(a[0] > 0.0)) return s;
        for (k = 1; k <= n; k++) if (a[k] < 0.0) neg++;
        if (!neg) return end;
        if (left < HUGE_VAL){
            for (k = n; k >= 1; k--) lose = lose * left + (a[k] < 0.0 ? -a[k] : 0.0);
            if (lose * left < a[0]) return end;
        }
        for (k = 1; k <= n; k++) if (a[k] < 0.0) h = i_math_min(h, pow(a[0] / (neg * -a[k]), 1.0 / k));
        if (!(h > s * 1e-15)) return s;
        s += h;
    }
    return s;
}

static void i_fold(i_MathLens *L){
    int i;
    L->fold = L->fisheye ? I_MATH_PI : HUGE_VAL;
    for (i = 0; i < L->bounds; i++) L->fold = i_math_min(L->fold, i_reach(L->bound[i], L->degree[i], HUGE_VAL));
}

RantMathResult i_math_lens(const RantCameraIntrinsics *c, int many, i_MathLens *L){
    int i, count;
    if (!c || !(c->fx > 0.0 && c->fy > 0.0) || !i_math_finite(c->fx) || !i_math_finite(c->fy)
        || !i_math_finite(c->cx) || !i_math_finite(c->cy)) return RANT_MATH_ERR_ARGUMENT;
    switch (c->model){
    case RANT_DISTORTION_MODEL_NONE: count = 0; break;
    case RANT_DISTORTION_MODEL_BROWN_CONRADY: count = 5; break;
    case RANT_DISTORTION_MODEL_FISHEYE: count = 4; break;
    case RANT_DISTORTION_MODEL_RATIONAL: count = 8; break;
    default: return RANT_MATH_ERR_ARGUMENT;
    }
    memset(L, 0, sizeof *L);
    L->c = c;
    for (i = 0; i < count; i++){
        if (!i_math_finite(c->coeffs[i])) return RANT_MATH_ERR_ARGUMENT;
        L->k[i] = c->coeffs[i];
    }
    L->fisheye = c->model == RANT_DISTORTION_MODEL_FISHEYE;
    if (L->fisheye){
        /* theta g(theta^2) grows as 1 + 3 k1 theta^2 + 5 k2 theta^4 + 7 k3 theta^6 + 9 k4 theta^8 */
        L->bounds = 1;
        L->bound[0][0] = 1.0;
        for (i = 0; i < 4; i++) L->bound[0][2 * i + 2] = (2 * i + 3) * L->k[i];
        L->degree[0] = 8;
    } else {
        /* The Jacobian is symmetric: radial eigenvalues g = N/M and slope G/M^2, plus a tangential
         * part under t r. While both stay above t r over the whole disk, the map is one to one. */
        double n[4], m[4], dn[3], dm[3], nm[7], a[6], b[6], g[7], mm[7], t = 6.0 * (fabs(L->k[2]) + fabs(L->k[3]));
        n[0] = 1.0; n[1] = L->k[0]; n[2] = L->k[1]; n[3] = L->k[4];
        m[0] = 1.0; m[1] = L->k[5]; m[2] = L->k[6]; m[3] = L->k[7];
        for (i = 0; i < 3; i++){ dn[i] = (i + 1) * n[i + 1]; dm[i] = (i + 1) * m[i + 1]; }
        i_poly_mul(n, 3, m, 3, nm);
        i_poly_mul(dn, 2, m, 3, a);
        i_poly_mul(n, 3, dm, 2, b);
        i_poly_mul(m, 3, m, 3, mm);
        for (i = 0; i <= 6; i++) g[i] = nm[i] + (i ? 2.0 * (a[i - 1] - b[i - 1]) : 0.0);
        /* in r, each power of q lands on an even power and the tangential bound on the odd ones */
        L->bounds = 3;
        for (i = 0; i <= 3; i++){
            L->bound[0][2 * i] = n[i];
            L->bound[0][2 * i + 1] = -t * m[i];
            L->bound[2][2 * i] = m[i];
        }
        for (i = 0; i <= 6; i++){
            L->bound[1][2 * i] = g[i];
            L->bound[1][2 * i + 1] = -t * mm[i];
        }
        L->degree[0] = 7;
        L->degree[1] = 13;
        L->degree[2] = 6;
    }
    for (i = 0; i < L->bounds; i++) while (L->degree[i] > 0 && L->bound[i][L->degree[i]] == 0.0) L->degree[i]--;
    L->fold = -1.0;
    if (many) i_fold(L);
    return RANT_MATH_OK;
}

/* 1 when the lens is one to one on the whole disk out to r, the normalized radius or a fisheye's angle. */
static int i_grows(const i_MathLens *L, double r){
    int i;
    if (L->fisheye && !(r < I_MATH_PI)) return 0;
    if (L->fold >= 0.0) return r < L->fold;
    for (i = 0; i < L->bounds; i++) if (i_reach(L->bound[i], L->degree[i], r) < r) return 0;
    return 1;
}

/* The radius a ray r off the axis lands at, r g(r^2) with r an angle for a fisheye, and its slope. */
static double i_bend(const i_MathLens *L, double r, double *slope){
    const double *k = L->k;
    double q = r * r, g, dg;
    if (L->fisheye){
        g = 1.0 + q * (k[0] + q * (k[1] + q * (k[2] + q * k[3])));
        dg = k[0] + q * (2.0 * k[1] + q * (3.0 * k[2] + q * 4.0 * k[3]));
    } else {
        double n = 1.0 + q * (k[0] + q * (k[1] + q * k[4])), m = 1.0 + q * (k[5] + q * (k[6] + q * k[7]));
        double dn = k[0] + q * (2.0 * k[1] + q * 3.0 * k[4]), dm = k[5] + q * (2.0 * k[6] + q * 3.0 * k[7]);
        g = n / m;
        dg = (dn * m - n * dm) / (m * m);
    }
    if (slope) *slope = g + 2.0 * q * dg;
    return r * g;
}

/* A pinhole lens on the normalized point a, b, and its Jacobian, which is symmetric. */
static void i_distort(const i_MathLens *L, double a, double b, double *xd, double *yd, double j[3]){
    const double *k = L->k;
    double p1 = k[2], p2 = k[3], q = a * a + b * b, dg, g;
    double n = 1.0 + q * (k[0] + q * (k[1] + q * k[4])), m = 1.0 + q * (k[5] + q * (k[6] + q * k[7]));
    double dn = k[0] + q * (2.0 * k[1] + q * 3.0 * k[4]), dm = k[5] + q * (2.0 * k[6] + q * 3.0 * k[7]);
    g = n / m;
    dg = (dn * m - n * dm) / (m * m);
    *xd = a * g + 2.0 * p1 * a * b + p2 * (q + 2.0 * a * a);
    *yd = b * g + p1 * (q + 2.0 * b * b) + 2.0 * p2 * a * b;
    if (j){
        j[0] = g + 2.0 * a * a * dg + 2.0 * p1 * b + 6.0 * p2 * a;
        j[1] = 2.0 * a * b * dg + 2.0 * p1 * a + 2.0 * p2 * b;
        j[2] = g + 2.0 * b * b * dg + 6.0 * p1 * b + 2.0 * p2 * a;
    }
}

RantMathResult i_math_lens_project(i_MathLens *L, RantDouble3 o, RantDouble2 *pixel){
    double xd, yd;
    if (!i_math_finite(o.x) || !i_math_finite(o.y) || !i_math_finite(o.z)) return RANT_MATH_ERR_ARGUMENT;
    if (L->fisheye){
        double r = hypot(o.x, o.y), t = atan2(r, o.z), rd;
        if (!(r > 0.0 || o.z > 0.0) || !i_grows(L, t)) return RANT_MATH_ERR_OUTSIDE;
        rd = i_bend(L, t, NULL);
        xd = r > 0.0 ? rd * o.x / r : 0.0;
        yd = r > 0.0 ? rd * o.y / r : 0.0;
    } else {
        double a, b;
        if (!(o.z > 0.0)) return RANT_MATH_ERR_OUTSIDE;
        a = o.x / o.z;
        b = o.y / o.z;
        /* a lens found one to one everywhere needs no radius */
        if (L->fold != HUGE_VAL && !i_grows(L, hypot(a, b))) return RANT_MATH_ERR_OUTSIDE;
        i_distort(L, a, b, &xd, &yd, NULL);
    }
    pixel->x = L->c->fx * xd + L->c->cx;
    pixel->y = L->c->fy * yd + L->c->cy;
    return RANT_MATH_OK;
}

/* The r on the growing branch that lands at rd, or 0 when the lens reaches no farther. */
static int i_unbend(i_MathLens *L, double rd, double *out){
    double lo = 0.0, hi = rd, r, f, slope, next;
    int i;
    for (i = 0; ; i++){
        if (!i_grows(L, hi)){
            /* the farthest it reaches is at its fold */
            if (L->fold < 0.0) i_fold(L);
            hi = i_math_max(lo, L->fold);
            if (!(i_bend(L, hi, NULL) >= rd)) return 0;
            break;
        }
        if (i_bend(L, hi, NULL) >= rd) break;
        if (i == 200) return 0;
        lo = hi;
        hi *= 2.0;
    }
    /* Newton kept inside the bracket, bisecting when it would leave it */
    r = hi;
    for (i = 0; i < 200; i++){
        f = i_bend(L, r, &slope) - rd;
        if (f == 0.0) break;
        if (f < 0.0) lo = r; else hi = r;
        next = r - f / slope;
        if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
        if (next == r || hi - lo <= 1e-17 * hi) break;
        r = next;
    }
    *out = r;
    return 1;
}

RantMathResult i_math_lens_ray(i_MathLens *L, RantDouble2 pixel, RantDouble3 *o){
    const RantCameraIntrinsics *c = L->c;
    double xd = (pixel.x - c->cx) / c->fx, yd = (pixel.y - c->cy) / c->fy, rd, r = 0.0, a, b;
    int i, tangential = !L->fisheye && (L->k[2] != 0.0 || L->k[3] != 0.0);
    if (!i_math_finite(xd) || !i_math_finite(yd)) return RANT_MATH_ERR_ARGUMENT;
    if (c->model == RANT_DISTORTION_MODEL_NONE){
        *o = rant_double3(xd, yd, 1.0);
        return RANT_MATH_OK;
    }
    rd = hypot(xd, yd);
    if (rd > 0.0 && !i_unbend(L, rd, &r)){
        /* past the radial reach, the tangential terms may still bring the pixel within the disk */
        if (!tangential) return RANT_MATH_ERR_OUTSIDE;
        if (L->fold < 0.0) i_fold(L);
        if (!(L->fold < HUGE_VAL)) return RANT_MATH_ERR_OUTSIDE;
        r = L->fold * (1.0 - 1e-9);
    }
    if (L->fisheye){
        double s = rd > 0.0 ? sin(r) / rd : 0.0;
        *o = rant_double3(xd * s, yd * s, cos(r));
        return RANT_MATH_OK;
    }
    a = rd > 0.0 ? xd * r / rd : 0.0;
    b = rd > 0.0 ? yd * r / rd : 0.0;
    if (tangential){
        /* the radial answer starts Newton on the tangential terms, each step halved until it gains */
        double ex, ey, e, j[3];
        i_distort(L, a, b, &ex, &ey, j);
        ex -= xd; ey -= yd;
        e = ex * ex + ey * ey;
        for (i = 0; i < 60 && e > 0.0; i++){
            double det = j[0] * j[2] - j[1] * j[1], da, db, t = 1.0, na = a, nb = b, nx = 0.0, ny = 0.0, ne = e, nj[3];
            int h;
            if (!(det > 0.0)) return RANT_MATH_ERR_OUTSIDE;
            da = (j[2] * ex - j[1] * ey) / det;
            db = (j[0] * ey - j[1] * ex) / det;
            for (h = 0; h < 40; h++, t *= 0.5){
                na = a - t * da;
                nb = b - t * db;
                if (!i_grows(L, hypot(na, nb))) continue;
                i_distort(L, na, nb, &nx, &ny, nj);
                nx -= xd; ny -= yd;
                ne = nx * nx + ny * ny;
                if (ne < e) break;
            }
            if (h == 40) break;
            a = na; b = nb; ex = nx; ey = ny; e = ne;
            j[0] = nj[0]; j[1] = nj[1]; j[2] = nj[2];
        }
        if (!(sqrt(e) <= 1e-10 * (1.0 + rd))) return RANT_MATH_ERR_OUTSIDE;
    }
    *o = rant_double3(a, b, 1.0);
    return RANT_MATH_OK;
}

/* A camera's +x ahead, y left and z up from the optical frame's x right, y down and z ahead. */
static RantDouble3 i_body(RantDouble3 o){ return rant_double3(o.z, -o.x, -o.y); }
static RantDouble3 i_optical(RantDouble3 b){ return rant_double3(-b.y, -b.z, b.x); }

RantMathResult rant_camera_project(const RantCameraIntrinsics *camera, RantDouble3 point, RantDouble2 *pixel){
    i_MathLens L;
    RantDouble2 px;
    RantMathResult r;
    if (!pixel) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_lens(camera, 0, &L)) != RANT_MATH_OK) return r;
    if ((r = i_math_lens_project(&L, i_optical(point), &px)) == RANT_MATH_OK) *pixel = px;
    return r;
}

RantMathResult rant_camera_project_points(const RantCameraIntrinsics *camera, const RantDouble3 *points,
                                          RantDouble2 *pixels, uint32_t count){
    i_MathLens L;
    RantMathResult r;
    uint32_t i;
    if (count && (!points || !pixels)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_lens(camera, 1, &L)) != RANT_MATH_OK) return r;
    for (i = 0; i < count; i++)
        if (i_math_lens_project(&L, i_optical(points[i]), &pixels[i]) != RANT_MATH_OK)
            pixels[i] = rant_double2(NAN, NAN);
    return RANT_MATH_OK;
}

RantMathResult rant_camera_ray(const RantCameraIntrinsics *camera, RantDouble2 pixel, RantRay *out){
    i_MathLens L;
    RantDouble3 o;
    RantMathResult r;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_lens(camera, 0, &L)) != RANT_MATH_OK) return r;
    if ((r = i_math_lens_ray(&L, pixel, &o)) != RANT_MATH_OK) return r;
    out->origin = rant_double3(0.0, 0.0, 0.0);
    out->direction = rant_double3_normalize(i_body(o));
    return RANT_MATH_OK;
}

RantMathResult rant_camera_unproject(const RantCameraIntrinsics *camera, RantDouble2 pixel, double depth, RantDouble3 *out){
    i_MathLens L;
    RantDouble3 o;
    RantMathResult r;
    if (!out || !(depth > 0.0) || !i_math_finite(depth)) return RANT_MATH_ERR_ARGUMENT;
    if ((r = i_math_lens(camera, 0, &L)) != RANT_MATH_OK) return r;
    if ((r = i_math_lens_ray(&L, pixel, &o)) != RANT_MATH_OK) return r;
    if (!(o.z > 0.0)) return RANT_MATH_ERR_OUTSIDE;
    *out = rant_double3(depth, -o.x * (depth / o.z), -o.y * (depth / o.z));
    return RANT_MATH_OK;
}

RantMathResult rant_camera_scale(const RantCameraIntrinsics *camera, uint32_t width, uint32_t height,
                                 RantCameraIntrinsics *out){
    i_MathLens L;
    RantCameraIntrinsics o;
    double sx, sy;
    if (!out || i_math_lens(camera, 0, &L) != RANT_MATH_OK || !camera->width || !camera->height || !width || !height)
        return RANT_MATH_ERR_ARGUMENT;
    o = *camera;
    sx = (double)width / camera->width;
    sy = (double)height / camera->height;
    o.width = width;
    o.height = height;
    o.fx = camera->fx * sx;
    o.fy = camera->fy * sy;
    /* pixel centers sit at whole numbers, so the image's own edge at -0.5 is what scales */
    o.cx = (camera->cx + 0.5) * sx - 0.5;
    o.cy = (camera->cy + 0.5) * sy - 0.5;
    *out = o;
    return RANT_MATH_OK;
}

RantMathResult rant_camera_crop(const RantCameraIntrinsics *camera, uint32_t x, uint32_t y,
                                uint32_t width, uint32_t height, RantCameraIntrinsics *out){
    i_MathLens L;
    RantCameraIntrinsics o;
    if (!out || i_math_lens(camera, 0, &L) != RANT_MATH_OK || !width || !height
        || (uint64_t)x + width > camera->width || (uint64_t)y + height > camera->height)
        return RANT_MATH_ERR_ARGUMENT;
    o = *camera;
    o.width = width;
    o.height = height;
    o.cx = camera->cx - x;
    o.cy = camera->cy - y;
    *out = o;
    return RANT_MATH_OK;
}

RantMathResult rant_camera_fov(const RantCameraIntrinsics *camera, RantDouble2 *out){
    i_MathLens L;
    RantDouble3 l, r, t, b;
    RantMathResult e;
    if (!out) return RANT_MATH_ERR_ARGUMENT;
    if ((e = i_math_lens(camera, 1, &L)) != RANT_MATH_OK) return e;
    if (!camera->width || !camera->height) return RANT_MATH_ERR_ARGUMENT;
    if ((e = i_math_lens_ray(&L, rant_double2(-0.5, camera->cy), &l)) != RANT_MATH_OK
        || (e = i_math_lens_ray(&L, rant_double2(camera->width - 0.5, camera->cy), &r)) != RANT_MATH_OK
        || (e = i_math_lens_ray(&L, rant_double2(camera->cx, -0.5), &t)) != RANT_MATH_OK
        || (e = i_math_lens_ray(&L, rant_double2(camera->cx, camera->height - 0.5), &b)) != RANT_MATH_OK) return e;
    *out = rant_double2(atan2(r.x, r.z) - atan2(l.x, l.z), atan2(b.y, b.z) - atan2(t.y, t.z));
    return RANT_MATH_OK;
}

RantMathResult rant_camera_from_fov(uint32_t width, uint32_t height, RantDouble2 fov, RantCameraIntrinsics *out){
    RantCameraIntrinsics o;
    if (!out || !width || !height || !(fov.x > 0.0 && fov.x < I_MATH_PI) || !(fov.y >= 0.0 && fov.y < I_MATH_PI))
        return RANT_MATH_ERR_ARGUMENT;
    memset(&o, 0, sizeof o);
    o.width = width;
    o.height = height;
    o.fx = 0.5 * width / tan(0.5 * fov.x);
    o.fy = fov.y > 0.0 ? 0.5 * height / tan(0.5 * fov.y) : o.fx;
    o.cx = 0.5 * (width - 1.0);
    o.cy = 0.5 * (height - 1.0);
    o.model = RANT_DISTORTION_MODEL_NONE;
    *out = o;
    return RANT_MATH_OK;
}
