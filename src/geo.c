#include "internal.h"
#include <float.h>

/* WGS84: the equator's radius and the flattening, and from them e^2, (1 - f)^2 and e^4. */
#define I_A 6378137.0
#define I_F (1.0 / 298.257223563)
#define I_E2 (I_F * (2.0 - I_F))
#define I_E2M ((1.0 - I_F) * (1.0 - I_F))
#define I_E4 (I_E2 * I_E2)

/* Degrees reduced to [-45, 45] first, so whole quarter turns come out exact. */
static void i_sincosd(double x, double *s, double *c){
    int q = 0;
    double r = remquo(x, 90.0, &q) * (I_MATH_PI / 180.0), a = sin(r), b = cos(r);
    switch ((unsigned)q & 3u){
    case 0u: *s = a; *c = b; break;
    case 1u: *s = b; *c = -a; break;
    case 2u: *s = -a; *c = -b; break;
    default: *s = -b; *c = a; break;
    }
}

static double i_atan2d(double y, double x){
    int q = 0;
    double a;
    if (fabs(y) > fabs(x)){ a = x; x = y; y = a; q = 2; }
    if (x < 0.0){ x = -x; q++; }
    a = atan2(y, x) * (180.0 / I_MATH_PI);
    switch (q){
    case 1: return (y >= 0.0 ? 180.0 : -180.0) - a;
    case 2: return 90.0 - a;
    case 3: return -90.0 + a;
    default: return a;
    }
}

RantDouble3 rant_geo_point_to_ecef(RantGeoPoint point){
    double sp, cp, sl, cl, n;
    i_sincosd(point.lat, &sp, &cp);
    i_sincosd(point.lon, &sl, &cl);
    n = I_A / sqrt(1.0 - I_E2 * sp * sp);
    return rant_double3((n + point.alt) * cp * cl, (n + point.alt) * cp * sl, (n * I_E2M + point.alt) * sp);
}

/* Vermeille's closed form as Karney's GeographicLib arranges it, exact to rounding at any height. */
RantGeoPoint rant_geo_point_from_ecef(RantDouble3 ecef){
    RantGeoPoint g;
    double x = ecef.x, y = ecef.y, z = ecef.z;
    double r = hypot(x, y), sl = r != 0.0 ? y / r : 0.0, cl = r != 0.0 ? x / r : 1.0, h = hypot(r, z), sp, cp;
    if (h > 2.0 * I_A / DBL_EPSILON){
        /* so far off that the Earth is a point, halved so the squares stay finite */
        double hh;
        r = hypot(x / 2.0, y / 2.0);
        sl = r != 0.0 ? (y / 2.0) / r : 0.0;
        cl = r != 0.0 ? (x / 2.0) / r : 1.0;
        hh = hypot(z / 2.0, r);
        sp = (z / 2.0) / hh;
        cp = r / hh;
    } else {
        double p = i_math_sqr(r / I_A), q = I_E2M * i_math_sqr(z / I_A), t = (p + q - I_E4) / 6.0;
        if (!(I_E4 * q == 0.0 && t <= 0.0)){
            double s = I_E4 * p * q / 4.0, t2 = t * t, t3 = t * t2, disc = s * (2.0 * t3 + s), u = t, v, uv, w, k, d, hh;
            if (disc >= 0.0){
                double c3 = s + t3, c;
                /* the root's sign is picked against cancellation, which leaves u the same */
                c3 += c3 < 0.0 ? -sqrt(disc) : sqrt(disc);
                c = cbrt(c3);
                u += c + (c != 0.0 ? t2 / c : 0.0);
            } else {
                u += 2.0 * t * cos(atan2(sqrt(-disc), -(s + t3)) / 3.0);
            }
            v = sqrt(u * u + I_E4 * q);
            uv = u < 0.0 ? I_E4 * q / (v - u) : u + v;
            w = i_math_max(0.0, I_E2 * (uv - q) / (2.0 * v));
            k = uv / (sqrt(uv + w * w) + w);
            d = k * r / (k + I_E2);
            hh = hypot(z / k, r / (k + I_E2));
            sp = (z / k) / hh;
            cp = (r / (k + I_E2)) / hh;
            h = (1.0 - I_E2M / k) * hypot(d, z);
        } else {
            /* on the equator's disc deep inside, where the limits of the formulas above are taken */
            double zz = sqrt((I_E4 - p) / I_E2M), xx = sqrt(p), hh = hypot(zz, xx);
            sp = zz / hh;
            cp = xx / hh;
            if (z < 0.0) sp = -sp;
            h = -I_A * I_E2M * hh / I_E2;
        }
    }
    g.lat = i_atan2d(sp, cp);
    g.lon = i_atan2d(sl, cl);
    g.alt = h;
    return g;
}

/* East, north and up at origin as rows, in ECEF. */
static void i_enu(RantGeoPoint origin, RantDouble3 axes[3]){
    double sp, cp, sl, cl;
    i_sincosd(origin.lat, &sp, &cp);
    i_sincosd(origin.lon, &sl, &cl);
    axes[0] = rant_double3(-sl, cl, 0.0);
    axes[1] = rant_double3(-sp * cl, -sp * sl, cp);
    axes[2] = rant_double3(cp * cl, cp * sl, sp);
}

RantDouble3 rant_geo_point_to_enu(RantGeoPoint origin, RantGeoPoint point){
    RantDouble3 axes[3], d = rant_double3_subtract(rant_geo_point_to_ecef(point), rant_geo_point_to_ecef(origin));
    i_enu(origin, axes);
    return rant_double3(rant_double3_dot(axes[0], d), rant_double3_dot(axes[1], d), rant_double3_dot(axes[2], d));
}

RantGeoPoint rant_geo_point_from_enu(RantGeoPoint origin, RantDouble3 enu){
    RantDouble3 axes[3], e;
    i_enu(origin, axes);
    e = rant_double3_add(rant_double3_add(rant_double3_scale(axes[0], enu.x), rant_double3_scale(axes[1], enu.y)),
                         rant_double3_scale(axes[2], enu.z));
    return rant_geo_point_from_ecef(rant_double3_add(rant_geo_point_to_ecef(origin), e));
}
