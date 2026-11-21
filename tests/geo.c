#include "check.h"
#include "reference.h"

#define WGS84_A 6378137.0
#define WGS84_B 6356752.314245179

static RantGeoPoint geo(double lat, double lon, double alt){
    RantGeoPoint g;
    g.lat = lat; g.lon = lon; g.alt = alt;
    return g;
}

/* Against 40 digit answers, from the ground to past the satellites. */
static void reference(void){
    double worst_xyz = 0.0, worst_deg = 0.0, worst_alt = 0.0;
    int i, bad = 0;
    for (i = 0; i < REF_GEO_CASES; i++){
        const double *r = REF_GEO + 9 * i;
        RantDouble3 e = rant_geo_point_to_ecef(geo(r[0], r[1], r[2])), want = rant_double3(r[3], r[4], r[5]);
        RantGeoPoint back = rant_geo_point_from_ecef(want);
        double scale = 1.0 + rant_double3_length(want) / WGS84_A, dx = rant_double3_distance(e, want) / scale;
        double dd = fabs(back.lat - r[6]) > fabs(back.lon - r[7]) ? fabs(back.lat - r[6]) : fabs(back.lon - r[7]);
        double da = fabs(back.alt - r[8]) / scale;
        if (dx > worst_xyz) worst_xyz = dx;
        if (dd > worst_deg) worst_deg = dd;
        if (da > worst_alt) worst_alt = da;
        if (!(dx < 5e-9) || !(dd < 1e-13) || !(da < 5e-9)) bad++;
    }
    CHECK(bad == 0, "%d of %d off: ECEF %.3g m, angles %.3g deg, altitude %.3g m", bad, REF_GEO_CASES, worst_xyz, worst_deg, worst_alt);
    printf("geo worst: ECEF %.3g m, angles %.3g deg, altitude %.3g m, per Earth radius of distance\n", worst_xyz, worst_deg, worst_alt);
}

static void exact(void){
    RantDouble3 e;
    RantGeoPoint g;
    e = rant_geo_point_to_ecef(geo(0, 0, 0));
    CHECK(e.x == WGS84_A && e.y == 0.0 && e.z == 0.0, "the equator at the meridian");
    e = rant_geo_point_to_ecef(geo(90, 33, 0));
    CHECK(e.x == 0.0 && e.y == 0.0 && near(e.z, WGS84_B, 1e-9), "the north pole, on the axis exactly");
    e = rant_geo_point_to_ecef(geo(0, 90, 0));
    CHECK(e.x == 0.0 && e.y == WGS84_A, "a quarter turn east");
    e = rant_geo_point_to_ecef(geo(0, 180, 0));
    CHECK(e.x == -WGS84_A && e.y == 0.0, "half a turn");
    e = rant_geo_point_to_ecef(geo(-90, 0, 100));
    CHECK(near(e.z, -WGS84_B - 100.0, 1e-9), "below the south pole");
    g = rant_geo_point_from_ecef(rant_double3(-WGS84_A, 0.0, 0.0));
    CHECK(g.lat == 0.0 && g.lon == 180.0 && near(g.alt, 0.0, 1e-9), "longitude 180, never -180");
    g = rant_geo_point_from_ecef(rant_double3(0.0, 0.0, WGS84_B + 5.0));
    CHECK(g.lat == 90.0 && g.lon == 0.0 && near(g.alt, 5.0, 1e-9), "above the pole, longitude 0");
    g = rant_geo_point_from_ecef(rant_double3(0.0, 0.0, 0.0));
    CHECK(g.lat == 90.0 && near(g.alt, -WGS84_B, 1e-6), "the center is the pole's depth below it");
    {   /* deep inside, where many surface normals pass through a point, any answer must lead back */
        static const double inside[5][3] = { {1000, 0, 0}, {30000, 0, 0}, {0, 0, 1000}, {1e5, 2e5, 1e5}, {4e4, 1e4, -1.0} };
        int k, bad = 0;
        for (k = 0; k < 5; k++){
            RantDouble3 p = rant_double3(inside[k][0], inside[k][1], inside[k][2]);
            if (!(rant_double3_distance(rant_geo_point_to_ecef(rant_geo_point_from_ecef(p)), p) < 1e-8)) bad++;
        }
        CHECK(bad == 0, "%d points near the center do not lead back", bad);
    }
    g = rant_geo_point_from_ecef(rant_double3(1e30, 2e30, -1e30));
    CHECK(near(g.lat, -24.0948425521107, 1e-9) && near(g.lon, 63.434948822922, 1e-9), "far past the Earth, it is a point");
}

static void round_trips(void){
    int i, bad = 0;
    double worst = 0.0;
    for (i = 0; i < 2000; i++){
        RantGeoPoint g = geo(random_range(-90, 90), random_range(-180, 180), random_range(-5000, 1e5));
        RantDouble3 e = rant_geo_point_to_ecef(g), again;
        RantGeoPoint back = rant_geo_point_from_ecef(e);
        again = rant_geo_point_to_ecef(back);
        if (rant_double3_distance(e, again) > worst) worst = rant_double3_distance(e, again);
        if (!(rant_double3_distance(e, again) < 5e-9) || !(fabs(back.alt - g.alt) < 5e-9)) bad++;
    }
    CHECK(bad == 0, "%d round trips drift, worst %.3g m", bad, worst);
}

static void enu(void){
    int i, bad = 0, back_bad = 0;
    double worst = 0.0;
    RantGeoPoint o = geo(48.1, 11.5, 520.0);
    RantDouble3 v;
    for (i = 0; i < REF_ENU_CASES; i++){
        const double *r = REF_ENU + 9 * i;
        RantGeoPoint a = geo(r[0], r[1], r[2]), b = geo(r[3], r[4], r[5]), c;
        RantDouble3 got = rant_geo_point_to_enu(a, b);
        double d = rant_double3_distance(got, rant_double3(r[6], r[7], r[8]));
        if (d > worst) worst = d;
        if (!(d < 1e-8)) bad++;
        c = rant_geo_point_from_enu(a, got);
        if (!near(c.lat, b.lat, 1e-12) || !near(c.lon, b.lon, 1e-12) || !near(c.alt, b.alt, 1e-8)) back_bad++;
    }
    CHECK(bad == 0, "%d ENU offsets off, worst %.3g m", bad, worst);
    CHECK(back_bad == 0, "%d do not come back from ENU", back_bad);
    v = rant_geo_point_to_enu(o, geo(48.1, 11.5001, 520.0));
    CHECK(v.x > 7.0 && fabs(v.y) < 1e-3 && fabs(v.z) < 1e-5, "east is +x");
    v = rant_geo_point_to_enu(o, geo(48.1001, 11.5, 520.0));
    CHECK(v.y > 11.0 && fabs(v.x) < 1e-9 && fabs(v.z) < 1e-5, "north is +y");
    v = rant_geo_point_to_enu(o, geo(48.1, 11.5, 530.0));
    CHECK(near3(v, rant_double3(0, 0, 10.0), 1e-8), "up is +z");
}

void test_geo(void){
    reference();
    exact();
    round_trips();
    enu();
}
