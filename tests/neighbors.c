#include <stdlib.h>
#include <time.h>
#include "check.h"
#include "reference.h"

static const RantDouble3 *ref_cloud(void){ return (const RantDouble3 *)REF_CLOUD; }

/* The order the tree promises: nearer first, and the lower index among equals. */
typedef struct { double d2; uint32_t i; } Hit;
static int hit_order(const void *a, const void *b){
    const Hit *x = (const Hit *)a, *y = (const Hit *)b;
    if (x->d2 != y->d2) return x->d2 < y->d2 ? -1 : 1;
    return x->i < y->i ? -1 : (x->i > y->i);
}
static double d2_of(RantDouble3 a, RantDouble3 b){
    double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return x * x + y * y + z * z;
}
static int finite3(RantDouble3 p){ return p.x - p.x == 0.0 && p.y - p.y == 0.0 && p.z - p.z == 0.0; }

/* Every finite point by distance, the answer the tree must match exactly. */
static uint32_t brute(const RantDouble3 *p, uint32_t n, RantDouble3 q, Hit *out){
    uint32_t i, k = 0;
    for (i = 0; i < n; i++) if (finite3(p[i])){ out[k].d2 = d2_of(p[i], q); out[k++].i = i; }
    qsort(out, k, sizeof *out, hit_order);
    return k;
}

/* Points on a coarse grid with twins and NaN among them, so ties are everywhere. */
static uint32_t tied_points(RantDouble3 *p, uint32_t most){
    uint32_t i;
    for (i = 0; i < most; i++){
        double u = random_unit();
        if (u < 0.05) p[i] = rant_double3(NAN, 0.0, 0.0);
        else if (u < 0.15 && i) p[i] = p[(uint32_t)(random_unit() * i)];
        else p[i] = rant_double3(floor(random_range(-4, 4)), floor(random_range(-4, 4)), floor(random_range(-2, 2)));
    }
    return most;
}

static void tree_matches(void){
    static RantDouble3 p[700];
    static Hit want[700];
    static uint32_t idx[720];
    static double dist[720];
    RantKdTree *t;
    int set, q, bad = 0, missed = 0;
    for (set = 0; set < 6; set++){
        uint32_t n = set < 3 ? tied_points(p, 700) : 700, i, k;
        if (set >= 3) for (i = 0; i < n; i++) p[i] = random_double3(set == 5 ? 1e6 : 3.0);
        CHECK(rant_kdtree_create(p, n, &t) == RANT_MATH_OK, "set %d: a tree", set);
        for (q = 0; q < 60; q++){
            static const uint32_t ks[] = { 1, 2, 7, 33, 750 };
            RantDouble3 at = q % 3 == 0 && finite3(p[q]) ? p[q] : (set < 3 ? rant_double3(floor(random_range(-5, 5)) + 0.5 * (q & 1), floor(random_range(-5, 5)), 0.0) : random_double3(set == 5 ? 1e6 : 3.5));
            uint32_t m = brute(p, n, at, want), c, j;
            double r;
            for (j = 0; j < 5; j++){
                c = ks[j];
                if (rant_kdtree_nearest(t, at, idx, dist, &c) != RANT_MATH_OK || c != (ks[j] < m ? ks[j] : m)){ bad++; continue; }
                for (k = 0; k < c; k++) if (idx[k] != want[k].i || dist[k] != sqrt(want[k].d2)){ bad++; break; }
            }
            /* a radius on a tie, and one between */
            r = m > 20 ? sqrt(want[20].d2) * (q & 1 ? 1.0 : 1.0001) : 1.0;
            c = 720;
            if (rant_kdtree_within(t, at, r, idx, dist, &c) != RANT_MATH_OK){ bad++; continue; }
            for (k = 0; k < m && want[k].d2 <= r * r; k++)
                if (k >= c || idx[k] != want[k].i || dist[k] != sqrt(want[k].d2)){ missed++; break; }
            if (k != c) missed++;
        }
        rant_kdtree_destroy(t);
    }
    CHECK(bad == 0, "%d nearest searches differ from every point by distance", bad);
    CHECK(missed == 0, "%d radius searches differ", missed);
}

static void tree_edges(void){
    static RantDouble3 same[5000];
    RantDouble3 nan3 = rant_double3(NAN, 1, 2), q = rant_double3(1, 1, 1);
    RantKdTree *t;
    uint32_t idx[8], c, i;
    double d[8];
    clock_t start;
    for (i = 0; i < 5000; i++) same[i] = rant_double3(1, 2, 3);
    start = clock();
    CHECK(rant_kdtree_create(same, 5000, &t) == RANT_MATH_OK, "5000 copies of one point");
    c = 3;
    CHECK(rant_kdtree_nearest(t, q, idx, d, &c) == RANT_MATH_OK && c == 3 && idx[0] == 0 && idx[1] == 1 && idx[2] == 2,
          "the lowest indices first among equals");
    c = 0;
    CHECK(rant_kdtree_within(t, q, 10.0, NULL, NULL, &c) == RANT_MATH_ERR_SPACE && c == 5000, "all 5000 in reach, counted");
    rant_kdtree_destroy(t);
    CHECK((double)(clock() - start) / CLOCKS_PER_SEC < 1.0, "copies of one point stay cheap");
    CHECK(rant_kdtree_create(&nan3, 1, &t) == RANT_MATH_OK, "a tree of no finite point");
    c = 4;
    CHECK(rant_kdtree_nearest(t, q, idx, NULL, &c) == RANT_MATH_OK && c == 0, "finds nothing");
    c = 4;
    CHECK(rant_kdtree_within(t, q, 1e9, idx, NULL, &c) == RANT_MATH_OK && c == 0, "nothing in any reach");
    CHECK(rant_kdtree_nearest(t, nan3, idx, NULL, &c) == RANT_MATH_ERR_ARGUMENT, "a NaN point to search from");
    CHECK(rant_kdtree_within(t, q, -1.0, idx, NULL, &c) == RANT_MATH_ERR_ARGUMENT, "a negative radius");
    CHECK(rant_kdtree_within(t, q, NAN, idx, NULL, &c) == RANT_MATH_ERR_ARGUMENT, "a NaN radius");
    rant_kdtree_destroy(t);
    rant_kdtree_destroy(NULL);
    CHECK(rant_kdtree_create(NULL, 0, &t) == RANT_MATH_OK, "an empty tree");
    c = 2;
    CHECK(rant_kdtree_nearest(t, q, idx, NULL, &c) == RANT_MATH_OK && c == 0, "is empty");
    rant_kdtree_destroy(t);
    CHECK(rant_kdtree_create(NULL, 3, &t) == RANT_MATH_ERR_ARGUMENT, "3 points need data");
}

static void tree_speed(void){
    uint32_t n = 200000, i, c, idx[10];
    RantDouble3 *p = (RantDouble3 *)malloc(sizeof *p * n);
    RantKdTree *t;
    clock_t start;
    double built, searched;
    if (!p) return;
    for (i = 0; i < n; i++) p[i] = random_double3(20.0);
    start = clock();
    rant_kdtree_create(p, n, &t);
    built = (double)(clock() - start) / CLOCKS_PER_SEC;
    start = clock();
    for (i = 0; i < n; i++){ c = 10; rant_kdtree_nearest(t, p[i], idx, NULL, &c); }
    searched = (double)(clock() - start) / CLOCKS_PER_SEC;
    printf("kd tree of %u points: built in %.0f ms, 10 nearest of each in %.0f ms\n", n, 1e3 * built, 1e3 * searched);
    rant_kdtree_destroy(t);
    free(p);
}

/* The reference cloud with NaN points scattered through it, and where each original landed. */
static uint32_t with_nans(RantDouble3 *p, uint32_t *at){
    uint32_t i, n = 0;
    for (i = 0; i < REF_CLOUD_POINTS; i++){
        if (i % 37 == 5) p[n++] = rant_double3(0.0, NAN, 0.0);
        at[i] = n;
        p[n++] = ref_cloud()[i];
    }
    return n;
}

static void filters(void){
    static RantDouble3 p[REF_CLOUD_POINTS + 40];
    static uint32_t at[REF_CLOUD_POINTS];
    static uint8_t keep[REF_CLOUD_POINTS + 40];
    uint32_t n = with_nans(p, at), i, wrong;
    CHECK(rant_points_downsample(ref_cloud(), REF_CLOUD_POINTS, REF_VOXEL, keep) == RANT_MATH_OK, "downsamples");
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[i] != REF_DOWNSAMPLE[i];
    CHECK(wrong == 0, "%u points kept other than numpy keeps", wrong);
    rant_points_downsample(p, n, REF_VOXEL, keep);
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[at[i]] != REF_DOWNSAMPLE[i];
    for (i = 0; i < n; i++) wrong += !finite3(p[i]) && keep[i];
    CHECK(wrong == 0, "NaN points change nothing and are never kept");
    CHECK(rant_points_downsample(p, n, 0.0, keep) == RANT_MATH_ERR_ARGUMENT, "a voxel of 0");
    CHECK(rant_points_downsample(p, n, HUGE_VAL, keep) == RANT_MATH_ERR_ARGUMENT, "an endless voxel");

    CHECK(rant_points_remove_outliers(ref_cloud(), REF_CLOUD_POINTS, REF_OUTLIER_K, 1.0, keep) == RANT_MATH_OK, "removes outliers");
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[i] != REF_OUTLIERS[i];
    CHECK(wrong == 0, "%u points judged other than PCL's rule in numpy", wrong);
    rant_points_remove_outliers(p, n, REF_OUTLIER_K, 1.0, keep);
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[at[i]] != REF_OUTLIERS[i];
    for (i = 0; i < n; i++) wrong += !finite3(p[i]) && keep[i];
    CHECK(wrong == 0, "NaN points change nothing there either");
    CHECK(rant_points_remove_outliers(p, n, 0, 1.0, keep) == RANT_MATH_ERR_ARGUMENT, "no neighbors to judge by");
    {   RantDouble3 lone = rant_double3(NAN, 0.0, 0.0);
        CHECK(rant_points_remove_outliers(&lone, 1, 5, 1.0, keep) == RANT_MATH_OK && keep[0] == 0, "a lone NaN is never kept");
    }
    CHECK(rant_points_remove_outliers(ref_cloud(), 1, 5, 1.0, keep) == RANT_MATH_OK && keep[0] == 1, "a lone point stays");

    CHECK(rant_points_remove_isolated(ref_cloud(), REF_CLOUD_POINTS, REF_ISOLATED_R, 3, keep) == RANT_MATH_OK, "removes isolated points");
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[i] != REF_ISOLATED[i];
    CHECK(wrong == 0, "%u points judged other than scipy counts", wrong);
    rant_points_remove_isolated(p, n, REF_ISOLATED_R, 3, keep);
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += keep[at[i]] != REF_ISOLATED[i];
    CHECK(wrong == 0, "NaN points change nothing");
    rant_points_remove_isolated(p, n, 0.0, 0, keep);
    for (i = 0, wrong = 0; i < n; i++) wrong += keep[i] != finite3(p[i]);
    CHECK(wrong == 0, "needing no neighbors keeps every finite point");
}

static void normals(void){
    static RantDouble3 p[REF_CLOUD_POINTS + 40], nrm[REF_CLOUD_POINTS + 40];
    static uint32_t at[REF_CLOUD_POINTS];
    RantDouble3 view = rant_double3(REF_VIEW[0], REF_VIEW[1], REF_VIEW[2]), line[6];
    uint32_t n = with_nans(p, at), i, wrong = 0;
    double worst = 0.0;
    CHECK(rant_points_normals(p, n, 10, view, nrm) == RANT_MATH_OK, "normals");
    for (i = 0; i < REF_CLOUD_POINTS; i++){
        RantDouble3 want = rant_double3(REF_NORMALS[3 * i], REF_NORMALS[3 * i + 1], REF_NORMALS[3 * i + 2]);
        double a = rant_double3_angle(nrm[at[i]], want);
        if (!(a < 1e-9)) wrong++;
        if (a > worst) worst = a;
        if (rant_double3_dot(nrm[at[i]], rant_double3_subtract(view, p[at[i]])) < 0.0) wrong++;
    }
    for (i = 0; i < n; i++) if (!finite3(p[i]) && finite3(nrm[i])) wrong++;
    CHECK(wrong == 0, "%u normals differ from numpy's, worst %.3g rad", wrong, worst);
    for (i = 0; i < 6; i++) line[i] = rant_double3(0.5 * i, 1.0 + i, 2.0);
    CHECK(rant_points_normals(line, 6, 4, view, nrm) == RANT_MATH_OK, "points on a line");
    for (i = 0, wrong = 0; i < 6; i++) wrong += finite3(nrm[i]);
    CHECK(wrong == 0, "fix no plane, so NaN");
    CHECK(rant_points_normals(line, 2, 3, view, nrm) == RANT_MATH_OK && !finite3(nrm[0]), "two points fix none");
    CHECK(rant_points_normals(line, 6, 2, view, nrm) == RANT_MATH_ERR_ARGUMENT, "a plane needs 3 neighbors");
}

static void clusters(void){
    static RantDouble3 p[REF_CLOUD_POINTS + 40];
    static uint32_t at[REF_CLOUD_POINTS], label[REF_CLOUD_POINTS + 40];
    uint32_t n = with_nans(p, at), i, wrong = 0, count;
    CHECK(rant_points_clusters(ref_cloud(), REF_CLOUD_POINTS, 0.2, 5, label, &count) == RANT_MATH_OK && count == REF_CLUSTER_COUNT,
          "%u clusters, scipy finds %d", count, REF_CLUSTER_COUNT);
    for (i = 0; i < REF_CLOUD_POINTS; i++) wrong += label[i] != REF_CLUSTERS[i];
    CHECK(wrong == 0, "%u labels differ from scipy's connected parts", wrong);
    rant_points_clusters(p, n, 0.2, 5, label, &count);
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += label[at[i]] != REF_CLUSTERS[i];
    for (i = 0; i < n; i++) wrong += !finite3(p[i]) && label[i];
    CHECK(wrong == 0 && count == REF_CLUSTER_COUNT, "NaN points change nothing and get no label");
    rant_points_clusters(ref_cloud(), REF_CLOUD_POINTS, 0.0, 0, label, &count);
    for (i = 0, wrong = 0; i < REF_CLOUD_POINTS; i++) wrong += label[i] == 0 || label[i] > count;
    CHECK(wrong == 0 && count < REF_CLOUD_POINTS && count > REF_CLOUD_POINTS - 10, "a distance of 0 joins only twins: %u", count);
    CHECK(rant_points_clusters(p, n, -1.0, 1, label, &count) == RANT_MATH_ERR_ARGUMENT, "a negative distance");
}

/* The target moved by the inverse of truth, so aligning it back finds truth. */
static void align(void){
    static RantDouble3 src[REF_CLOUD_POINTS], nrm[REF_CLOUD_POINTS];
    static uint8_t in[REF_CLOUD_POINTS];
    const RantDouble3 *dst = ref_cloud();
    RantPose truth, got, start = rant_pose_identity();
    RantAlignOptions o;
    double error = -1.0;
    uint32_t i, n = REF_CLOUD_POINTS, paired = 0;
    truth.position = rant_double3(0.08, -0.05, 0.03);
    truth.orientation = rant_quaternion_from_axis_angle(rant_double3_normalize(rant_double3(0.3, -0.2, 1.0)), 0.08);
    for (i = 0; i < n; i++) src[i] = rant_pose_apply_inverse(truth, dst[i]);
    memset(&o, 0, sizeof o);
    o.iterations = 300;
    o.error = &error;
    o.inliers = in;
    CHECK(rant_points_align(src, n, dst, n, start, &o, &got) == RANT_MATH_OK, "point to point aligns");
    CHECK(same_pose(got, truth, 1e-9), "and finds the motion: off by %.3g m", rant_double3_distance(got.position, truth.position));
    for (i = 0; i < n; i++) paired += in[i];
    CHECK(error < 1e-9 && paired == n, "every point paired, RMS %.3g", error);
    CHECK(rant_points_normals(dst, n, 10, rant_double3(0, 0, 6), nrm) == RANT_MATH_OK, "target normals");
    o.normals = nrm;
    o.iterations = 25;
    CHECK(rant_points_align(src, n, dst, n, start, &o, &got) == RANT_MATH_OK && same_pose(got, truth, 1e-9),
          "point to plane finds it in 25 rounds: off by %.3g m", rant_double3_distance(got.position, truth.position));
    /* with noise and points the target never saw, which max_distance leaves out */
    for (i = 0; i < n; i++){
        src[i] = rant_double3_add(src[i], random_double3(0.002));
        if (i % 9 == 0) src[i] = rant_double3_add(src[i], rant_double3(0, 0, 3.0 + random_unit()));
    }
    o.max_distance = 0.1;
    o.iterations = 100;
    CHECK(rant_points_align(src, n, dst, n, start, &o, &got) == RANT_MATH_OK, "noisy points align");
    for (i = 0, paired = 0; i < n; i++) paired += in[i] && i % 9 != 0;
    CHECK(rant_double3_distance(got.position, truth.position) < 2e-3 && rant_quaternion_angle(got.orientation, truth.orientation) < 2e-3,
          "near the motion: off by %.3g m", rant_double3_distance(got.position, truth.position));
    CHECK(error < 0.005 && paired > n * 8 / 10, "RMS %.3g, the far points left out", error);
    for (i = 0, paired = 0; i < n; i += 9) paired += in[i];
    CHECK(paired == 0, "none of the lifted points paired");
    CHECK(rant_points_align(src, 2, dst, n, start, NULL, &got) == RANT_MATH_ERR_DEGENERATE, "two points fix no pose");
    o.max_distance = -1.0;
    CHECK(rant_points_align(src, n, dst, n, start, &o, &got) == RANT_MATH_ERR_ARGUMENT, "a negative max distance");
}

/* A floor alone holds height and tilt, and leaves the slide along it where the guess put it. */
static void align_floor(void){
    static RantDouble3 floor_pts[400], src[400], nrm[400];
    RantPose truth, got, guess = rant_pose_identity();
    RantAlignOptions o;
    uint32_t i;
    for (i = 0; i < 400; i++){
        floor_pts[i] = rant_double3(random_range(-3, 3), random_range(-3, 3), 0.0);
        nrm[i] = rant_double3(0, 0, 1);
    }
    truth.position = rant_double3(0.0, 0.0, 0.2);
    truth.orientation = rant_quaternion_from_axis_angle(rant_double3(1, 0, 0), 0.03);
    for (i = 0; i < 400; i++) src[i] = rant_pose_apply_inverse(truth, rant_double3(random_range(-2, 2), random_range(-2, 2), 0.0));
    memset(&o, 0, sizeof o);
    o.normals = nrm;
    CHECK(rant_points_align(src, 400, floor_pts, 400, guess, &o, &got) == RANT_MATH_OK, "a floor aligns");
    CHECK(near(got.position.z, 0.2, 1e-9) && rant_quaternion_angle(got.orientation, truth.orientation) < 1e-9,
          "height %.12f and tilt found", got.position.z);
    /* a turn about x moves nothing along x, and the slide along it is the floor's to leave alone */
    CHECK(near(got.position.x, 0.0, 1e-12), "and x stays where the guess put it: %.3g", got.position.x);
    for (i = 0; i < 400; i++) if (!near(rant_pose_apply(got, src[i]).z, 0.0, 1e-9)) break;
    CHECK(i == 400, "every point lands on the floor");
}

void test_neighbors(void){
    tree_matches();
    tree_edges();
    tree_speed();
    filters();
    normals();
    clusters();
    align();
    align_floor();
}
