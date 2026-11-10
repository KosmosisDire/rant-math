#include <stdlib.h>
#include "check.h"

static const char *LABELS[3] = { "car", "person", "" };

static RantOrientedBox2D box2(double x, double y, double angle, double w, double h){
    RantOrientedBox2D b;
    b.pose.position = rant_double2(x, y);
    b.pose.angle = angle;
    b.size = rant_double2(w, h);
    return b;
}

static int same_label(const char *a, size_t na, const char *b, size_t nb){ return na == nb && (!na || !memcmp(a, b, na)); }

/* Greedy suppression by its definition: in order of score, a detection stays unless a kept one
 * before it, of its label, overlaps it by more than the limit. */
static int greedy_holds(const RantDetection2DView *d, const RantDetection3DView *e, uint32_t n, double iou, const uint8_t *keep){
    uint32_t order[64], i, j, k;
    for (i = 0; i < n; i++) order[i] = i;
    for (i = 1; i < n; i++)
        for (j = i; j > 0; j--){
            double s1 = d ? d[order[j - 1]].score : e[order[j - 1]].score, s2 = d ? d[order[j]].score : e[order[j]].score;
            int later = (s1 != s1 && s2 == s2) || (s1 == s1 && s2 == s2 && s2 > s1);
            if (!later) break;
            k = order[j]; order[j] = order[j - 1]; order[j - 1] = k;
        }
    for (i = 0; i < n; i++){
        uint32_t a = order[i];
        int beaten = 0;
        for (j = 0; j < i && !beaten; j++){
            uint32_t b = order[j];
            if (!keep[b]) continue;
            if (d) beaten = same_label(d[a].label, d[a].label_len, d[b].label, d[b].label_len)
                            && rant_oriented_box2d_iou(d[a].bounds, d[b].bounds) > iou;
            else beaten = same_label(e[a].label, e[a].label_len, e[b].label, e[b].label_len)
                          && rant_oriented_box_iou(e[a].bounds, e[b].bounds) > iou;
        }
        if (keep[a] != !beaten) return 0;
    }
    return 1;
}

static void suppress2d(void){
    RantDetection2DView d[64];
    uint8_t keep[64];
    int round, bad = 0, gone = 0;
    uint32_t i;
    for (round = 0; round < 40; round++){
        uint32_t n = 5 + (uint32_t)(random_unit() * 50);
        double limit = random_range(0.1, 0.8);
        for (i = 0; i < n; i++){
            const char *l = LABELS[(uint32_t)(random_unit() * 3)];
            d[i].bounds = box2(random_range(0, 3), random_range(0, 3), random_range(-PI, PI), random_range(0.5, 2.5), random_range(0.5, 2.5));
            d[i].label = l;
            d[i].label_len = strlen(l);
            /* ties in score, so earlier first among equals is tried */
            d[i].score = floor(random_range(0, 8)) / 8.0;
        }
        if (round == 0) d[3].score = NAN;
        if (rant_detections2d_suppress(d, n, limit, keep) != RANT_MATH_OK || !greedy_holds(d, NULL, n, limit, keep)) bad++;
        for (i = 0; i < n; i++) gone += !keep[i];
    }
    CHECK(bad == 0, "%d rounds break greedy suppression", bad);
    CHECK(gone > 100, "and it suppresses: %d", gone);
    d[0].bounds = box2(0, 0, 0, 2, 2); d[0].score = 0.9; d[0].label = "car"; d[0].label_len = 3;
    d[1].bounds = box2(0.2, 0, 0, 2, 2); d[1].score = 0.8; d[1].label = "car"; d[1].label_len = 3;
    d[2].bounds = box2(0.4, 0, 0, 2, 2); d[2].score = 0.7; d[2].label = "cart"; d[2].label_len = 3;
    d[3].bounds = box2(0.2, 0, 0, 2, 2); d[3].score = 0.95; d[3].label = "dog"; d[3].label_len = 3;
    CHECK(rant_detections2d_suppress(d, 4, 0.5, keep) == RANT_MATH_OK && keep[0] && !keep[1] && !keep[2] && keep[3],
          "a car hides a car behind it, labels by their bytes, a dog stays");
    CHECK(rant_detections2d_suppress(d, 4, 1.0, keep) == RANT_MATH_OK && keep[0] && keep[1] && keep[2], "an IoU never passes 1");
    CHECK(rant_detections2d_suppress(d, 4, NAN, keep) == RANT_MATH_ERR_ARGUMENT, "a NaN limit");
    CHECK(rant_detections2d_suppress(NULL, 0, 0.5, NULL) == RANT_MATH_OK, "no detections");
    CHECK(rant_detections2d_suppress(NULL, 2, 0.5, keep) == RANT_MATH_ERR_ARGUMENT, "two detections need a list");
}

static void suppress3d(void){
    RantDetection3DView d[40];
    uint8_t keep[40];
    int round, bad = 0, gone = 0;
    uint32_t i;
    for (round = 0; round < 15; round++){
        uint32_t n = 5 + (uint32_t)(random_unit() * 30);
        for (i = 0; i < n; i++){
            const char *l = LABELS[(uint32_t)(random_unit() * 2)];
            d[i].bounds.pose.position = random_double3(0.7);
            d[i].bounds.pose.orientation = round & 1 ? random_quaternion() : rant_quaternion_from_axis_angle(rant_double3(0, 0, 1), random_range(-PI, PI));
            d[i].bounds.size = rant_double3(random_range(0.5, 2), random_range(0.5, 2), random_range(0.5, 2));
            d[i].label = l;
            d[i].label_len = strlen(l);
            d[i].score = random_unit();
        }
        if (rant_detections3d_suppress(d, n, 0.3, keep) != RANT_MATH_OK || !greedy_holds(NULL, d, n, 0.3, keep)) bad++;
        for (i = 0; i < n; i++) gone += !keep[i];
    }
    CHECK(bad == 0, "%d rounds break greedy suppression in 3D", bad);
    CHECK(gone > 20, "and it suppresses: %d", gone);
}

void test_detection(void){
    suppress2d();
    suppress3d();
}
