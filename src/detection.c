#include "internal.h"

/* Detections by score, highest first and earlier first among equals, a score that is NaN last. */
typedef struct { double score; uint32_t i; } i_Rank;

static int i_compare_ranks(const void *a, const void *b){
    const i_Rank *x = (const i_Rank *)a, *y = (const i_Rank *)b;
    int nx = x->score != x->score, ny = y->score != y->score;
    if (nx != ny) return nx - ny;
    if (!nx && x->score != y->score) return x->score > y->score ? -1 : 1;
    return x->i < y->i ? -1 : (x->i > y->i);
}

static int i_same(const char *a, size_t na, const char *b, size_t nb){
    return na == nb && (!na || (a && b && !memcmp(a, b, na)));
}

/* Greedy suppression over either kind, flat for the 2D list. */
static RantMathResult i_suppress(int flat, const RantDetection2DView *d2, const RantDetection3DView *d3, uint32_t count,
                                 double iou, uint8_t *keep){
    i_Rank *r;
    uint32_t i, j;
    if ((count && (!keep || (flat ? !d2 : !d3))) || iou != iou) return RANT_MATH_ERR_ARGUMENT;
    r = (i_Rank *)malloc(sizeof(i_Rank) * (count ? count : 1));
    if (!r) return RANT_MATH_ERR_MEMORY;
    for (i = 0; i < count; i++){
        r[i].score = flat ? d2[i].score : d3[i].score;
        r[i].i = i;
        keep[i] = 1;
    }
    qsort(r, count, sizeof *r, i_compare_ranks);
    for (i = 0; i < count; i++){
        uint32_t a = r[i].i;
        if (!keep[a]) continue;
        for (j = i + 1; j < count; j++){
            uint32_t b = r[j].i;
            if (!keep[b]) continue;
            if (flat){
                if (i_same(d2[a].label, d2[a].label_len, d2[b].label, d2[b].label_len)
                    && rant_oriented_box2d_iou(d2[a].bounds, d2[b].bounds) > iou) keep[b] = 0;
            } else if (i_same(d3[a].label, d3[a].label_len, d3[b].label, d3[b].label_len)
                       && rant_oriented_box_iou(d3[a].bounds, d3[b].bounds) > iou) keep[b] = 0;
        }
    }
    free(r);
    return RANT_MATH_OK;
}

RantMathResult rant_detections2d_suppress(const RantDetection2DView *detections, uint32_t count, double iou, uint8_t *keep){
    return i_suppress(1, detections, NULL, count, iou, keep);
}

RantMathResult rant_detections3d_suppress(const RantDetection3DView *detections, uint32_t count, double iou, uint8_t *keep){
    return i_suppress(0, NULL, detections, count, iou, keep);
}
