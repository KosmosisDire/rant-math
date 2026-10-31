#include <stdarg.h>
#include <stdlib.h>
#include "check.h"

int check_failures = 0;
int check_count = 0;

void check_report(const char *file, int line, const char *fmt, ...){
    va_list ap;
    fprintf(stderr, "FAIL %s:%d: ", file, line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

/* x - x is 0 only for a finite x, which avoids the classify macros some C libraries warn in. */
int near(double a, double b, double tol){
    if (a != a || b != b) return 0;
    if (a - a != 0.0 || b - b != 0.0) return a == b;
    return fabs(a - b) <= tol;
}
int near2(RantDouble2 a, RantDouble2 b, double tol){ return near(a.x, b.x, tol) && near(a.y, b.y, tol); }
int near3(RantDouble3 a, RantDouble3 b, double tol){
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}
int same_turn(RantQuaternion a, RantQuaternion b, double tol){
    double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    double na = sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
    double nb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
    return na > 0.0 && nb > 0.0 && 1.0 - d / (na * nb) <= tol;
}
int same_pose(RantPose a, RantPose b, double tol){
    return near3(a.position, b.position, tol) && same_turn(a.orientation, b.orientation, tol);
}

static uint64_t state = 0x9E3779B97F4A7C15ull;
void random_seed(uint64_t seed){ state = seed ? seed : 0x9E3779B97F4A7C15ull; }
static uint64_t next(void){
    state ^= state << 13; state ^= state >> 7; state ^= state << 17;
    return state;
}
double random_unit(void){ return (double)(next() >> 11) / 9007199254740992.0; }
double random_range(double lo, double hi){ return lo + (hi - lo) * random_unit(); }
RantDouble2 random_double2(double r){ return rant_double2(random_range(-r, r), random_range(-r, r)); }
RantDouble3 random_double3(double r){
    return rant_double3(random_range(-r, r), random_range(-r, r), random_range(-r, r));
}
/* Shoemake's uniform quaternion from three uniforms. */
RantQuaternion random_quaternion(void){
    double u1 = random_unit(), u2 = random_unit() * 2.0 * PI, u3 = random_unit() * 2.0 * PI;
    double a = sqrt(1.0 - u1), b = sqrt(u1);
    return rant_quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3));
}
RantPose random_pose(double r){
    RantPose p;
    p.position = random_double3(r);
    p.orientation = random_quaternion();
    return p;
}

uint32_t random_polygon(RantDouble2 *out, uint32_t most){
    uint32_t n = 3 + (uint32_t)(random_unit() * (most - 2)), i;
    double angles[256] = { 0.0 }, gap;
    if (n > most) n = most;
    do {
        for (i = 0; i < n; i++){
            double a = random_range(0.0, 2.0 * PI);
            uint32_t j = i;
            while (j > 0 && angles[j - 1] > a){ angles[j] = angles[j - 1]; j--; }
            angles[j] = a;
        }
        gap = angles[0] + 2.0 * PI - angles[n - 1];
        for (i = 1; i < n; i++) if (angles[i] - angles[i - 1] > gap) gap = angles[i] - angles[i - 1];
    } while (gap >= 0.95 * PI);
    for (i = 0; i < n; i++){
        double r = random_range(0.4, 2.0);
        out[i] = rant_double2(r * cos(angles[i]), r * sin(angles[i]));
    }
    if (random_unit() < 0.5)
        for (i = 0; i < n / 2; i++){ RantDouble2 t = out[i]; out[i] = out[n - 1 - i]; out[n - 1 - i] = t; }
    return n;
}

typedef struct { const char *name; void (*run)(void); } Part;
static const Part parts[] = {
    { "vector", test_vector }, { "rotation", test_rotation }, { "matrix", test_matrix },
    { "pose", test_pose }, { "motion", test_motion }, { "axes", test_axes },
    { "line", test_line },
    { "region", test_region },
    { "solid", test_solid },
    { "curve", test_curve },
    { "surface", test_surface },
    { "points", test_points },
    { "shape", test_shape },
    { "fit", test_fit },
    { "color", test_color },
    { "camera", test_camera },
    { "image", test_image },
    { "cloud", test_cloud },
    { "neighbors", test_neighbors },
    { "grid", test_grid },
    { "geo", test_geo },
    { "detection", test_detection }
};

int main(int argc, char **argv){
    size_t i;
    int ran = 0;
    for (i = 0; i < sizeof parts / sizeof parts[0]; i++){
        if (argc > 1 && strcmp(argv[1], parts[i].name) != 0) continue;
        random_seed(0x5EED0000u + i);
        parts[i].run();
        ran++;
    }
    if (!ran){ fprintf(stderr, "no part named %s\n", argv[1]); return 2; }
    printf("%d checks, %d failed\n", check_count, check_failures);
    return check_failures ? 1 : 0;
}
