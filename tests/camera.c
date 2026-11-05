#include "check.h"
#include "reference.h"

static RantCameraIntrinsics cam_of(int i){
    const double *c = REF_CAMS + 15 * i;
    RantCameraIntrinsics o;
    int k;
    o.width = (uint32_t)c[0];
    o.height = (uint32_t)c[1];
    o.fx = c[2]; o.fy = c[3]; o.cx = c[4]; o.cy = c[5];
    o.model = (RantDistortionModel)(int)c[6];
    for (k = 0; k < 8; k++) o.coeffs[k] = c[7 + k];
    return o;
}

static RantCameraIntrinsics lens(RantDistortionModel model, double k1, double k2, double k3, double k4){
    RantCameraIntrinsics c;
    memset(&c, 0, sizeof c);
    c.width = 800; c.height = 600;
    c.fx = 400.0; c.fy = 400.0; c.cx = 399.5; c.cy = 299.5;
    c.model = model;
    c.coeffs[0] = k1; c.coeffs[1] = k2;
    if (model == RANT_DISTORTION_MODEL_FISHEYE){ c.coeffs[2] = k3; c.coeffs[3] = k4; } else c.coeffs[4] = k3;
    (void)k4;
    return c;
}

/* OpenCV's optical frame, x right, y down and z ahead, as the camera's own x ahead, y left, z up. */
static RantDouble3 body(double x, double y, double z){ return rant_double3(z, -x, -y); }

/* Uniform over the sphere. */
static RantDouble3 random_direction(void){
    double z = random_range(-1.0, 1.0), t = random_range(0.0, 2.0 * PI), r = sqrt(1.0 - z * z);
    return rant_double3(r * cos(t), r * sin(t), z);
}

static double angle_between(RantDouble3 a, RantDouble3 b){
    return atan2(rant_double3_length(rant_double3_cross(a, b)), rant_double3_dot(a, b));
}

static void opencv(void){
    int i, k, bad = 0;
    RantDouble2 px, many[REF_CAM_CASES];
    RantDouble3 pts[REF_CAM_CASES];
    for (i = 0; i < REF_CAM_COUNT; i++){
        RantCameraIntrinsics c = cam_of(i);
        for (k = 0; k < REF_CAM_CASES; k++){
            const double *p = REF_CAM_POINTS + 3 * (i * REF_CAM_CASES + k), *want = REF_CAM_PIXELS + 2 * (i * REF_CAM_CASES + k);
            pts[k] = body(p[0], p[1], p[2]);
            if (rant_camera_project(&c, pts[k], &px) != RANT_MATH_OK || !near2(px, rant_double2(want[0], want[1]), 1e-8)) bad++;
        }
        CHECK(rant_camera_project_points(&c, pts, many, REF_CAM_CASES) == RANT_MATH_OK, "camera %d: many points", i);
        for (k = 0; k < REF_CAM_CASES; k++){
            const double *want = REF_CAM_PIXELS + 2 * (i * REF_CAM_CASES + k);
            if (!near2(many[k], rant_double2(want[0], want[1]), 1e-8)) bad++;
        }
    }
    CHECK(bad == 0, "%d projections differ from OpenCV's", bad);
}

static void frame(void){
    int i;
    RantDouble2 ahead, left, up;
    for (i = 0; i < REF_CAM_COUNT; i++){
        RantCameraIntrinsics c = cam_of(i);
        CHECK(rant_camera_project(&c, rant_double3(2.0, 0.0, 0.0), &ahead) == RANT_MATH_OK
              && ahead.x == c.cx && ahead.y == c.cy, "camera %d: straight ahead is the principal point", i);
        /* tangential distortion moves a point off the row or column it lies in */
        CHECK(rant_camera_project(&c, rant_double3(2.0, 0.3, 0.0), &left) == RANT_MATH_OK && left.x < c.cx - 0.1 * c.fx
              && near(left.y, c.cy, (c.coeffs[2] != 0.0 ? 1.0 : 1e-9) * c.fy), "camera %d: +y is left in the image", i);
        CHECK(rant_camera_project(&c, rant_double3(2.0, 0.0, 0.3), &up) == RANT_MATH_OK && up.y < c.cy - 0.1 * c.fy
              && near(up.x, c.cx, (c.coeffs[3] != 0.0 ? 1.0 : 1e-9) * c.fx), "camera %d: +z is up in the image", i);
    }
}

/* Every pixel a ray reaches maps back to itself, and a depth lands depth ahead. */
static void round_trip(void){
    int i, gx, gy, bad = 0, far = 0;
    for (i = 0; i < REF_CAM_COUNT; i++){
        RantCameraIntrinsics c = cam_of(i);
        for (gy = 0; gy <= 16; gy++)
            for (gx = 0; gx <= 16; gx++){
                RantDouble2 px = rant_double2(-0.5 + c.width * gx / 16.0, -0.5 + c.height * gy / 16.0), back;
                RantRay ray;
                RantDouble3 p;
                if (rant_camera_ray(&c, px, &ray) != RANT_MATH_OK){ bad++; continue; }
                if (!near(rant_double3_length(ray.direction), 1.0, 1e-15) || !near3(ray.origin, rant_double3(0, 0, 0), 0.0)) bad++;
                if (rant_camera_project(&c, rant_double3_scale(ray.direction, 3.0), &back) != RANT_MATH_OK
                    || !near2(back, px, 1e-8)) bad++;
                if (ray.direction.x <= 0.0){
                    if (c.model != RANT_DISTORTION_MODEL_FISHEYE) far++;
                    continue;
                }
                if (rant_camera_unproject(&c, px, 2.5, &p) != RANT_MATH_OK || p.x != 2.5
                    || rant_camera_project(&c, p, &back) != RANT_MATH_OK || !near2(back, px, 1e-8)
                    || angle_between(p, ray.direction) > 1e-14) bad++;
            }
    }
    CHECK(bad == 0, "%d pixels fail to come back through their ray", bad);
    CHECK(far == 0, "%d pinhole pixels see behind", far);
}

/* A point the lens accepts lands at a pixel whose ray points back at it, so none lands as a ghost
 * among the points the camera really sees. */
static void no_ghosts(const RantCameraIntrinsics *c, int whole_sphere, int *taken, int *refused){
    int k, bad = 0;
    *taken = *refused = 0;
    for (k = 0; k < 4000; k++){
        RantDouble3 d = random_direction();
        RantDouble2 px;
        RantRay ray;
        if (!whole_sphere) d.x = fabs(d.x);
        if (rant_camera_project(c, rant_double3_scale(d, 4.0), &px) != RANT_MATH_OK){ (*refused)++; continue; }
        (*taken)++;
        if (rant_camera_ray(c, px, &ray) != RANT_MATH_OK || angle_between(ray.direction, d) > 1e-9) bad++;
    }
    CHECK(bad == 0, "%d points land as ghosts", bad);
}

/* Just inside the edge of what a pinhole lens accepts, where the map turns least, each point still
 * comes back through its ray. */
static void edge(const RantCameraIntrinsics *c){
    int k, i, bad = 0;
    for (k = 0; k < 300; k++){
        double t = random_range(0.0, 2.0 * PI), lo = 0.0, hi = 50.0;
        RantDouble2 px;
        RantRay ray;
        for (i = 0; i < 60; i++){
            double mid = 0.5 * (lo + hi);
            if (rant_camera_project(c, body(mid * cos(t), mid * sin(t), 1.0), &px) == RANT_MATH_OK) lo = mid; else hi = mid;
        }
        for (i = 0; i < 5; i++){
            double r = lo * (i ? random_range(0.9, 1.0) : 1.0 - 1e-6);
            RantDouble3 p = body(r * cos(t), r * sin(t), 1.0);
            if (rant_camera_project(c, p, &px) != RANT_MATH_OK || rant_camera_ray(c, px, &ray) != RANT_MATH_OK
                || angle_between(ray.direction, rant_double3_normalize(p)) > 1e-8) bad++;
        }
    }
    CHECK(bad == 0, "model %d: %d points by the edge fail to come back", (int)c->model, bad);
}

static void folds(void){
    RantCameraIntrinsics c = lens(RANT_DISTORTION_MODEL_BROWN_CONRADY, -0.5, 0.0, 0.0, 0.0);
    double fold = sqrt(2.0 / 3.0), reach = fold * (1.0 - 0.5 * fold * fold);
    RantDouble2 px;
    RantRay ray;
    RantDouble3 p;
    int taken, refused, i;
    /* r (1 - 0.5 r^2) grows until r^2 = 2/3, then turns back toward the center */
    CHECK(rant_camera_project(&c, body(fold - 1e-6, 0.0, 1.0), &px) == RANT_MATH_OK, "just inside the fold");
    CHECK(rant_camera_project(&c, body(fold + 1e-6, 0.0, 1.0), &px) == RANT_MATH_ERR_OUTSIDE, "just past the fold");
    CHECK(rant_camera_project(&c, body(0.0, 1.2, 1.0), &px) == RANT_MATH_ERR_OUTSIDE, "far past the fold, where it would land inside");
    CHECK(rant_camera_ray(&c, rant_double2(c.cx + c.fx * (reach - 1e-6), c.cy), &ray) == RANT_MATH_OK
          && near(angle_between(ray.direction, rant_double3(1, 0, 0)), atan(fold), 2e-3), "the farthest pixel it reaches");
    CHECK(rant_camera_ray(&c, rant_double2(c.cx, c.cy - c.fy * (reach + 1e-6)), &ray) == RANT_MATH_ERR_OUTSIDE, "a pixel past its reach");
    no_ghosts(&c, 0, &taken, &refused);
    /* the fold's cone of 39 degrees holds a fifth of the half sphere */
    CHECK(near(taken / 4000.0, 1.0 - cos(atan(fold)), 0.03), "the fold splits the view: %d taken, %d refused", taken, refused);
    edge(&c);
    for (i = 0; i < REF_CAM_COUNT; i++){
        RantCameraIntrinsics r = cam_of(i);
        no_ghosts(&r, r.model == RANT_DISTORTION_MODEL_FISHEYE, &taken, &refused);
        CHECK(taken > 100, "camera %d sees %d of the points", i, taken);
        if (r.model != RANT_DISTORTION_MODEL_FISHEYE) edge(&r);
    }

    /* r / (1 - r^2) runs to infinity at r = 1, so every pixel has a ray and nothing past 1 lands */
    c = lens(RANT_DISTORTION_MODEL_RATIONAL, 0.0, 0.0, 0.0, 0.0);
    c.coeffs[5] = -1.0;
    CHECK(rant_camera_project(&c, body(0.99, 0.0, 1.0), &px) == RANT_MATH_OK && px.x > c.cx + 49.0 * c.fx, "near the pole lands far out");
    CHECK(rant_camera_project(&c, body(1.01, 0.0, 1.0), &px) == RANT_MATH_ERR_OUTSIDE, "past the pole");
    CHECK(rant_camera_ray(&c, rant_double2(c.cx + 1e6, c.cy), &ray) == RANT_MATH_OK && ray.direction.x > 0.0, "a far pixel still has its ray");
    no_ghosts(&c, 0, &taken, &refused);

    /* an equidistant fisheye sees past its side, though no depth lies ahead there */
    c = lens(RANT_DISTORTION_MODEL_FISHEYE, 0.0, 0.0, 0.0, 0.0);
    p = rant_double3(cos(100.0 * PI / 180.0), sin(100.0 * PI / 180.0), 0.0);
    CHECK(rant_camera_project(&c, p, &px) == RANT_MATH_OK && near(px.x, c.cx - c.fx * 100.0 * PI / 180.0, 1e-9), "100 degrees off its axis");
    CHECK(rant_camera_ray(&c, px, &ray) == RANT_MATH_OK && angle_between(ray.direction, p) < 1e-12, "and its ray goes back there");
    CHECK(rant_camera_unproject(&c, px, 1.0, &p) == RANT_MATH_ERR_OUTSIDE, "no depth ahead of a ray that points back");
    CHECK(rant_camera_project(&c, rant_double3(-1.0, 0.0, 0.0), &px) == RANT_MATH_ERR_OUTSIDE, "straight behind");
    CHECK(rant_camera_project(&c, rant_double3(0.0, 0.0, 0.0), &px) == RANT_MATH_ERR_OUTSIDE, "the camera's own center");
    CHECK(rant_camera_ray(&c, rant_double2(c.cx + c.fx * 3.2, c.cy), &ray) == RANT_MATH_ERR_OUTSIDE, "past the turn behind it");
    no_ghosts(&c, 1, &taken, &refused);
    CHECK(taken > 3900, "it sees nearly everything: %d", taken);
    /* 1 - 0.6 theta^2 reaches 0 at 74 degrees */
    c = lens(RANT_DISTORTION_MODEL_FISHEYE, -0.2, 0.0, 0.0, 0.0);
    CHECK(rant_camera_project(&c, rant_double3(cos(1.25), sin(1.25), 0.0), &px) == RANT_MATH_OK, "inside a fisheye's fold");
    CHECK(rant_camera_project(&c, rant_double3(cos(1.33), sin(1.33), 0.0), &px) == RANT_MATH_ERR_OUTSIDE, "past a fisheye's fold");
    no_ghosts(&c, 1, &taken, &refused);
    CHECK(refused > 1000, "the fold refuses the rest: %d", refused);
    c = lens(RANT_DISTORTION_MODEL_NONE, 0.0, 0.0, 0.0, 0.0);
    CHECK(rant_camera_project(&c, rant_double3(0.0, 1.0, 0.0), &px) == RANT_MATH_ERR_OUTSIDE, "a pinhole sees nothing at its side");
    CHECK(rant_camera_project(&c, rant_double3(-1.0, 0.0, 0.0), &px) == RANT_MATH_ERR_OUTSIDE, "or behind");
}

static void many(void){
    RantCameraIntrinsics c = cam_of(1);
    RantDouble3 pts[4];
    RantDouble2 px[4], one;
    pts[0] = rant_double3(2.0, 0.1, -0.2);
    pts[1] = rant_double3(-2.0, 0.1, -0.2);
    pts[2] = rant_double3(NAN, 0.0, 0.0);
    pts[3] = rant_double3(1.0, 0.0, 0.0);
    CHECK(rant_camera_project_points(&c, pts, px, 4) == RANT_MATH_OK, "many points");
    CHECK(rant_camera_project(&c, pts[0], &one) == RANT_MATH_OK && px[0].x == one.x && px[0].y == one.y, "the same as one at a time");
    CHECK(px[1].x != px[1].x && px[1].y != px[1].y && px[2].x != px[2].x, "NaN where it cannot project");
    CHECK(px[3].x == c.cx && px[3].y == c.cy, "and on after them");
    CHECK(rant_camera_project_points(&c, NULL, px, 1) == RANT_MATH_ERR_ARGUMENT, "no points");
    CHECK(rant_camera_project_points(&c, NULL, NULL, 0) == RANT_MATH_OK, "none to do");
}

static void scale_crop(void){
    RantCameraIntrinsics c = cam_of(3), s, k;
    int i, bad = 0;
    CHECK(rant_camera_scale(&c, 640, 360, &s) == RANT_MATH_OK && s.width == 640 && s.height == 360, "a third the size");
    CHECK(rant_camera_crop(&c, 100, 50, 800, 600, &k) == RANT_MATH_OK && k.width == 800 && k.height == 600, "a crop");
    for (i = 0; i < 200; i++){
        RantDouble3 p = rant_double3(random_range(1.0, 5.0), random_range(-1.5, 1.5), random_range(-1.0, 1.0));
        RantDouble2 a, b, e;
        if (rant_camera_project(&c, p, &a) != RANT_MATH_OK) continue;
        /* the image's edge at -0.5 scales, so a pixel center moves by half a pixel less */
        if (rant_camera_project(&s, p, &b) != RANT_MATH_OK
            || !near2(b, rant_double2((a.x + 0.5) / 3.0 - 0.5, (a.y + 0.5) / 3.0 - 0.5), 1e-9)) bad++;
        if (rant_camera_project(&k, p, &e) != RANT_MATH_OK || !near2(e, rant_double2(a.x - 100.0, a.y - 50.0), 1e-9)) bad++;
    }
    CHECK(bad == 0, "%d points land elsewhere after a scale or crop", bad);
    CHECK(rant_camera_scale(&c, 0, 360, &s) == RANT_MATH_ERR_ARGUMENT, "no width");
    CHECK(rant_camera_crop(&c, 1500, 0, 421, 10, &k) == RANT_MATH_ERR_ARGUMENT, "a crop past the right edge");
    CHECK(rant_camera_crop(&c, 1500, 0, 420, 10, &k) == RANT_MATH_OK, "a crop to the right edge");
    CHECK(rant_camera_crop(&c, 0, 0xFFFFFFFFu, 1, 2, &k) == RANT_MATH_ERR_ARGUMENT, "a crop whose end overflows");
    c.width = 0;
    CHECK(rant_camera_scale(&c, 640, 360, &s) == RANT_MATH_ERR_ARGUMENT, "a camera of unstated size cannot scale");
    CHECK(rant_camera_crop(&c, 0, 0, 1, 1, &k) == RANT_MATH_ERR_ARGUMENT, "or crop");
}

static void fov(void){
    RantCameraIntrinsics c, back;
    RantDouble2 f;
    CHECK(rant_camera_from_fov(640, 480, rant_double2(PI / 2.0, 0.0), &c) == RANT_MATH_OK, "from a fov");
    CHECK(near(c.fx, 320.0, 1e-12) && near(c.fy, 320.0, 1e-12) && c.cx == 319.5 && c.cy == 239.5
          && c.model == RANT_DISTORTION_MODEL_NONE && c.width == 640 && c.coeffs[0] == 0.0, "square pixels, centered");
    CHECK(rant_camera_fov(&c, &f) == RANT_MATH_OK && near(f.x, PI / 2.0, 1e-14) && near(f.y, 2.0 * atan(240.0 / 320.0), 1e-14), "its fov back");
    CHECK(rant_camera_from_fov(1280, 720, rant_double2(1.2, 0.8), &back) == RANT_MATH_OK
          && rant_camera_fov(&back, &f) == RANT_MATH_OK && near(f.x, 1.2, 1e-14) && near(f.y, 0.8, 1e-14), "both angles");
    /* an equidistant fisheye whose edges reach 100 degrees to each side */
    c = lens(RANT_DISTORTION_MODEL_FISHEYE, 0.0, 0.0, 0.0, 0.0);
    c.fx = c.fy = 400.0 / (100.0 * PI / 180.0);
    CHECK(rant_camera_fov(&c, &f) == RANT_MATH_OK && near(f.x, 200.0 * PI / 180.0, 1e-12), "a fisheye spans more than half a turn");
    c = cam_of(1);
    CHECK(rant_camera_fov(&c, &f) == RANT_MATH_OK && f.x > 2.0 * atan(640.0 / 900.0) && f.y > 2.0 * atan(360.0 / 905.0),
          "barrel distortion widens the view");
    CHECK(rant_camera_from_fov(640, 480, rant_double2(PI, 0.0), &c) == RANT_MATH_ERR_ARGUMENT, "a pinhole spans less than half a turn");
    CHECK(rant_camera_from_fov(640, 0, rant_double2(1.0, 0.0), &c) == RANT_MATH_ERR_ARGUMENT, "no height");
    CHECK(rant_camera_from_fov(640, 480, rant_double2(1.0, -0.1), &c) == RANT_MATH_ERR_ARGUMENT, "a negative angle");
}

static void failures(void){
    RantCameraIntrinsics c = cam_of(1), bad;
    RantDouble2 px;
    RantRay ray;
    RantDouble3 p;
    CHECK(rant_camera_project(NULL, rant_double3(1, 0, 0), &px) == RANT_MATH_ERR_ARGUMENT, "no camera");
    CHECK(rant_camera_project(&c, rant_double3(1, 0, 0), NULL) == RANT_MATH_ERR_ARGUMENT, "no output");
    CHECK(rant_camera_project(&c, rant_double3(1, NAN, 0), &px) == RANT_MATH_ERR_ARGUMENT, "a NaN point");
    CHECK(rant_camera_ray(&c, rant_double2(HUGE_VAL, 0.0), &ray) == RANT_MATH_ERR_ARGUMENT, "an endless pixel");
    bad = c; bad.fx = 0.0;
    CHECK(rant_camera_project(&bad, rant_double3(1, 0, 0), &px) == RANT_MATH_ERR_ARGUMENT, "fx of 0");
    bad = c; bad.fy = -900.0;
    CHECK(rant_camera_ray(&bad, rant_double2(1, 1), &ray) == RANT_MATH_ERR_ARGUMENT, "a negative fy");
    bad = c; bad.cx = NAN;
    CHECK(rant_camera_ray(&bad, rant_double2(1, 1), &ray) == RANT_MATH_ERR_ARGUMENT, "a NaN cx");
    bad = c; bad.model = (RantDistortionModel)9;
    CHECK(rant_camera_project(&bad, rant_double3(1, 0, 0), &px) == RANT_MATH_ERR_ARGUMENT, "an unknown model");
    bad = c; bad.coeffs[4] = HUGE_VAL;
    CHECK(rant_camera_project(&bad, rant_double3(1, 0, 0), &px) == RANT_MATH_ERR_ARGUMENT, "an endless k3");
    bad = c; bad.coeffs[6] = NAN;
    CHECK(rant_camera_project(&bad, rant_double3(1, 0, 0), &px) == RANT_MATH_OK, "past the model's count is never read");
    CHECK(rant_camera_unproject(&c, rant_double2(5, 5), 0.0, &p) == RANT_MATH_ERR_ARGUMENT, "a depth of 0, as a depth image says nothing");
    CHECK(rant_camera_unproject(&c, rant_double2(5, 5), -1.0, &p) == RANT_MATH_ERR_ARGUMENT, "a negative depth");
    CHECK(rant_camera_unproject(&c, rant_double2(5, 5), NAN, &p) == RANT_MATH_ERR_ARGUMENT, "a NaN depth");
    px = rant_double2(7.0, 7.0);
    CHECK(rant_camera_project(&c, rant_double3(-1, 0, 0), &px) == RANT_MATH_ERR_OUTSIDE && px.x == 7.0, "a failure leaves the output alone");
}

void test_camera(void){
    opencv();
    frame();
    round_trip();
    folds();
    many();
    scale_crop();
    fov();
    failures();
}
