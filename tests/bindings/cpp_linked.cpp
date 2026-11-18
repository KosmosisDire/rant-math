/* rant_math.hpp over the library built as C, as a program that links the rant_math target does. */
#include "rant_math.hpp"

#include <cstdio>

int main() {
    using namespace rant::math;
    Pose pose{ { 1, 2, 3 }, quaternion_from_axis_angle({ 0, 0, 1 }, 0.5) };
    Double3 back = pose_inverse(pose) * Double3{ 1, 2, 3 };
    std::vector<Double3> points = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    Plane plane = points_fit_plane(points);
    bool threw = false;
    try {
        matrix3x3_inverse(Matrix3x3{});
    } catch (const MathError &e) {
        threw = e.kind() == ErrorKind::Degenerate;
    }
    if (back.x * back.x + back.y * back.y + back.z * back.z > 1e-20 || plane.normal.z * plane.normal.z < 0.99 || !threw) {
        std::printf("the answers are wrong\n");
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
