/* The C++ binding: every case of vectors.json through its call, then what vectors cannot show. */
#include "rant_math.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>

using namespace rant::math;

/* Just enough JSON to read vectors.json and compare answers. */
struct J {
    enum Kind { Null, Bool, Number, Text, Array, Object } kind = Null;
    bool flag = false;
    double number = 0;
    std::string str;
    std::vector<J> items;
    std::vector<std::pair<std::string, J>> members;

    static J object() { J j; j.kind = Object; return j; }
    static J array() { J j; j.kind = Array; return j; }
    bool is_null() const { return kind == Null; }
    const J &at(const char *key) const {
        for (const auto &m : members) if (m.first == key) return m.second;
        static const J none;
        return none;
    }
    bool has(const char *key) const {
        for (const auto &m : members) if (m.first == key) return true;
        return false;
    }
    void add(const char *key, J v) { members.emplace_back(key, std::move(v)); }
};

struct Reader {
    const char *p;
    void space() { while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++; }
    J value() {
        J j;
        space();
        if (*p == '{') {
            j.kind = J::Object;
            p++;
            space();
            while (*p != '}') {
                space();
                std::string key = text();
                space();
                p++;  /* the colon */
                j.members.emplace_back(std::move(key), value());
                space();
                if (*p == ',') p++;
                space();
            }
            p++;
        } else if (*p == '[') {
            j.kind = J::Array;
            p++;
            space();
            while (*p != ']') {
                j.items.push_back(value());
                space();
                if (*p == ',') p++;
                space();
            }
            p++;
        } else if (*p == '"') {
            j.kind = J::Text;
            j.str = text();
        } else if (std::strncmp(p, "true", 4) == 0) {
            j.kind = J::Bool;
            j.flag = true;
            p += 4;
        } else if (std::strncmp(p, "false", 5) == 0) {
            j.kind = J::Bool;
            p += 5;
        } else if (std::strncmp(p, "null", 4) == 0) {
            p += 4;
        } else {
            char *end;
            j.kind = J::Number;
            j.number = std::strtod(p, &end);
            p = end;
        }
        return j;
    }
    std::string text() {
        std::string s;
        p++;
        while (*p != '"') s += *p++;
        p++;
        return s;
    }
};

/* Numbers agree within the case's tolerance of their size, all else exactly. */
static bool same(const J &want, const J &got, double tol) {
    if (want.kind != got.kind) return false;
    switch (want.kind) {
    case J::Null: return true;
    case J::Bool: return want.flag == got.flag;
    case J::Number: {
        double larger = std::fmax(1.0, std::fmax(std::fabs(want.number), std::fabs(got.number)));
        return want.number == got.number || std::fabs(want.number - got.number) <= tol * larger;
    }
    case J::Text: return want.str == got.str;
    case J::Array:
        if (want.items.size() != got.items.size()) return false;
        for (size_t i = 0; i < want.items.size(); i++) if (!same(want.items[i], got.items[i], tol)) return false;
        return true;
    case J::Object:
        if (want.members.size() != got.members.size()) return false;
        for (const auto &m : want.members) {
            if (!got.has(m.first.c_str()) || !same(m.second, got.at(m.first.c_str()), tol)) return false;
        }
        return true;
    }
    return false;
}

static std::string show(const J &j) {
    std::ostringstream s;
    switch (j.kind) {
    case J::Null: s << "null"; break;
    case J::Bool: s << (j.flag ? "true" : "false"); break;
    case J::Number: s.precision(17); s << j.number; break;
    case J::Text: s << '"' << j.str << '"'; break;
    case J::Array:
        s << '[';
        for (size_t i = 0; i < j.items.size(); i++) s << (i ? "," : "") << show(j.items[i]);
        s << ']';
        break;
    case J::Object:
        s << '{';
        for (size_t i = 0; i < j.members.size(); i++) s << (i ? "," : "") << j.members[i].first << ':' << show(j.members[i].second);
        s << '}';
        break;
    }
    return s.str();
}

template <class T> T get(const J &j);
template <> double get<double>(const J &j) {
    if (j.kind == J::Text) return j.str == "nan" ? NAN : (j.str == "inf" ? HUGE_VAL : -HUGE_VAL);
    return j.number;
}
template <> float get<float>(const J &j) { return float(get<double>(j)); }
template <> int get<int>(const J &j) { return int(j.number); }
template <> uint8_t get<uint8_t>(const J &j) { return uint8_t(j.number); }
template <> uint32_t get<uint32_t>(const J &j) { return uint32_t(j.number); }
template <> uint64_t get<uint64_t>(const J &j) { return uint64_t(j.number); }

template <class T, class = std::enable_if_t<std::is_arithmetic_v<T>>> J put(T v) {
    J j;
    if constexpr (std::is_same_v<T, bool>) {
        j.kind = J::Bool;
        j.flag = v;
    } else if constexpr (std::is_floating_point_v<T>) {
        if (v != v) { j.kind = J::Text; j.str = "nan"; }
        else if (std::isinf(v)) { j.kind = J::Text; j.str = v > 0 ? "inf" : "-inf"; }
        else { j.kind = J::Number; j.number = double(v); }
    } else {
        j.kind = J::Number;
        j.number = double(v);
    }
    return j;
}
inline J put(const char *s) { J j; j.kind = J::Text; j.str = s; return j; }
inline J put(const std::string &s) { J j; j.kind = J::Text; j.str = s; return j; }

template <class T, size_t N> void get_array(T (&a)[N], const J &j) {
    for (size_t i = 0; i < N; i++) {
        if constexpr (std::is_array_v<T>) get_array(a[i], j.items[i]);
        else a[i] = T(get<double>(j.items[i]));
    }
}
template <class T, size_t N> J put_array(const T (&a)[N]) {
    J o = J::array();
    for (size_t i = 0; i < N; i++) {
        if constexpr (std::is_array_v<T>) o.items.push_back(put_array(a[i]));
        else o.items.push_back(put(a[i]));
    }
    return o;
}

#define THUNK_DECLS
#include "generated/cpp_thunks.inc"
#undef THUNK_DECLS

static J put(const Matrix &m);
static J put(const Image &i);
static J put(const PointField &f);
static J put(const PointCloud &c);
static J put(const GridField &f);
static J put(const Grid &g);
static J put(const Prism &p);

static std::string hex(const uint8_t *d, size_t n) {
    static const char digits[] = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; i++) { s += digits[d[i] >> 4]; s += digits[d[i] & 15]; }
    return s;
}
static std::vector<uint8_t> unhex(const std::string &s) {
    std::vector<uint8_t> out(s.size() / 2);
    auto nib = [](char c) { return uint8_t(c <= '9' ? c - '0' : c - 'a' + 10); };
    for (size_t i = 0; i < out.size(); i++) out[i] = uint8_t(nib(s[2 * i]) << 4 | nib(s[2 * i + 1]));
    return out;
}

template <class T> J put(const std::optional<T> &v) { return v ? put(*v) : J(); }
template <class T> J put(const std::vector<T> &v) {
    if constexpr (std::is_same_v<T, uint8_t>) {
        return put(hex(v.data(), v.size()));
    } else {
        J o = J::array();
        for (const T &x : v) o.items.push_back(put(x));
        return o;
    }
}
template <class T, size_t N> J put(const std::array<T, N> &v) {
    J o = J::array();
    for (const T &x : v) o.items.push_back(put(x));
    return o;
}
template <class... T> J put(const std::tuple<T...> &v) {
    J o = J::array();
    std::apply([&](const auto &...x) { (o.items.push_back(put(x)), ...); }, v);
    return o;
}
static J put(const Matrix &m) {
    J o = J::object();
    o.add("rows", put(m.rows));
    o.add("cols", put(m.cols));
    o.add("data", put(m.data));
    return o;
}
static J put(const Image &i) {
    J o = J::object();
    o.add("width", put(i.width));
    o.add("height", put(i.height));
    o.add("stride", put(i.stride));
    o.add("format", put(int(i.format)));
    o.add("data", put(i.data));
    return o;
}
static J put(const PointField &f) {
    J o = J::object();
    o.add("name", put(f.name));
    o.add("offset", put(f.offset));
    o.add("kind", put(int(f.kind)));
    o.add("count", put(f.count));
    return o;
}
static J put(const PointCloud &c) {
    J o = J::object();
    o.add("fields", put(c.fields));
    o.add("point_step", put(c.point_step));
    o.add("data", put(c.data));
    return o;
}
static J put(const GridField &f) {
    J o = J::object();
    o.add("name", put(f.name));
    o.add("kind", put(int(f.kind)));
    o.add("data", put(f.data));
    return o;
}
static J put(const Grid &g) {
    J o = J::object();
    o.add("origin", put(g.origin.pose));
    o.add("resolution", put(g.resolution));
    o.add("cells_x", put(g.cells_x));
    o.add("cells_y", put(g.cells_y));
    o.add("cells_z", put(g.cells_z));
    o.add("fields", put(g.fields));
    return o;
}
static J put(const Prism &p) {
    J o = J::object();
    J base = J::object();
    base.add("points", put(p.base.points));
    o.add("pose", put(p.pose));
    o.add("base", std::move(base));
    o.add("height", put(p.height));
    return o;
}

template <class T> std::vector<T> get_list(const J &j) {
    std::vector<T> out;
    out.reserve(j.items.size());
    for (const J &x : j.items) out.push_back(get<T>(x));
    return out;
}
static Prism get_prism(const J &j) {
    Prism p;
    p.pose = get<Pose>(j.at("pose"));
    p.base.points = get_list<Double2>(j.at("base").at("points"));
    p.height = get<double>(j.at("height"));
    return p;
}
static Image get_image(const J &j) {
    Image i;
    i.width = get<uint32_t>(j.at("width"));
    i.height = get<uint32_t>(j.at("height"));
    i.stride = get<uint32_t>(j.at("stride"));
    i.format = ImageFormat(int(j.at("format").number));
    i.data = unhex(j.at("data").str);
    return i;
}
static PointCloud get_cloud(const J &j) {
    PointCloud c;
    for (const J &f : j.at("fields").items) {
        PointField field;
        field.name = f.at("name").str;
        field.offset = get<uint32_t>(f.at("offset"));
        field.kind = ScalarKind(int(f.at("kind").number));
        field.count = get<uint32_t>(f.at("count"));
        c.fields.push_back(field);
    }
    c.point_step = get<uint32_t>(j.at("point_step"));
    c.data = unhex(j.at("data").str);
    return c;
}
static Grid get_grid(const J &j) {
    Grid g;
    g.origin.pose = get<Pose>(j.at("origin"));
    g.resolution = get<double>(j.at("resolution"));
    g.cells_x = get<uint32_t>(j.at("cells_x"));
    g.cells_y = get<uint32_t>(j.at("cells_y"));
    g.cells_z = get<uint32_t>(j.at("cells_z"));
    for (const J &f : j.at("fields").items) {
        GridField field;
        field.name = f.at("name").str;
        field.kind = ScalarKind(int(f.at("kind").number));
        field.data = unhex(f.at("data").str);
        g.fields.push_back(field);
    }
    return g;
}
static LaserScan get_scan(const J &j) {
    LaserScan s;
    s.angle_min = get<float>(j.at("angle_min"));
    s.angle_step = get<float>(j.at("angle_step"));
    s.range_min = get<float>(j.at("range_min"));
    s.range_max = get<float>(j.at("range_max"));
    s.echoes = get<uint8_t>(j.at("echoes"));
    s.ranges = get_list<float>(j.at("ranges"));
    return s;
}
static std::vector<Detection2D> get_det2d(const J &j) {
    std::vector<Detection2D> out;
    for (const J &d : j.items) {
        Detection2D x;
        x.bounds = get<OrientedBox2D>(d.at("bounds"));
        x.label = d.at("label").str;
        x.score = get<float>(d.at("score"));
        out.push_back(x);
    }
    return out;
}
static std::vector<Detection3D> get_det3d(const J &j) {
    std::vector<Detection3D> out;
    for (const J &d : j.items) {
        Detection3D x;
        x.bounds = get<OrientedBox>(d.at("bounds"));
        x.label = d.at("label").str;
        x.score = get<float>(d.at("score"));
        out.push_back(x);
    }
    return out;
}
static Matrix get_matrix(const J &j) {
    Matrix m;
    m.rows = get<uint32_t>(j.at("rows"));
    m.cols = get<uint32_t>(j.at("cols"));
    m.data = get_list<double>(j.at("data"));
    return m;
}
static Shape get_shape(const J &j) {
    const std::string &k = j.at("kind").str;
    const J &v = j.at("value");
    if (k == "aligned_box") return shape_from_aligned_box(get<AlignedBox>(v));
    if (k == "oriented_box") return shape_from_oriented_box(get<OrientedBox>(v));
    if (k == "sphere") return shape_from_sphere(get<Sphere>(v));
    if (k == "capsule") return shape_from_capsule(get<Capsule>(v));
    if (k == "cylinder") return shape_from_cylinder(get<Cylinder>(v));
    if (k == "cone") return shape_from_cone(get<Cone>(v));
    if (k == "prism") return shape_from_prism(get_prism(v));
    if (k == "segment") return shape_from_segment(get<Segment>(v));
    if (k == "polyline") return shape_from_polyline(get_list<Double3>(v));
    if (k == "polygon") return shape_from_polygon(get_list<Double3>(v));
    if (k == "mesh") return shape_from_mesh(get_list<Double3>(v.at("vertices")), get_list<uint32_t>(v.at("triangles")));
    throw std::runtime_error("no shape " + k);
}
static Shape2D get_shape2d(const J &j) {
    const std::string &k = j.at("kind").str;
    const J &v = j.at("value");
    if (k == "aligned_box2d") return shape2d_from_aligned_box2d(get<AlignedBox2D>(v));
    if (k == "oriented_box2d") return shape2d_from_oriented_box2d(get<OrientedBox2D>(v));
    if (k == "circle") return shape2d_from_circle(get<Circle>(v));
    if (k == "capsule2d") return shape2d_from_capsule2d(get<Capsule2D>(v));
    if (k == "polygon2d") return shape2d_from_polygon2d(get_list<Double2>(v));
    if (k == "segment2d") return shape2d_from_segment2d(get<Segment2D>(v));
    if (k == "polyline2d") return shape2d_from_polyline2d(get_list<Double2>(v));
    throw std::runtime_error("no 2D shape " + k);
}

/* What an options object gives back goes beside the answer, as vectors.json keeps it. */
static J extras = J::object();

struct FitHolder {
    FitOptions options;
    std::vector<double> weights;
    std::vector<uint8_t> inliers;
    FitHolder(const J &j, size_t points) {
        if (j.is_null()) return;
        if (!j.at("weights").is_null()) {
            weights = get_list<double>(j.at("weights"));
            options.weights = weights;
        }
        options.tolerance = get<double>(j.at("tolerance"));
        options.tries = get<uint32_t>(j.at("tries"));
        options.seed = get<uint64_t>(j.at("seed"));
        if (j.at("inliers").flag) {
            inliers.resize(points);
            options.inliers = inliers;
        }
    }
    void add_extras(J &) {
        if (!inliers.empty()) extras.add("inliers", put(inliers));
    }
};

struct AlignHolder {
    AlignOptions options;
    std::vector<Double3> normals;
    std::vector<uint8_t> inliers;
    double error = 0;
    bool want_error = false;
    AlignHolder(const J &j, size_t source, size_t) {
        if (j.is_null()) return;
        options.max_distance = get<double>(j.at("max_distance"));
        options.iterations = get<uint32_t>(j.at("iterations"));
        if (!j.at("normals").is_null()) {
            normals = get_list<Double3>(j.at("normals"));
            options.normals = normals;
        }
        if (j.at("inliers").flag) {
            inliers.resize(source);
            options.inliers = inliers;
        }
        if (j.at("error").flag) {
            want_error = true;
            options.error = &error;
        }
    }
    void add_extras(J &) {
        if (!inliers.empty()) extras.add("inliers", put(inliers));
        if (want_error) extras.add("error", put(error));
    }
};

using Thunk = std::function<J(const std::vector<J> &)>;

#include "generated/cpp_thunks.inc"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

template <class F> static void throws(ErrorKind kind, F f) {
    try {
        f();
        std::printf("FAILED: no throw, wanted kind %d\n", int(kind));
        failures++;
    } catch (const MathError &e) {
        CHECK(e.kind() == kind);
        CHECK(e.what()[0] != 0);
    }
}

static bool near(double a, double b, double tol = 1e-12) { return std::fabs(a - b) <= tol; }

static void replay(const char *path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::printf("cannot read %s\n", path);
        failures++;
        return;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();
    Reader reader{ text.c_str() };
    J all = reader.value();
    size_t cases = 0, functions = 0;
    for (const J &entry : all.at("functions").items) {
        const std::string &name = entry.at("f").str;
        double tol = entry.at("tol").number;
        auto thunk = thunks.find(name);
        if (thunk == thunks.end()) {
            std::printf("no thunk for %s\n", name.c_str());
            failures++;
            continue;
        }
        functions++;
        for (const J &c : entry.at("cases").items) {
            J got = J::object();
            extras = J::object();
            try {
                J v = thunk->second(c.at("a").items);
                got.add("v", std::move(v));
                for (auto &m : extras.members) got.add(m.first.c_str(), m.second);
            } catch (const MathError &e) {
                got.add("e", put(int(e.kind())));
            }
            J want = c;
            want.members.erase(std::remove_if(want.members.begin(), want.members.end(), [](const auto &m) { return m.first == "a"; }), want.members.end());
            if (!same(want, got, tol)) {
                std::printf("%s %s\n  want %s\n  got  %s\n", name.c_str(), show(c.at("a")).substr(0, 300).c_str(), show(want).substr(0, 400).c_str(), show(got).substr(0, 400).c_str());
                failures++;
            }
            cases++;
        }
    }
    std::printf("replayed %zu cases of %zu functions\n", cases, functions);
}

static void spans() {
    std::vector<Double3> v = { { 1, 0, 0 }, { 3, 0, 0 } };
    std::array<Double3, 2> a = { Double3{ 1, 0, 0 }, Double3{ 3, 0, 0 } };
    Double3 c[2] = { { 1, 0, 0 }, { 3, 0, 0 } };
    CHECK(polyline_length(v) == 2.0);
    CHECK(polyline_length(a) == 2.0);
    CHECK(polyline_length(c) == 2.0);
    CHECK(polyline_length({ Double3{ 1, 0, 0 }, Double3{ 3, 0, 0 } }) == 2.0);
    CHECK(polyline_length(Span<const Double3>(v.data(), 1)) == 0.0);
    Polyline line;
    line.points = v;
    CHECK(polyline_length(line.points) == 2.0);
    CHECK(polyline_length(as_points(line)) == 2.0);
    CHECK(polygon2d_area({ Double2{ 0, 0 }, Double2{ 1, 0 }, Double2{ 1, 1 }, Double2{ 0, 1 } }) == 1.0);
}

static void shapes() {
    Shape kept;
    {
        std::vector<Double3> points = { { 0, 0, 0 }, { 2, 0, 0 } };
        Shape line = shape_from_polyline(points);
        points.clear();
        points.shrink_to_fit();
        Shape copy = line;
        Shape moved = std::move(line);
        CHECK(near(shape_distance(copy, { 1, 1, 0 }), 1.0));
        CHECK(near(shape_distance(moved, { 1, 1, 0 }), 1.0));
        kept = copy;
    }
    CHECK(near(shape_distance(kept, { 1, 3, 0 }), 3.0));

    Shape mesh_copy;
    {
        std::vector<Double3> v = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
        std::vector<uint32_t> t = { 0, 1, 2 };
        mesh_copy = shape_from_mesh(v, t);
    }
    Shape again = mesh_copy;
    mesh_copy = Shape();
    CHECK(near(shape_distance(again, { 0.25, 0.25, 2 }), 2.0));

    Shape prism_shape;
    {
        Prism p;
        p.pose = identity_pose();
        p.base.points = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
        p.height = 2;
        prism_shape = shape_from_prism(p);
    }
    Shape prism_copy = prism_shape;
    CHECK(shape_contains(prism_copy, { 0, 0, 1 }));
    CHECK(!shape_contains(prism_copy, { 0, 0, 3 }));

    Shape2D flat;
    {
        std::vector<Double2> points = { { 0, 0 }, { 2, 0 }, { 2, 2 }, { 0, 2 } };
        flat = shape2d_from_polygon2d(points);
    }
    Shape2D flat_copy = flat;
    flat = Shape2D();
    CHECK(shape2d_contains(flat_copy, { 1, 1 }));

    throws(ErrorKind::Argument, [] { shape_from_mesh(std::vector<Double3>(3), std::vector<uint32_t>{ 0, 1 }); });
}

static void results() {
    /* an answer, a thrown error, a missing hit and a plain yes or no */
    CHECK(matrix3x3_inverse(matrix3x3_identity()).m[1][1] == 1.0f);
    throws(ErrorKind::Degenerate, [] { matrix3x3_inverse(Matrix3x3{}); });
    Plane plane{ { 0, 0, 0 }, { 0, 0, 1 } };
    Ray down{ { 0, 0, 5 }, { 0, 0, -1 } };
    Ray up{ { 0, 0, 5 }, { 0, 0, 1 } };
    CHECK(plane_raycast(plane, down).has_value());
    CHECK(near(plane_raycast(plane, down)->distance, 5.0));
    CHECK(!plane_raycast(plane, up).has_value());
    CHECK(sphere_contains(Sphere{ { 0, 0, 0 }, 1 }, { 0, 0, 0 }));
    CHECK(!sphere_contains(Sphere{ { 0, 0, 0 }, 1 }, { 2, 0, 0 }));
    auto [axis, angle] = quaternion_to_axis_angle(quaternion_from_axis_angle({ 0, 0, 1 }, 0.5));
    CHECK(near(axis.z, 1.0) && near(angle, 0.5));
    auto corners = aligned_box_corners(AlignedBox{ { 0, 0, 0 }, { 1, 1, 1 } });
    CHECK(corners.size() == 8);
}

static void lists() {
    std::vector<Double3> from = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    std::vector<Double3> to = from;
    /* lists that must match in length say so */
    std::vector<Double3> shorter(2);
    throws(ErrorKind::Argument, [&] { points_fit_pose(from, shorter); });
    /* an options list must fit the points or be empty */
    std::vector<uint8_t> wrong(2);
    FitOptions bad;
    bad.inliers = wrong;
    throws(ErrorKind::Argument, [&] { points_fit_plane(from, bad); });
    std::vector<uint8_t> marks(3);
    FitOptions good;
    good.inliers = marks;
    Plane fitted = points_fit_plane(from, good);
    CHECK(marks[0] == 1 && marks[1] == 1 && marks[2] == 1);
    CHECK(near(fabs(fitted.normal.z), 1.0));
    double error = -1;
    AlignOptions align;
    align.error = &error;
    std::vector<Double3> more = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    Pose moved = points_align(more, more, identity_pose(), align);
    CHECK(near(error, 0.0, 1e-9));
    CHECK(near(moved.orientation.w, 1.0, 1e-9));
    throws(ErrorKind::Argument, [] { polynomial_at(Span<const double>{}, 1.0); });
    CHECK(polynomial_at({ 1.0, 2.0 }, 3.0) == 7.0);

    KdTree tree = kdtree_create(more);
    KdTree other = std::move(tree);
    auto [indices, distances] = kdtree_nearest(other, { 0, 0, 0.1 }, 2);
    CHECK(indices.size() == 2 && indices[0] == 0 && near(distances[0], 0.1));
    auto [inside, far] = kdtree_within(other, { 0, 0, 0 }, 0.5);
    CHECK(inside.size() == 1 && far.size() == 1);
    auto [vertices, triangles] = shape_mesh(shape_from_sphere(Sphere{ { 0, 0, 0 }, 1 }), 8);
    CHECK(!vertices.empty() && triangles.size() % 3 == 0 && !triangles.empty());
}

static void images() {
    Image image;
    image.width = 2;
    image.height = 1;
    image.format = ImageFormat::Rgb8;
    image.data = { 1, 2, 3, 4, 5, 6 };
    CHECK(image_color_at(image, 1, 0).g == 5);
    ImageView view(image);
    CHECK(view.data == image.data.data());
    Image gray = image_convert(image, ImageFormat::Mono8);
    CHECK(gray.data.size() == 2 && gray.format == ImageFormat::Mono8);
    Image turned = image_rotate(image, 1);
    CHECK(turned.width == 1 && turned.height == 2);
    Image part = image_crop(image, 1, 0, 1, 1);
    CHECK(part.width == 1 && part.data[0] == 4);
    image.data[3] = 99;
    CHECK(part.data[0] == 4);
    image_draw(image, shape2d_from_circle(Circle{ { 0, 0 }, 5 }), Color{ 7, 8, 9, 255 }, 0);
    CHECK(image.data[0] == 7);
    throws(ErrorKind::Format, [] { Image jpeg; jpeg.format = ImageFormat::Jpeg; jpeg.width = jpeg.height = 2; jpeg.data = { 1 }; image_check(jpeg); });
}

static void clouds() {
    PointCloud cloud = point_cloud_layout(PointCloudLayout::XyzIntensity);
    CHECK(cloud.point_step == 16);
    CHECK(point_cloud_field(cloud, "intensity").has_value());
    CHECK(!point_cloud_field(cloud, "nothing").has_value());
    cloud.data.resize(2 * cloud.point_step);
    std::vector<Double3> points = { { 1, 2, 3 }, { 4, 5, 6 } };
    point_cloud_write_points(cloud, points);
    CHECK(near(point_cloud_read_points(cloud)[1].y, 5.0));
    point_cloud_write_values(cloud, "intensity", std::vector<double>{ 0.5, 0.25 });
    CHECK(near(point_cloud_read_values(cloud, "intensity")[1], 0.25));
    PointCloud one = point_cloud_select(cloud, std::vector<uint8_t>{ 0, 1 });
    CHECK(one.data.size() == 16 && near(point_cloud_read_points(one)[0].x, 4.0));
    throws(ErrorKind::Argument, [&] { point_cloud_select(cloud, std::vector<uint8_t>{ 1 }); });
    point_cloud_transform(cloud, Pose{ { 1, 0, 0 }, { 0, 0, 0, 1 } });
    CHECK(near(point_cloud_read_points(cloud)[0].x, 2.0));
}

static void grids() {
    Grid grid;
    grid.origin.pose = identity_pose();
    grid.resolution = 1;
    grid.cells_x = 2;
    grid.cells_y = 2;
    grid.cells_z = 1;
    GridField cost;
    cost.name = "cost";
    cost.kind = ScalarKind::U8;
    cost.data = { 0, 1, 2, 3 };
    grid.fields.push_back(cost);
    grid_write_value(grid, "cost", Int3{ 1, 1, 0 }, 9);
    CHECK(grid.fields[0].data[3] == 9);
    CHECK(grid_read_value(grid, "cost", Int3{ 1, 1, 0 }) == 9.0);
    throws(ErrorKind::Outside, [&] { grid_cell(grid, Double3{ 50, 0, 0 }); });
}

static void operators() {
    Double3 a{ 1, 2, 3 }, b{ 4, 5, 6 };
    CHECK(a + b == (Double3{ 5, 7, 9 }));
    CHECK(b - a == (Double3{ 3, 3, 3 }));
    CHECK(-a == (Double3{ -1, -2, -3 }));
    CHECK(a * 2.0 == (Double3{ 2, 4, 6 }));
    CHECK(2.0 * a == (Double3{ 2, 4, 6 }));
    CHECK(b / 2.0 == (Double3{ 2, 2.5, 3 }));
    Double3 c = a;
    c += b;
    c -= a;
    c *= 2.0;
    c /= 2.0;
    CHECK(c == b);
    Double2 d{ 1, 2 };
    CHECK(d + d == (Double2{ 2, 4 }));
    CHECK(d != (Double2{ 2, 4 }));
    Quaternion q = quaternion_from_axis_angle({ 0, 0, 1 }, 1.5707963267948966);
    Double3 turned = q * Double3{ 1, 0, 0 };
    CHECK(near(turned.y, 1.0));
    Quaternion twice = q * q;
    CHECK(near((twice * Double3{ 1, 0, 0 }).x, -1.0));
    Pose p{ { 1, 0, 0 }, q };
    CHECK(near((p * Double3{ 1, 0, 0 }).y, 1.0));
    Pose chain = p * pose_inverse(p);
    CHECK(near(chain.position.x, 0.0) && near(chain.orientation.w, 1.0));
    Pose2D p2{ { 1, 0 }, 0 };
    CHECK(near((p2 * p2).position.x, 2.0) && near((p2 * Double2{ 1, 1 }).x, 2.0));
}

static void axes() {
    Axes unity = axes_of(AxesPreset::Unity);
    Double3 forward = axes_point_to(unity, { 1, 0, 0 });
    CHECK(forward.x == 0 && forward.y == 0 && forward.z == 1);
    CHECK(near(axes_point_from(unity, forward).x, 1.0));
    throws(ErrorKind::Argument, [] { axes_make(1, 1, 3, 1.0); });
}

#if defined(RANT_MATH_HAS_EIGEN)
static void eigen() {
    Double3 v{ 1, 2, 3 };
    CHECK(to_eigen(v).y() == 2.0);
    CHECK(from_eigen(to_eigen(v)) == v);
    Quaternion q = quaternion_from_axis_angle({ 0, 0, 1 }, 0.5);
    CHECK(near(to_eigen(q).w(), q.w));
    Pose p{ { 1, 2, 3 }, q };
    Double3 by_math = pose_apply(p, { 4, 5, 6 });
    Eigen::Vector3d by_eigen = to_eigen(p) * Eigen::Vector3d(4, 5, 6);
    CHECK(near(by_math.x, by_eigen.x(), 1e-9) && near(by_math.y, by_eigen.y(), 1e-9));
    Pose back = from_eigen(to_eigen(p));
    CHECK(near(back.position.z, 3.0, 1e-9) && near(back.orientation.w, q.w, 1e-9));
    Matrix m = from_eigen(Eigen::MatrixXd::Identity(2, 3));
    CHECK(m.rows == 2 && m.cols == 3 && m.at(1, 1) == 1.0 && m.at(1, 2) == 0.0);
    CHECK(to_eigen(m)(1, 1) == 1.0);
    std::vector<Double3> points = { { 1, 2, 3 }, { 4, 5, 6 } };
    auto map = points_to_eigen(points);
    CHECK(map(1, 1) == 5.0 && map.data() == &points[0].x);
    Eigen::Matrix3Xd columns = map;
    CHECK(polyline_length(points_from_eigen(columns)) == polyline_length(points));
    Matrix3x3 r = matrix3x3_from_quaternion(q);
    CHECK(near(to_eigen(r)(1, 0), r.m[1][0]));
    Matrix3x3 r2 = from_eigen(to_eigen(r));
    CHECK(r2.m[1][0] == r.m[1][0] && r2.m[0][1] == r.m[0][1]);
    Double3 sum = from_eigen(to_eigen(v) + to_eigen(v));
    CHECK(sum.z == 6.0);
    Covariance3 c;
    c.m[0][1] = 2.5;
    CHECK(from_eigen(to_eigen(c)).m[0][1] == 2.5);
    std::printf("Eigen conversions checked\n");
}
#endif

#if defined(RANT_MATH_HAS_OPENCV)
static void opencv() {
    cv::Mat mat(2, 3, CV_8UC3, cv::Scalar(10, 20, 30));
    CHECK(image_color_at(mat, 1, 1).r == 30);
    Image gray = image_convert(mat, ImageFormat::Mono8);
    CHECK(gray.width == 3 && gray.height == 2);
    cv::Mat part = mat(cv::Rect(1, 0, 2, 2));
    CHECK(image_color_at(part, 0, 0).r == 30);
    image_draw(mat, shape2d_from_circle(Circle{ { 0, 0 }, 9 }), Color{ 1, 2, 3, 255 }, 0);
    CHECK(mat.at<cv::Vec3b>(0, 0)[2] == 1);
    Image rgb = image_convert(mat, ImageFormat::Rgb8);
    cv::Mat over = to_mat(rgb);
    CHECK(over.data == rgb.data.data() && over.cols == 3);
    Image copy = from_mat(mat);
    CHECK(copy.format == ImageFormat::Bgr8 && copy.data.data() != mat.data);
    throws(ErrorKind::Format, [] { cv::Mat two(2, 2, CV_8UC2); image_check(two); });
    std::printf("OpenCV conversions checked\n");
}
#endif

#if defined(RANT_MATH_HAS_PCL)
static void pcl_clouds() {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI one;
    one.x = 1; one.y = 2; one.z = 3; one.intensity = 0.5f;
    cloud.push_back(one);
    one.x = 4;
    cloud.push_back(one);
    CHECK(near(point_cloud_read_points(cloud)[1].x, 4.0));
    CHECK(near(point_cloud_read_values(cloud, "intensity")[0], 0.5));
    point_cloud_transform(cloud, Pose{ { 1, 0, 0 }, { 0, 0, 0, 1 } });
    CHECK(near(cloud.points[0].x, 2.0, 1e-5));
    PointCloud ours = from_pcl(cloud);
    CHECK(ours.point_step == sizeof(pcl::PointXYZI) && ours.data.size() == 2 * sizeof(pcl::PointXYZI));
    CHECK(near(point_cloud_read_points(ours)[1].x, 5.0));
    pcl::PointCloud<pcl::PointXYZ> xyz = to_pcl<pcl::PointXYZ>(ours);
    CHECK(xyz.size() == 2 && near(xyz.points[1].x, 5.0, 1e-5));
    std::printf("PCL conversions checked\n");
}
#endif

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("usage: rant_math_cpp_test <vectors.json>\n");
        return 2;
    }
    replay(argv[1]);
    spans();
    shapes();
    results();
    lists();
    images();
    clouds();
    grids();
    operators();
    axes();
#if defined(RANT_MATH_HAS_EIGEN)
    eigen();
#endif
#if defined(RANT_MATH_HAS_OPENCV)
    opencv();
#endif
#if defined(RANT_MATH_HAS_PCL)
    pcl_clouds();
#endif
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
