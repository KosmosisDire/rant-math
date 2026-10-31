/* Rant Math for C++17: every call of rant_math.h as rant::math::<name> on the standard types of
 * rant::types. A failure throws MathError. One file defines RANT_MATH_IMPLEMENTATION before the
 * include, or links the rant_math target. It goes beside rant.hpp in either order. */
#ifndef RANT_MATH_HPP
#define RANT_MATH_HPP

#if defined(RANT_MATH_H)
#error "include rant_math.hpp instead of rant_math.h (do not include rant_math.h before rant_math.hpp)"
#endif
#if defined(RANT_MATH_IMPLEMENTATION) && defined(RANT_HPP_INCLUDED) && !defined(RANT_IMPLEMENTATION)
#error "define RANT_MATH_IMPLEMENTATION in a file of its own, apart from rant.hpp"
#endif

/* The C standard headers math's C pulls, so their guards are set before the embed and no std name
 * is dragged into rant::detail. */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* With RANT_MATH_IMPLEMENTATION the C lands at global scope as the implementation and nothing else
 * is compiled, else inside rant::detail as declarations only. */
#if !defined(RANT_MATH_IMPLEMENTATION)
namespace rant { namespace detail {
#endif
#include "rant_math.h"   /* @RANT_EMBED@ */
#if !defined(RANT_MATH_IMPLEMENTATION)
} }
#endif

#if !defined(RANT_MATH_IMPLEMENTATION)

#include <array>
#include <cstring>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "rant_types.hpp"   /* @RANT_EMBED@ */

/* Conversions appear for each library the compiler can find. RANT_MATH_NO_EIGEN and its siblings
 * leave one out of a project that has it on the path but wants no part of it here. */
#if defined(__has_include)
  #if __has_include(<Eigen/Core>) && __has_include(<Eigen/Geometry>) && !defined(RANT_MATH_NO_EIGEN)
    #define RANT_MATH_HAS_EIGEN 1
    #include <Eigen/Core>
    #include <Eigen/Geometry>
  #endif
  #if __has_include(<opencv2/core.hpp>) && !defined(RANT_MATH_NO_OPENCV)
    #define RANT_MATH_HAS_OPENCV 1
    #include <opencv2/core.hpp>
  #endif
  #if __has_include(<pcl/point_cloud.h>) && __has_include(<pcl/common/io.h>) && !defined(RANT_MATH_NO_PCL)
    #define RANT_MATH_HAS_PCL 1
    #include <pcl/common/io.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
  #endif
#endif

namespace rant { namespace math {

using namespace rant::types;

/* What a call that failed reports, the C result it came from. */
enum class ErrorKind : int {
    Argument = -1,    /* a list too short, or a value out of range */
    Degenerate = -2,  /* no single answer: a singular matrix, points on one line */
    Space = -3,       /* never seen here, a binding always gives the room asked for */
    Format = -4,      /* an image or cloud laid out in a way this call cannot read */
    Memory = -5,      /* out of memory */
    Outside = -6      /* a point the call cannot map, such as one behind the camera */
};

class MathError : public std::runtime_error {
public:
    explicit MathError(ErrorKind kind)
        : std::runtime_error(detail::rant_math_result_text(static_cast<detail::RantMathResult>(static_cast<int>(kind)))),
          kind_(kind) {}
    ErrorKind kind() const noexcept { return kind_; }
private:
    ErrorKind kind_;
};

/* A list a call only borrows: a vector, an array, a std::span or { ... } in place. It reads
 * through the pointer, so what it names must live through the call. */
template <class T> class Span {
public:
    using element_type = T;
    constexpr Span() noexcept : data_(nullptr), size_(0) {}
    constexpr Span(T *data, size_t size) noexcept : data_(data), size_(size) {}
    template <size_t N> constexpr Span(T (&array)[N]) noexcept : data_(array), size_(N) {}
    template <class C, class = std::enable_if_t<
        !std::is_same_v<std::decay_t<C>, Span> &&
        std::is_convertible_v<std::remove_pointer_t<decltype(std::declval<C &>().data())> (*)[], T (*)[]>>>
    constexpr Span(C &&c) noexcept : data_(c.data()), size_(c.size()) {}
    /* { ... } in a call lives through that call, which is all a Span is for. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winit-list-lifetime"
#endif
    template <class U = T, class = std::enable_if_t<std::is_const_v<U>>>
    constexpr Span(std::initializer_list<std::remove_const_t<U>> list) noexcept : data_(list.begin()), size_(list.size()) {}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
    constexpr T *data() const noexcept { return data_; }
    constexpr size_t size() const noexcept { return size_; }
    constexpr bool empty() const noexcept { return size_ == 0; }
    constexpr T &operator[](size_t i) const noexcept { return data_[i]; }
    constexpr T *begin() const noexcept { return data_; }
    constexpr T *end() const noexcept { return data_ + size_; }
private:
    T *data_;
    size_t size_;
};

/* A dense matrix of doubles, rows one after another. */
struct Matrix {
    uint32_t rows = 0;
    uint32_t cols = 0;
    std::vector<double> data;
    double &at(uint32_t row, uint32_t col) { return data[size_t(row) * cols + col]; }
    double at(uint32_t row, uint32_t col) const { return data[size_t(row) * cols + col]; }
};

/* How a fit treats its points. All default, every point counts alike. */
struct FitOptions {
    Span<const double> weights;  /* one per point: how many times its squared error counts */
    double tolerance = 0;        /* above 0, RANSAC: a point farther than this from a model is an outlier of it */
    uint32_t tries = 0;          /* the most models RANSAC tries, 0 for 1000 */
    uint64_t seed = 0;           /* RANSAC's random draws: the same seed gives the same answer */
    Span<uint8_t> inliers;       /* one per point: gets 1 within tolerance of the answer, else 0 */
};

/* How points_align pairs points. All default, each pairs with its nearest. */
struct AlignOptions {
    double max_distance = 0;       /* above 0, a source point with no target point this near is left out */
    uint32_t iterations = 0;       /* the most rounds, 0 for 50 */
    Span<const Double3> normals;   /* one per target point: each pair counts by its distance along the normal */
    Span<uint8_t> inliers;         /* one per source point: gets 1 when the answer pairs it, else 0 */
    double *error = nullptr;       /* gets the RMS distance of the pairs of the answer */
};

/* The pixels of an Image or a cv::Mat, read only and with no copy. */
struct ImageView {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;  /* bytes from one row to the next, 0 when rows are packed */
    ImageFormat format = ImageFormat::Mono8;
    const uint8_t *data = nullptr;
    size_t size = 0;

    ImageView() = default;
    ImageView(const Image &image)
        : width(image.width), height(image.height), stride(image.stride), format(image.format),
          data(image.data.data()), size(image.data.size()) {}
#if defined(RANT_MATH_HAS_OPENCV)
    ImageView(const cv::Mat &mat);
#endif
};

/* The pixels of an Image or a cv::Mat a call draws on. */
struct MutableImageView {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    ImageFormat format = ImageFormat::Mono8;
    uint8_t *data = nullptr;
    size_t size = 0;

    MutableImageView() = default;
    MutableImageView(Image &image)
        : width(image.width), height(image.height), stride(image.stride), format(image.format),
          data(image.data.data()), size(image.data.size()) {}
#if defined(RANT_MATH_HAS_OPENCV)
    MutableImageView(cv::Mat &mat);
#endif
};

class KdTree;
class Shape;
class Shape2D;

namespace impl {

/* A C struct and its standard type hold the same bytes, which rant_types.hpp pins. */
template <class To, class From> inline To bits(const From &from) noexcept {
    static_assert(sizeof(To) == sizeof(From), "a C struct and its mirror must have one size");
    To to;
    std::memcpy(static_cast<void *>(&to), &from, sizeof to);
    return to;
}

inline void check(detail::RantMathResult r) {
    if (r != detail::RANT_MATH_OK) throw MathError(static_cast<ErrorKind>(static_cast<int>(r)));
}

/* The answer to asking a list's size: too small is what the question expects. */
inline void room(detail::RantMathResult r) {
    if (r != detail::RANT_MATH_OK && r != detail::RANT_MATH_ERR_SPACE) check(r);
}

[[noreturn]] inline void argument() { throw MathError(ErrorKind::Argument); }

inline uint32_t count(size_t n) {
    if (n > 0xFFFFFFFFu) argument();
    return static_cast<uint32_t>(n);
}

template <class A, class B> inline uint32_t count(const A &a, const B &b) {
    if (a.size() != b.size()) argument();
    return count(a.size());
}

template <class C, class T> inline const C *ptr(Span<const T> s) noexcept { return reinterpret_cast<const C *>(s.data()); }
template <class C, class T> inline C *mut(T *p) noexcept { return reinterpret_cast<C *>(p); }

template <class V, class C, class T> inline V view(Span<const T> s) {
    return V{ reinterpret_cast<const C *>(s.data()), count(s.size()) };
}

inline detail::RantMeshView mesh(Span<const Double3> vertices, Span<const uint32_t> triangles) {
    if (triangles.size() % 3) argument();
    detail::RantMeshView m;
    m.vertices = ptr<detail::RantDouble3>(vertices);
    m.vertex_count = count(vertices.size());
    m.triangles = triangles.data();
    m.triangle_count = count(triangles.size() / 3);
    return m;
}

inline detail::RantPrismView prism(const Prism &p) {
    detail::RantPrismView v;
    v.pose = bits<detail::RantPose>(p.pose);
    v.base.points = reinterpret_cast<const detail::RantDouble2 *>(p.base.points.data());
    v.base.count = count(p.base.points.size());
    v.height = p.height;
    return v;
}

inline Prism prism_of(const detail::RantPrismView &v) {
    Prism p;
    p.pose = bits<Pose>(v.pose);
    const Double2 *points = reinterpret_cast<const Double2 *>(v.base.points);
    p.base.points.assign(points, points + v.base.count);
    p.height = v.height;
    return p;
}

inline detail::RantMatrixView matrix(const Matrix &m) {
    if (m.data.size() != size_t(m.rows) * m.cols) argument();
    detail::RantMatrixView v;
    v.data = const_cast<double *>(m.data.data());
    v.rows = m.rows;
    v.cols = m.cols;
    return v;
}

inline Matrix matrix_out(uint32_t rows, uint32_t cols) {
    Matrix m;
    m.rows = rows;
    m.cols = cols;
    m.data.resize(size_t(rows) * cols);
    return m;
}

inline detail::RantImageView image(const ImageView &i) {
    detail::RantImageView v;
    v.width = i.width;
    v.height = i.height;
    v.stride = i.stride;
    v.format = static_cast<detail::RantImageFormat>(i.format);
    v.data = const_cast<uint8_t *>(i.data);
    v.size = i.size;
    return v;
}

inline detail::RantImageView image(const MutableImageView &i) {
    detail::RantImageView v;
    v.width = i.width;
    v.height = i.height;
    v.stride = i.stride;
    v.format = static_cast<detail::RantImageFormat>(i.format);
    v.data = i.data;
    v.size = i.size;
    return v;
}

/* A new packed image a call fills. */
inline Image image_out(ImageFormat format, uint32_t width, uint32_t height) {
    Image out;
    out.width = width;
    out.height = height;
    out.format = format;
    out.data.resize(detail::rant_image_bytes(static_cast<detail::RantImageFormat>(format), width, height));
    return out;
}

inline Image image_copy(const detail::RantImageView &part) {
    Image out = image_out(static_cast<ImageFormat>(part.format), part.width, part.height);
    detail::RantImageView to = image(MutableImageView(out));
    check(detail::rant_image_convert(&part, &to));
    return out;
}

inline uint32_t turned_width(const ImageView &src, int turns) { return ((turns % 4 + 4) % 4) & 1 ? src.height : src.width; }
inline uint32_t turned_height(const ImageView &src, int turns) { return ((turns % 4 + 4) % 4) & 1 ? src.width : src.height; }

inline std::vector<detail::RantPointFieldView> field_views(const std::vector<PointField> &fields) {
    std::vector<detail::RantPointFieldView> out(fields.size());
    for (size_t i = 0; i < fields.size(); i++) {
        out[i].name = fields[i].name.data();
        out[i].name_len = fields[i].name.size();
        out[i].offset = fields[i].offset;
        out[i].kind = static_cast<detail::RantScalarKind>(fields[i].kind);
        out[i].count = fields[i].count;
    }
    return out;
}

inline std::vector<PointField> fields_of(const detail::RantPointFieldView *fields, uint32_t count) {
    std::vector<PointField> out(count);
    for (uint32_t i = 0; i < count; i++) {
        out[i].name.assign(fields[i].name, fields[i].name_len);
        out[i].offset = fields[i].offset;
        out[i].kind = static_cast<ScalarKind>(fields[i].kind);
        out[i].count = fields[i].count;
    }
    return out;
}

inline PointCloud cloud_like(const detail::RantPointCloudView &v) {
    PointCloud out;
    out.fields = fields_of(v.fields, v.field_count);
    out.point_step = v.point_step;
    return out;
}

/* The C view of a grid, its fields held beside it. */
struct GridRef {
    detail::RantGridView raw;
    std::vector<detail::RantGridFieldView> fields;

    explicit GridRef(const Grid &g) : fields(g.fields.size()) {
        for (size_t i = 0; i < g.fields.size(); i++) {
            fields[i].name = g.fields[i].name.data();
            fields[i].name_len = g.fields[i].name.size();
            fields[i].kind = static_cast<detail::RantScalarKind>(g.fields[i].kind);
            fields[i].data = const_cast<uint8_t *>(g.fields[i].data.data());
            fields[i].size = g.fields[i].data.size();
        }
        raw.origin = bits<detail::RantPose>(g.origin.pose);
        raw.resolution = g.resolution;
        raw.cells_x = g.cells_x;
        raw.cells_y = g.cells_y;
        raw.cells_z = g.cells_z;
        raw.fields = fields.data();
        raw.field_count = count(fields.size());
    }
};

inline detail::RantLaserScanView scan(const LaserScan &s) {
    detail::RantLaserScanView v;
    v.angle_min = s.angle_min;
    v.angle_step = s.angle_step;
    v.range_min = s.range_min;
    v.range_max = s.range_max;
    v.echoes = s.echoes;
    v.ranges = s.ranges.data();
    v.range_count = count(s.ranges.size());
    return v;
}

inline detail::RantDetection2DView detection(const Detection2D &d) {
    detail::RantDetection2DView v;
    v.bounds = bits<detail::RantOrientedBox2D>(d.bounds);
    v.label = d.label.data();
    v.label_len = d.label.size();
    v.score = d.score;
    return v;
}

inline detail::RantDetection3DView detection(const Detection3D &d) {
    detail::RantDetection3DView v;
    v.bounds = bits<detail::RantOrientedBox>(d.bounds);
    v.label = d.label.data();
    v.label_len = d.label.size();
    v.score = d.score;
    return v;
}

template <class V, class D> inline std::vector<V> detections(Span<const D> list) {
    std::vector<V> out(list.size());
    for (size_t i = 0; i < list.size(); i++) out[i] = detection(list[i]);
    return out;
}

/* A list's count, which must be the points' count or none at all. */
inline void same_or_empty(size_t n, size_t points) {
    if (n != 0 && n != points) argument();
}

inline detail::RantFitOptions fit(const FitOptions &o, size_t points) {
    same_or_empty(o.weights.size(), points);
    same_or_empty(o.inliers.size(), points);
    detail::RantFitOptions v;
    v.weights = o.weights.empty() ? nullptr : o.weights.data();
    v.tolerance = o.tolerance;
    v.tries = o.tries;
    v.seed = o.seed;
    v.inliers = o.inliers.empty() ? nullptr : o.inliers.data();
    return v;
}

inline detail::RantAlignOptions align(const AlignOptions &o, size_t source, size_t target) {
    same_or_empty(o.normals.size(), target);
    same_or_empty(o.inliers.size(), source);
    detail::RantAlignOptions v;
    v.max_distance = o.max_distance;
    v.iterations = o.iterations;
    v.normals = o.normals.empty() ? nullptr : ptr<detail::RantDouble3>(o.normals);
    v.inliers = o.inliers.empty() ? nullptr : o.inliers.data();
    v.error = o.error;
    return v;
}

}   /* namespace impl */

/* The fields and bytes of a PointCloud or a pcl::PointCloud, read only and with no copy of the
 * points. */
class PointCloudView {
public:
    PointCloudView(const PointCloud &cloud) : fields_(impl::field_views(cloud.fields)) {
        raw_.fields = fields_.data();
        raw_.field_count = impl::count(fields_.size());
        raw_.point_step = cloud.point_step;
        raw_.data = const_cast<uint8_t *>(cloud.data.data());
        raw_.size = cloud.data.size();
    }
#if defined(RANT_MATH_HAS_PCL)
    template <class P> PointCloudView(const pcl::PointCloud<P> &cloud);
#endif
    const detail::RantPointCloudView *raw() const noexcept { return &raw_; }
private:
    detail::RantPointCloudView raw_;
    std::vector<PointField> names_;
    std::vector<detail::RantPointFieldView> fields_;
};

/* The same for a cloud a call writes to. */
class MutablePointCloudView {
public:
    MutablePointCloudView(PointCloud &cloud) : fields_(impl::field_views(cloud.fields)) {
        raw_.fields = fields_.data();
        raw_.field_count = impl::count(fields_.size());
        raw_.point_step = cloud.point_step;
        raw_.data = cloud.data.data();
        raw_.size = cloud.data.size();
    }
#if defined(RANT_MATH_HAS_PCL)
    template <class P> MutablePointCloudView(pcl::PointCloud<P> &cloud);
#endif
    detail::RantPointCloudView *raw() noexcept { return &raw_; }
private:
    detail::RantPointCloudView raw_;
    std::vector<PointField> names_;
    std::vector<detail::RantPointFieldView> fields_;
};

/* A kd tree over a copy of some points, freed with the object. Queries never change it, so threads
 * may share one. */
class KdTree {
public:
    KdTree() noexcept : tree_(nullptr) {}
    explicit KdTree(detail::RantKdTree *tree) noexcept : tree_(tree) {}
    KdTree(const KdTree &) = delete;
    KdTree &operator=(const KdTree &) = delete;
    KdTree(KdTree &&other) noexcept : tree_(other.tree_) { other.tree_ = nullptr; }
    KdTree &operator=(KdTree &&other) noexcept {
        if (this != &other) {
            detail::rant_kdtree_destroy(tree_);
            tree_ = other.tree_;
            other.tree_ = nullptr;
        }
        return *this;
    }
    ~KdTree() { detail::rant_kdtree_destroy(tree_); }
    const detail::RantKdTree *raw() const noexcept { return tree_; }
private:
    detail::RantKdTree *tree_;
};

/* Any shape behind one type, made by a shape_from_ call. It keeps its own copy of any list. */
class Shape {
public:
    Shape() noexcept { std::memset(static_cast<void *>(&raw_), 0, sizeof raw_); }
    Shape(const Shape &other) : raw_(other.raw_), points_(other.points_), flat_(other.flat_), triangles_(other.triangles_) { aim(); }
    Shape(Shape &&other) noexcept
        : raw_(other.raw_), points_(std::move(other.points_)), flat_(std::move(other.flat_)), triangles_(std::move(other.triangles_)) {
        std::memset(static_cast<void *>(&other.raw_), 0, sizeof other.raw_);
    }
    Shape &operator=(Shape other) noexcept {
        std::memcpy(static_cast<void *>(&raw_), &other.raw_, sizeof raw_);
        points_ = std::move(other.points_);
        flat_ = std::move(other.flat_);
        triangles_ = std::move(other.triangles_);
        return *this;
    }
    const detail::RantShape *raw() const noexcept { return &raw_; }

    static Shape plain(const detail::RantShape &raw) {
        Shape s;
        s.raw_ = raw;
        return s;
    }
    static Shape polyline(Span<const Double3> points, detail::RantShapeKind kind) {
        Shape s;
        s.raw_.kind = kind;
        s.points_.assign(points.begin(), points.end());
        s.aim();
        return s;
    }
    static Shape mesh(Span<const Double3> vertices, Span<const uint32_t> triangles) {
        impl::mesh(vertices, triangles);
        Shape s;
        s.raw_.kind = detail::RANT_SHAPE_MESH;
        s.points_.assign(vertices.begin(), vertices.end());
        s.triangles_.assign(triangles.begin(), triangles.end());
        s.aim();
        return s;
    }
    static Shape prism(const Prism &p) {
        Shape s;
        s.raw_.kind = detail::RANT_SHAPE_PRISM;
        s.raw_.as.prism = impl::prism(p);
        s.flat_ = p.base.points;
        s.aim();
        return s;
    }

private:
    /* A copy's lists live elsewhere, so what it points at must follow them. */
    void aim() noexcept {
        const detail::RantDouble3 *points = reinterpret_cast<const detail::RantDouble3 *>(points_.data());
        uint32_t n = static_cast<uint32_t>(points_.size());
        switch (raw_.kind) {
        case detail::RANT_SHAPE_POLYLINE: raw_.as.polyline.points = points; raw_.as.polyline.count = n; break;
        case detail::RANT_SHAPE_POLYGON: raw_.as.polygon.points = points; raw_.as.polygon.count = n; break;
        case detail::RANT_SHAPE_MESH:
            raw_.as.mesh.vertices = points;
            raw_.as.mesh.vertex_count = n;
            raw_.as.mesh.triangles = triangles_.data();
            raw_.as.mesh.triangle_count = static_cast<uint32_t>(triangles_.size() / 3);
            break;
        case detail::RANT_SHAPE_PRISM:
            raw_.as.prism.base.points = reinterpret_cast<const detail::RantDouble2 *>(flat_.data());
            raw_.as.prism.base.count = static_cast<uint32_t>(flat_.size());
            break;
        default: break;
        }
    }
    detail::RantShape raw_;
    std::vector<Double3> points_;
    std::vector<Double2> flat_;
    std::vector<uint32_t> triangles_;
};

/* The same in 2D. */
class Shape2D {
public:
    Shape2D() noexcept { std::memset(static_cast<void *>(&raw_), 0, sizeof raw_); }
    Shape2D(const Shape2D &other) : raw_(other.raw_), points_(other.points_) { aim(); }
    Shape2D(Shape2D &&other) noexcept : raw_(other.raw_), points_(std::move(other.points_)) {
        std::memset(static_cast<void *>(&other.raw_), 0, sizeof other.raw_);
    }
    Shape2D &operator=(Shape2D other) noexcept {
        std::memcpy(static_cast<void *>(&raw_), &other.raw_, sizeof raw_);
        points_ = std::move(other.points_);
        return *this;
    }
    const detail::RantShape2D *raw() const noexcept { return &raw_; }

    static Shape2D plain(const detail::RantShape2D &raw) {
        Shape2D s;
        s.raw_ = raw;
        return s;
    }
    static Shape2D polyline(Span<const Double2> points, detail::RantShape2DKind kind) {
        Shape2D s;
        s.raw_.kind = kind;
        s.points_.assign(points.begin(), points.end());
        s.aim();
        return s;
    }

private:
    void aim() noexcept {
        const detail::RantDouble2 *points = reinterpret_cast<const detail::RantDouble2 *>(points_.data());
        uint32_t n = static_cast<uint32_t>(points_.size());
        if (raw_.kind == detail::RANT_SHAPE2D_POLYGON) { raw_.as.polygon.points = points; raw_.as.polygon.count = n; }
        if (raw_.kind == detail::RANT_SHAPE2D_POLYLINE) { raw_.as.polyline.points = points; raw_.as.polyline.count = n; }
    }
    detail::RantShape2D raw_;
    std::vector<Double2> points_;
};

#include "rant_math_api.inc"   /* @RANT_EMBED@ */

/* A point list and its size in one place, for a call that wants neither copied. */
inline Span<const Double3> as_points(const Polygon &p) { return p.points; }
inline Span<const Double2> as_points(const Polygon2D &p) { return p.points; }
inline Span<const Double3> as_points(const Polyline &p) { return p.points; }
inline Span<const Double2> as_points(const Polyline2D &p) { return p.points; }

#if defined(RANT_MATH_HAS_EIGEN)

inline Eigen::Vector2d to_eigen(const Double2 &v) { return { v.x, v.y }; }
inline Eigen::Vector3d to_eigen(const Double3 &v) { return { v.x, v.y, v.z }; }
inline Eigen::Vector4d to_eigen(const Double4 &v) { return { v.x, v.y, v.z, v.w }; }
inline Eigen::Quaterniond to_eigen(const Quaternion &q) { return Eigen::Quaterniond(q.w, q.x, q.y, q.z); }
inline Eigen::Isometry3d to_eigen(const Pose &p) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = to_eigen(p.orientation).toRotationMatrix();
    t.translation() = to_eigen(p.position);
    return t;
}
inline Eigen::Matrix3f to_eigen(const Matrix3x3 &m) { return Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&m.m[0][0]); }
inline Eigen::Matrix4f to_eigen(const Matrix4x4 &m) { return Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(&m.m[0][0]); }
inline Eigen::Matrix3d to_eigen(const Covariance3 &m) { return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&m.m[0][0]); }
inline Eigen::Matrix<double, 6, 6> to_eigen(const Covariance6 &m) { return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(&m.m[0][0]); }
inline Eigen::MatrixXd to_eigen(const Matrix &m) {
    return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(m.data.data(), m.rows, m.cols);
}

/* Any matrix or expression, by its size and scalar: a 3 by 1 is a Double3, a 3 by 3 of float a
 * Matrix3x3, one of double a Covariance3, and a size known only at run time a Matrix. */
template <class D> inline auto from_eigen(const Eigen::MatrixBase<D> &m) {
    constexpr int rows = D::RowsAtCompileTime, cols = D::ColsAtCompileTime;
    constexpr bool single = std::is_same_v<typename D::Scalar, float>;
    if constexpr (rows == 2 && cols == 1) {
        return Double2{ double(m(0)), double(m(1)) };
    } else if constexpr (rows == 3 && cols == 1) {
        return Double3{ double(m(0)), double(m(1)), double(m(2)) };
    } else if constexpr (rows == 4 && cols == 1) {
        return Double4{ double(m(0)), double(m(1)), double(m(2)), double(m(3)) };
    } else if constexpr (rows == 3 && cols == 3 && single) {
        Matrix3x3 out;
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&out.m[0][0]) = m;
        return out;
    } else if constexpr (rows == 4 && cols == 4 && single) {
        Matrix4x4 out;
        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(&out.m[0][0]) = m;
        return out;
    } else if constexpr (rows == 3 && cols == 3) {
        Covariance3 out;
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&out.m[0][0]) = m.template cast<double>();
        return out;
    } else if constexpr (rows == 6 && cols == 6) {
        Covariance6 out;
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(&out.m[0][0]) = m.template cast<double>();
        return out;
    } else {
        Matrix out = impl::matrix_out(impl::count(size_t(m.rows())), impl::count(size_t(m.cols())));
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(out.data.data(), m.rows(), m.cols()) = m.template cast<double>();
        return out;
    }
}
inline Quaternion from_eigen(const Eigen::Quaterniond &q) { return { q.x(), q.y(), q.z(), q.w() }; }
inline Pose from_eigen(const Eigen::Isometry3d &t) {
    return { from_eigen(Eigen::Vector3d(t.translation())), from_eigen(Eigen::Quaterniond(t.rotation())) };
}

/* Three doubles per point in both: a list as a 3 by n matrix and back, with no copy. */
inline Eigen::Map<const Eigen::Matrix3Xd> points_to_eigen(Span<const Double3> points) {
    return Eigen::Map<const Eigen::Matrix3Xd>(reinterpret_cast<const double *>(points.data()), 3, Eigen::Index(points.size()));
}
inline Span<const Double3> points_from_eigen(const Eigen::Matrix3Xd &m) {
    return Span<const Double3>(reinterpret_cast<const Double3 *>(m.data()), size_t(m.cols()));
}

#endif   /* RANT_MATH_HAS_EIGEN */

#if defined(RANT_MATH_HAS_OPENCV)

namespace impl {
/* A cv::Mat names no colour order, so three channels are OpenCV's own BGR. */
inline ImageFormat format_of(const cv::Mat &m) {
    switch (m.type()) {
    case CV_8UC1: return ImageFormat::Mono8;
    case CV_8UC3: return ImageFormat::Bgr8;
    case CV_16UC1: return ImageFormat::Mono16;
    case CV_32FC1: return ImageFormat::Monof32;
    default: throw MathError(ErrorKind::Format);
    }
}
inline int mat_type(ImageFormat f) {
    switch (f) {
    case ImageFormat::Mono8: return CV_8UC1;
    case ImageFormat::Mono16: return CV_16UC1;
    case ImageFormat::Rgb8: case ImageFormat::Bgr8: return CV_8UC3;
    case ImageFormat::Rgba8: return CV_8UC4;
    case ImageFormat::Monof32: return CV_32FC1;
    default: throw MathError(ErrorKind::Format);
    }
}
}   /* namespace impl */

inline ImageView::ImageView(const cv::Mat &mat)
    : width(uint32_t(mat.cols)), height(uint32_t(mat.rows)), stride(uint32_t(mat.step[0])), format(impl::format_of(mat)),
      data(mat.data), size(mat.step[0] * size_t(mat.rows)) {}
inline MutableImageView::MutableImageView(cv::Mat &mat)
    : width(uint32_t(mat.cols)), height(uint32_t(mat.rows)), stride(uint32_t(mat.step[0])), format(impl::format_of(mat)),
      data(mat.data), size(mat.step[0] * size_t(mat.rows)) {}

/* A cv::Mat over the image's own pixels, so the image must outlive it. An Rgb8 image's channels stay
 * in that order, and OpenCV calls that BGR. */
inline cv::Mat to_mat(Image &image) {
    int stride = int(image.stride ? image.stride : (image.height ? image.data.size() / image.height : 0));
    return cv::Mat(int(image.height), int(image.width), impl::mat_type(image.format), image.data.data(), size_t(stride));
}

/* A new Image holding a copy of the Mat's pixels. */
inline Image from_mat(const cv::Mat &mat) {
    Image out = impl::image_out(impl::format_of(mat), uint32_t(mat.cols), uint32_t(mat.rows));
    size_t line = out.height ? out.data.size() / out.height : 0;
    for (uint32_t y = 0; y < out.height; y++) std::memcpy(out.data.data() + y * line, mat.ptr(int(y)), line);
    return out;
}

#endif   /* RANT_MATH_HAS_OPENCV */

#if defined(RANT_MATH_HAS_PCL)

namespace impl {
template <class P> inline std::vector<PointField> pcl_fields() {
    std::vector<PointField> out;
    for (const pcl::PCLPointField &f : pcl::getFields<P>()) {
        PointField field;
        field.name = f.name;
        field.offset = f.offset;
        /* PCL numbers its datatypes as ScalarKind does. */
        field.kind = static_cast<ScalarKind>(f.datatype);
        field.count = f.count;
        out.push_back(std::move(field));
    }
    return out;
}
}   /* namespace impl */

template <class P> inline PointCloudView::PointCloudView(const pcl::PointCloud<P> &cloud)
    : names_(impl::pcl_fields<P>()), fields_(impl::field_views(names_)) {
    raw_.fields = fields_.data();
    raw_.field_count = impl::count(fields_.size());
    raw_.point_step = sizeof(P);
    raw_.data = reinterpret_cast<uint8_t *>(const_cast<P *>(cloud.points.data()));
    raw_.size = cloud.points.size() * sizeof(P);
}

template <class P> inline MutablePointCloudView::MutablePointCloudView(pcl::PointCloud<P> &cloud)
    : names_(impl::pcl_fields<P>()), fields_(impl::field_views(names_)) {
    raw_.fields = fields_.data();
    raw_.field_count = impl::count(fields_.size());
    raw_.point_step = sizeof(P);
    raw_.data = reinterpret_cast<uint8_t *>(cloud.points.data());
    raw_.size = cloud.points.size() * sizeof(P);
}

/* A PointCloud with a copy of the points, in PCL's layout. */
template <class P> inline PointCloud from_pcl(const pcl::PointCloud<P> &cloud) {
    PointCloud out;
    out.fields = impl::pcl_fields<P>();
    out.point_step = sizeof(P);
    out.width = cloud.height > 1 ? cloud.width : 0;
    out.dense = cloud.is_dense;
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(cloud.points.data());
    out.data.assign(bytes, bytes + cloud.points.size() * sizeof(P));
    return out;
}

/* The points as P, each field copied from the field of its name and kind, others left as P starts. */
template <class P> inline pcl::PointCloud<P> to_pcl(const PointCloud &cloud) {
    if (!cloud.point_step || cloud.data.size() % cloud.point_step) throw MathError(ErrorKind::Format);
    pcl::PointCloud<P> out;
    size_t n = cloud.data.size() / cloud.point_step;
    out.points.resize(n);
    out.width = uint32_t(n);
    out.height = 1;
    out.is_dense = cloud.dense;
    for (const pcl::PCLPointField &want : pcl::getFields<P>()) {
        for (const PointField &have : cloud.fields) {
            if (have.name != want.name || uint8_t(have.kind) != want.datatype || have.count != want.count) continue;
            size_t bytes = have.count * pcl::getFieldSize(want.datatype);
            if (have.offset + bytes > cloud.point_step) throw MathError(ErrorKind::Format);
            for (size_t i = 0; i < n; i++)
                std::memcpy(reinterpret_cast<uint8_t *>(&out.points[i]) + want.offset,
                            cloud.data.data() + i * cloud.point_step + have.offset, bytes);
        }
    }
    return out;
}

#endif   /* RANT_MATH_HAS_PCL */

} }   /* namespace rant::math */

/* Operators live beside the types so a call finds them with no using. */
namespace rant { namespace types {

inline Double2 operator+(Double2 a, Double2 b) { return math::double2_add(a, b); }
inline Double2 operator-(Double2 a, Double2 b) { return math::double2_subtract(a, b); }
inline Double2 operator-(Double2 a) { return math::double2_scale(a, -1.0); }
inline Double2 operator*(Double2 a, double k) { return math::double2_scale(a, k); }
inline Double2 operator*(double k, Double2 a) { return math::double2_scale(a, k); }
inline Double2 operator/(Double2 a, double k) { return math::double2_scale(a, 1.0 / k); }
inline Double2 &operator+=(Double2 &a, Double2 b) { return a = a + b; }
inline Double2 &operator-=(Double2 &a, Double2 b) { return a = a - b; }
inline Double2 &operator*=(Double2 &a, double k) { return a = a * k; }
inline Double2 &operator/=(Double2 &a, double k) { return a = a / k; }
inline bool operator==(Double2 a, Double2 b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(Double2 a, Double2 b) { return !(a == b); }

inline Double3 operator+(Double3 a, Double3 b) { return math::double3_add(a, b); }
inline Double3 operator-(Double3 a, Double3 b) { return math::double3_subtract(a, b); }
inline Double3 operator-(Double3 a) { return math::double3_scale(a, -1.0); }
inline Double3 operator*(Double3 a, double k) { return math::double3_scale(a, k); }
inline Double3 operator*(double k, Double3 a) { return math::double3_scale(a, k); }
inline Double3 operator/(Double3 a, double k) { return math::double3_scale(a, 1.0 / k); }
inline Double3 &operator+=(Double3 &a, Double3 b) { return a = a + b; }
inline Double3 &operator-=(Double3 &a, Double3 b) { return a = a - b; }
inline Double3 &operator*=(Double3 &a, double k) { return a = a * k; }
inline Double3 &operator/=(Double3 &a, double k) { return a = a / k; }
inline bool operator==(Double3 a, Double3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool operator!=(Double3 a, Double3 b) { return !(a == b); }

inline Double4 operator+(Double4 a, Double4 b) { return math::double4_add(a, b); }
inline Double4 operator-(Double4 a, Double4 b) { return math::double4_subtract(a, b); }
inline Double4 operator-(Double4 a) { return math::double4_scale(a, -1.0); }
inline Double4 operator*(Double4 a, double k) { return math::double4_scale(a, k); }
inline Double4 operator*(double k, Double4 a) { return math::double4_scale(a, k); }
inline Double4 operator/(Double4 a, double k) { return math::double4_scale(a, 1.0 / k); }
inline bool operator==(Double4 a, Double4 b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }
inline bool operator!=(Double4 a, Double4 b) { return !(a == b); }

/* a * b is a's frame then b's, as quaternion_compose and pose_compose read: a parent to child chain. */
inline Quaternion operator*(Quaternion a, Quaternion b) { return math::quaternion_compose(a, b); }
inline Double3 operator*(Quaternion q, Double3 v) { return math::quaternion_rotate(q, v); }
inline Pose operator*(Pose a, Pose b) { return math::pose_compose(a, b); }
inline Double3 operator*(Pose p, Double3 point) { return math::pose_apply(p, point); }
inline Pose2D operator*(Pose2D a, Pose2D b) { return math::pose2d_compose(a, b); }
inline Double2 operator*(Pose2D p, Double2 point) { return math::pose2d_apply(p, point); }

} }   /* namespace rant::types */

#endif   /* !RANT_MATH_IMPLEMENTATION */
#endif   /* RANT_MATH_HPP */
