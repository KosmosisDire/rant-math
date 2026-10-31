"""The Python binding: every case of vectors.json through its call, again with numpy lists where a
call takes them, then what vectors cannot show. Takes the path of vectors.json."""
import dataclasses
import gc
import json
import math
import sys

import rant_math
from rant_math import MathError, ErrorKind

failures = 0
try:
    import numpy  # noqa: F401
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False
ALIASES = {"RantMatrix3x3", "RantMatrix4x4", "RantCovariance3", "RantCovariance6"}


def check(cond, what):
    global failures
    if not cond:
        print("FAILED", what)
        failures += 1


def number(v):
    if v == "nan":
        return math.nan
    if v == "inf":
        return math.inf
    if v == "-inf":
        return -math.inf
    return v


def make(types, enums, t, v):
    """A native value of type t from its JSON."""
    if t in ALIASES:
        return v["m"]
    if t in enums:
        return getattr(rant_math, t[4:])(v)
    if t in types:
        cls = getattr(rant_math, t[4:])
        args = []
        for f in types[t]:
            value = v[f["name"]]
            ft = f["type"]
            if t == "RantCameraIntrinsics" and f["name"] == "model":
                value = rant_math.DistortionModel(value)
            elif f.get("dims"):
                value = [number(x) for x in value]
            else:
                value = make(types, enums, ft, value)
            args.append(value)
        return cls(*args)
    return number(v)


def make_prism(types, enums, v):
    return rant_math.Prism(make(types, enums, "RantPose", v["pose"]),
                           rant_math.Polygon2D(items(types, enums, "RantDouble2", v["base"]["points"])), number(v["height"]))


def items(types, enums, t, values):
    return [make(types, enums, t, x) for x in values]


def make_shape(types, enums, v, two):
    kind, value = v["kind"], v["value"]
    build = getattr(rant_math, ("shape2d_from_" if two else "shape_from_") + kind)
    if kind == "mesh":
        return build(items(types, enums, "RantDouble3", value["vertices"]), value["triangles"])
    if kind == "prism":
        return build(make_prism(types, enums, value))
    if kind in ("polygon", "polyline"):
        return build(items(types, enums, "RantDouble3", value))
    if kind in ("polygon2d", "polyline2d"):
        return build(items(types, enums, "RantDouble2", value))
    names = {"aligned_box": "RantAlignedBox", "oriented_box": "RantOrientedBox", "sphere": "RantSphere", "capsule": "RantCapsule",
             "cylinder": "RantCylinder", "cone": "RantCone", "segment": "RantSegment", "aligned_box2d": "RantAlignedBox2D",
             "oriented_box2d": "RantOrientedBox2D", "circle": "RantCircle", "capsule2d": "RantCapsule2D", "segment2d": "RantSegment2D"}
    return build(make(types, enums, names[kind], value))


def make_image(v, writable=False):
    data = bytes.fromhex(v["data"])
    return rant_math.Image(v["width"], v["height"], v["stride"], rant_math.ImageFormat(v["format"]), bytearray(data) if writable else data)


def make_cloud(v, writable=False):
    fields = [rant_math.PointField(f["name"], f["offset"], rant_math.ScalarKind(f["kind"]), f["count"]) for f in v["fields"]]
    data = bytes.fromhex(v["data"])
    return rant_math.PointCloud(rant_math.Transform(), 0, fields, v["point_step"], False, data)


def make_grid(types, enums, v):
    fields = [rant_math.GridField(f["name"], rant_math.ScalarKind(f["kind"]), bytes.fromhex(f["data"])) for f in v["fields"]]
    origin = rant_math.Transform(make(types, enums, "RantPose", v["origin"]))
    return rant_math.Grid(origin, v["resolution"], v["cells_x"], v["cells_y"], v["cells_z"], fields)


def make_scan(v):
    return rant_math.LaserScan(v["angle_min"], v["angle_step"], 0.0, v["range_min"], v["range_max"], v["echoes"], [number(x) for x in v["ranges"]])


class Extras:
    def __init__(self):
        self.fit = None
        self.align = None


def make_input(types, enums, kind, v, extras, points, source, use_numpy):
    k, _, t = kind.partition(":")
    if k == "value" or k == "ref":
        return make(types, enums, t, v)
    if k == "opt_ref":
        return None if v is None else make(types, enums, t, v)
    if k == "list":
        if use_numpy:
            import numpy
            if t in ("double", "uint32_t", "uint8_t"):
                return numpy.array([number(x) for x in v], dtype={"double": "float64", "uint32_t": "uint32", "uint8_t": "uint8"}[t])
            if t in ("RantDouble2", "RantDouble3", "RantPose", "RantPose2D"):
                return numpy.array([flatten(x) for x in v], dtype="float64").reshape(len(v), {"RantDouble2": 2, "RantDouble3": 3, "RantPose": 7, "RantPose2D": 3}[t])
        if t in ("double", "uint32_t", "uint8_t"):
            return [number(x) for x in v]
        return items(types, enums, t, v)
    if k == "string":
        return v
    if k == "prism":
        return make_prism(types, enums, v)
    if k == "shape":
        return make_shape(types, enums, v, False)
    if k == "shape2d":
        return make_shape(types, enums, v, True)
    if k in ("image", "image_mut"):
        return make_image(v)
    if k in ("cloud", "cloud_mut"):
        return make_cloud(v)
    if k in ("grid", "grid_mut"):
        return make_grid(types, enums, v)
    if k == "scan":
        return make_scan(v)
    if k == "det2d":
        return [rant_math.Detection2D(make(types, enums, "RantOrientedBox2D", d["bounds"]), d["label"], d["score"]) for d in v]
    if k == "det3d":
        return [rant_math.Detection3D(make(types, enums, "RantOrientedBox", d["bounds"]), d["label"], d["score"]) for d in v]
    if k == "matrix":
        return rant_math.Matrix(v["rows"], v["cols"], [number(x) for x in v["data"]])
    if k == "fit_options":
        if v is None:
            return None
        o = rant_math.FitOptions(None if v["weights"] is None else v["weights"], v["tolerance"], v["tries"], v["seed"],
                                 bytearray(points) if v["inliers"] else None)
        extras.fit = o
        return o
    if k == "align_options":
        if v is None:
            return None
        o = rant_math.AlignOptions(v["max_distance"], v["iterations"],
                                   None if v["normals"] is None else items(types, enums, "RantDouble3", v["normals"]),
                                   bytearray(source) if v["inliers"] else None)
        extras.align = (o, v["error"])
        return o
    if k == "tree":
        return rant_math.kdtree_create(items(types, enums, "RantDouble3", v["points"]))
    raise KeyError(k)


def flatten(v):
    out = []
    if isinstance(v, dict):
        for x in v.values():
            out += flatten(x)
    elif isinstance(v, (list, tuple)):
        for x in v:
            out += flatten(x)
    elif isinstance(v, str):
        out.append(number(v))
    else:
        out.append(v)
    return out


def to_json(v, top=True):
    if v is None or isinstance(v, bool):
        return v
    if hasattr(v, "tolist"):
        return v
    if isinstance(v, (bytes, bytearray)):
        return bytes(v).hex()
    if isinstance(v, rant_math.Image):
        return {"width": v.width, "height": v.height, "stride": v.stride, "format": int(v.format), "data": bytes(v.data).hex()}
    if isinstance(v, rant_math.PointCloud):
        return {"fields": [to_json(f) for f in v.fields], "point_step": v.point_step, "data": bytes(v.data).hex()}
    if isinstance(v, rant_math.PointField):
        return {"name": v.name, "offset": v.offset, "kind": int(v.kind), "count": v.count}
    if isinstance(v, rant_math.GridField):
        return {"name": v.name, "kind": int(v.kind), "data": bytes(v.data).hex()}
    if isinstance(v, rant_math.Grid):
        return {"origin": to_json(v.origin.pose), "resolution": v.resolution, "cells_x": v.cells_x, "cells_y": v.cells_y,
                "cells_z": v.cells_z, "fields": [to_json(f) for f in v.fields]}
    if isinstance(v, rant_math.Prism):
        return {"pose": to_json(v.pose), "base": {"points": [to_json(x) for x in v.base.points]}, "height": v.height}
    if isinstance(v, rant_math.Matrix):
        return {"rows": v.rows, "cols": v.cols, "data": [to_json(x) for x in v.data]}
    if dataclasses.is_dataclass(v):
        return {f.name: to_json(getattr(v, f.name)) for f in dataclasses.fields(v)}
    if isinstance(v, (list, tuple)):
        return [to_json(x) for x in v]
    if isinstance(v, float):
        if v != v:
            return "nan"
        if v in (math.inf, -math.inf):
            return "inf" if v > 0 else "-inf"
    return int(v) if isinstance(v, int) else v


def close(want, got, tol):
    if isinstance(want, bool) or isinstance(got, bool):
        return want == got
    if isinstance(want, (int, float)) and isinstance(got, (int, float)):
        return want == got or abs(want - got) <= tol * max(1.0, abs(want), abs(got))
    if isinstance(want, list) and isinstance(got, list):
        return len(want) == len(got) and all(close(a, b, tol) for a, b in zip(want, got))
    if isinstance(want, dict) and isinstance(got, dict):
        return want.keys() == got.keys() and all(close(want[k], got[k], tol) for k in want)
    return want == got


def alias_wrap(entry, v, f):
    return v


def expected_of(case):
    return {k: x for k, x in case.items() if k != "a"}


def run_case(entry, case, types, enums, use_numpy):
    extras = Extras()
    args = case["a"]
    kinds = entry["in"]
    points = len(args[0]) if kinds and kinds[0].startswith("list:") else 0
    source = points
    natives = [make_input(types, enums, k, v, extras, points, source, use_numpy) for k, v in zip(kinds, args)]
    fn = getattr(rant_math, entry["f"])
    try:
        value = fn(*natives)
    except MathError as e:
        return {"e": int(e.kind)}
    for i, k in enumerate(kinds):
        if k in ("cloud_mut", "image_mut", "grid_mut"):
            value = natives[i]
    got = {"v": value}
    if extras.fit is not None and extras.fit.inliers is not None:
        got["inliers"] = bytes(extras.fit.inliers).hex()
    if extras.align is not None:
        o, want_error = extras.align
        if o.inliers is not None:
            got["inliers"] = bytes(o.inliers).hex()
        if want_error:
            got["error"] = o.error
    return got


def wrap_matrix(entry, json_value, tail):
    return json_value


def replay(path):
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    types, enums = doc["types"], set(doc["enums"])
    done = 0
    numpy_done = 0
    for entry in doc["functions"]:
        tol = entry["tol"]
        for case in entry["cases"]:
            want = expected_of(case)
            got = run_case(entry, case, types, enums, False)
            if "v" in got:
                got["v"] = shaped(entry, to_json(got["v"]))
            if not close(want, to_json_error(got), tol):
                check(False, f"{entry['f']} {json.dumps(case['a'])[:200]}\n  want {json.dumps(want)[:300]}\n  got  {json.dumps(got, default=str)[:300]}")
            done += 1
            if HAVE_NUMPY and any(k.startswith("list:") for k in entry["in"]) and numpy_ok(entry):
                got = run_case(entry, case, types, enums, True)
                if "e" in want or "e" in got:
                    check(want.get("e") == got.get("e"), f"numpy {entry['f']} {json.dumps(case['a'])[:200]}: want {want} got {got}")
                else:
                    a, b = numeric_flat(want["v"]), numeric_flat(to_json(got["v"]))
                    same = len(a) == len(b) and all(x == y or (x != x and y != y) or abs(x - y) <= tol * max(1.0, abs(x), abs(y)) for x, y in zip(a, b))
                    check(same, f"numpy {entry['f']} {json.dumps(case['a'])[:200]}\n  want {str(a)[:300]}\n  got  {str(b)[:300]}")
                numpy_done += 1
    print(f"replayed {done} cases of {len(doc['functions'])} functions, {numpy_done} again with numpy")


def numpy_ok(entry):
    return not any(k in ("list:RantColor",) for k in entry["in"])


def numeric_flat(v):
    if hasattr(v, "tolist"):
        return numeric_flat(v.tolist())
    if v is None:
        return []
    if isinstance(v, bool):
        return [float(v)]
    if isinstance(v, (bytes, bytearray)):
        return [float(x) for x in v]
    if isinstance(v, str):
        if v in ("nan", "inf", "-inf"):
            return [number(v)]
        return [float(x) for x in bytes.fromhex(v)]
    if isinstance(v, dict):
        out = []
        for k, x in v.items():
            if k != "name":
                out += numeric_flat(x)
        return out
    if isinstance(v, (list, tuple)):
        out = []
        for x in v:
            out += numeric_flat(x)
        return out
    if dataclasses.is_dataclass(v):
        return numeric_flat({f.name: getattr(v, f.name) for f in dataclasses.fields(v)})
    if isinstance(v, (int, float)):
        return [float(v)]
    return numeric_flat(to_json(v))


def shaped(entry, value):
    # A matrix type is nested lists in Python, and vectors.json keeps it under its field name.
    out = entry.get("out", [])
    if len(out) == 1 and out[0] in ALIASES:
        return {"m": value}
    if len(out) > 1:
        return [({"m": x} if o in ALIASES else x) for o, x in zip(out, value)]
    return value


def to_json_error(got):
    return got


def expect_error(kind, fn, *args):
    try:
        fn(*args)
        check(False, f"no error from {fn.__name__}, wanted {kind}")
    except MathError as e:
        check(e.kind == kind, f"{fn.__name__} raised {e.kind}, wanted {kind}")
        check(bool(str(e)), "an error says something")


def hand_tests():
    import rant_math as m
    d3 = m.Double3
    # the standard types and identity helpers come through
    check(m.identity_pose().orientation.w == 1.0, "identity")
    # results
    check(m.plane_raycast(m.Plane(d3(0, 0, 0), d3(0, 0, 1)), m.Ray(d3(0, 0, 5), d3(0, 0, 1))) is None, "a miss is None")
    check(m.plane_raycast(m.Plane(d3(0, 0, 0), d3(0, 0, 1)), m.Ray(d3(0, 0, 5), d3(0, 0, -1))).distance == 5.0, "a hit")
    check(m.sphere_contains(m.Sphere(d3(), 1.0), d3()) is True, "a bool")
    axis, angle = m.quaternion_to_axis_angle(m.quaternion_from_axis_angle(d3(0, 0, 1), 0.5))
    check(abs(angle - 0.5) < 1e-12, "a pair")
    expect_error(ErrorKind.Degenerate, m.matrix3x3_inverse, [[0.0] * 3] * 3)
    # lists: iterables and mismatches
    check(m.polyline_length(iter([d3(0, 0, 0), d3(2, 0, 0)])) == 2.0, "any iterable")
    expect_error(ErrorKind.Argument, m.points_fit_pose, [d3()] * 3, [d3()] * 2)
    expect_error(ErrorKind.Argument, m.polynomial_at, [], 1.0)
    # a shape holds its own data
    pts = [d3(0, 0, 0), d3(2, 0, 0)]
    line = m.shape_from_polyline(pts)
    pts.clear()
    gc.collect()
    check(abs(m.shape_distance(line, d3(1, 1, 0)) - 1.0) < 1e-12, "a shape keeps its list")
    # options write back
    inliers = bytearray(3)
    plane = m.points_fit_plane([d3(0, 0, 0), d3(1, 0, 0), d3(0, 1, 0)], m.FitOptions(inliers=inliers))
    check(bytes(inliers) == b"\x01\x01\x01", "inliers come back")
    check(abs(abs(plane.normal.z) - 1.0) < 1e-12, "the plane")
    expect_error(ErrorKind.Argument, m.points_fit_plane, [d3()] * 3, m.FitOptions(inliers=bytearray(2)))
    o = m.AlignOptions()
    more = [d3(0, 0, 0), d3(1, 0, 0), d3(0, 1, 0), d3(0, 0, 1)]
    m.points_align(more, more, m.identity_pose(), o)
    check(o.error is not None and o.error < 1e-9, "the error comes back")
    # kd tree
    with m.kdtree_create(more) as tree:
        indices, distances = m.kdtree_nearest(tree, d3(0, 0, 0.1), 2)
        check(indices[0] == 0 and abs(distances[0] - 0.1) < 1e-12, "nearest")
        check(len(m.kdtree_within(tree, d3(0, 0, 0), 0.5)[0]) == 1, "within")
    check(tree._handle is None, "the tree is freed")
    m.kdtree_create(more)
    gc.collect()
    # images: reads never copy, a call writes the image in place
    img = m.Image(2, 1, 0, m.ImageFormat.Rgb8, bytes([1, 2, 3, 4, 5, 6]))
    check(m.image_color_at(img, 1, 0).g == 5, "read from bytes")
    gray = m.image_convert(img, m.ImageFormat.Mono8)
    check(len(gray.data) == 2, "a new image")
    m.image_draw(img, m.shape2d_from_circle(m.Circle(m.Double2(0, 0), 5.0)), m.Color(7, 8, 9, 255), 0.0)
    check(img.data[0] == 7, "drawn in place")
    expect_error(ErrorKind.Format, m.image_check, m.Image(2, 2, 0, m.ImageFormat.Jpeg, b"\x01"))
    # clouds and grids write in place from bytes
    cloud = m.point_cloud_layout(m.PointCloudLayout.Xyz)
    cloud.data = bytes(24)
    m.point_cloud_write_points(cloud, [d3(1, 2, 3), d3(4, 5, 6)])
    check(abs(m.point_cloud_read_points(cloud)[1].y - 5.0) < 1e-6, "cloud round trip")
    one = m.point_cloud_select(cloud, [0, 1])
    check(len(one.data) == 12, "select")
    expect_error(ErrorKind.Argument, m.point_cloud_select, cloud, [1])
    check(m.point_cloud_field(cloud, "z").offset == 8 and m.point_cloud_field(cloud, "q") is None, "field")
    field = m.GridField("cost", m.ScalarKind.U8, bytes([0, 1, 2, 3]))
    grid = m.Grid(m.identity_transform(), 1.0, 2, 2, 1, [field])
    m.grid_write_value(grid, "cost", m.Int3(1, 1, 0), 9.0)
    check(grid.fields[0].data[3] == 9 and m.grid_read_value(grid, "cost", m.Int3(1, 1, 0)) == 9.0, "grid")
    numpy_tests()


def numpy_tests():
    try:
        import numpy as np
    except ImportError:
        print("no numpy, its tests are left out")
        return
    import rant_math as m
    a = np.array([[0.0, 0, 0], [2, 0, 0]])
    check(m.polyline_length(a) == 2.0, "numpy in")
    moved = m.pose_apply_points(m.Pose(m.Double3(1, 0, 0), m.identity_rotation()), a)
    check(isinstance(moved, np.ndarray) and moved.shape == (2, 3) and moved[1, 0] == 3.0, "numpy out")
    check(isinstance(m.pose_apply_points(m.identity_pose(), [m.Double3()]), list), "a list in gives a list out")
    expect_error(ErrorKind.Argument, m.polyline_length, np.zeros((2, 2)))
    check(m.polyline_length(np.zeros(3)) == 0.0, "a single row counts")
    hull = m.points2d_hull(np.array([[0.0, 0], [1, 0], [0, 1], [0.1, 0.1]]))
    check(isinstance(hull, np.ndarray) and hull.shape == (3, 2), "numpy from a list call")
    keep = m.points_downsample(np.array([[0.0, 0, 0], [0.01, 0, 0], [5, 5, 5]]), 1.0)
    check(isinstance(keep, np.ndarray) and keep.dtype == np.uint8 and keep.sum() == 2, "a numpy mask")
    pose = m.pose_apply_points(m.identity_pose(), np.zeros((0, 3)))
    check(pose.shape == (0, 3), "an empty array")
    # a numpy matrix in gives one out
    inv = m.matrix_inverse(np.array([[2.0, 0], [0, 4]]))
    check(isinstance(inv, np.ndarray) and inv[1, 1] == 0.25, "numpy matrix")
    u, s, v = m.matrix_svd(np.array([[3.0, 0], [0, 2]]))
    check(isinstance(s, np.ndarray) and abs(s[0] - 3.0) < 1e-12, "numpy svd")
    # a shape copies numpy too
    b = np.array([[0.0, 0, 0], [2, 0, 0]])
    line = m.shape_from_polyline(b)
    b[:] = 99.0
    check(abs(m.shape_distance(line, m.Double3(1, 1, 0)) - 1.0) < 1e-12, "a shape copies a numpy list")
    # options take numpy
    inliers = np.zeros(3, dtype=np.uint8)
    m.points_fit_plane(np.array([[0.0, 0, 0], [1, 0, 0], [0, 1, 0]]), m.FitOptions(inliers=inliers))
    check(inliers.sum() == 3, "numpy inliers")
    # images
    rows = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    image = m.image_from_numpy(rows, m.ImageFormat.Rgb8)
    check(m.image_color_at(image, 1, 1).r == rows[1, 1, 0], "image from numpy")
    back = m.image_to_numpy(image)
    check((back == rows).all(), "image to numpy")
    back[0, 0, 0] = 200
    check(image.data[0] == 200, "image to numpy shares its pixels")
    gray = m.image_from_numpy(np.zeros((2, 2), dtype=np.float32))
    check(gray.format == m.ImageFormat.Monof32, "a format from the array")
    expect_error(ErrorKind.Argument, m.image_from_numpy, rows)
    # clouds
    structured = np.zeros(2, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    structured["y"] = [1.0, 2.0]
    cloud = m.cloud_from_numpy(structured)
    check(m.point_cloud_read_points(cloud)[1].y == 2.0, "cloud from numpy")
    view = m.cloud_to_numpy(cloud)
    check(view["y"][0] == 1.0 and view.dtype.names == ("x", "y", "z", "intensity"), "cloud to numpy")
    # points
    pts = m.points_from_numpy(np.array([[1.0, 2, 3]]))
    check(pts[0] == m.Double3(1.0, 2.0, 3.0), "points from numpy")
    check((m.points_to_numpy(pts) == np.array([[1.0, 2, 3]])).all(), "points to numpy")
    poses = m.points_from_numpy(np.array([[1.0, 2, 3, 0, 0, 0, 1]]), m.Pose)
    check(poses[0].orientation.w == 1.0, "poses from numpy")
    check(m.points_to_numpy(poses).shape == (1, 7), "poses to numpy")


def main():
    replay(sys.argv[1])
    hand_tests()
    if failures:
        print(failures, "failures")
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
