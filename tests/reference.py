"""Writes reference.h: answers from numpy, scipy, OpenCV, matplotlib and mpmath that the C tests compare against.

Run with python tests/reference.py after changing a case. The output is committed so the
tests need no Python.
"""
import cv2
import matplotlib
import numpy as np
from numpy.polynomial import polynomial as P
from scipy.optimize import least_squares, linprog
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, HalfspaceIntersection, cKDTree
from scipy.spatial.transform import Rotation, Slerp
import mpmath
import os

rng = np.random.default_rng(20261030)
out = []


def arr(name, values, ctype="double"):
    flat = np.asarray(values, dtype=float).ravel()
    body = ",\n    ".join(", ".join(f"{v:.17g}" for v in flat[i:i + 4]) for i in range(0, len(flat), 4))
    out.append(f"static const {ctype} {name}[{len(flat)}] = {{\n    {body}\n}};")


# The enum order of RantEulerOrder: the twelve fixed orders, then the twelve moving ones.
ORDERS = ["xyz", "xzy", "yxz", "yzx", "zxy", "zyx", "xyx", "xzx", "yxy", "yzy", "zxz", "zyz"]
SEQS = ORDERS + [o.upper() for o in ORDERS]

N_EULER = 16
angles = rng.uniform(-np.pi, np.pi, size=(N_EULER, 3))
angles[:, 1] = rng.uniform(-1.4, 1.4, size=N_EULER)
from_euler = []
for seq in SEQS:
    proper = seq[0].lower() == seq[2].lower()
    for a in angles:
        b = a.copy()
        if proper:
            b[1] = abs(b[1]) + 0.1
        from_euler.append(list(b) + list(Rotation.from_euler(seq, b).as_quat()))
arr("REF_FROM_EULER", from_euler)
out.append(f"#define REF_EULER_CASES {N_EULER}")

quats = Rotation.random(N_EULER, random_state=7)
to_euler = []
for seq in SEQS:
    for q in quats:
        to_euler.append(list(q.as_quat()) + list(q.as_euler(seq)))
arr("REF_TO_EULER", to_euler)

N_ROT = 24
rots = Rotation.random(N_ROT, random_state=11)
rotvec = [list(r.as_quat()) + list(r.as_rotvec()) for r in rots]
arr("REF_ROTVEC", rotvec)
out.append(f"#define REF_ROT_CASES {N_ROT}")
mats = [list(r.as_quat()) + list(r.as_matrix().ravel()) for r in rots]
arr("REF_MATRIX", mats)

slerp_cases = []
for i in range(N_ROT // 2):
    a, b = rots[2 * i], rots[2 * i + 1]
    t = rng.uniform(0, 1)
    s = Slerp([0, 1], Rotation.concatenate([a, b]))(t)
    slerp_cases.append(list(a.as_quat()) + list(b.as_quat()) + [t] + list(s.as_quat()))
arr("REF_SLERP", slerp_cases)
out.append(f"#define REF_SLERP_CASES {N_ROT // 2}")

# Dense matrices: sizes and the values numpy gives.
def mat_case(name, m, n):
    a = rng.normal(size=(m, n))
    arr(f"{name}_A", a)
    return a

a = mat_case("REF_SVD", 7, 4)
arr("REF_SVD_S", np.linalg.svd(a, compute_uv=False))
a = mat_case("REF_SVD_WIDE", 3, 6)
arr("REF_SVD_WIDE_S", np.linalg.svd(a, compute_uv=False))
b = rng.normal(size=(6, 6))
sym = b + b.T
arr("REF_EIGEN_A", sym)
arr("REF_EIGEN_VALUES", np.linalg.eigvalsh(sym))
a = rng.normal(size=(5, 5))
arr("REF_INVERSE_A", a)
arr("REF_INVERSE", np.linalg.inv(a))
out.append(f"static const double REF_DETERMINANT = {np.linalg.det(a):.17g};")
a = rng.normal(size=(9, 3))
y = rng.normal(size=(9, 2))
arr("REF_LSQ_A", a)
arr("REF_LSQ_B", y)
arr("REF_LSQ_X", np.linalg.lstsq(a, y, rcond=None)[0])
spd = b @ b.T + 6 * np.eye(6)
arr("REF_CHOLESKY_A", spd)
arr("REF_CHOLESKY_L", np.linalg.cholesky(spd))

# Fits: noisy measurements, their weights, and what numpy and scipy fit to them.
N_FIT = 40
out.append(f"#define REF_FIT_COUNT {N_FIT}")
weights = rng.uniform(0.2, 2.0, size=N_FIT)
arr("REF_FIT_WEIGHTS", weights)
tight = dict(method="lm", xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=100000)


def polish(pts, w, p):
    """Gauss Newton steps on a circle or sphere fit, past where scipy's tolerances stop it."""
    root = np.sqrt(w)[:, None]
    for _ in range(20):
        d = pts - p[:-1]
        length = np.linalg.norm(d, axis=1)
        jac = np.hstack([-d / length[:, None], -np.ones((len(pts), 1))]) * root
        p = p - np.linalg.lstsq(jac, (length - p[-1]) * root[:, 0], rcond=None)[0]
    return p

normal = rng.normal(size=3)
normal /= np.linalg.norm(normal)
u = np.cross(normal, [0.3, 0.5, 0.8])
u /= np.linalg.norm(u)
v = np.cross(normal, u)
st = rng.uniform(-2, 2, size=(N_FIT, 2))
pts = np.array([4, -3, 2]) + st[:, :1] * u + st[:, 1:] * v + rng.normal(scale=0.02, size=(N_FIT, 1)) * normal
arr("REF_FIT_PLANE_POINTS", pts)
c = pts.mean(0)
arr("REF_FIT_PLANE", list(c) + list(np.linalg.svd(pts - c)[2][2]))

along = rng.uniform(-3, 5, size=(N_FIT, 1))
pts = np.array([1, 2, -1]) + along * u + rng.normal(scale=0.03, size=(N_FIT, 3))
arr("REF_FIT_LINE_POINTS", pts)
c = pts.mean(0)
arr("REF_FIT_LINE", list(c) + list(np.linalg.svd(pts - c)[2][0]))

angle = rng.uniform(0.0, 2.2, size=N_FIT)
pts = np.array([1.0, -2.0]) + 3.0 * np.stack([np.cos(angle), np.sin(angle)], 1) + rng.normal(scale=0.05, size=(N_FIT, 2))
arr("REF_FIT_CIRCLE_POINTS", pts)
for name, w in (("REF_FIT_CIRCLE", np.ones(N_FIT)), ("REF_FIT_CIRCLE_WEIGHTED", weights)):
    fit = least_squares(lambda p: np.sqrt(w) * (np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1]) - p[2]), [1.0, -2.0, 3.0], **tight)
    arr(name, polish(pts, w, fit.x))

polar = np.arccos(rng.uniform(np.cos(1.2), 1.0, size=N_FIT))
around = rng.uniform(0, 2 * np.pi, size=N_FIT)
dirs = np.stack([np.sin(polar) * np.cos(around), np.sin(polar) * np.sin(around), np.cos(polar)], 1)
pts = np.array([-1.0, 0.5, 2.0]) + 2.0 * dirs + rng.normal(scale=0.03, size=(N_FIT, 3))
arr("REF_FIT_SPHERE_POINTS", pts)
for name, w in (("REF_FIT_SPHERE", np.ones(N_FIT)), ("REF_FIT_SPHERE_WEIGHTED", weights)):
    fit = least_squares(lambda p: np.sqrt(w) * (np.linalg.norm(pts - p[:3], axis=1) - p[3]), [-1.0, 0.5, 2.0, 2.0], **tight)
    arr(name, polish(pts, w, fit.x))

turn = Rotation.random(random_state=13)
a = rng.uniform(-2, 2, size=(N_FIT, 3))
b = turn.apply(a) + np.array([0.5, -1.5, 3.0]) + rng.normal(scale=0.05, size=(N_FIT, 3))
arr("REF_FIT_POSE_FROM", a)
arr("REF_FIT_POSE_TO", b)
for name, w in (("REF_FIT_POSE", np.ones(N_FIT)), ("REF_FIT_POSE_WEIGHTED", weights)):
    ma, mb = np.average(a, axis=0, weights=w), np.average(b, axis=0, weights=w)
    r = Rotation.align_vectors(b - mb, a - ma, weights=w)[0]
    arr(name, list(mb - r.apply(ma)) + list(r.as_quat()))

x = rng.uniform(-1.0, 4.0, size=N_FIT)
y = 0.5 - 1.2 * x + 0.3 * x ** 2 + 0.05 * x ** 3 + rng.normal(scale=0.1, size=N_FIT)
arr("REF_FIT_POLY_X", x)
arr("REF_FIT_POLY_Y", y)
arr("REF_FIT_POLY", P.polyfit(x, y, 3))
arr("REF_FIT_POLY_WEIGHTED", P.polyfit(x, y, 3, w=np.sqrt(weights)))
x = rng.uniform(1000.0, 1010.0, size=N_FIT)
y = 2.0 + 0.01 * (x - 1004.0) ** 2 + rng.normal(scale=0.01, size=N_FIT)
arr("REF_FIT_POLY_FAR_X", x)
arr("REF_FIT_POLY_FAR_Y", y)
arr("REF_FIT_POLY_FAR_VALUES", P.polyval(x, P.polyfit(x, y, 2)))

# Colours: the HMI's OKLab mix written again here, linear light, and matplotlib's maps.
def to_byte(v):
    return 0 if not v > 0 else (255 if v >= 1 else int(np.floor(v * 255.0 + 0.5)))


def to_linear(v):
    return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4


def to_gamma(v):
    return 12.92 * max(v, 0.0) if v <= 0.0031308 else 1.055 * v ** (1 / 2.4) - 0.055


def mix(a, b, t):
    def lab(c):
        r, g, bl = (to_linear(x / 255.0) for x in c[:3])
        l = np.cbrt(0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * bl)
        m = np.cbrt(0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * bl)
        s = np.cbrt(0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * bl)
        return (0.2104542553 * l + 0.793617785 * m - 0.0040720468 * s, 1.9779984951 * l - 2.428592205 * m + 0.4505937099 * s,
                0.0259040371 * l + 0.7827717662 * m - 0.808675766 * s, c[3] / 255.0)
    p, q = lab(a), lab(b)
    L, A, B, al = (x + (y - x) * t for x, y in zip(p, q))
    l = (L + 0.3963377774 * A + 0.2158037573 * B) ** 3
    m = (L - 0.1055613458 * A - 0.0638541728 * B) ** 3
    s = (L - 0.0894841775 * A - 1.291485548 * B) ** 3
    return [to_byte(to_gamma(4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s)),
            to_byte(to_gamma(-1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s)),
            to_byte(to_gamma(-0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s)), to_byte(al)]


N_MIX = 200
mixes = []
for _ in range(N_MIX):
    a, b = rng.integers(0, 256, 4), rng.integers(0, 256, 4)
    t = rng.uniform(-0.25, 1.25)
    mixes.append(list(a) + list(b) + [t] + mix(a, b, t))
arr("REF_MIX", mixes)
out.append(f"#define REF_MIX_CASES {N_MIX}")
arr("REF_LINEAR", [to_linear(i / 255.0) for i in range(256)])

MAPS = ["gray", "viridis", "plasma", "inferno", "magma", "turbo"]
ts = list(np.arange(256) / 255.0) + list(rng.uniform(0, 1, 200)) + [-0.5, 0.0, 1e-12, 1 / 256, 1 - 1e-12, 1.0, 1.5]
arr("REF_MAP_T", ts)
out.append(f"#define REF_MAP_CASES {len(ts)}")
arr("REF_MAP", [[to_byte(v) for v in matplotlib.colormaps[name](t)[:3]] for name in MAPS for t in ts])

# Cameras: OpenCV's projections of points in its optical frame, x right, y down, z ahead.
CAMS = [
    (640, 480, 500.0, 510.0, 319.5, 239.5, 0, []),
    (1280, 720, 900.0, 905.0, 641.3, 358.2, 1, [-0.28, 0.09, 0.0012, -0.0007, -0.012]),
    (1024, 1024, 330.0, 331.0, 511.2, 512.9, 2, [0.05, -0.01, 0.002, -0.0003]),
    (1920, 1080, 1400.0, 1398.0, 962.1, 538.7, 3, [0.4, -0.1, 0.0008, 0.0011, 0.02, 0.7, -0.05, 0.03]),
]
N_CAM = 64
out.append(f"#define REF_CAM_COUNT {len(CAMS)}")
out.append(f"#define REF_CAM_CASES {N_CAM}")
arr("REF_CAMS", [list(c[:7]) + c[7] + [0.0] * (8 - len(c[7])) for c in CAMS])
cam_points, cam_pixels = [], []
for w, h, fx, fy, cx, cy, model, k in CAMS:
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    if model == 2:
        angle = rng.uniform(0, 1.35, N_CAM)
        turn = rng.uniform(0, 2 * np.pi, N_CAM)
        d = np.stack([np.sin(angle) * np.cos(turn), np.sin(angle) * np.sin(turn), np.cos(angle)], 1)
    else:
        d = np.stack([rng.uniform(-0.6, 0.6, N_CAM) * w / fx, rng.uniform(-0.6, 0.6, N_CAM) * h / fy, np.ones(N_CAM)], 1)
    pts = d * rng.uniform(0.3, 8.0, (N_CAM, 1))
    if model == 2:
        px = cv2.fisheye.projectPoints(pts.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, np.array(k))[0]
    else:
        px = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, np.array(k) if k else np.zeros(5))[0]
    cam_points.append(pts)
    cam_pixels.append(px.reshape(-1, 2))
arr("REF_CAM_POINTS", cam_points)
arr("REF_CAM_PIXELS", cam_pixels)

# Images: BT.601 limited range written out, OpenCV's own answer beside it, and resizes.
KR, KG, KB = 0.299, 0.587, 0.114


def from_yuv(y, u, v):
    l, pb, pr = (y - 16) * (255.0 / 219.0), (u - 128) * (255.0 / 224.0), (v - 128) * (255.0 / 224.0)
    return [u8(l + 2.0 * (1.0 - KR) * pr),
            u8(l - 2.0 * KB * (1.0 - KB) / KG * pb - 2.0 * KR * (1.0 - KR) / KG * pr), u8(l + 2.0 * (1.0 - KB) * pb)]


def u8(v):
    return 0 if not v > 0 else (255 if v >= 255 else int(np.floor(v + 0.5)))


def to_yuv(c):
    r, g, b = (float(x) for x in c)
    l = KR * r + KG * g + KB * b
    return 16.0 + l * (219.0 / 255.0), 128.0 + (b - l) * (224.0 / 255.0) / (2.0 * (1.0 - KB)), 128.0 + (r - l) * (224.0 / 255.0) / (2.0 * (1.0 - KR))


yuyv = rng.integers(0, 256, (4, 8, 2), dtype=np.uint8)
arr("REF_YUYV", yuyv, "uint8_t")
arr("REF_YUYV_RGB", [from_yuv(int(yuyv[y, x, 0]), int(yuyv[y, x & ~1, 1]), int(yuyv[y, x | 1, 1])) for y in range(4) for x in range(8)], "uint8_t")
arr("REF_YUYV_CV", cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUYV), "uint8_t")
nv12 = rng.integers(0, 256, (9, 8), dtype=np.uint8)
arr("REF_NV12", nv12, "uint8_t")
arr("REF_NV12_RGB", [from_yuv(int(nv12[y, x]), int(nv12[6 + y // 2, x & ~1]), int(nv12[6 + y // 2, x | 1])) for y in range(6) for x in range(8)], "uint8_t")
arr("REF_NV12_CV", cv2.cvtColor(nv12, cv2.COLOR_YUV2RGB_NV12), "uint8_t")
rgb = rng.integers(0, 256, (5, 7, 3), dtype=np.uint8)
arr("REF_RGB", rgb, "uint8_t")
arr("REF_RGB_GRAY", [u8(KR * float(p[0]) + KG * float(p[1]) + KB * float(p[2])) for p in rgb.reshape(-1, 3)], "uint8_t")
arr("REF_RGB_GRAY_CV", cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), "uint8_t")
yu = [[to_yuv(rgb[y, x]) for x in range(7)] for y in range(5)]
packed = []
for y in range(5):
    for x in range(0, 7, 2):
        pair = [yu[y][x]] + ([yu[y][x + 1]] if x + 1 < 7 else [])
        packed += [u8(pair[0][0]), u8(np.mean([p[1] for p in pair])), u8(pair[-1][0]), u8(np.mean([p[2] for p in pair]))]
arr("REF_RGB_YUYV", packed, "uint8_t")
planar = [u8(yu[y][x][0]) for y in range(5) for x in range(7)]
chroma = []
for y in range(0, 5, 2):
    for x in range(0, 7, 2):
        block = [yu[j][i] for j in (y, y + 1) if j < 5 for i in (x, x + 1) if i < 7]
        chroma += [u8(np.mean([p[1] for p in block])), u8(np.mean([p[2] for p in block]))]
arr("REF_RGB_NV12_Y", planar, "uint8_t")
arr("REF_RGB_NV12_UV", chroma, "uint8_t")


def axis_weights(n_from, n_to, linear):
    """The weights of each output pixel on the input ones, as the C resize defines them."""
    w = np.zeros((n_to, n_from))
    s = n_from / n_to
    for d in range(n_to):
        if not linear:
            w[d, min(int(np.floor((d + 0.5) * s)), n_from - 1)] = 1
        elif n_from > n_to:
            for i in range(n_from):
                w[d, i] = max(0.0, min((d + 1) * s, i + 1) - max(d * s, i))
        else:
            c = (d + 0.5) * s - 0.5
            f = c - np.floor(c)
            w[d, int(np.clip(np.floor(c), 0, n_from - 1))] += 1 - f
            w[d, int(np.clip(np.floor(c) + 1, 0, n_from - 1))] += f
    return w / w.sum(1, keepdims=True)


source = rng.integers(0, 256, (7, 9, 3), dtype=np.uint8)
arr("REF_RESIZE_SRC", source, "uint8_t")
SIZES = [(4, 3), (17, 11), (5, 14), (9, 7), (3, 2)]
out.append(f"#define REF_RESIZE_SIZES {len(SIZES)}")
arr("REF_RESIZE_DIMS", SIZES, "uint32_t")
for name, linear in (("NEAREST", False), ("LINEAR", True)):
    flat = []
    for w, h in SIZES:
        wx, wy = axis_weights(9, w, linear), axis_weights(7, h, linear)
        flat += list(np.einsum("yi,xj,ijc->yxc", wy, wx, source.astype(float)).ravel())
    arr(f"REF_RESIZE_{name}", flat)
arr("REF_RESIZE_CV_EXACT", np.concatenate([cv2.resize(source, s, interpolation=cv2.INTER_NEAREST_EXACT).ravel() for s in SIZES]), "uint8_t")
area = rng.integers(0, 256, (8, 12, 3), dtype=np.uint8)
arr("REF_AREA_SRC", area, "uint8_t")
arr("REF_AREA_CV", cv2.resize(area, (4, 4), interpolation=cv2.INTER_AREA), "uint8_t")
arr("REF_LINEAR_CV", cv2.resize(area, (19, 13), interpolation=cv2.INTER_LINEAR), "uint8_t")

# Point sets: a scene of blobs, a plane patch, scattered points and exact twins, and what numpy
# and scipy make of it by the rules the C calls state.
blobs = [rng.normal(c, s, (n, 3)) for c, s, n in (((0, 0, 1), 0.15, 150), ((2, 1, 0.5), 0.1, 100), ((-1.5, 2, 2), 0.2, 60))]
patch = np.column_stack([rng.uniform(-1, 1, 200), rng.uniform(-3, -1, 200), rng.normal(0, 0.004, 200)])
scatter = rng.uniform(-4, 4, (40, 3))
cloud = np.concatenate(blobs + [patch, scatter])
cloud = np.concatenate([cloud, cloud[rng.choice(len(cloud), 6, replace=False)]])
arr("REF_CLOUD", cloud)
out.append(f"#define REF_CLOUD_POINTS {len(cloud)}")
tree = cKDTree(cloud)

VOXEL = 0.3
keys = np.floor(cloud / VOXEL)
keep = np.zeros(len(cloud), dtype=np.uint8)
for key in np.unique(keys, axis=0):
    idx = np.flatnonzero((keys == key).all(1))
    mean = cloud[idx[0]] + (cloud[idx] - cloud[idx[0]]).sum(0) / len(idx)
    d = ((cloud[idx] - mean) ** 2).sum(1)
    keep[idx[np.argmin(d)]] = 1
out.append(f"#define REF_VOXEL {VOXEL!r}")
arr("REF_DOWNSAMPLE", keep, "uint8_t")

OUTLIER_K, OUTLIER_STD = 8, 1.0
dist, _ = tree.query(cloud, OUTLIER_K + 1)
mean = dist[:, 1:].mean(1)
arr("REF_OUTLIERS", mean <= mean.mean() + OUTLIER_STD * mean.std(ddof=1), "uint8_t")
out.append(f"#define REF_OUTLIER_K {OUTLIER_K}")

ISOLATED_R, ISOLATED_N = 0.25, 3
counts = np.array([len(tree.query_ball_point(p, ISOLATED_R)) - 1 for p in cloud])
arr("REF_ISOLATED", counts >= ISOLATED_N, "uint8_t")
out.append(f"#define REF_ISOLATED_R {ISOLATED_R!r}")

NORMAL_K = 10
VIEW = np.array([0.5, -0.5, 6.0])
_, near = tree.query(cloud, NORMAL_K)
normals = []
for i, idx in enumerate(near):
    q = cloud[idx] - cloud[idx].mean(0)
    n = np.linalg.eigh(q.T @ q)[1][:, 0]
    normals.append(-n if n @ (VIEW - cloud[i]) < 0 else n)
arr("REF_NORMALS", normals)
arr("REF_VIEW", VIEW)

CLUSTER_D, CLUSTER_MIN = 0.2, 5
pairs = np.array(sorted(tree.query_pairs(CLUSTER_D)))
graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(cloud), len(cloud)))
_, comp = connected_components(graph, directed=False)
groups = sorted(range(comp.max() + 1), key=lambda g: (-(comp == g).sum(), np.flatnonzero(comp == g)[0]))
rank = {g: (k + 1 if (comp == g).sum() >= CLUSTER_MIN else 0) for k, g in enumerate(groups)}
arr("REF_CLUSTERS", [rank[g] for g in comp], "uint32_t")
out.append(f"#define REF_CLUSTER_COUNT {sum(1 for g in groups if (comp == g).sum() >= CLUSTER_MIN)}")

# Places on the Earth: WGS84 worked at 40 digits.
mpmath.mp.dps = 40
WA = mpmath.mpf(6378137)
WF = 1 / mpmath.mpf("298.257223563")
WE2 = WF * (2 - WF)


def to_ecef(lat, lon, alt):
    p, l = mpmath.radians(lat), mpmath.radians(lon)
    n = WA / mpmath.sqrt(1 - WE2 * mpmath.sin(p) ** 2)
    return [(n + alt) * mpmath.cos(p) * mpmath.cos(l), (n + alt) * mpmath.cos(p) * mpmath.sin(l),
            (n * (1 - WE2) + alt) * mpmath.sin(p)]


def from_ecef(x, y, z):
    x, y, z = mpmath.mpf(x), mpmath.mpf(y), mpmath.mpf(z)
    r = mpmath.hypot(x, y)
    phi = mpmath.atan2(z, r * (1 - WE2))
    for _ in range(200):
        n = WA / mpmath.sqrt(1 - WE2 * mpmath.sin(phi) ** 2)
        h = r / mpmath.cos(phi) - n if abs(mpmath.cos(phi)) > abs(mpmath.sin(phi)) else z / mpmath.sin(phi) - n * (1 - WE2)
        phi = mpmath.atan2(z, r * (1 - WE2 * n / (n + h)))
    return [mpmath.degrees(phi), mpmath.degrees(mpmath.atan2(y, x)), h]


geo = []
for k in range(60):
    lat, lon = rng.uniform(-90, 90), rng.uniform(-180, 180)
    alt = [rng.uniform(-1e4, 1e4), rng.uniform(-1e5, 4e5), 10 ** rng.uniform(4, 7.5)][k % 3]
    e = [float(v) for v in to_ecef(lat, lon, alt)]
    geo.append([lat, lon, alt] + e + [float(v) for v in from_ecef(*e)])
arr("REF_GEO", geo)
out.append(f"#define REF_GEO_CASES {len(geo)}")

enu = []
for k in range(30):
    o = [rng.uniform(-89, 89), rng.uniform(-180, 180), rng.uniform(-100, 3000)]
    p = [o[0] + rng.normal(0, 0.05), o[1] + rng.normal(0, 0.05), o[2] + rng.normal(0, 200)]
    a, b = to_ecef(*o), to_ecef(*p)
    pl, ll = mpmath.radians(o[0]), mpmath.radians(o[1])
    d = [b[i] - a[i] for i in range(3)]
    east = -mpmath.sin(ll) * d[0] + mpmath.cos(ll) * d[1]
    north = -mpmath.sin(pl) * mpmath.cos(ll) * d[0] - mpmath.sin(pl) * mpmath.sin(ll) * d[1] + mpmath.cos(pl) * d[2]
    up = mpmath.cos(pl) * mpmath.cos(ll) * d[0] + mpmath.cos(pl) * mpmath.sin(ll) * d[1] + mpmath.sin(pl) * d[2]
    enu.append(o + p + [float(east), float(north), float(up)])
arr("REF_ENU", enu)
out.append(f"#define REF_ENU_CASES {len(enu)}")


# Boxes: the volume two boxes share, from qhull's intersection of their half spaces.
def shared(halves, dim):
    a, b = halves[:, :dim], halves[:, dim]
    norm = np.linalg.norm(a, axis=1)
    lp = linprog(np.r_[np.zeros(dim), -1], A_ub=np.c_[a, norm], b_ub=-b, bounds=[(None, None)] * dim + [(0, None)])
    if lp.status != 0 or lp.x[-1] < 1e-9:
        return 0.0
    return ConvexHull(HalfspaceIntersection(halves, lp.x[:dim]).intersections).volume


def box_halves(center, axes, size):
    rows = []
    for k in range(len(size)):
        rows.append(np.r_[axes[:, k], -axes[:, k] @ center - size[k] / 2])
        rows.append(np.r_[-axes[:, k], axes[:, k] @ center - size[k] / 2])
    return np.array(rows)


iou2 = []
for k in range(40):
    boxes = [(rng.uniform(-1, 1, 2), rng.uniform(-np.pi, np.pi), rng.uniform(0.4, 2.5, 2)) for _ in range(2)]
    halves = []
    for c, ang, s in boxes:
        axes = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        halves.append(box_halves(c, axes, s))
    both = shared(np.concatenate(halves), 2)
    either = sum(s.prod() for _, _, s in boxes) - both
    iou2.append([v for c, ang, s in boxes for v in (*c, ang, *s)] + [both / either])
arr("REF_IOU2D", iou2)
out.append(f"#define REF_IOU2D_CASES {len(iou2)}")

iou3 = []
turns = Rotation.random(80, random_state=31)
for k in range(40):
    boxes = [(rng.uniform(-1, 1, 3), turns[2 * k + j], rng.uniform(0.4, 2.5, 3)) for j in range(2)]
    halves = np.concatenate([box_halves(c, r.as_matrix(), s) for c, r, s in boxes])
    both = shared(halves, 3)
    either = sum(s.prod() for _, _, s in boxes) - both
    iou3.append([v for c, r, s in boxes for v in (*c, *r.as_quat(), *s)] + [both / either])
arr("REF_IOU3D", iou3)
out.append(f"#define REF_IOU3D_CASES {len(iou3)}")

here = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(here, "reference.h"), "w", newline="\n") as f:
    f.write("/* GENERATED by reference.py from numpy, scipy, OpenCV and matplotlib. DO NOT EDIT. */\n")
    f.write("#ifndef RANT_MATH_REFERENCE_H\n#define RANT_MATH_REFERENCE_H\n\n")
    f.write("\n\n".join(out))
    f.write("\n\n#endif\n")
print("wrote reference.h")
