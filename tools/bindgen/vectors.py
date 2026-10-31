# The proof every binding shares: the inputs of each case, and a C program that calls the library
# with them and writes down what comes back. Each binding must answer the same, errors and all.
import json
import math
import struct

CASES_PER_FUNCTION = 4
FLOAT_TOLERANCE = 1e-5
DOUBLE_TOLERANCE = 1e-9


def d2(x, y): return {"x": x, "y": y}
def d3(x, y, z): return {"x": x, "y": y, "z": z}
def d4(x, y, z, w): return {"x": x, "y": y, "z": z, "w": w}


QUATS = [d4(0.0, 0.0, 0.0, 1.0), d4(0.0, 0.0, 0.6, 0.8), d4(0.5, 0.5, 0.5, 0.5),
         d4(0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214)]
V3 = [d3(1.0, 2.0, 3.0), d3(-0.5, 0.25, 4.0), d3(2.0, -3.0, 0.5), d3(0.0, 0.0, 0.0)]
V2 = [d2(1.0, 2.0), d2(-0.5, 0.25), d2(3.0, -4.0), d2(0.0, 0.0)]
POSES = [{"position": V3[i], "orientation": QUATS[i]} for i in range(4)]
POSES2 = [{"position": V2[i], "angle": a} for i, a in enumerate([0.0, 0.5, -2.5, 1.5707963267948966])]


def m3(rows): return {"m": rows}


# What a type is asked with, cycled through. A value that makes a call fail belongs here too.
POOLS = {
    "RantDouble2": V2,
    "RantDouble3": V3,
    "RantDouble4": [d4(1.0, 2.0, 3.0, 4.0), d4(-0.5, 0.25, 0.0, 1.0), d4(0.0, 0.0, 0.0, 0.0), d4(2.0, -1.0, 0.5, 0.25)],
    "RantFloat2": [d2(1.5, -2.5), d2(0.25, 0.5), d2(0.0, 0.0), d2(100.0, -0.125)],
    "RantFloat3": [d3(1.5, -2.5, 0.75), d3(0.25, 0.5, 8.0), d3(0.0, 0.0, 0.0), d3(100.0, -0.125, 4.0)],
    "RantFloat4": [d4(1.0, 0.5, 0.25, 1.0), d4(0.25, 0.5, 0.75, 0.5), d4(0.0, 0.0, 0.0, 0.0), d4(2.0, -1.0, 0.5, 1.0)],
    "RantInt2": [d2(1, 2), d2(-3, 4), d2(0, 0), d2(100, -200)],
    "RantInt3": [d3(1, 2, 0), d3(-3, 4, 0), d3(0, 0, 0), d3(2, 1, 0)],
    "RantQuaternion": QUATS,
    "RantPose": POSES,
    "RantPose2D": POSES2,
    "RantMatrix3x3": [
        m3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        m3([[2.0, 1.0, 0.0], [0.0, 3.0, 1.0], [1.0, 0.0, 4.0]]),
        m3([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 1.0, 1.0]]),
        m3([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
    ],
    "RantMatrix4x4": [
        m3([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
        m3([[0.0, -1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 2.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]),
        m3([[1.0, 2.0, 3.0, 4.0], [2.0, 4.0, 6.0, 8.0], [1.0, 1.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0]]),
        m3([[2.0, 0.0, 0.0, 0.5], [0.0, 2.0, 0.0, 0.5], [0.0, 0.0, 2.0, 0.5], [0.0, 0.0, 0.5, 1.0]]),
    ],
    "RantCovariance3": [
        m3([[1.0, 0.1, 0.0], [0.1, 2.0, 0.2], [0.0, 0.2, 3.0]]),
        m3([[0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.5]]),
    ],
    "RantCovariance6": [
        m3([[float(i == j) * (i + 1) + 0.01 * (i + j) for j in range(6)] for i in range(6)]),
        m3([[float(i == j) for j in range(6)] for i in range(6)]),
    ],
    "RantPlane": [
        {"position": d3(0.0, 0.0, 1.0), "normal": d3(0.0, 0.0, 1.0)},
        {"position": d3(1.0, 2.0, 3.0), "normal": d3(0.6, 0.0, 0.8)},
        {"position": d3(0.0, 0.0, 0.0), "normal": d3(1.0, 0.0, 0.0)},
        {"position": d3(0.0, 1.0, 0.0), "normal": d3(0.0, 1.0, 0.0)},
    ],
    "RantPlane2D": [
        {"position": d2(0.0, 1.0), "normal": d2(0.0, 1.0)},
        {"position": d2(1.0, 2.0), "normal": d2(0.6, 0.8)},
        {"position": d2(0.0, 0.0), "normal": d2(1.0, 0.0)},
        {"position": d2(4.0, 0.0), "normal": d2(1.0, 0.0)},
    ],
    "RantRay": [
        {"origin": d3(-5.0, 0.25, 0.5), "direction": d3(1.0, 0.0, 0.0)},
        {"origin": d3(0.5, 0.25, 9.0), "direction": d3(0.0, 0.0, -1.0)},
        {"origin": d3(0.0, 40.0, 0.0), "direction": d3(1.0, 0.0, 0.0)},
        {"origin": d3(4.0, 5.0, 6.0), "direction": d3(-0.5, -0.5, -0.7071067811865476)},
    ],
    "RantRay2D": [
        {"origin": d2(-5.0, 0.25), "direction": d2(1.0, 0.0)},
        {"origin": d2(0.5, 9.0), "direction": d2(0.0, -1.0)},
        {"origin": d2(0.0, 40.0), "direction": d2(1.0, 0.0)},
        {"origin": d2(4.0, 5.0), "direction": d2(-0.6, -0.8)},
    ],
    "RantSegment": [
        {"a": d3(0.0, 0.0, 0.0), "b": d3(1.0, 2.0, 3.0)},
        {"a": d3(-1.0, 1.0, 0.5), "b": d3(2.0, 1.0, -0.5)},
        {"a": d3(1.0, 1.0, 1.0), "b": d3(1.0, 1.0, 1.0)},
        {"a": d3(3.0, 0.0, 0.0), "b": d3(3.0, 4.0, 0.0)},
    ],
    "RantSegment2D": [
        {"a": d2(0.0, 0.0), "b": d2(2.0, 2.0)},
        {"a": d2(0.0, 2.0), "b": d2(2.0, 0.0)},
        {"a": d2(5.0, 5.0), "b": d2(6.0, 5.0)},
        {"a": d2(1.0, 1.0), "b": d2(1.0, 1.0)},
    ],
    "RantAlignedBox": [
        {"min": d3(-1.0, -1.0, -1.0), "max": d3(1.0, 1.0, 1.0)},
        {"min": d3(0.0, 0.0, 0.0), "max": d3(2.0, 3.0, 4.0)},
        {"min": d3(5.0, 5.0, 5.0), "max": d3(6.0, 7.0, 8.0)},
        {"min": d3(-2.0, 0.5, 0.0), "max": d3(0.5, 2.0, 1.0)},
    ],
    "RantAlignedBox2D": [
        {"min": d2(-1.0, -1.0), "max": d2(1.0, 1.0)},
        {"min": d2(0.0, 0.0), "max": d2(2.0, 3.0)},
        {"min": d2(5.0, 5.0), "max": d2(6.0, 7.0)},
        {"min": d2(-2.0, 0.5), "max": d2(0.5, 2.0)},
    ],
    "RantOrientedBox": [
        {"pose": POSES[3], "size": d3(1.0, 2.0, 3.0)},
        {"pose": POSES[0], "size": d3(2.0, 2.0, 2.0)},
        {"pose": POSES[1], "size": d3(1.0, 1.0, 4.0)},
        {"pose": POSES[2], "size": d3(-1.0, 2.0, 0.5)},
    ],
    "RantOrientedBox2D": [
        {"pose": POSES2[0], "size": d2(2.0, 2.0)},
        {"pose": POSES2[1], "size": d2(1.0, 3.0)},
        {"pose": POSES2[2], "size": d2(4.0, 0.5)},
        {"pose": POSES2[3], "size": d2(-1.0, 2.0)},
    ],
    "RantSphere": [
        {"center": d3(0.0, 0.0, 0.0), "radius": 1.0},
        {"center": d3(1.0, 2.0, 3.0), "radius": 2.5},
        {"center": d3(-4.0, 0.0, 1.0), "radius": 0.5},
        {"center": d3(0.0, 40.0, 0.0), "radius": 1.0},
    ],
    "RantCircle": [
        {"center": d2(0.0, 0.0), "radius": 1.0},
        {"center": d2(1.0, 2.0), "radius": 2.5},
        {"center": d2(-4.0, 0.0), "radius": 0.5},
        {"center": d2(0.0, 40.0), "radius": 1.0},
    ],
    "RantCapsule": [
        {"axis": {"a": d3(-1.0, 0.0, 0.0), "b": d3(1.0, 0.0, 0.0)}, "radius": 0.5},
        {"axis": {"a": d3(0.0, 0.0, 0.0), "b": d3(0.0, 0.0, 3.0)}, "radius": 1.0},
        {"axis": {"a": d3(4.0, 4.0, 4.0), "b": d3(5.0, 4.0, 4.0)}, "radius": 0.25},
        {"axis": {"a": d3(0.0, 40.0, 0.0), "b": d3(1.0, 40.0, 1.0)}, "radius": 0.5},
    ],
    "RantCapsule2D": [
        {"axis": {"a": d2(-1.0, 0.0), "b": d2(1.0, 0.0)}, "radius": 0.5},
        {"axis": {"a": d2(0.0, 0.0), "b": d2(0.0, 3.0)}, "radius": 1.0},
        {"axis": {"a": d2(4.0, 4.0), "b": d2(5.0, 4.0)}, "radius": 0.25},
        {"axis": {"a": d2(0.0, 40.0), "b": d2(1.0, 40.0)}, "radius": 0.5},
    ],
    "RantCylinder": [
        {"axis": {"a": d3(0.0, 0.0, -1.0), "b": d3(0.0, 0.0, 1.0)}, "radius": 1.0},
        {"axis": {"a": d3(-1.0, 0.25, 0.5), "b": d3(2.0, 0.25, 0.5)}, "radius": 0.5},
        {"axis": {"a": d3(4.0, 4.0, 4.0), "b": d3(5.0, 5.0, 5.0)}, "radius": 0.25},
        {"axis": {"a": d3(0.0, 40.0, 0.0), "b": d3(0.0, 41.0, 0.0)}, "radius": 2.0},
    ],
    "RantCone": [
        {"base": d3(0.0, 0.0, 0.0), "tip": d3(0.0, 0.0, 2.0), "radius": 1.0},
        {"base": d3(-1.0, 0.25, 0.5), "tip": d3(2.0, 0.25, 0.5), "radius": 0.5},
        {"base": d3(4.0, 4.0, 4.0), "tip": d3(5.0, 5.0, 5.0), "radius": 0.25},
        {"base": d3(0.0, 40.0, 0.0), "tip": d3(0.0, 41.0, 0.0), "radius": 2.0},
    ],
    "RantTwist": [
        {"linear": d3(1.0, 0.0, 0.5), "angular": d3(0.0, 0.0, 0.25)},
        {"linear": d3(0.0, 2.0, 0.0), "angular": d3(0.5, 0.0, 0.0)},
        {"linear": d3(0.0, 0.0, 0.0), "angular": d3(0.0, 0.0, 0.0)},
        {"linear": d3(-1.0, 1.0, 1.0), "angular": d3(0.25, 0.25, 0.25)},
    ],
    "RantTwist2D": [
        {"linear": d2(1.0, 0.5), "angular": 0.25},
        {"linear": d2(0.0, 2.0), "angular": 0.5},
        {"linear": d2(0.0, 0.0), "angular": 0.0},
        {"linear": d2(-1.0, 1.0), "angular": -0.25},
    ],
    "RantWrench": [
        {"force": d3(1.0, 0.0, 0.5), "torque": d3(0.0, 0.0, 0.25)},
        {"force": d3(0.0, 2.0, 0.0), "torque": d3(0.5, 0.0, 0.0)},
        {"force": d3(0.0, 0.0, 0.0), "torque": d3(0.0, 0.0, 0.0)},
        {"force": d3(-1.0, 1.0, 1.0), "torque": d3(0.25, 0.25, 0.25)},
    ],
    "RantWrench2D": [
        {"force": d2(1.0, 0.5), "torque": 0.25},
        {"force": d2(0.0, 2.0), "torque": 0.5},
        {"force": d2(0.0, 0.0), "torque": 0.0},
        {"force": d2(-1.0, 1.0), "torque": -0.25},
    ],
    "RantAccel": [
        {"linear": d3(1.0, 0.0, 9.8), "angular": d3(0.0, 0.0, 0.25)},
        {"linear": d3(0.0, 2.0, 0.0), "angular": d3(0.5, 0.0, 0.0)},
    ],
    "RantInertia": [
        {"mass": 2.0, "center": d3(0.1, 0.0, 0.0), "ixx": 1.0, "ixy": 0.0, "ixz": 0.0, "iyy": 2.0, "iyz": 0.0, "izz": 3.0},
        {"mass": 0.5, "center": d3(0.0, 0.5, 0.25), "ixx": 0.2, "ixy": 0.01, "ixz": 0.02, "iyy": 0.3, "iyz": 0.03, "izz": 0.4},
        {"mass": 4.0, "center": d3(1.0, 0.0, 0.5), "ixx": 2.0, "ixy": 0.0, "ixz": 0.5, "iyy": 2.0, "iyz": 0.0, "izz": 1.0},
    ],
    "RantAxes": [
        {"x": -2, "y": 3, "z": 1, "scale": 1.0},
        {"x": 1, "y": -2, "z": 3, "scale": 100.0},
        {"x": -2, "y": -3, "z": 1, "scale": 1.0},
        {"x": 1, "y": 2, "z": 3, "scale": 0.5},
    ],
    "RantColor": [
        {"r": 255, "g": 0, "b": 0, "a": 255},
        {"r": 10, "g": 200, "b": 30, "a": 128},
        {"r": 0, "g": 0, "b": 0, "a": 0},
        {"r": 255, "g": 255, "b": 255, "a": 255},
    ],
    "RantGeoPoint": [
        {"lat": 51.5, "lon": -0.125, "alt": 35.0},
        {"lat": -33.25, "lon": 151.5, "alt": 10.0},
        {"lat": 90.0, "lon": 0.0, "alt": 0.0},
        {"lat": 0.0, "lon": 180.0, "alt": 1000.0},
    ],
    "RantCameraIntrinsics": [
        {"width": 640, "height": 480, "fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0, "coeffs": [0.0] * 8, "model": 0},
        {"width": 1280, "height": 720, "fx": 900.0, "fy": 910.0, "cx": 640.0, "cy": 360.0,
         "coeffs": [-0.1, 0.05, 0.001, -0.002, 0.0, 0.0, 0.0, 0.0], "model": 1},
        {"width": 800, "height": 600, "fx": 300.0, "fy": 300.0, "cx": 400.0, "cy": 300.0,
         "coeffs": [0.01, -0.002, 0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], "model": 2},
        {"width": 640, "height": 480, "fx": 0.0, "fy": 500.0, "cx": 320.0, "cy": 240.0, "coeffs": [0.0] * 8, "model": 0},
    ],
    "RantEulerOrder": [0, 5, 13, 22],
    "RantAxesPreset": [0, 1, 6, 8],
    "RantColormap": [0, 1, 5, 3],
    "RantImageFormat": [0, 2, 3, 7],
    "RantImageFilter": [0, 1],
    "RantPointCloudLayout": [0, 1, 2, 3],
    "double": [0.5, -1.25, 2.0, 0.0],
    "float": [0.5, -1.25, 2.0, 0.0],
    "int": [1, -1, 2, 0],
    "uint32_t": [1, 3, 0, 2],
    "uint64_t": [1, 42, 0, 7],
    "char": ["x", "intensity", "missing", "z"],
}

# A name says more than its type, where the plain pool would only ever fail.
NAMED = {
    ("double", "t"): [0.25, 0.5, 1.0, 0.0],
    ("double", "mass"): [2.0, 0.5, 1.0, 0.0],
    ("double", "dt"): [0.1, 0.5, 1.0, 0.0],
    ("double", "margin"): [0.25, 1.0, 0.0, -0.1],
    ("double", "distance"): [1.0, 0.5, 100.0, -1.0],
    ("double", "tolerance"): [0.1, 0.5, 0.01, 0.0],
    ("double", "radius"): [1.5, 0.5, 10.0, 0.0],
    ("double", "depth"): [2.0, 0.5, 10.0, -1.0],
    ("double", "iou"): [0.5, 0.1, 0.9, 0.0],
    ("double", "voxel"): [0.5, 2.0, 100.0, 0.0],
    ("double", "deviations"): [1.0, 2.0, 0.5, 0.0],
    ("double", "width"): [0.0, 1.0, 2.5, 0.5],
    ("double", "low"): [0.0, -1.0, 0.0, 10.0],
    ("double", "high"): [1.0, 1.0, 255.0, 10.0],
    ("double", "scale"): [1.0, 100.0, 0.5, 0.0],
    ("uint32_t", "count"): [5, 2, 1, 0],
    ("uint32_t", "neighbors"): [3, 4, 1, 0],
    ("uint32_t", "min_points"): [2, 1, 3, 100],
    ("uint32_t", "segments"): [8, 0, 3, 2],
    ("uint32_t", "width"): [3, 2, 1, 0],
    ("uint32_t", "height"): [2, 3, 1, 0],
    ("uint32_t", "degree"): [2, 1, 0, 9],
    ("int", "x"): [-2, 1, 1, 4],
    ("int", "y"): [3, -2, 2, 2],
    ("int", "z"): [1, 3, 3, 3],
    ("int", "quarter_turns"): [1, 2, -1, 7],
    ("RantDouble2", "fov"): [d2(1.5, 1.0), d2(1.0, 0.0), d2(0.0, 0.0), d2(4.0, 1.0)],
    ("RantDouble2", "pixel"): [d2(320.0, 240.0), d2(100.5, 50.25), d2(-10.0, 0.0), d2(1000.0, 2000.0)],
    ("RantDouble3", "onto"): [d3(1.0, 0.0, 0.0), d3(0.0, 2.0, 0.0), d3(0.0, 0.0, 0.0), d3(1.0, 1.0, 1.0)],
    ("RantDouble3", "normal"): [d3(0.0, 0.0, 1.0), d3(1.0, 0.0, 0.0), d3(0.6, 0.8, 0.0), d3(0.0, 1.0, 0.0)],
    ("RantDouble2", "normal"): [d2(0.0, 1.0), d2(1.0, 0.0), d2(0.6, 0.8), d2(0.0, -1.0)],
    ("RantDouble2", "onto"): [d2(1.0, 0.0), d2(0.0, 2.0), d2(0.0, 0.0), d2(1.0, 1.0)],
    ("RantDouble3", "point"): [d3(0.5, 0.25, 0.125), d3(4.0, 5.0, 6.0), d3(-1.5, 0.0, 0.5), d3(0.0, 0.0, 0.0)],
    ("RantDouble2", "point"): [d2(0.5, 0.25), d2(4.0, 5.0), d2(-1.5, 0.0), d2(0.0, 0.0)],
    ("RantDouble3", "viewpoint"): [d3(0.0, 0.0, 10.0), d3(5.0, 5.0, 5.0)],
}

CLOUD8 = [d3(0.0, 0.0, 0.0), d3(1.0, 0.0, 0.1), d3(0.0, 1.0, -0.1), d3(1.0, 1.0, 0.05),
          d3(0.5, 0.5, 2.0), d3(2.0, 0.5, 1.0), d3(0.5, 2.0, 0.5), d3(-1.0, -1.0, 0.25)]
FLAT6 = [d3(float(x), float(y), 0.5) for x, y in [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1.5)]]
LINE4 = [d3(float(i), 2.0 * i, -1.0 * i) for i in range(4)]
SPHERE6 = [d3(2.0, 0.0, 0.0), d3(-2.0, 0.0, 0.0), d3(0.0, 2.0, 0.0), d3(0.0, -2.0, 0.0), d3(0.0, 0.0, 2.0), d3(0.0, 0.0, -2.0)]
NAN_THREE = [d3(0.0, 0.0, 0.0), d3("nan", 1.0, 2.0), d3(1.0, 1.0, 1.0)]

CLOUD2 = [d2(0.0, 0.0), d2(2.0, 0.0), d2(2.0, 1.0), d2(1.0, 0.5), d2(0.0, 1.0), d2(1.0, 3.0)]
LINE2 = [d2(float(i), 2.0 * i) for i in range(4)]

LISTS = {
    "RantDouble3": [CLOUD8, LINE4, [], FLAT6],
    "RantDouble2": [CLOUD2, LINE2, [], [d2(1.0, 1.0)]],
    "RantPose": [POSES[:3], [POSES[1]], [], POSES],
    "RantPose2D": [POSES2[:3], [POSES2[1]], [], POSES2],
    "double": [[1.0, 2.0, 0.5, 4.0], [0.5], [], [1.0, -1.0]],
    "uint32_t": [[0, 1, 2], [0, 2, 1, 1, 2, 3], [], [0, 1, 99]],
    "uint8_t": [[1, 0, 1], [1, 1, 1], [0, 0, 0], [1, 0, 0]],
    "RantColor": [POOLS["RantColor"][:3], POOLS["RantColor"], [], POOLS["RantColor"][:1]],
}

SQUARE = [d2(-1.0, -1.0), d2(1.0, -1.0), d2(1.0, 1.0), d2(-1.0, 1.0)]
ELL = [d2(0.0, 0.0), d2(3.0, 0.0), d2(3.0, 1.0), d2(1.0, 1.0), d2(1.0, 3.0), d2(0.0, 3.0)]
CLOCKWISE = list(reversed(SQUARE))
FLAT_LINE = [d2(0.0, 0.0), d2(1.0, 1.0), d2(2.0, 2.0)]
POLYGON3 = [d3(0.0, 0.0, 1.0), d3(2.0, 0.0, 1.0), d3(2.0, 2.0, 1.5), d3(0.0, 2.0, 1.5)]
TRIANGLE3 = [d3(1.0, 0.0, 0.0), d3(0.0, 1.0, 0.0), d3(0.0, 0.0, 1.0)]
LINE3 = [d3(0.0, 0.0, 0.0), d3(1.0, 1.0, 1.0), d3(2.0, 2.0, 2.0)]
ZIGZAG3 = [d3(0.0, 0.0, 0.0), d3(1.0, 0.0, 0.0), d3(1.0, 1.0, 0.0), d3(2.0, 1.0, 1.0), d3(2.0, 2.0, 1.01)]
ZIGZAG2 = [d2(0.0, 0.0), d2(1.0, 0.0), d2(1.0, 1.0), d2(2.0, 1.0), d2(2.0, 1.01), d2(3.0, 2.0)]

BY_PREFIX = {
    "polygon2d_": [ELL, SQUARE, FLAT_LINE, []],
    "polygon_": [POLYGON3, TRIANGLE3, LINE3, []],
    "polyline_": [ZIGZAG3, LINE3, [d3(1.0, 2.0, 3.0)], []],
    "polyline2d_": [ZIGZAG2, FLAT_LINE, [d2(1.0, 2.0)], []],
}

CUBE_V = [d3(float(x), float(y), float(z)) for z in (0, 1) for y in (0, 1) for x in (0, 1)]
CUBE_T = [0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5]
TETRA_V = [d3(0.0, 0.0, 0.0), d3(2.0, 0.0, 0.0), d3(0.0, 2.0, 0.0), d3(0.0, 0.0, 2.0)]
TETRA_T = [0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2]
MESHES = [
    (CUBE_V, CUBE_T),
    (TETRA_V, TETRA_T),
    (TRIANGLE3, [0, 1, 2]),
    (TETRA_V, [0, 1, 9]),
]

PRISMS = [
    {"pose": {"position": V3[3], "orientation": QUATS[0]}, "base": {"points": SQUARE}, "height": 2.0},
    {"pose": POSES[3], "base": {"points": ELL}, "height": 1.5},
    {"pose": POSES[1], "base": {"points": CLOCKWISE}, "height": 0.5},
    {"pose": POSES[0], "base": {"points": FLAT_LINE}, "height": 1.0},
]


def hexof(data):
    return bytes(data).hex()


def rgb(w, h, c=3, offset=0):
    return hexof((offset + (i * 7)) % 256 for i in range(w * h * c))


def image(w, h, fmt, data, stride=0):
    return {"width": w, "height": h, "stride": stride, "format": fmt, "data": data}


def floats(values):
    return struct.pack("<%df" % len(values), *values).hex()


def u16s(values):
    return struct.pack("<%dH" % len(values), *values).hex()


RGB = image(4, 3, 2, rgb(4, 3))
RGBA = image(3, 2, 3, rgb(3, 2, 4))
BGR = image(2, 2, 4, rgb(2, 2))
MONO = image(4, 3, 0, rgb(4, 3, 1))
MONO16 = image(2, 2, 1, u16s([0, 1000, 2500, 65535]))
MONOF = image(2, 2, 7, floats([0.0, 0.5, 1.5, float("nan")]))
YUYV = image(4, 2, 5, rgb(4, 2, 2, 60))
NV12 = image(4, 2, 6, rgb(4, 3, 1, 90))
PADDED = image(2, 2, 2, hexof(range(16)), stride=8)
JPEG = image(2, 2, 16, "ffd8ffe0")
SHORT = image(4, 3, 2, "010203")
IMAGES = [RGB, MONO, RGBA, BGR, PADDED, MONO16, MONOF, YUYV, NV12, JPEG, SHORT]


def cloud(fields, step, data):
    return {"fields": [{"name": n, "offset": o, "kind": k, "count": c} for n, o, k, c in fields],
            "point_step": step, "data": data}


XYZ = [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)]
CLOUD_XYZ = cloud(XYZ, 12, floats([1.0, 2.0, 3.0, -1.5, 0.5, 0.25, float("nan"), 0.0, 1.0]))
CLOUD_I = cloud(XYZ + [("intensity", 12, 7, 1)], 16, floats([1.0, 2.0, 3.0, 0.5, 0.0, 1.0, 0.0, 0.25]))
CLOUD_RGB = cloud(XYZ + [("rgb", 12, 7, 1)], 16, floats([1.0, 2.0, 3.0]) + "0a141eff" + floats([0.0, 1.0, 0.0]) + "ff000080")
CLOUD_N = cloud(XYZ + [("normal_x", 12, 7, 1), ("normal_y", 16, 7, 1), ("normal_z", 20, 7, 1)], 24,
                floats([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0]))
CLOUD_NONE = cloud([("q", 0, 8, 1)], 8, struct.pack("<2d", 1.5, 2.5).hex())
CLOUD_BAD = cloud(XYZ, 12, "0102")
CLOUDS = [CLOUD_XYZ, CLOUD_I, CLOUD_RGB, CLOUD_N, CLOUD_NONE, CLOUD_BAD]

GRID_A = {"origin": POSES[0], "resolution": 0.5, "cells_x": 3, "cells_y": 2, "cells_z": 1,
          "fields": [{"name": "occupancy", "kind": 2, "data": hexof([0, 10, 20, 30, 40, 50])},
                     {"name": "height", "kind": 7, "data": floats([0.5, 1.0, 1.5, 2.0, 2.5, 3.0])}]}
GRID_B = {"origin": POSES[1], "resolution": 1.0, "cells_x": 2, "cells_y": 2, "cells_z": 2,
          "fields": [{"name": "cost", "kind": 8, "data": struct.pack("<8d", *[float(i) / 4 for i in range(8)]).hex()}]}

SCANS = [
    {"angle_min": -1.5, "angle_step": 0.5, "range_min": 0.25, "range_max": 10.0, "echoes": 1,
     "ranges": [1.0, 2.0, "inf", 0.125, 20.0, "nan", 3.5]},
    {"angle_min": 0.0, "angle_step": 0.25, "range_min": 0.0, "range_max": 100.0, "echoes": 2,
     "ranges": [1.0, 1.5, 2.0, 2.5, 3.0, 3.5]},
    {"angle_min": 0.0, "angle_step": 0.25, "range_min": 0.0, "range_max": 100.0, "echoes": 0, "ranges": []},
]


def box2(x, y, w, h):
    return {"pose": {"position": d2(x, y), "angle": 0.0}, "size": d2(w, h)}


def box3(x, y, z, s):
    return {"pose": {"position": d3(x, y, z), "orientation": QUATS[0]}, "size": d3(s, s, s)}


DETS2 = [
    [{"bounds": box2(0.0, 0.0, 2.0, 2.0), "label": "car", "score": 0.5},
     {"bounds": box2(0.25, 0.0, 2.0, 2.0), "label": "car", "score": 0.75},
     {"bounds": box2(0.25, 0.0, 2.0, 2.0), "label": "dog", "score": 0.25},
     {"bounds": box2(9.0, 9.0, 1.0, 1.0), "label": "", "score": 0.125}],
    [{"bounds": box2(0.0, 0.0, 1.0, 1.0), "label": "", "score": 0.5}],
    [],
]
DETS3 = [
    [{"bounds": box3(0.0, 0.0, 0.0, 2.0), "label": "car", "score": 0.5},
     {"bounds": box3(0.25, 0.0, 0.0, 2.0), "label": "car", "score": 0.75},
     {"bounds": box3(0.25, 0.0, 0.0, 2.0), "label": "dog", "score": 0.25}],
    [{"bounds": box3(0.0, 0.0, 0.0, 1.0), "label": "x", "score": 0.5}],
    [],
]

SHAPES = [
    {"kind": "aligned_box", "value": POOLS["RantAlignedBox"][0]},
    {"kind": "oriented_box", "value": POOLS["RantOrientedBox"][0]},
    {"kind": "sphere", "value": POOLS["RantSphere"][1]},
    {"kind": "capsule", "value": POOLS["RantCapsule"][0]},
    {"kind": "cylinder", "value": POOLS["RantCylinder"][0]},
    {"kind": "cone", "value": POOLS["RantCone"][0]},
    {"kind": "prism", "value": PRISMS[0]},
    {"kind": "segment", "value": POOLS["RantSegment"][0]},
    {"kind": "polyline", "value": ZIGZAG3},
    {"kind": "polygon", "value": POLYGON3},
    {"kind": "mesh", "value": {"vertices": CUBE_V, "triangles": CUBE_T}},
    {"kind": "mesh", "value": {"vertices": TRIANGLE3, "triangles": [0, 1, 2]}},
    {"kind": "polygon", "value": []},
]
SHAPES2 = [
    {"kind": "aligned_box2d", "value": POOLS["RantAlignedBox2D"][0]},
    {"kind": "oriented_box2d", "value": POOLS["RantOrientedBox2D"][1]},
    {"kind": "circle", "value": POOLS["RantCircle"][1]},
    {"kind": "capsule2d", "value": POOLS["RantCapsule2D"][0]},
    {"kind": "polygon2d", "value": ELL},
    {"kind": "segment2d", "value": POOLS["RantSegment2D"][0]},
    {"kind": "polyline2d", "value": ZIGZAG2},
    {"kind": "polygon2d", "value": []},
    {"kind": "circle", "value": POOLS["RantCircle"][3]},
]

MATRICES = [
    {"rows": 3, "cols": 3, "data": [4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0]},
    {"rows": 2, "cols": 2, "data": [1.0, 2.0, 2.0, 4.0]},
    {"rows": 4, "cols": 2, "data": [1.0, 0.0, 1.0, 1.0, 1.0, 2.0, 1.0, 3.0]},
    {"rows": 2, "cols": 3, "data": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]},
]
WEIGHTS8 = [1.0, 2.0, 1.0, 0.5, 1.0, 1.0, 2.0, 1.0]


def fit(points, tolerance=0.0, tries=0, seed=0, weights=False, inliers=False):
    return {"weights": [1.0 + 0.25 * (i % 3) for i in range(len(points))] if weights else None,
            "tolerance": tolerance, "tries": tries, "seed": seed, "inliers": inliers}


def fit_cases(points, *rest):
    return [
        [points, *rest, None],
        [points, *rest, fit(points, weights=True, inliers=True)],
        [points, *rest, fit(points, tolerance=0.2, tries=40, seed=7, inliers=True)],
    ]


def fit_pose_cases(a, b):
    return [
        [a, b, None],
        [a, b, fit(a, weights=True, inliers=True)],
        [a, b, fit(a, tolerance=0.5, tries=40, seed=9, inliers=True)],
        [a[:1], b[:1], None],
    ]


def turned3(points):
    return [d3(1.0 - p["y"], 2.0 + p["x"], 3.0 + p["z"]) for p in points]


def turned2(points):
    return [d2(1.0 - p["y"], 2.0 + p["x"]) for p in points]


# Whole cases for what cycling a pool cannot reach. Each row is one call's inputs.
EXPLICIT = {
    "image_bytes": [[0, 4, 3], [2, 4, 3], [6, 4, 2], [16, 4, 3]],
    "image_check": [[i] for i in [RGB, PADDED, JPEG, SHORT]],
    "image_color_at": [[RGB, 1, 2], [MONO, 3, 0], [BGR, 0, 1], [RGB, 9, 0], [YUYV, 1, 1], [NV12, 2, 1], [MONO16, 0, 0]],
    "image_value_at": [[MONO, 1, 2], [MONO16, 1, 1], [MONOF, 1, 0], [MONOF, 1, 1], [RGB, 0, 0], [MONO, 8, 8]],
    "image_convert": [[RGB, 0], [RGB, 3], [BGR, 2], [YUYV, 2], [NV12, 3], [MONO, 2], [JPEG, 2], [MONO16, 2], [PADDED, 2]],
    "image_colorize": [[MONO, 0.0, 255.0, 1, 2], [MONO16, 0.0, 5000.0, 5, 3], [MONOF, 0.0, 2.0, 0, 0], [RGB, 0.0, 1.0, 0, 2]],
    "image_crop": [[RGB, 1, 1, 2, 2], [MONO, 0, 0, 4, 3], [YUYV, 2, 0, 2, 1], [NV12, 0, 0, 2, 2], [RGB, 3, 2, 5, 5], [PADDED, 1, 0, 1, 2]],
    "image_flip_horizontal": [[RGB], [MONO16], [MONOF], [YUYV]],
    "image_flip_vertical": [[RGBA], [MONO], [MONO16], [NV12]],
    "image_rotate": [[RGB, 1], [MONO, 2], [BGR, 3], [RGBA, -1], [RGB, 4], [YUYV, 1]],
    "image_resize": [[RGB, 1, 2, 2], [MONO, 0, 8, 6], [MONO16, 1, 4, 4], [MONOF, 0, 1, 1], [YUYV, 0, 2, 2], [RGB, 1, 0, 0]],
    "image_undistort": [
        [image(4, 4, 0, rgb(4, 4, 1)), {**POOLS["RantCameraIntrinsics"][1], "width": 4, "height": 4, "fx": 4.0, "fy": 4.0, "cx": 2.0, "cy": 2.0}, None, 1],
        [image(4, 4, 2, rgb(4, 4)), {**POOLS["RantCameraIntrinsics"][1], "width": 4, "height": 4, "fx": 4.0, "fy": 4.0, "cx": 2.0, "cy": 2.0},
         {**POOLS["RantCameraIntrinsics"][0], "width": 2, "height": 2, "fx": 2.0, "fy": 2.0, "cx": 1.0, "cy": 1.0}, 0],
        [RGB, POOLS["RantCameraIntrinsics"][0], None, 0],
        [JPEG, POOLS["RantCameraIntrinsics"][0], None, 0],
    ],
    "image_draw": [
        [image(8, 6, 2, rgb(8, 6)), {"kind": "circle", "value": {"center": d2(3.0, 2.0), "radius": 2.0}}, POOLS["RantColor"][0], 0.0],
        [image(8, 6, 0, rgb(8, 6, 1)), {"kind": "segment2d", "value": {"a": d2(0.0, 0.0), "b": d2(7.0, 5.0)}}, POOLS["RantColor"][1], 1.0],
        [image(8, 6, 3, rgb(8, 6, 4)), {"kind": "polygon2d", "value": [d2(1.0, 1.0), d2(6.0, 1.0), d2(3.0, 5.0)]}, POOLS["RantColor"][1], 0.0],
        [MONO16, {"kind": "circle", "value": {"center": d2(1.0, 1.0), "radius": 1.0}}, POOLS["RantColor"][0], 0.0],
    ],
    "point_cloud_layout": [[0], [1], [2], [3]],
    "point_cloud_check": [[c] for c in [CLOUD_XYZ, CLOUD_BAD, cloud([("x", 20, 7, 1)], 12, "00" * 12)]],
    "point_cloud_field": [[CLOUD_I, "intensity"], [CLOUD_XYZ, "z"], [CLOUD_XYZ, "missing"]],
    "point_cloud_read_points": [[c] for c in [CLOUD_XYZ, CLOUD_N, CLOUD_NONE, CLOUD_BAD]],
    "point_cloud_write_points": [[CLOUD_XYZ, TRIANGLE3], [CLOUD_I, TRIANGLE3[:2]], [CLOUD_XYZ, TRIANGLE3[:1]], [CLOUD_NONE, TRIANGLE3[:2]]],
    "point_cloud_read_values": [[CLOUD_I, "intensity"], [CLOUD_XYZ, "x"], [CLOUD_XYZ, "missing"], [CLOUD_BAD, "x"]],
    "point_cloud_write_values": [[CLOUD_I, "intensity", [9.5, -1.5]], [CLOUD_XYZ, "y", [1.0, 2.0, 3.0]], [CLOUD_XYZ, "missing", [1.0, 2.0, 3.0]], [CLOUD_XYZ, "y", [1.0]]],
    "point_cloud_read_colors": [[CLOUD_RGB], [CLOUD_XYZ]],
    "point_cloud_write_colors": [[CLOUD_RGB, POOLS["RantColor"][:2]], [CLOUD_XYZ, POOLS["RantColor"][:3]], [CLOUD_RGB, POOLS["RantColor"][:1]]],
    "point_cloud_transform": [[CLOUD_XYZ, POSES[1]], [CLOUD_N, POSES[3]], [CLOUD_NONE, POSES[1]], [CLOUD_BAD, POSES[0]]],
    "point_cloud_select": [[CLOUD_XYZ, [1, 0, 1]], [CLOUD_I, [0, 0]], [CLOUD_I, [1, 1]], [CLOUD_BAD, [1]]],
    "laser_scan_points": [[s] for s in SCANS],
    "image_depth_points": [
        [image(2, 2, 1, u16s([0, 1000, 2500, 65535])), {**POOLS["RantCameraIntrinsics"][0], "width": 2, "height": 2, "fx": 2.0, "fy": 2.0, "cx": 1.0, "cy": 1.0}],
        [image(2, 2, 7, floats([0.0, 0.5, 1.5, float("nan")])), {**POOLS["RantCameraIntrinsics"][1], "width": 2, "height": 2, "fx": 2.0, "fy": 2.0, "cx": 1.0, "cy": 1.0}],
        [image(2, 2, 7, floats([0.5] * 4)), POOLS["RantCameraIntrinsics"][0]],
        [RGB, {**POOLS["RantCameraIntrinsics"][0], "width": 4, "height": 3}],
    ],
    "kdtree_nearest": [
        [{"points": CLOUD8}, V3[0], 3],
        [{"points": CLOUD8}, d3(0.5, 0.5, 0.0), 100],
        [{"points": NAN_THREE}, V3[3], 2],
        [{"points": CLOUD8}, d3("nan", 0.0, 0.0), 1],
        [{"points": CLOUD8}, V3[0], 0],
    ],
    "kdtree_within": [
        [{"points": CLOUD8}, d3(0.5, 0.5, 0.0), 1.0],
        [{"points": CLOUD8}, V3[0], 0.01],
        [{"points": CLOUD8}, d3("nan", 0.0, 0.0), 1.0],
    ],
    "points_downsample": [[CLOUD8, 0.5], [CLOUD8, 100.0], [NAN_THREE, 1.0], [CLOUD8, 0.0], [[], 1.0]],
    "points_remove_outliers": [[CLOUD8, 3, 1.0], [CLOUD8 + [d3(50.0, 50.0, 50.0)], 3, 1.0], [NAN_THREE, 1, 1.0], [CLOUD8, 0, 1.0]],
    "points_remove_isolated": [[CLOUD8, 1.5, 2], [CLOUD8, 0.01, 1], [NAN_THREE, 2.0, 1], [CLOUD8, -1.0, 1]],
    "points_normals": [[FLAT6, 4, V3[1]], [CLOUD8, 5, d3(0.0, 0.0, 10.0)], [NAN_THREE, 3, V3[0]], [CLOUD8, 0, V3[0]]],
    "points_clusters": [[CLOUD8, 1.5, 2], [CLOUD8 + [d3(50.0, 50.0, 50.0)], 1.5, 1], [NAN_THREE, 2.0, 1], [CLOUD8, -1.0, 1]],
    "points_align": [
        [CLOUD8, turned3(CLOUD8), POSES[0], None],
        [CLOUD8, CLOUD8, POSES[0], {"max_distance": 2.0, "iterations": 20, "normals": None, "inliers": True, "error": True}],
        [CLOUD8, CLOUD8, POSES[1], {"max_distance": 0.0, "iterations": 10, "normals": [d3(0.0, 0.0, 1.0)] * 8, "inliers": False, "error": True}],
        [CLOUD8[:2], CLOUD8, POSES[0], None],
        [[], CLOUD8, POSES[0], None],
    ],
    "points_fit_plane": fit_cases(FLAT6) + fit_cases(LINE4)[:1] + [[[], None]],
    "points_fit_line": fit_cases(LINE4) + [[[], None]],
    "points_fit_sphere": fit_cases(SPHERE6) + fit_cases(FLAT6)[:1],
    "points_fit_pose": fit_pose_cases(CLOUD8, turned3(CLOUD8)),
    "points2d_fit_line": fit_cases(LINE2) + [[[d2(1.0, 1.0)], None]],
    "points2d_fit_circle": fit_cases([d2(2.0, 0.0), d2(-2.0, 0.0), d2(0.0, 2.0), d2(0.0, -2.0)]) + [[LINE2, None]],
    "points2d_fit_pose": fit_pose_cases(CLOUD2, turned2(CLOUD2)),
    "polynomial_fit": [
        [[0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 7.0, 13.0], 2, None],
        [[0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 7.0, 14.0], 1, fit([0, 0, 0, 0], weights=True, inliers=True)],
        [[0.0, 1.0], [1.0, 2.0], 5, None],
        [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], 1, None],
    ],
    "polynomial_at": [[[1.0, 2.0, 3.0], 2.0], [[0.5], 9.0], [[1.0, -1.0], -3.0], [[1.0, 0.0, 0.0, 2.0], 0.5]],
    "grid_cell": [[GRID_A, d3(1.75, 2.25, 3.0)], [GRID_A, d3(90.0, 0.0, 0.0)], [GRID_B, d3(1.5, 2.5, 3.5)]],
    "grid_center": [[GRID_A, d3(1, 1, 0)], [GRID_B, d3(0, 0, 1)], [GRID_A, d3(9, 9, 9)]],
    "grid_read_value": [[GRID_A, "occupancy", d3(2, 1, 0)], [GRID_A, "height", d3(1, 0, 0)], [GRID_A, "missing", d3(0, 0, 0)], [GRID_A, "height", d3(9, 0, 0)], [GRID_B, "cost", d3(1, 1, 1)]],
    "grid_write_value": [[GRID_A, "occupancy", d3(2, 1, 0), 300.0], [GRID_A, "height", d3(0, 0, 0), 0.25], [GRID_A, "missing", d3(0, 0, 0), 1.0], [GRID_B, "cost", d3(9, 9, 9), 1.0]],
    "detections2d_suppress": [[DETS2[0], 0.5], [DETS2[0], 0.95], [DETS2[1], 0.5], [DETS2[2], 0.5]],
    "detections3d_suppress": [[DETS3[0], 0.5], [DETS3[0], 0.95], [DETS3[1], 0.5], [DETS3[2], 0.5]],
    "shape_mesh": [[s, seg] for s, seg in zip(SHAPES, [0, 8, 8, 8, 6, 6, 0, 0, 0, 0, 0, 0, 0])] + [[SHAPES[2], 2]],
    "shape_gap": [[SHAPES[i], SHAPES[j]] for i, j in [(0, 2), (2, 3), (4, 5), (6, 7), (8, 9), (10, 11), (0, 0), (12, 0)]],
    "shape2d_gap": [[SHAPES2[i], SHAPES2[j]] for i, j in [(0, 2), (2, 3), (4, 5), (6, 0), (1, 1), (7, 0)]],
    "shape_overlaps": [[SHAPES[i], SHAPES[j]] for i, j in [(0, 2), (2, 3), (0, 0), (4, 10)]],
    "shape2d_overlaps": [[SHAPES2[i], SHAPES2[j]] for i, j in [(0, 2), (2, 3), (0, 8), (4, 8)]],
    "oriented_box_iou": [[POOLS["RantOrientedBox"][1], {**POOLS["RantOrientedBox"][1], "size": d3(1.0, 2.0, 2.0)}], [POOLS["RantOrientedBox"][0], POOLS["RantOrientedBox"][3]], [POOLS["RantOrientedBox"][1], POOLS["RantOrientedBox"][2]]],
    "oriented_box2d_iou": [[POOLS["RantOrientedBox2D"][0], {**POOLS["RantOrientedBox2D"][0], "size": d2(1.0, 2.0)}], [POOLS["RantOrientedBox2D"][1], POOLS["RantOrientedBox2D"][3]], [POOLS["RantOrientedBox2D"][0], POOLS["RantOrientedBox2D"][2]]],
    "shape_sample_inside": [[SHAPES[i], 7, 4] for i in [0, 2, 3, 6, 10, 7]],
    "shape_sample_surface": [[SHAPES[i], 9, 4] for i in [0, 4, 5, 8, 9, 10]] + [[SHAPES[12], 1, 2]],
    "shape2d_sample_inside": [[SHAPES2[i], 7, 4] for i in [0, 2, 4, 5]],
    "shape2d_sample_surface": [[SHAPES2[i], 9, 4] for i in [1, 3, 6]] + [[SHAPES2[7], 1, 2]],
    "polygon_flatten": [[POLYGON3], [TRIANGLE3], [LINE3], [[]]],
    "polygon_plane": [[POLYGON3], [TRIANGLE3], [LINE3], [[]]],
    "points_oriented_box": [[CLOUD8], [LINE4], [[d3(1.0, 1.0, 1.0)]], [[]]],
    "points_enclosing_sphere": [[CLOUD8], [SPHERE6], [[d3(1.0, 1.0, 1.0)]], [[]]],
    "points_bounds": [[CLOUD8], [[d3(1.0, 1.0, 1.0)]], [[]]],
    "points_mean": [[CLOUD8], [[d3(1.0, 1.0, 1.0)]], [[]]],
    "points_covariance": [[CLOUD8], [LINE4], [[]]],
    "points2d_bounds": [[CLOUD2], [[d2(1.0, 1.0)]], [[]]],
    "points2d_mean": [[CLOUD2], [[d2(1.0, 1.0)]], [[]]],
    "points2d_hull": [[CLOUD2], [LINE2], [[d2(1.0, 1.0)]], [[]]],
    "points2d_oriented_box": [[CLOUD2], [LINE2], [[d2(1.0, 1.0)]], [[]]],
    "points2d_enclosing_circle": [[CLOUD2], [LINE2], [[d2(1.0, 1.0)]], [[]]],
    "camera_project": [[POOLS["RantCameraIntrinsics"][0], d3(5.0, 1.0, 0.5)], [POOLS["RantCameraIntrinsics"][1], d3(4.0, -1.0, 0.5)],
                       [POOLS["RantCameraIntrinsics"][0], d3(-1.0, 0.0, 0.0)], [POOLS["RantCameraIntrinsics"][3], d3(1.0, 0.0, 0.0)],
                       [POOLS["RantCameraIntrinsics"][2], d3(4.0, 1.0, 1.0)]],
    "camera_project_points": [[POOLS["RantCameraIntrinsics"][0], [d3(5.0, 1.0, 0.5), d3(-1.0, 0.0, 0.0)]],
                              [POOLS["RantCameraIntrinsics"][1], CLOUD8[4:]], [POOLS["RantCameraIntrinsics"][3], [d3(1.0, 0.0, 0.0)]],
                              [POOLS["RantCameraIntrinsics"][0], []]],
    "camera_ray": [[POOLS["RantCameraIntrinsics"][0], d2(100.0, 50.0)], [POOLS["RantCameraIntrinsics"][1], d2(100.0, 50.0)], [POOLS["RantCameraIntrinsics"][3], d2(1.0, 1.0)], [POOLS["RantCameraIntrinsics"][2], d2(0.0, 0.0)]],
    "camera_unproject": [[POOLS["RantCameraIntrinsics"][0], d2(100.0, 50.0), 2.0], [POOLS["RantCameraIntrinsics"][1], d2(100.0, 50.0), 0.5], [POOLS["RantCameraIntrinsics"][3], d2(1.0, 1.0), 1.0]],
    "camera_scale": [[POOLS["RantCameraIntrinsics"][0], 320, 240], [POOLS["RantCameraIntrinsics"][1], 640, 360], [POOLS["RantCameraIntrinsics"][0], 0, 0]],
    "camera_crop": [[POOLS["RantCameraIntrinsics"][0], 10, 20, 100, 100], [POOLS["RantCameraIntrinsics"][1], 0, 0, 640, 360], [POOLS["RantCameraIntrinsics"][0], 600, 0, 100, 100]],
    "camera_fov": [[POOLS["RantCameraIntrinsics"][0]], [POOLS["RantCameraIntrinsics"][1]], [POOLS["RantCameraIntrinsics"][3]]],
    "camera_from_fov": [[640, 480, d2(1.5, 1.0)], [640, 480, d2(1.5, 0.0)], [0, 480, d2(1.5, 1.0)], [640, 480, d2(4.0, 1.0)]],
    "matrix_multiply": [[MATRICES[0], MATRICES[0]], [MATRICES[3], MATRICES[0]], [MATRICES[1], MATRICES[0]]],
    "matrix_transpose": [[MATRICES[3]], [MATRICES[0]], [{"rows": 0, "cols": 0, "data": []}]],
    "matrix_determinant": [[MATRICES[0]], [MATRICES[1]], [MATRICES[3]]],
    "matrix_inverse": [[MATRICES[0]], [MATRICES[1]], [MATRICES[3]]],
    "matrix_solve": [[MATRICES[0], {"rows": 3, "cols": 1, "data": [1.0, 2.0, 3.0]}],
                     [MATRICES[2], {"rows": 4, "cols": 1, "data": [1.0, 2.0, 2.0, 4.0]}],
                     [MATRICES[1], {"rows": 2, "cols": 1, "data": [1.0, 2.0]}],
                     [MATRICES[0], {"rows": 2, "cols": 1, "data": [1.0, 2.0]}]],
    "matrix_svd": [[MATRICES[0]], [MATRICES[2]], [MATRICES[3]], [{"rows": 0, "cols": 0, "data": []}]],
    "matrix_eigen": [[MATRICES[0]], [MATRICES[3]]],
    "matrix_cholesky": [[MATRICES[0]], [MATRICES[1]], [MATRICES[3]]],
    "axes_make": [[-2, 3, 1, 1.0], [1, -2, 3, 100.0], [1, 1, 3, 1.0], [1, 2, 3, 0.0]],
    "axes_of": [[i] for i in range(10)],
    "quaternion_from_euler": [[0.25, -0.5, 1.0, o] for o in [0, 5, 7, 12, 17, 23]],
    "quaternion_to_euler": [[q, o] for q, o in zip(QUATS, [0, 5, 13, 22])],
    "colormap_at": [[m, t] for m, t in [(0, 0.5), (1, 0.25), (5, 1.0), (3, "nan"), (2, -1.0), (4, 2.0)]],
    "pose_from_points": [[V3[3], d3(1.0, 0.0, 0.0), d3(0.0, 1.0, 0.0)], [V3[0], V3[1], V3[2]], [V3[3], V3[3], V3[3]]],
    "pose_look_at": [[V3[0], V3[1], d3(0.0, 0.0, 1.0)], [V3[3], d3(1.0, 0.0, 0.0), d3(0.0, 0.0, 1.0)], [V3[0], V3[0], d3(0.0, 0.0, 1.0)]],
    "plane_from_points": [[d3(0.0, 0.0, 0.0), d3(1.0, 0.0, 0.0), d3(0.0, 1.0, 0.0)], [V3[0], V3[1], V3[2]], [V3[0], V3[0], V3[0]]],
    "plane_intersect": [[POOLS["RantPlane"][0], POOLS["RantPlane"][2]], [POOLS["RantPlane"][1], POOLS["RantPlane"][3]], [POOLS["RantPlane"][0], POOLS["RantPlane"][0]]],
    "plane_intersect3": [[POOLS["RantPlane"][0], POOLS["RantPlane"][2], POOLS["RantPlane"][3]], [POOLS["RantPlane"][0], POOLS["RantPlane"][0], POOLS["RantPlane"][3]]],
    "plane2d_from_points": [[d2(0.0, 0.0), d2(1.0, 0.0)], [V2[0], V2[1]], [V2[0], V2[0]]],
    "plane2d_intersect": [[POOLS["RantPlane2D"][0], POOLS["RantPlane2D"][2]], [POOLS["RantPlane2D"][2], POOLS["RantPlane2D"][3]]],
    "plane_raycast": [[POOLS["RantPlane"][0], POOLS["RantRay"][1]], [POOLS["RantPlane"][2], POOLS["RantRay"][0]], [POOLS["RantPlane"][0], POOLS["RantRay"][0]]],
    "plane2d_raycast": [[POOLS["RantPlane2D"][0], POOLS["RantRay2D"][1]], [POOLS["RantPlane2D"][2], POOLS["RantRay2D"][0]], [POOLS["RantPlane2D"][0], POOLS["RantRay2D"][0]]],
    "segment2d_intersect": [[POOLS["RantSegment2D"][0], POOLS["RantSegment2D"][1]], [POOLS["RantSegment2D"][0], POOLS["RantSegment2D"][2]]],
    "segment2d_raycast": [[POOLS["RantSegment2D"][1], POOLS["RantRay2D"][0]], [POOLS["RantSegment2D"][2], POOLS["RantRay2D"][0]]],
    "aligned_box_overlap": [[POOLS["RantAlignedBox"][0], POOLS["RantAlignedBox"][1]], [POOLS["RantAlignedBox"][0], POOLS["RantAlignedBox"][2]]],
    "aligned_box2d_overlap": [[POOLS["RantAlignedBox2D"][0], POOLS["RantAlignedBox2D"][1]], [POOLS["RantAlignedBox2D"][0], POOLS["RantAlignedBox2D"][2]]],
    "twist_between": [[POSES[0], POSES[1], 0.5], [POSES[1], POSES[2], 0.1], [POSES[0], POSES[1], 0.0]],
    "twist2d_between": [[POSES2[0], POSES2[1], 0.5], [POSES2[1], POSES2[2], 0.1], [POSES2[0], POSES2[1], 0.0]],
    "matrix3x3_inverse": [[POOLS["RantMatrix3x3"][1]], [POOLS["RantMatrix3x3"][2]]],
    "matrix4x4_inverse": [[POOLS["RantMatrix4x4"][1]], [POOLS["RantMatrix4x4"][2]]],
    "prism_raycast": [[PRISMS[0], POOLS["RantRay"][0]], [PRISMS[1], POOLS["RantRay"][3]], [PRISMS[0], POOLS["RantRay"][2]], [PRISMS[3], POOLS["RantRay"][0]]],
    "geo_point_to_enu": [[POOLS["RantGeoPoint"][0], {"lat": 51.501, "lon": -0.12, "alt": 40.0}], [POOLS["RantGeoPoint"][1], POOLS["RantGeoPoint"][3]], [POOLS["RantGeoPoint"][0], POOLS["RantGeoPoint"][0]]],
    "shape_raycast": [[SHAPES[i], POOLS["RantRay"][r]] for i, r in [(0, 0), (2, 3), (0, 2), (10, 0), (9, 1), (7, 0)]],
    "shape2d_raycast": [[SHAPES2[i], POOLS["RantRay2D"][r]] for i, r in [(0, 0), (2, 3), (0, 2), (4, 0), (5, 3)]],
}


def enum_or_scalar(model, t):
    return t in model.enums or t in POOLS


def scalar_literal(value):
    return value


def pool_value(t, name, i):
    pool = NAMED.get((t, name)) or POOLS.get(t)
    if pool is None:
        raise KeyError(f"no samples for {t} {name}")
    return pool[i % len(pool)]


def list_value(f, inp, i):
    if f.name.startswith("path2d_"):
        return [POSES2[:3], POSES2, [POSES2[1]], []][i % 4]
    if f.name.startswith("path_"):
        return [POSES[:3], POSES, [POSES[1]], []][i % 4]
    for prefix, pool in BY_PREFIX.items():
        if f.name.startswith(prefix):
            return pool[i % 4]
    if inp.name in ("vertices",):
        return MESHES[i % 4][0]
    if inp.name in ("triangles",):
        return MESHES[i % 4][1]
    return LISTS[inp.type][i % 4]


def auto_inputs(f, i):
    row = []
    for j, inp in enumerate(f.inputs):
        k = inp.kind
        if k == "value" or k == "ref":
            # A query point keeps the shape's index, so some lie inside it.
            row.append(pool_value(inp.type, inp.name, i if inp.name == "point" else i + j))
        elif k == "list":
            row.append(list_value(f, inp, i))
        elif k == "prism":
            row.append(PRISMS[i % 4])
        elif k == "shape":
            row.append(SHAPES[(i * 3 + j) % len(SHAPES)])
        elif k == "shape2d":
            row.append(SHAPES2[(i * 3 + j) % len(SHAPES2)])
        else:
            raise KeyError(f"{f.name}: nothing to ask with for {k} {inp.name}")
    return row


def build_cases(model):
    out = []
    for f in model.functions:
        if not f.vector:
            continue
        if f.name in EXPLICIT:
            rows = EXPLICIT[f.name]
        else:
            n = max(CASES_PER_FUNCTION, 12 if any(i.kind in ("shape", "shape2d") for i in f.inputs) else 0)
            rows = [auto_inputs(f, i) for i in range(n)]
        for row in rows:
            if len(row) != len(f.inputs):
                raise ValueError(f"{f.name}: a case has {len(row)} inputs, it takes {len(f.inputs)}")
            out.append((f, row))
    return out


def json_number(v):
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return "nan" if math.isnan(v) else ("inf" if v > 0 else "-inf")
    return v


def clean(v):
    if isinstance(v, dict):
        return {k: clean(x) for k, x in v.items()}
    if isinstance(v, list):
        return [clean(x) for x in v]
    return json_number(v)


# A C program: one function per case, each writing its answer as one line of JSON.

C_HEAD = r"""/* GENERATED by tools/bindgen/gen.py. Asks the library each case and writes down its answers. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rant_math.h"

static FILE *o;
static void pd(double v){
    if (v != v) fputs("\"nan\"", o);
    else if (v == HUGE_VAL) fputs("\"inf\"", o);
    else if (v == -HUGE_VAL) fputs("\"-inf\"", o);
    else fprintf(o, "%.17g", v);
}
static void pf(float v){
    if (v != v) fputs("\"nan\"", o);
    else if (v == (float)HUGE_VAL) fputs("\"inf\"", o);
    else if (v == (float)-HUGE_VAL) fputs("\"-inf\"", o);
    else fprintf(o, "%.9g", (double)v);
}
static void pi(long long v){ fprintf(o, "%lld", v); }
static void pu(unsigned long long v){ fprintf(o, "%llu", v); }
static void pt(const char *s, size_t n){ fputc('"', o); fwrite(s, 1, n, o); fputc('"', o); }
static void px(const uint8_t *d, size_t n){
    size_t i;
    fputc('"', o);
    for (i = 0; i < n; i++) fprintf(o, "%02x", d[i]);
    fputc('"', o);
}
static void *room(size_t n){ void *p = calloc(n ? n : 1, 1); if (!p) abort(); return p; }
static size_t out_bytes(RantImageFormat format, uint32_t w, uint32_t h){ return rant_image_bytes(format, w, h); }
static void pimage(const RantImageView *v, size_t size){
    fprintf(o, "{\"width\":%u,\"height\":%u,\"stride\":%u,\"format\":%d,\"data\":", v->width, v->height, v->stride, (int)v->format);
    px(v->data, size);
    fputc('}', o);
}
static void pfield(const RantPointFieldView *f){
    fputs("{\"name\":", o);
    pt(f->name, f->name_len);
    fprintf(o, ",\"offset\":%u,\"kind\":%d,\"count\":%u}", f->offset, (int)f->kind, f->count);
}
static void pcloud(const RantPointCloudView *c, const uint8_t *data, size_t size){
    uint32_t i;
    fputs("{\"fields\":[", o);
    for (i = 0; i < c->field_count; i++){
        if (i) fputc(',', o);
        pfield(&c->fields[i]);
    }
    fprintf(o, "],\"point_step\":%u,\"data\":", c->point_step);
    px(data, size);
    fputc('}', o);
}
"""


def c_float_literal(v):
    if v == "nan" or (isinstance(v, float) and math.isnan(v)):
        return "NAN"
    if v == "inf" or v == float("inf"):
        return "HUGE_VAL"
    if v == "-inf" or v == float("-inf"):
        return "(-HUGE_VAL)"
    return repr(float(v))


class CWriter:
    def __init__(self, model):
        self.m = model
        self.printers = {}
        self.head = []

    def literal(self, t, v, dims=()):
        """A C initializer for value v of type t."""
        if dims:
            return "{" + ",".join(self.literal(t, x, dims[1:]) for x in v) + "}"
        if t == "double":
            return c_float_literal(v)
        if t == "float":
            s = c_float_literal(v)
            return s if s in ("NAN", "HUGE_VAL", "(-HUGE_VAL)") else s + "f"
        if t in ("uint64_t", "size_t"):
            return str(int(v)) + "ULL"
        if t in self.m.enums:
            return f"({t})({int(v)})"
        if t in self.m.structs:
            s = self.m.structs[t]
            return "{" + ",".join(self.literal(f.type, v[f.name], f.dims) for f in s.fields) + "}"
        return str(int(v))

    def printer(self, t):
        """The name of a function that prints a value of type t."""
        if t in self.printers:
            return self.printers[t]
        s = self.m.structs[t]
        name = "p_" + t
        lines = [f"static void {name}(const {t} *v){{", '    fputs("{", o);']
        for i, f in enumerate(s.fields):
            comma = "," if i else ""
            lines.append(f'    fputs("{comma}\\"{f.name}\\":", o);')
            lines.append("    " + self.print_field(f))
        lines += ['    fputs("}", o);', "}"]
        self.head.append("\n".join(lines))
        self.printers[t] = name
        return name

    def print_field(self, f):
        return self.print_nested(f.type, f"v->{f.name}", f.dims)

    def print_nested(self, t, expr, dims):
        if not dims:
            return self.print_value(t, expr) + ";"
        i = f"i{len(dims)}"
        inner = self.print_nested(t, f"{expr}[{i}]", dims[1:])
        return (f'{{ int {i}; fputc(\'[\', o); for ({i} = 0; {i} < {dims[0]}; {i}++){{ if ({i}) fputc(\',\', o); {inner} }} fputc(\']\', o); }}')

    def print_value(self, t, expr):
        if t == "double":
            return f"pd({expr})"
        if t == "float":
            return f"pf({expr})"
        if t in ("uint8_t", "uint16_t", "uint32_t", "uint64_t", "size_t"):
            return f"pu((unsigned long long)({expr}))"
        if t in ("int", "int8_t", "int16_t", "int32_t", "int64_t") or t in self.m.enums:
            return f"pi((long long)({expr}))"
        return f"{self.printer(t)}(&({expr}))"


def c_array(w, name, t, values, per=""):
    if not values:
        return f"{t} *{name} = NULL;", "0"
    body = ",".join(w.literal(t, v) for v in values)
    return f"static {t} {name}[] = {{{body}}};", str(len(values))


def c_bytes(name, hexdata):
    data = bytes.fromhex(hexdata)
    if not data:
        return f"static uint8_t {name}[1] = {{0}};", "0"
    return f"static uint8_t {name}[] = {{{','.join(str(b) for b in data)}}};", str(len(data))


def c_string(s):
    return '"' + s + '"'


class CCase:
    """One call: the declarations before it, the C argument for each parameter, and the answer's print."""

    def __init__(self, w, f, row, index):
        self.w, self.f, self.row = w, f, row
        self.lines = []
        self.names = {}
        self.index = index

    def decl(self, line):
        self.lines.append("    " + line)

    def inputs(self):
        for i, (inp, value) in enumerate(zip(self.f.inputs, self.row)):
            self.names[id(inp)] = self.emit_input(inp, value, f"a{i}")

    def emit_input(self, inp, v, name):
        k = inp.kind
        w = self.w
        if k == "value":
            if inp.type in w.m.structs:
                self.decl(f"{inp.type} {name} = {w.literal(inp.type, v)};")
            else:
                self.decl(f"{inp.type} {name} = {w.literal(inp.type, v)};")
            return {"value": name}
        if k == "ref":
            self.decl(f"{inp.type} {name} = {w.literal(inp.type, v)};")
            return {"ref": "&" + name}
        if k == "opt_ref":
            if v is None:
                return {"ref": "NULL"}
            self.decl(f"{inp.type} {name} = {w.literal(inp.type, v)};")
            return {"ref": "&" + name}
        if k == "list":
            line, n = c_array(w, name, inp.type, v)
            self.decl(line)
            return {"ptr": name, "count": n}
        if k == "string":
            return {"str": c_string(v)}
        if k == "prism":
            line, n = c_array(w, name + "b", "RantDouble2", v["base"]["points"])
            self.decl(line)
            self.decl(f"RantPrismView {name}; {name}.pose = ({w.pose_literal(v['pose'])}); {name}.base.points = {name}b; {name}.base.count = {n}; {name}.height = {c_float_literal(v['height'])};")
            return {"view": name}
        if k == "matrix":
            line, n = c_array(w, name + "d", "double", v["data"])
            self.decl(line)
            self.decl(f"RantMatrixView {name}; {name}.data = {name}d; {name}.rows = {v['rows']}; {name}.cols = {v['cols']};")
            return {"view": name, "rows": v["rows"], "cols": v["cols"]}
        if k == "image" or k == "image_mut":
            line, n = c_bytes(name + "d", v["data"])
            self.decl(line)
            self.decl(f"RantImageView {name}; {name}.width = {v['width']}; {name}.height = {v['height']}; {name}.stride = {v['stride']}; "
                      f"{name}.format = (RantImageFormat){v['format']}; {name}.data = {name}d; {name}.size = {n};")
            return {"image": name, "width": v["width"], "height": v["height"], "format": v["format"], "size": n}
        if k == "cloud" or k == "cloud_mut":
            self.emit_fields(name, v["fields"])
            line, n = c_bytes(name + "d", v["data"])
            self.decl(line)
            self.decl(f"RantPointCloudView {name}; {name}.fields = {name}f; {name}.field_count = {len(v['fields'])}; {name}.point_step = {v['point_step']}; "
                      f"{name}.data = {name}d; {name}.size = {n};")
            return {"cloud": name, "size": n}
        if k == "grid" or k == "grid_mut":
            parts = []
            for j, fld in enumerate(v["fields"]):
                line, n = c_bytes(f"{name}d{j}", fld["data"])
                self.decl(line)
                parts.append(f'{{"{fld["name"]}", {len(fld["name"])}, (RantScalarKind){fld["kind"]}, {name}d{j}, {n}}}')
            self.decl(f"static RantGridFieldView {name}f[] = {{{','.join(parts)}}};")
            self.decl(f"RantGridView {name}; {name}.origin = ({w.pose_literal(v['origin'])}); {name}.resolution = {c_float_literal(v['resolution'])}; "
                      f"{name}.cells_x = {v['cells_x']}; {name}.cells_y = {v['cells_y']}; {name}.cells_z = {v['cells_z']}; {name}.fields = {name}f; {name}.field_count = {len(parts)};")
            return {"grid": name, "value": v}
        if k == "scan":
            line, n = c_array(w, name + "r", "float", v["ranges"])
            self.decl(line)
            self.decl(f"RantLaserScanView {name}; {name}.angle_min = {c_float_literal(v['angle_min'])}; {name}.angle_step = {c_float_literal(v['angle_step'])}; "
                      f"{name}.range_min = {c_float_literal(v['range_min'])}; {name}.range_max = {c_float_literal(v['range_max'])}; {name}.echoes = {v['echoes']}; "
                      f"{name}.ranges = {name}r; {name}.range_count = {n};")
            return {"ref": "&" + name}
        if k in ("det2d", "det3d"):
            t = "RantOrientedBox2D" if k == "det2d" else "RantOrientedBox"
            view = "RantDetection2DView" if k == "det2d" else "RantDetection3DView"
            if not v:
                self.decl(f"{view} *{name} = NULL;")
                return {"ptr": name, "count": "0"}
            items = []
            for d in v:
                items.append(f'{{{w.literal(t, d["bounds"])}, "{d["label"]}", {len(d["label"])}, {c_float_literal(d["score"])}}}')
            self.decl(f"static {view} {name}[] = {{{','.join(items)}}};")
            return {"ptr": name, "count": str(len(v))}
        if k in ("shape", "shape2d"):
            return self.emit_shape(k, v, name)
        if k == "tree":
            line, n = c_array(w, name + "p", "RantDouble3", v["points"])
            self.decl(line)
            self.decl(f"RantKdTree *{name} = NULL; if (rant_kdtree_create({name}p, {n}, &{name}) != RANT_MATH_OK) abort();")
            return {"tree": name}
        if k == "fit_options":
            if v is None:
                return {"options": "NULL"}
            self.decl(f"RantFitOptions {name}; memset(&{name}, 0, sizeof {name});")
            if v["weights"] is not None:
                line, _ = c_array(w, name + "w", "double", v["weights"])
                self.decl(line)
                self.decl(f"{name}.weights = {name}w;")
            self.decl(f"{name}.tolerance = {c_float_literal(v['tolerance'])}; {name}.tries = {v['tries']}; {name}.seed = {v['seed']};")
            return {"options": "&" + name, "name": name, "value": v}
        if k == "align_options":
            if v is None:
                return {"options": "NULL"}
            self.decl(f"RantAlignOptions {name}; memset(&{name}, 0, sizeof {name});")
            if v["normals"] is not None:
                line, _ = c_array(w, name + "n", "RantDouble3", v["normals"])
                self.decl(line)
                self.decl(f"{name}.normals = {name}n;")
            self.decl(f"{name}.max_distance = {c_float_literal(v['max_distance'])}; {name}.iterations = {v['iterations']};")
            return {"options": "&" + name, "name": name, "value": v}
        raise KeyError(k)

    def emit_fields(self, name, fields):
        if not fields:
            self.decl(f"RantPointFieldView *{name}f = NULL;")
            return
        parts = [f'{{"{x["name"]}", {len(x["name"])}, {x["offset"]}, (RantScalarKind){x["kind"]}, {x["count"]}}}' for x in fields]
        self.decl(f"static RantPointFieldView {name}f[] = {{{','.join(parts)}}};")

    def emit_shape(self, k, v, name):
        kind = v["kind"]
        value = v["value"]
        w = self.w
        prefix = "rant_shape_from_" if k == "shape" else "rant_shape2d_from_"
        t = "RantShape" if k == "shape" else "RantShape2D"
        if kind == "mesh":
            l1, n1 = c_array(w, name + "v", "RantDouble3", value["vertices"])
            l2, n2 = c_array(w, name + "t", "uint32_t", value["triangles"])
            self.decl(l1)
            self.decl(l2)
            self.decl(f"RantMeshView {name}m; {name}m.vertices = {name}v; {name}m.vertex_count = {n1}; {name}m.triangles = {name}t; {name}m.triangle_count = {n2} / 3;")
            self.decl(f"{t} {name} = {prefix}mesh({name}m);")
        elif kind == "prism":
            l1, n1 = c_array(w, name + "b", "RantDouble2", value["base"]["points"])
            self.decl(l1)
            self.decl(f"RantPrismView {name}m; {name}m.pose = ({w.pose_literal(value['pose'])}); {name}m.base.points = {name}b; {name}m.base.count = {n1}; {name}m.height = {c_float_literal(value['height'])};")
            self.decl(f"{t} {name} = {prefix}prism({name}m);")
        elif kind in ("polygon", "polyline", "polygon2d", "polyline2d"):
            el = "RantDouble3" if kind in ("polygon", "polyline") else "RantDouble2"
            view = {"polygon": "RantPolygonView", "polyline": "RantPolylineView", "polygon2d": "RantPolygon2DView", "polyline2d": "RantPolyline2DView"}[kind]
            l1, n1 = c_array(w, name + "p", el, value)
            self.decl(l1)
            self.decl(f"{view} {name}m; {name}m.points = {name}p; {name}m.count = {n1};")
            self.decl(f"{t} {name} = {prefix}{kind}({name}m);")
        else:
            ct = {"aligned_box": "RantAlignedBox", "oriented_box": "RantOrientedBox", "sphere": "RantSphere", "capsule": "RantCapsule",
                  "cylinder": "RantCylinder", "cone": "RantCone", "segment": "RantSegment", "aligned_box2d": "RantAlignedBox2D",
                  "oriented_box2d": "RantOrientedBox2D", "circle": "RantCircle", "capsule2d": "RantCapsule2D", "segment2d": "RantSegment2D"}[kind]
            self.decl(f"{ct} {name}m = {w.literal(ct, value)};")
            self.decl(f"{t} {name} = {prefix}{kind}({name}m);")
        return {"ref": "&" + name}

    def size_of(self, out):
        """A C expression for how many items an output holds."""
        if out.kind == "room":
            return "n0"
        if out.kind == "query":
            if isinstance(out.size, type(out)):
                return f"n{self.f.outputs.index(out.size)}"
            return f"n{self.f.outputs.index(out)}"
        if out.kind in ("same",):
            inp = out.size
            return self.names[id(inp)]["count"]
        if out.kind == "sized":
            return self.names[id(out.size)]["value"]
        if out.kind == "fixed":
            return str(out.size)
        if out.kind == "expr":
            return self.expr(out.size)
        raise KeyError(out.kind)

    def expr(self, words):
        return self.matrix_expr(words)

    def matrix_expr(self, words):
        by_name = {i.name: self.names[id(i)] for i in self.f.inputs}
        if words == "min":
            a = by_name["a"]
            return str(min(a["rows"], a["cols"]))
        if words == "degree + 1":
            return by_name["degree"]["value"] + " + 1"
        input_name, _, word = words.partition(".")
        return str(by_name[input_name][word])

    def run(self):
        f, w = self.f, self.w
        self.inputs()
        args, after, pre_args = [], [], []
        self.options_extra = []
        for inp in f.inputs:
            n = self.names[id(inp)]
            if inp.kind in ("fit_options", "align_options") and n.get("name"):
                v, name = n["value"], n["name"]
                count = self.names[id(f.inputs[0])]["count"]
                if v.get("inliers"):
                    self.decl(f"uint8_t *{name}i = (uint8_t *)room({count}); {name}.inliers = {name}i;")
                    self.options_extra.append(("inliers", f"px({name}i, {count})"))
                if v.get("error"):
                    self.decl(f"double {name}e = 0; {name}.error = &{name}e;")
                    self.options_extra.append(("error", f"pd({name}e)"))
        queries = [o for o in f.outputs if o.kind in ("query", "cloud_select") and not isinstance(o.size, type(o))]
        for j, out in enumerate(f.outputs):
            self.emit_output(out, j)
        for a in f.args:
            args.append(self.c_arg(a, False))
            pre_args.append(self.c_arg(a, True))
        call = f"{f.c}({', '.join(args)})"
        ret = f.ret
        print_lines = self.print_outputs()
        if ret == "RantMathResult":
            self.decl("RantMathResult r;")
            if queries:
                self.decl(f"r = {f.c}({', '.join(pre_args)});")
                self.decl("if (r == RANT_MATH_OK || r == RANT_MATH_ERR_SPACE){")
                for j, out in enumerate(f.outputs):
                    if out.kind == "query" and not isinstance(out.size, type(out)):
                        el = self.element_size(out)
                        self.decl(f"    o{j} = room(n{j} * {out.per} * {el});")
                    elif out.kind == "query":
                        k = f.outputs.index(out.size)
                        self.decl(f"    o{j} = room(n{k} * {out.per} * {self.element_size(out)});")
                    elif out.kind == "cloud_select":
                        src = self.names[id(out.size)]
                        self.decl(f"    o{j} = room(n{j} * {src['cloud']}.point_step);")
                self.decl(f"    r = {call};")
                self.decl("}")
            else:
                self.decl(f"r = {call};")
            self.decl('if (r != RANT_MATH_OK){ fprintf(o, "{\\"e\\":%d", (int)r); } else {')
            self.decl('fputs("{\\"v\\":", o);')
            for line in print_lines:
                self.decl(line)
            for key, stmt in self.options_extra:
                self.decl(f'fputs(",\\"{key}\\":", o); {stmt};')
            self.decl("}")
        elif ret == "int":
            self.decl(f"int r = {call};")
            self.decl('fputs("{\\"v\\":", o);')
            if f.outputs:
                self.decl('if (!r) fputs("null", o); else {')
                for line in print_lines:
                    self.decl(line)
                self.decl("}")
            else:
                self.decl('fputs(r ? "true" : "false", o);')
        elif ret == "void":
            self.decl(f"{call};")
            self.decl('fputs("{\\"v\\":", o);')
            for line in print_lines:
                self.decl(line)
        else:
            self.decl(self.return_decl(ret, call))
            self.decl('fputs("{\\"v\\":", o);')
            for line in print_lines:
                self.decl(line)
        for inp in f.inputs:
            if inp.kind == "tree":
                self.decl(f"rant_kdtree_destroy({self.names[id(inp)]['tree']});")
        self.decl('fputs("}\\n", o);')
        return f"static void case_{self.index}(void){{\n" + "\n".join(self.lines) + "\n}\n"

    def return_decl(self, ret, call):
        ret = ret.replace("const ", "const ")
        return f"{ret} r = {call};"

    def element_size(self, out):
        return f"sizeof({out.type})"

    def emit_output(self, out, j):
        k = out.kind
        by_name = {i.name: self.names[id(i)] for i in self.f.inputs}
        if k == "return" or k in ("mutated", "field", "cloud_layout", "prism"):
            return
        if k == "out":
            self.decl(f"{out.type} o{j}; memset(&o{j}, 0, sizeof o{j});")
        elif k == "fixed":
            self.decl(f"{out.type} o{j}[{out.size}];")
        elif k == "query":
            if isinstance(out.size, type(out)):
                self.decl(f"{out.type} *o{j} = NULL;")
            else:
                self.decl(f"{out.type} *o{j} = NULL; uint32_t n{j} = 0;")
        elif k == "room":
            # kdtree_nearest: the count asked for is the room.
            self.decl(f"{out.type} *o{j} = ({out.type} *)room(sizeof({out.type}) * ({self.names[id(out.size)]['value']} ? {self.names[id(out.size)]['value']} : 1));")
            if j == 0:
                self.decl(f"uint32_t n0 = {self.names[id(out.size)]['value']};")
        elif k in ("same", "sized", "expr"):
            size = self.size_of(out)
            self.decl(f"{out.type} *o{j} = ({out.type} *)room(sizeof({out.type}) * ({size}));")
        elif k == "matrix":
            rows, cols = self.matrix_expr(out.rows), self.matrix_expr(out.cols)
            self.decl(f"RantMatrixView o{j}; o{j}.rows = {rows}; o{j}.cols = {cols}; o{j}.data = (double *)room(sizeof(double) * {rows} * {cols});")
        elif k == "image":
            fmt = self.image_word(out.size, by_name)
            width = self.image_word(out.cols, by_name)
            height = self.image_word(out.rows, by_name)
            self.decl(f"uint32_t w{j} = {width}, h{j} = {height}; RantImageFormat f{j} = (RantImageFormat){fmt}; size_t s{j} = out_bytes(f{j}, w{j}, h{j});")
            self.decl(f"RantImageView o{j}; o{j}.width = w{j}; o{j}.height = h{j}; o{j}.stride = 0; o{j}.format = f{j}; o{j}.data = (uint8_t *)room(s{j}); o{j}.size = s{j};")
        elif k == "image_copy":
            self.decl(f"RantImageView o{j}; memset(&o{j}, 0, sizeof o{j});")
        elif k == "cloud_select":
            self.decl(f"uint8_t *o{j} = NULL; uint32_t n{j} = 0;")
        else:
            raise KeyError(k)

    def image_word(self, word, by_name):
        src = by_name.get("src") or by_name.get("image")
        if word == "src.format":
            return str(src["format"])
        if word == "src.width":
            return str(src["width"])
        if word == "src.height":
            return str(src["height"])
        if word == "turned_width":
            return f"(((({by_name['quarter_turns']['value']}) % 4 + 4) % 4) & 1 ? {src['height']} : {src['width']})"
        if word == "turned_height":
            return f"(((({by_name['quarter_turns']['value']}) % 4 + 4) % 4) & 1 ? {src['width']} : {src['height']})"
        if word in ("ideal_width", "ideal_height"):
            axis = word[6:]
            ideal = self.row[[i.name for i in self.f.inputs].index("ideal")]
            camera = self.row[[i.name for i in self.f.inputs].index("camera")]
            return str((ideal or camera)[axis])
        return by_name[word]["value"]

    def c_arg(self, a, pre):
        k = a.kind
        names = self.names
        if k in ("value",):
            return names[id(a.a)]["value"]
        if k in ("ref", "opt_ref", "scan"):
            return names[id(a.a)]["ref"]
        if k == "list_ptr" or k == "det_ptr":
            return names[id(a.a)]["ptr"]
        if k == "list_count" or k == "det_count":
            return names[id(a.a)]["count"]
        if k == "degree":
            return f"(uint32_t)({names[id(a.a)]['count']} - 1)"
        if k == "view":
            n = names[id(a.a)]
            return f"rv_{a.b}({n['ptr']}, {n['count']})"
        if k == "mesh":
            v, t = names[id(a.a)], names[id(a.b)]
            return f"rv_mesh({v['ptr']}, {v['count']}, {t['ptr']}, {t['count']})"
        if k == "prism" or k == "matrix":
            return names[id(a.a)]["view"]
        if k in ("shape", "shape2d"):
            return names[id(a.a)]["ref"]
        if k == "image":
            return "&" + names[id(a.a)]["image"]
        if k == "image_mut":
            return "&" + names[id(a.a)]["image"]
        if k == "cloud" or k == "cloud_mut":
            return "&" + names[id(a.a)]["cloud"]
        if k == "grid" or k == "grid_mut":
            return "&" + names[id(a.a)]["grid"]
        if k == "string":
            return names[id(a.a)]["str"]
        if k in ("fit_options", "align_options"):
            return names[id(a.a)]["options"]
        if k == "tree":
            return names[id(a.a)]["tree"]
        if k == "out":
            j = self.f.outputs.index(a.a)
            out = a.a
            if out.kind == "out":
                return f"&o{j}"
            if out.kind == "image_copy":
                return f"&o{j}"
            return f"o{j}"
        if k == "count_ptr":
            j = self.f.outputs.index(a.a)
            return f"&n{j}" if a.a.kind != "room" else "&n0"
        if k == "matrix_out":
            return f"o{self.f.outputs.index(a.a)}"
        if k == "image_out":
            return f"&o{self.f.outputs.index(a.a)}"
        raise KeyError(k)

    def print_outputs(self):
        outs = self.f.outputs
        lines = []
        many = len(outs) > 1
        if many:
            lines.append("fputc('[', o);")
        for j, out in enumerate(outs):
            if j:
                lines.append("fputc(',', o);")
            lines += self.print_output(out, j)
        if many:
            lines.append("fputc(']', o);")
        if not outs:
            lines.append('fputs("null", o);')
        return lines

    def print_list(self, t, expr, n):
        if t == "uint8_t":
            return [f"px({expr}, {n});"]
        w = self.w
        return ["{ size_t k; fputc('[', o); for (k = 0; k < (size_t)(" + n + "); k++){ if (k) fputc(',', o); " + w.print_value(t, f"{expr}[k]") + "; } fputc(']', o); }"]

    def print_output(self, out, j):
        k = out.kind
        w = self.w
        if k == "return":
            if out.type == "string":
                return ['fputs("\\"\\"", o);']
            if len(self.f.outputs) > 1:
                return [w.print_value(out.type, "r") + ";"]
            return [w.print_value(out.type, "r") + ";"]
        if k == "out":
            return [w.print_value(out.type, f"o{j}") + ";"]
        if k in ("fixed", "same", "sized", "expr", "query", "room"):
            n = self.size_of(out)
            if out.per != 1:
                n = f"{n} * {out.per}"
            return self.print_list(out.type, f"o{j}", n)
        if k == "matrix":
            return [f'fprintf(o, "{{\\"rows\\":%u,\\"cols\\":%u,\\"data\\":", o{j}.rows, o{j}.cols);'] + \
                self.print_list("double", f"o{j}.data", f"(size_t)o{j}.rows * o{j}.cols") + ["fputc('}', o);"]
        if k == "image":
            return [f"pimage(&o{j}, s{j});"]
        if k == "image_copy":
            # The binding's copy is packed, so the answer is written packed too.
            return [f"{{ size_t s = out_bytes(o{j}.format, o{j}.width, o{j}.height); RantImageView c = o{j}; c.stride = 0; c.data = (uint8_t *)room(s); c.size = s; "
                    f"if (rant_image_convert(&o{j}, &c) != RANT_MATH_OK) abort(); pimage(&c, s); }}"]
        if k == "mutated":
            inp = out.size
            n = self.names[id(inp)]
            if inp.kind == "image_mut":
                return [f"pimage(&{n['image']}, (size_t)({n['size']}));"]
            if inp.kind == "cloud_mut":
                return [f"pcloud(&{n['cloud']}, {n['cloud']}.data, (size_t)({n['size']}));"]
            if inp.kind == "grid_mut":
                return self.print_grid(n)
        if k == "cloud_layout":
            return ["pcloud(&r, NULL, 0);"]
        if k == "cloud_select":
            src = self.names[id(out.size)]
            return [f"pcloud(&{src['cloud']}, o{j}, (size_t)n{j} * {src['cloud']}.point_step);"]
        if k == "field":
            return ['if (!r) fputs("null", o); else pfield(r);']
        if k == "prism":
            return ['fputs("{\\"pose\\":", o);', w.print_value("RantPose", "r.pose") + ";",
                    'fputs(",\\"base\\":{\\"points\\":", o);'] + self.print_list("RantDouble2", "r.base.points", "r.base.count") + [
                    'fputs("},\\"height\\":", o);', "pd(r.height);", "fputc('}', o);"]
        raise KeyError(k)

    def print_grid(self, n):
        name = n["grid"]
        w = self.w
        return [f'fputs("{{\\"origin\\":", o);', w.print_value("RantPose", f"{name}.origin") + ";",
                f'fputs(",\\"resolution\\":", o); pd({name}.resolution);',
                f'fprintf(o, ",\\"cells_x\\":%u,\\"cells_y\\":%u,\\"cells_z\\":%u,\\"fields\\":[", {name}.cells_x, {name}.cells_y, {name}.cells_z);',
                f'{{ uint32_t k; for (k = 0; k < {name}.field_count; k++){{ const RantGridFieldView *g = &{name}.fields[k]; if (k) fputc(\',\', o); fputs("{{\\"name\\":", o); pt(g->name, g->name_len); fprintf(o, ",\\"kind\\":%d,\\"data\\":", (int)g->kind); px(g->data, g->size); fputc(\'}}\', o); }} }}',
                "fputs(\"]}\", o);"]


def add_pose_literal(w):
    w.pose_literal = lambda v: "rv_pose((RantPose)" + w.literal("RantPose", v) + ")"


def write_record_program(model, cases):
    w = CWriter(model)
    add_pose_literal(w)
    bodies = []
    for i, (f, row) in enumerate(cases):
        bodies.append(CCase(w, f, row, i).run())
    head = [C_HEAD, "static RantPose rv_pose(RantPose p){ return p; }\n"]
    for view, (el, word) in {
        "RantPolygonView": ("RantDouble3", "points"), "RantPolygon2DView": ("RantDouble2", "points"),
        "RantPolylineView": ("RantDouble3", "points"), "RantPolyline2DView": ("RantDouble2", "points"),
        "RantPathView": ("RantPose", "poses"), "RantPath2DView": ("RantPose2D", "poses"),
    }.items():
        head.append(f"static {view} rv_{view}(const {el} *p, uint32_t n){{ {view} v; v.{word} = p; v.count = n; return v; }}\n")
    head.append("static RantMeshView rv_mesh(const RantDouble3 *v, uint32_t nv, const uint32_t *t, uint32_t nt){ "
                "RantMeshView m; m.vertices = v; m.vertex_count = nv; m.triangles = t; m.triangle_count = nt / 3; return m; }\n")
    tail = ["int main(int argc, char **argv){", "    (void)argc;", '    o = fopen(argv[1], "w");', "    if (!o) return 1;"]
    for i in range(len(cases)):
        tail.append(f"    case_{i}();")
    tail += ["    fclose(o);", "    return 0;", "}", ""]
    return "".join(head) + "\n".join(w.head) + "\n" + "\n".join(bodies) + "\n" + "\n".join(tail)


def tolerance_of(model, f):
    types = [o.type for o in f.outputs]
    def has_float(t, seen=()):
        if t == "float":
            return True
        s = model.structs.get(t)
        return bool(s) and any(has_float(x.type) for x in s.fields)
    return FLOAT_TOLERANCE if any(has_float(t) for t in types) else DOUBLE_TOLERANCE


def input_words(i):
    k = i.kind
    if k in ("value", "ref", "opt_ref", "list"):
        return f"{k}:{i.type}"
    return k


def vectors_file(model, cases, answers):
    """vectors.json: the functions, each with its case rows, one line per case."""
    if len(cases) != len(answers):
        raise ValueError(f"{len(cases)} cases, {len(answers)} answers")
    lines = []
    last = None
    for (f, row), answer in zip(cases, answers):
        if f is not last:
            lines.append({"f": f.name, "in": [input_words(i) for i in f.inputs], "tol": tolerance_of(model, f),
                          "many": len(f.outputs) > 1, "cases": []})
            last = f
        lines[-1]["cases"].append({"a": clean(row), **answer})
    out = ['{"functions":[']
    for k, entry in enumerate(lines):
        cases_text = ",\n".join(json.dumps(c, separators=(",", ":")) for c in entry["cases"])
        head = {x: entry[x] for x in ("f", "in", "tol", "many")}
        text = json.dumps(head, separators=(",", ":"))[:-1] + ',"cases":[\n' + cases_text + "\n]}"
        out.append(text + ("," if k + 1 < len(lines) else ""))
    out.append("]}")
    return "\n".join(out) + "\n"


def close(expected, got, tol):
    """Whether two answers agree, numbers within tol of their size and everything else exactly."""
    if isinstance(expected, bool) or isinstance(got, bool):
        return expected == got
    if isinstance(expected, (int, float)) and isinstance(got, (int, float)):
        if expected == got:
            return True
        return abs(expected - got) <= tol * max(1.0, abs(expected), abs(got))
    if isinstance(expected, list) and isinstance(got, list):
        return len(expected) == len(got) and all(close(a, b, tol) for a, b in zip(expected, got))
    if isinstance(expected, dict) and isinstance(got, dict):
        return expected.keys() == got.keys() and all(close(expected[k], got[k], tol) for k in expected)
    return expected == got
