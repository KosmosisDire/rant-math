# What every backend reads: math's C headers parsed into types and functions, and each function
# sorted by the kinds of its parameters into what a binding takes and gives back.
import os
import re
from dataclasses import dataclass, field

SCALARS = {
    "double", "float", "int", "int8_t", "uint8_t", "int16_t", "uint16_t", "int32_t", "uint32_t",
    "int64_t", "uint64_t", "size_t",
}

# A C view of a list by value becomes the list itself in a binding.
LIST_VIEWS = {
    "RantPolygonView": ("RantDouble3", "points"),
    "RantPolygon2DView": ("RantDouble2", "points"),
    "RantPolylineView": ("RantDouble3", "points"),
    "RantPolyline2DView": ("RantDouble2", "points"),
    "RantPathView": ("RantPose", "poses"),
    "RantPath2DView": ("RantPose2D", "poses"),
}

# Words C may use for a parameter that some language keeps for itself.
RENAMES = {"in": "points", "l": "points"}

# Pointer parameters that stand for one value by their type alone.
OBJECT_KINDS = {
    "RantShape": "shape", "RantShape2D": "shape2d", "RantImageView": "image",
    "RantPointCloudView": "cloud", "RantGridView": "grid", "RantLaserScanView": "scan",
    "RantFitOptions": "fit_options", "RantAlignOptions": "align_options", "RantKdTree": "tree",
}

# Struct types the bindings never see as they are, since a binding's own type stands for them.
HIDDEN_STRUCTS = {
    "RantShape", "RantShape2D", "RantImageView", "RantPointFieldView", "RantPointCloudView",
    "RantGridFieldView", "RantGridView", "RantLaserScanView", "RantFitOptions", "RantAlignOptions",
    "RantMatrixView", "RantPrismView", "RantMeshView", "RantDetection2DView", "RantDetection3DView",
} | set(LIST_VIEWS)

HIDDEN_ENUMS = {"RantMathResult", "RantShapeKind", "RantShape2DKind"}


@dataclass
class CField:
    name: str
    type: str
    pointer: bool = False
    dims: tuple = ()


@dataclass
class Struct:
    name: str
    fields: list
    std: bool  # a standard type, which each language has from Rant

    @property
    def short(self):
        return self.name[4:]


@dataclass
class Enum:
    name: str
    options: list  # (constant, value)
    std: bool

    @property
    def short(self):
        return self.name[4:]

    def option_names(self):
        # RANT_AXES_OPENCV_CAMERA is OpencvCamera, once the prefix all options share is gone.
        prefix = "RANT_" + camel_to_upper(self.short) + "_"
        names = []
        words = [c for c, _ in self.options]
        common = os.path.commonprefix(words)
        common = common[: common.rfind("_") + 1]
        if all(w.startswith(prefix) for w in words):
            common = prefix
        for c, _ in self.options:
            names.append("".join(w.capitalize() for w in c[len(common):].split("_")))
        return names


def camel_to_upper(s):
    s = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", s)
    return s.upper()


@dataclass
class CParam:
    name: str
    type: str
    const: bool = False
    pointer: int = 0
    dim: int = 0


@dataclass
class Input:
    name: str
    kind: str   # value ref opt_ref list prism shape shape2d image image_mut cloud cloud_mut grid grid_mut
                # scan det2d det3d matrix fit_options align_options tree string
    type: str = ""


@dataclass
class Output:
    name: str
    kind: str   # return out fixed query sized same expr matrix image cloud field mutated handle
    type: str = ""
    size: object = None     # fixed: N. sized: the input that gives the count. same: the input it matches.
                            # expr: a size in words a backend reads, see SIZE_EXPRS.
    per: int = 1            # how many numbers per counted item, 3 for triangles
    rows: str = ""
    cols: str = ""
    optional: bool = False  # the C pointer may be NULL


@dataclass
class Arg:
    kind: str   # value ref opt_ref list_ptr list_count degree view mesh prism shape shape2d image cloud grid scan
                # det_ptr det_count matrix string options tree out out_size count_ptr null
                # matrix_out image_out cloud_data
    a: object = None
    b: object = None


@dataclass
class Func:
    c: str
    name: str
    header: str
    ret: str                    # the C return type
    params: list
    doc: str = ""
    inputs: list = field(default_factory=list)
    outputs: list = field(default_factory=list)
    args: list = field(default_factory=list)
    errors: str = "none"        # none throw optional bool
    hand: bool = False          # each backend writes this one's body itself
    vector: bool = True         # takes part in the shared vectors


@dataclass
class Model:
    structs: dict
    enums: dict
    functions: list
    version: str

    def func(self, c):
        for f in self.functions:
            if f.c == c:
                return f
        raise KeyError(c)

    def own_structs(self):
        return [s for s in self.structs.values() if not s.std and s.name not in HIDDEN_STRUCTS]

    def own_enums(self):
        return [e for e in self.enums.values() if not e.std and e.name not in HIDDEN_ENUMS]

    def used_structs(self):
        # Each struct some function passes or returns by value, inner ones first.
        seen, out = set(), []
        def visit(t):
            s = self.structs.get(t)
            if s is None or t in seen or t in HIDDEN_STRUCTS:
                return
            seen.add(t)
            for f in s.fields:
                visit(f.type)
            out.append(s)
        for f in self.functions:
            for i in f.inputs:
                visit(i.type)
            for o in f.outputs:
                visit(o.type)
        return out

    def used_enums(self):
        seen, out = set(), []
        for f in self.functions:
            for t in [i.type for i in f.inputs] + [o.type for o in f.outputs]:
                e = self.enums.get(t)
                if e and t not in seen and t not in HIDDEN_ENUMS:
                    seen.add(t)
                    out.append(e)
        return out


def strip_comments(text):
    return re.sub(r"/\*.*?\*/", " ", text, flags=re.S)


def parse_decl(text):
    """A declaration without its name split off: returns (type, const, pointers, name, dims)."""
    t = " ".join(text.split())
    dims = tuple(int(d) for d in re.findall(r"\[(\d+)\]", t))
    t = re.sub(r"\[\d*\]", "", t).strip()
    m = re.match(r"(const\s+)?([A-Za-z_]\w*)\s*(\*+)?\s*(?:const\s+)?([A-Za-z_]\w*)?$", t)
    if not m:
        raise ValueError("cannot read declaration: " + text)
    return m.group(2), bool(m.group(1)), len(m.group(3) or ""), m.group(4) or "", dims


def parse_structs(text, std, structs, enums):
    clean = strip_comments(text)
    clean = re.sub(r"^\s*#.*$", " ", clean, flags=re.M)
    for m in re.finditer(r"typedef\s+struct\s*\w*\s*\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}\s*(\w+)\s*;", clean, flags=re.S):
        body, name = m.group(1), m.group(2)
        fields = []
        if "union" not in body:
            for decl in body.split(";"):
                decl = decl.strip()
                if not decl:
                    continue
                parts = [p.strip() for p in decl.split(",")]
                t, _, ptrs, n, dims = parse_decl(parts[0])
                fields.append(CField(n, t, bool(ptrs), dims))
                for p in parts[1:]:
                    extra_ptrs = p.count("*")
                    extra_dims = tuple(int(d) for d in re.findall(r"\[(\d+)\]", p))
                    n2 = re.sub(r"\[\d*\]", "", p).replace("*", "").strip()
                    fields.append(CField(n2, t, bool(extra_ptrs or ptrs), extra_dims))
        structs[name] = Struct(name, fields, std)
    for m in re.finditer(r"typedef\s+enum\s*\w*\s*\{(.*?)\}\s*(\w+)\s*;", clean, flags=re.S):
        options, value = [], 0
        for item in m.group(1).split(","):
            item = item.strip()
            if not item:
                continue
            if "=" in item:
                c, v = item.split("=")
                c, value = c.strip(), int(v.strip())
            else:
                c = item
            options.append((c, value))
            value += 1
        enums[m.group(2)] = Enum(m.group(2), options, std)


def clean_doc(comment):
    # Only what also holds in a binding: the sentences that speak of C's pointers and counts go.
    words = " ".join(comment.replace("*", " ").split())
    sentences = re.split(r"(?<=[.:])\s+", words)
    keep = [s for s in sentences if not re.search(
        r"\bNULL\b|\bcount\b|\bcounts\b|rant_|RANT_|ERR_|\bOK\b|\bout\b|out_count|\bpointer\b|holds the room|\bdst\b|\bview\b", s)]
    return " ".join(keep)


def parse_functions(path, name):
    text = open(path, encoding="utf-8").read()
    pattern = re.compile(r"(/\*(?:(?!\*/).)*?\*/\s*)?RANT_MATH_API\s+([^;{(]+?)\s*\b(rant_\w+)\s*\(([^;]*?)\)\s*;([ \t]*/\*.*?\*/)?", re.S)
    out = []
    no_cpp = re.sub(r"^\s*#.*$", " ", text, flags=re.M)
    for m in pattern.finditer(no_cpp):
        comment = (m.group(5) or "") or (m.group(1) or "")
        comment = re.sub(r"/\*|\*/", " ", comment)
        ret = " ".join(m.group(2).split())
        fn = m.group(3)
        params = []
        arglist = strip_comments(m.group(4)).strip()
        if arglist != "void":
            for a in arglist.split(","):
                t, const, ptrs, n, dims = parse_decl(a)
                params.append(CParam(n, t, const, ptrs, dims[0] if dims else 0))
        out.append(Func(fn, fn[5:], name, ret, params, clean_doc(comment)))
    return out


SPECIAL = {}


def special(c):
    def wrap(fn):
        SPECIAL[c] = fn
        return fn
    return wrap


def classify(f, m):
    if f.c in SPECIAL:
        SPECIAL[f.c](f, m)
    else:
        auto(f, m)
    if f.ret == "RantMathResult":
        f.errors = "throw"
    elif f.ret == "int":
        f.errors = "optional" if f.outputs else "bool"


def auto(f, m):
    """The rule for every regular function: read the C parameters left to right."""
    ps = f.params
    i = 0
    list_inputs = {}
    last_list = None
    while i < len(ps):
        p = ps[i]
        name = RENAMES.get(p.name, p.name)
        nxt = ps[i + 1] if i + 1 < len(ps) else None
        if p.dim:                                                    # T out[N]
            out = Output(p.name, "fixed", p.type, p.dim)
            f.outputs.append(out)
            f.args.append(Arg("out", out))
        elif p.type in LIST_VIEWS and not p.pointer:                 # a view of a list
            elem, word = LIST_VIEWS[p.type]
            inp = Input(word, "list", elem)
            f.inputs.append(inp)
            f.args.append(Arg("view", inp, p.type))
            last_list = inp
        elif p.type == "RantMeshView" and not p.pointer:
            v = Input("vertices", "list", "RantDouble3")
            t = Input("triangles", "list", "uint32_t")
            f.inputs += [v, t]
            f.args.append(Arg("mesh", v, t))
            last_list = v
        elif p.type == "RantPrismView" and not p.pointer:
            inp = Input("prism", "prism", "RantPrism")
            f.inputs.append(inp)
            f.args.append(Arg("prism", inp))
        elif p.pointer and p.const and p.type in OBJECT_KINDS:
            inp = Input(name, OBJECT_KINDS[p.type], p.type)
            f.inputs.append(inp)
            f.args.append(Arg(OBJECT_KINDS[p.type], inp))
        elif p.pointer and p.const and nxt is not None and nxt.type == "uint32_t" and not nxt.pointer and "count" in nxt.name:
            inp = Input(name, "list", p.type)                        # const T *a, uint32_t count
            f.inputs.append(inp)
            f.args.append(Arg("list_ptr", inp))
            f.args.append(Arg("list_count", inp))
            last_list = inp
            i += 1
        elif p.pointer and p.const and p.type == "char":
            inp = Input(name, "string", "char")
            f.inputs.append(inp)
            f.args.append(Arg("string", inp))
        elif p.pointer and p.const:
            if nxt is not None and nxt.pointer and not nxt.const and i + 2 < len(ps) and ps[i + 2].type == "uint32_t" and not ps[i + 2].pointer:
                inp = Input(name, "list", p.type)                    # const T *in, U *out, uint32_t count
                out = Output(nxt.name, "same", nxt.type, inp)
                f.inputs.append(inp)
                f.outputs.append(out)
                f.args += [Arg("list_ptr", inp), Arg("out", out), Arg("list_count", inp)]
                last_list = inp
                i += 3
                continue
            if nxt is not None and nxt.pointer and nxt.const and i + 2 < len(ps) and ps[i + 2].type == "uint32_t" and not ps[i + 2].pointer:
                a = Input(name, "list", p.type)                      # const T *a, const T *b, uint32_t count
                b = Input(RENAMES.get(nxt.name, nxt.name), "list", nxt.type)
                f.inputs += [a, b]
                f.args += [Arg("list_ptr", a), Arg("list_ptr", b), Arg("list_count", a, b)]
                last_list = a
                i += 3
                continue
            inp = Input(name, "ref", p.type)                         # const T *one
            f.inputs.append(inp)
            f.args.append(Arg("ref", inp))
        elif p.pointer:
            if nxt is not None and nxt.type == "uint32_t" and nxt.pointer:
                out = Output(p.name, "query", p.type)                # T *out, uint32_t *count
                f.outputs.append(out)
                f.args += [Arg("out", out), Arg("count_ptr", out)]
                i += 2
                continue
            if nxt is not None and nxt.type == "uint32_t" and not nxt.pointer:
                count = Input(RENAMES.get(nxt.name, nxt.name), "value", "uint32_t")
                out = Output(p.name, "sized", p.type, count)         # T *out, uint32_t count
                f.inputs.append(count)
                f.outputs.append(out)
                f.args += [Arg("out", out), Arg("value", count)]
                i += 2
                continue
            if p.type == "uint8_t" or p.name in ("normals", "labels"):
                out = Output(p.name, "same", p.type, last_list)      # one for each of the points before
                f.outputs.append(out)
                f.args.append(Arg("out", out))
            else:
                out = Output(p.name, "out", p.type)                  # T *out
                f.outputs.append(out)
                f.args.append(Arg("out", out))
        else:
            inp = Input(name, "value", p.type)
            f.inputs.append(inp)
            f.args.append(Arg("value", inp))
        i += 1
    if f.ret in ("RantShape", "RantShape2D"):
        f.outputs.insert(0, Output("return", "shape_new", f.ret))     # a binding's own Shape holds the data
        f.vector = False
    elif f.ret not in ("void", "int", "RantMathResult"):
        f.outputs.insert(0, Output("return", "return", normal_type(f.ret)))
    if not f.outputs and f.ret == "void":
        raise ValueError("no answer from " + f.c)


def normal_type(t):
    return t.replace("const ", "").replace("*", "").strip()


# The functions no rule can read from their C types alone.

@special("rant_math_result_text")
def _(f, m):
    f.vector = False
    f.hand = True
    f.outputs = [Output("return", "return", "string")]
    f.inputs = [Input("r", "value", "RantMathResult")]
    f.args = [Arg("value", f.inputs[0])]


@special("rant_quaternion_to_axis_angle")
def _(f, m):
    q = Input("q", "value", "RantQuaternion")
    axis = Output("axis", "out", "RantDouble3")
    angle = Output("angle", "return", "double")
    f.inputs = [q]
    f.outputs = [axis, angle]
    f.args = [Arg("value", q), Arg("out", axis)]


def matrix_inputs(f, names):
    inputs = [Input(n, "matrix") for n in names]
    f.inputs = inputs
    return inputs


def matrix_fn(rows, cols, inputs=("a",)):
    def apply(f, m):
        ins = matrix_inputs(f, inputs)
        out = Output("out", "matrix", rows=rows, cols=cols)
        f.outputs = [out]
        f.args = [Arg("matrix", i) for i in ins] + [Arg("matrix_out", out)]
    return apply


SPECIAL["rant_matrix_multiply"] = matrix_fn("a.rows", "b.cols", ("a", "b"))
SPECIAL["rant_matrix_transpose"] = matrix_fn("a.cols", "a.rows")
SPECIAL["rant_matrix_inverse"] = matrix_fn("a.rows", "a.cols")
SPECIAL["rant_matrix_solve"] = matrix_fn("a.cols", "b.cols", ("a", "b"))
SPECIAL["rant_matrix_cholesky"] = matrix_fn("a.rows", "a.cols")


@special("rant_matrix_determinant")
def _(f, m):
    a, = matrix_inputs(f, ["a"])
    out = Output("out", "out", "double")
    f.outputs = [out]
    f.args = [Arg("matrix", a), Arg("out", out)]


@special("rant_matrix_svd")
def _(f, m):
    a, = matrix_inputs(f, ["a"])
    u = Output("u", "matrix", rows="a.rows", cols="min")
    s = Output("s", "expr", "double", "min")
    v = Output("v", "matrix", rows="a.cols", cols="min")
    f.outputs = [u, s, v]
    f.args = [Arg("matrix", a), Arg("matrix_out", u), Arg("out", s), Arg("matrix_out", v)]


@special("rant_matrix_eigen")
def _(f, m):
    a, = matrix_inputs(f, ["a"])
    values = Output("values", "expr", "double", "a.rows")
    vectors = Output("vectors", "matrix", rows="a.rows", cols="a.rows")
    f.outputs = [values, vectors]
    f.args = [Arg("matrix", a), Arg("out", values), Arg("matrix_out", vectors)]


@special("rant_polygon_flatten")
def _(f, m):
    pts = Input("points", "list", "RantDouble3")
    frame = Output("frame", "out", "RantPose")
    out = Output("out", "same", "RantDouble2", pts)
    f.inputs = [pts]
    f.outputs = [frame, out]
    f.args = [Arg("view", pts, "RantPolygonView"), Arg("out", frame), Arg("out", out)]


@special("rant_mesh_normals")
def _(f, m):
    v = Input("vertices", "list", "RantDouble3")
    t = Input("triangles", "list", "uint32_t")
    out = Output("normals", "same", "RantDouble3", v)
    f.inputs = [v, t]
    f.outputs = [out]
    f.args = [Arg("mesh", v, t), Arg("out", out)]


@special("rant_polygon2d_triangulate")
def _(f, m):
    pts = Input("points", "list", "RantDouble2")
    out = Output("triangles", "query", "uint32_t", per=3)
    f.inputs = [pts]
    f.outputs = [out]
    f.args = [Arg("view", pts, "RantPolygon2DView"), Arg("out", out), Arg("count_ptr", out)]


def shape_gap(kind, seg):
    def apply(f, m):
        a, b = Input("a", kind, f.params[0].type), Input("b", kind, f.params[0].type)
        gap = Output("gap", "out", "double")
        between = Output("between", "out", seg)
        f.inputs = [a, b]
        f.outputs = [gap, between]
        f.args = [Arg(kind, a), Arg(kind, b), Arg("out", gap), Arg("out", between)]
    return apply


SPECIAL["rant_shape_gap"] = shape_gap("shape", "RantSegment")
SPECIAL["rant_shape2d_gap"] = shape_gap("shape2d", "RantSegment2D")


@special("rant_shape_mesh")
def _(f, m):
    s = Input("s", "shape", "RantShape")
    seg = Input("segments", "value", "uint32_t")
    v = Output("vertices", "query", "RantDouble3")
    t = Output("triangles", "query", "uint32_t", per=3)
    f.inputs = [s, seg]
    f.outputs = [v, t]
    f.args = [Arg("shape", s), Arg("value", seg), Arg("out", v), Arg("count_ptr", v), Arg("out", t), Arg("count_ptr", t)]


@special("rant_polynomial_fit")
def _(f, m):
    x = Input("x", "list", "double")
    y = Input("y", "list", "double")
    degree = Input("degree", "value", "uint32_t")
    options = Input("options", "fit_options", "RantFitOptions")
    out = Output("coefficients", "expr", "double", "degree + 1")
    f.inputs = [x, y, degree, options]
    f.outputs = [out]
    f.args = [Arg("list_ptr", x), Arg("list_ptr", y), Arg("list_count", x, y), Arg("value", degree),
              Arg("fit_options", options), Arg("out", out)]


@special("rant_polynomial_at")
def _(f, m):
    c = Input("coefficients", "list", "double")
    x = Input("x", "value", "double")
    f.inputs = [c, x]
    f.outputs = [Output("return", "return", "double")]
    f.args = [Arg("list_ptr", c), Arg("degree", c), Arg("value", x)]


@special("rant_points_clusters")
def _(f, m):
    pts = Input("points", "list", "RantDouble3")
    d = Input("distance", "value", "double")
    mp = Input("min_points", "value", "uint32_t")
    labels = Output("labels", "same", "uint32_t", pts)
    clusters = Output("clusters", "out", "uint32_t")
    f.inputs = [pts, d, mp]
    f.outputs = [labels, clusters]
    f.args = [Arg("list_ptr", pts), Arg("list_count", pts), Arg("value", d), Arg("value", mp), Arg("out", labels), Arg("out", clusters)]


@special("rant_points_align")
def _(f, m):
    s = Input("source", "list", "RantDouble3")
    t = Input("target", "list", "RantDouble3")
    g = Input("guess", "value", "RantPose")
    o = Input("options", "align_options", "RantAlignOptions")
    out = Output("out", "out", "RantPose")
    f.inputs = [s, t, g, o]
    f.outputs = [out]
    f.args = [Arg("list_ptr", s), Arg("list_count", s), Arg("list_ptr", t), Arg("list_count", t),
              Arg("value", g), Arg("align_options", o), Arg("out", out)]


@special("rant_kdtree_create")
def _(f, m):
    pts = Input("points", "list", "RantDouble3")
    out = Output("tree", "handle", "RantKdTree")
    f.inputs = [pts]
    f.outputs = [out]
    f.args = [Arg("list_ptr", pts), Arg("list_count", pts), Arg("out", out)]
    f.vector = False


@special("rant_kdtree_destroy")
def _(f, m):
    f.vector = False
    f.hand = True


@special("rant_kdtree_nearest")
def _(f, m):
    tree = Input("tree", "tree", "RantKdTree")
    p = Input("point", "value", "RantDouble3")
    k = Input("count", "value", "uint32_t")
    i = Output("indices", "room", "uint32_t", k)
    d = Output("distances", "room", "double", k)
    f.inputs = [tree, p, k]
    f.outputs = [i, d]
    f.args = [Arg("tree", tree), Arg("value", p), Arg("out", i), Arg("out", d), Arg("count_ptr", i)]


@special("rant_kdtree_within")
def _(f, m):
    tree = Input("tree", "tree", "RantKdTree")
    p = Input("point", "value", "RantDouble3")
    r = Input("radius", "value", "double")
    i = Output("indices", "query", "uint32_t")
    d = Output("distances", "query", "double", i)   # shares the count of indices
    f.inputs = [tree, p, r]
    f.outputs = [i, d]
    f.args = [Arg("tree", tree), Arg("value", p), Arg("value", r), Arg("out", i), Arg("out", d), Arg("count_ptr", i)]


def image_out(name="dst", format="src.format", width="src.width", height="src.height"):
    return Output(name, "image", "RantImageView", rows=height, cols=width, size=format)


def image_fn(extra, dst=None):
    # (const RantImageView *src, extras..., RantImageView *dst)
    def apply(f, m):
        src = Input("src", "image", "RantImageView")
        f.inputs = [src]
        f.args = [Arg("image", src)]
        for p in f.params[1:-1]:
            if p.pointer:
                raise ValueError(f.c)
            inp = Input(p.name, "value", p.type)
            f.inputs.append(inp)
            f.args.append(Arg("value", inp))
        out = dst() if dst else image_out()
        for e in extra:
            f.inputs.append(e)
        f.outputs = [out]
        f.args.append(Arg("image_out", out))
    return apply


def named_format():
    return Input("format", "value", "RantImageFormat")


SPECIAL["rant_image_convert"] = image_fn([named_format()], lambda: image_out(format="format"))
SPECIAL["rant_image_colorize"] = image_fn([named_format()], lambda: image_out(format="format"))
SPECIAL["rant_image_flip_horizontal"] = image_fn([])
SPECIAL["rant_image_flip_vertical"] = image_fn([])
SPECIAL["rant_image_rotate"] = image_fn([], lambda: image_out(width="turned_width", height="turned_height"))
SPECIAL["rant_image_resize"] = image_fn([Input("width", "value", "uint32_t"), Input("height", "value", "uint32_t")],
                                        lambda: image_out(width="width", height="height"))


@special("rant_image_undistort")
def _(f, m):
    src = Input("src", "image", "RantImageView")
    cam = Input("camera", "ref", "RantCameraIntrinsics")
    ideal = Input("ideal", "opt_ref", "RantCameraIntrinsics")
    filt = Input("filter", "value", "RantImageFilter")
    out = image_out(width="ideal_width", height="ideal_height")
    f.inputs = [src, cam, ideal, filt]
    f.outputs = [out]
    f.args = [Arg("image", src), Arg("ref", cam), Arg("opt_ref", ideal), Arg("value", filt), Arg("image_out", out)]


@special("rant_image_crop")
def _(f, m):
    # Crop shares data in C, where the caller holds the lifetime. A binding hands back its own copy.
    image = Input("image", "image", "RantImageView")
    ins = [Input(n, "value", "uint32_t") for n in ("x", "y", "width", "height")]
    out = Output("out", "image_copy", "RantImageView")
    f.inputs = [image] + ins
    f.outputs = [out]
    f.args = [Arg("image", image)] + [Arg("value", i) for i in ins] + [Arg("out", out)]


@special("rant_image_draw")
def _(f, m):
    image = Input("image", "image_mut", "RantImageView")
    shape = Input("shape", "shape2d", "RantShape2D")
    color = Input("color", "value", "RantColor")
    width = Input("width", "value", "double")
    f.inputs = [image, shape, color, width]
    f.outputs = [Output("image", "mutated", "RantImageView", image)]
    f.args = [Arg("image_mut", image), Arg("shape2d", shape), Arg("value", color), Arg("value", width)]


@special("rant_point_cloud_layout")
def _(f, m):
    layout = Input("layout", "value", "RantPointCloudLayout")
    f.inputs = [layout]
    f.outputs = [Output("return", "cloud_layout", "RantPointCloudView")]
    f.args = [Arg("value", layout)]


@special("rant_point_cloud_field")
def _(f, m):
    cloud = Input("cloud", "cloud", "RantPointCloudView")
    name = Input("name", "string", "char")
    f.inputs = [cloud, name]
    f.outputs = [Output("return", "field", "RantPointFieldView")]
    f.args = [Arg("cloud", cloud), Arg("string", name)]
    f.errors = "none"


def cloud_write(element, words=None):
    def apply(f, m):
        cloud = Input("cloud", "cloud_mut", "RantPointCloudView")
        f.inputs = [cloud]
        f.args = [Arg("cloud_mut", cloud)]
        if len(f.params) == 4:
            field_in = Input(f.params[1].name, "string", "char")
            f.inputs.append(field_in)
            f.args.append(Arg("string", field_in))
        values = Input(words or f.params[-2].name, "list", element)
        f.inputs.append(values)
        f.args += [Arg("list_ptr", values), Arg("list_count", values)]
        f.outputs = [Output("cloud", "mutated", "RantPointCloudView", cloud)]
    return apply


SPECIAL["rant_point_cloud_write_points"] = cloud_write("RantDouble3")
SPECIAL["rant_point_cloud_write_values"] = cloud_write("double")
SPECIAL["rant_point_cloud_write_colors"] = cloud_write("RantColor")


@special("rant_point_cloud_transform")
def _(f, m):
    cloud = Input("cloud", "cloud_mut", "RantPointCloudView")
    pose = Input("pose", "value", "RantPose")
    f.inputs = [cloud, pose]
    f.outputs = [Output("cloud", "mutated", "RantPointCloudView", cloud)]
    f.args = [Arg("cloud_mut", cloud), Arg("value", pose)]


@special("rant_point_cloud_select")
def _(f, m):
    cloud = Input("cloud", "cloud", "RantPointCloudView")
    keep = Input("keep", "list", "uint8_t")
    out = Output("out", "cloud_select", "RantPointCloudView", cloud)
    f.inputs = [cloud, keep]
    f.outputs = [out]
    f.args = [Arg("cloud", cloud), Arg("list_ptr", keep), Arg("out", out), Arg("count_ptr", out)]


@special("rant_grid_write_value")
def _(f, m):
    grid = Input("grid", "grid_mut", "RantGridView")
    name = Input("field", "string", "char")
    cell = Input("cell", "value", "RantInt3")
    value = Input("value", "value", "double")
    f.inputs = [grid, name, cell, value]
    f.outputs = [Output("grid", "mutated", "RantGridView", grid)]
    f.args = [Arg("grid_mut", grid), Arg("string", name), Arg("value", cell), Arg("value", value)]


def detections(kind, element):
    def apply(f, m):
        d = Input("detections", kind, element)
        iou = Input("iou", "value", "double")
        keep = Output("keep", "same", "uint8_t", d)
        f.inputs = [d, iou]
        f.outputs = [keep]
        f.args = [Arg("det_ptr", d), Arg("det_count", d), Arg("value", iou), Arg("out", keep)]
    return apply


SPECIAL["rant_detections2d_suppress"] = detections("det2d", "RantDetection2DView")
SPECIAL["rant_detections3d_suppress"] = detections("det3d", "RantDetection3DView")


@special("rant_prism_transform")
def _(f, m):
    prism = Input("prism", "prism", "RantPrism")
    pose = Input("pose", "value", "RantPose")
    f.inputs = [prism, pose]
    f.outputs = [Output("return", "prism", "RantPrism")]
    f.args = [Arg("prism", prism), Arg("value", pose)]


def read_model(root, types_path):
    src = os.path.join(root, "src")
    order = re.findall(r'#include "(\w+\.h)"', open(os.path.join(src, "rant_math.h"), encoding="utf-8").read())
    structs, enums = {}, {}
    parse_structs(open(types_path, encoding="utf-8").read(), True, structs, enums)
    functions = []
    for name in order:
        path = os.path.join(src, name)
        parse_structs(open(path, encoding="utf-8").read(), False, structs, enums)
        functions += parse_functions(path, name)
    # Prism's C view holds the standard type's three fields.
    version = open(os.path.join(root, "VERSION"), encoding="utf-8").read().strip()
    model = Model(structs, enums, functions, version)
    for f in functions:
        classify(f, model)
    return model


def signature_words(f):
    ins = ", ".join(f"{i.kind}:{i.type} {i.name}" if i.type else f"{i.kind} {i.name}" for i in f.inputs)
    outs = ", ".join(f"{o.kind}:{o.type} {o.name}" for o in f.outputs)
    return f"{f.name}({ins}) -> [{f.errors}] {outs}"
