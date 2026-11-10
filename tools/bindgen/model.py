# What every backend reads: math's C headers parsed into types and functions, and each function
# sorted by the kinds of its parameters into what a binding takes and gives back.
import json
import os
import re
import shutil
import subprocess
import tempfile
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

# Words C may use for a list of points that some language keeps for itself.
RENAMES = {"in": "points", "l": "points"}


def renamed(p):
    return RENAMES.get(p.name, p.name) if p.type in ("RantDouble2", "RantDouble3") or p.type.endswith("View") else p.name

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
    "RantMathArg", "RantMathFunction", "RantMathMethod", "RantMathOperator",
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
    tags: list = field(default_factory=list)  # (name, words) from the comment beside it


@dataclass
class Input:
    name: str
    kind: str   # value ref opt_ref list prism shape shape2d image image_mut cloud cloud_mut grid grid_mut
                # scan det2d det3d matrix fit_options align_options tree string mesh
    type: str = ""


@dataclass
class Output:
    name: str
    kind: str   # return out fixed query sized same expr matrix image cloud field mutated handle mesh_new
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
                # matrix_out image_out cloud_data mesh_out
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
    tags: list = field(default_factory=list)  # (name, words) from a comment between its ) and ;


@dataclass
class Method:
    type: str               # what it hangs on: a standard type's name, or math's own such as Shape, Axes, KdTree
    name: str
    func: Func
    static: bool = False    # made from other values, as Quaternion.from_axis_angle
    field: str = ""         # the field the first input reads, points for a Polygon whose C call takes the list
    self_index: int = 0     # the input that is the value itself, 1 for rant_twist_transform(pose, twist)


@dataclass
class Operator:
    op: str                 # + - * / or neg
    left: str
    right: str              # "" for neg, double for a scale
    func: Func
    how: str                # call, scale (a k) or scale_left (k a)


@dataclass
class Model:
    structs: dict
    enums: dict
    functions: list
    version: str
    methods: list = field(default_factory=list)
    operators: list = field(default_factory=list)
    cpp_types: dict = field(default_factory=dict)   # Rant's C++ type names to struct or their enum declaration
    cpp_fields: dict = field(default_factory=dict)  # each of Rant's C++ structs to its field names
    rant_stub: str = None                           # Rant's stub of its Python types, when its checkout has one

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


def clang_ast(clang, root, types_path):
    """The headers as clang's JSON tree, with the file and line clang leaves out when unchanged filled in."""
    with tempfile.TemporaryDirectory() as shim:
        # base.h includes rant_types.h, which is Rant's stdtypes.h under its released name.
        with open(os.path.join(shim, "rant_types.h"), "w", encoding="utf-8") as f:
            f.write('#include "%s"\n' % os.path.abspath(types_path).replace("\\", "/"))
        run = subprocess.run([clang, "-x", "c", "-std=c99", "-fsyntax-only", "-Xclang", "-ast-dump=json",
                              "-I", shim, os.path.join(root, "src", "rant_math.h")],
                             capture_output=True, text=True, encoding="utf-8")
    if run.returncode != 0:
        raise RuntimeError("clang could not read the headers:\n" + run.stderr)
    tree = json.loads(run.stdout)
    last = {"file": "", "line": 0}

    def fill(node):
        if isinstance(node, list):
            for n in node:
                fill(n)
        elif isinstance(node, dict):
            if "offset" in node:
                for k in ("file", "line"):
                    if k in node:
                        last[k] = node[k]
                    else:
                        node[k] = last[k]
            for k, v in node.items():
                if k != "includedFrom":
                    fill(v)
    fill(tree)
    return tree


def where(loc):
    # A location inside a macro holds the place it was written out as its expansion.
    return loc.get("expansionLoc", loc)


def read_type(qual):
    """A clang type spelling as (name, const, pointers, dims)."""
    dims = tuple(int(d) for d in re.findall(r"\[(\d+)\]", qual))
    t = re.sub(r"\[\d*\]", "", qual).strip()
    m = re.match(r"(const\s+)?(?:struct\s+|enum\s+)?([A-Za-z_]\w*)\s*(\*+)?\s*(?:const)?$", t)
    if not m:
        raise ValueError("cannot read type: " + qual)
    return m.group(2), bool(m.group(1)), len(m.group(3) or ""), dims


def read_record(node):
    # A struct holding a union is a tagged view each binding writes by hand, so it lists no fields.
    if any(n.get("kind") == "RecordDecl" and n.get("tagUsed") == "union" for n in node.get("inner", [])):
        return []
    fields = []
    for n in node.get("inner", []):
        if n.get("kind") == "FieldDecl":
            t, _, ptrs, dims = read_type(n["type"]["qualType"])
            fields.append(CField(n["name"], t, bool(ptrs), dims))
    return fields


def read_enum(node):
    options, value = [], 0
    for n in node.get("inner", []):
        if n.get("kind") != "EnumConstantDecl":
            continue
        for e in n.get("inner", []):
            if e.get("kind") == "ConstantExpr":
                value = int(e["value"])
        options.append((n["name"], value))
        value += 1
    return options


def doc_of(text, begin, end):
    """The comment on a declaration: after its ; on the same line, else just before RANT_MATH_API."""
    m = re.match(r"\s*;[ \t]*(/\*.*?\*/)", text[end:], re.S)
    if m:
        return m.group(1)
    head = text[:begin].rstrip()
    if head.endswith("RANT_MATH_API"):
        head = head[: -len("RANT_MATH_API")].rstrip()
    if head.endswith("*/"):
        start = head.rfind("/*")
        # One that ends the line of the declaration before belongs to that one.
        if not head[head.rfind("\n", 0, start) + 1: start].strip():
            return head[start:]
    return ""


def read_function(node, text, header):
    begin = where(node["range"]["begin"])["offset"]
    close = where(node["range"]["end"])
    ret = node["type"]["qualType"].split("(", 1)[0].strip()
    params = []
    nodes = [p for p in node.get("inner", []) if p.get("kind") == "ParmVarDecl"]
    for k, p in enumerate(nodes):
        t, const, ptrs, _ = read_type(p["type"]["qualType"])
        # An array parameter's type is already a pointer, so its size comes from the text it spans.
        name_end = where(p["loc"])["offset"] + where(p["loc"])["tokLen"]
        end = where(p["range"]["end"])["offset"] + where(p["range"]["end"])["tokLen"]
        dim = re.search(r"\[(\d+)\]", text[name_end:end])
        upto = where(nodes[k + 1]["range"]["begin"])["offset"] if k + 1 < len(nodes) else close["offset"]
        params.append(CParam(p["name"], t, const, 0 if dim else ptrs, int(dim.group(1)) if dim else 0,
                             read_tags(text[end:upto])))
    after = close["offset"] + close["tokLen"]
    comment = re.sub(r"/\*|\*/", " ", doc_of(text, begin, after))
    f = Func(node["name"], node["name"][5:], header, ret, params, clean_doc(comment))
    f.tags = read_tags(text[after: text.index(";", after)])
    return f


def read_tags(text):
    """The tags of a comment beside a declaration, (name words) each, for what its C type cannot say."""
    tags = []
    for comment in re.findall(r"/\*(.*?)\*/", text, re.S):
        tags += [(n, w.strip()) for n, w in re.findall(r"\((\w+)([^()]*)\)", comment)]
    return tags


def read_headers(clang, root, types_path, order):
    """Each header's structs, enums and functions, the types file first and then math's in build order."""
    tree = clang_ast(clang, root, types_path)
    key = lambda p: os.path.normcase(os.path.abspath(p))
    files = {key(types_path): ""}
    files.update({key(os.path.join(root, "src", n)): n for n in order})
    found = {name: ([], [], []) for name in files.values()}
    texts = {}
    records = {}
    for node in tree["inner"]:
        loc = where(node.get("loc", {}))
        if "file" not in loc or key(loc["file"]) not in files:
            continue
        header = files[key(loc["file"])]
        if node["kind"] in ("RecordDecl", "EnumDecl"):
            records[node["id"]] = node
        elif node["kind"] == "TypedefDecl":
            owned = next((n for n in node.get("inner", []) if n.get("isTagOwned")), None)
            tag = records.get(owned["decl"]["id"]) if owned else None
            if tag is None or not tag.get("completeDefinition", tag["kind"] == "EnumDecl"):
                continue
            if tag["kind"] == "RecordDecl":
                found[header][0].append(Struct(node["name"], read_record(tag), header == ""))
            else:
                found[header][1].append(Enum(node["name"], read_enum(tag), header == ""))
        elif node["kind"] == "FunctionDecl" and header:
            if header not in texts:
                texts[header] = open(loc["file"], encoding="utf-8", newline="").read()
            found[header][2].append(read_function(node, texts[header], header))
    structs, enums, functions = {}, {}, []
    for name in [""] + list(order):
        s, e, f = found[name]
        structs.update((x.name, x) for x in s)
        enums.update((x.name, x) for x in e)
        functions += f
    return structs, enums, functions


def read_cpp_types(clang, path):
    """Rant's C++ types: each struct as "struct", each enum as its declaration, and each struct's fields."""
    run = subprocess.run([clang, "-x", "c++", "-std=c++17", "-fsyntax-only", "-Xclang", "-ast-dump=json", path],
                         capture_output=True, text=True, encoding="utf-8")
    if run.returncode != 0:
        raise RuntimeError("clang could not read " + path + ":\n" + run.stderr)
    found, fields = {}, {}

    def walk(node, where):
        for n in node.get("inner", []):
            if n.get("kind") == "NamespaceDecl":
                walk(n, where + [n.get("name")])
            elif where == ["rant", "types"] and n.get("kind") == "CXXRecordDecl" and n.get("completeDefinition"):
                found[n["name"]] = n["tagUsed"]
                fields[n["name"]] = [c["name"] for c in n.get("inner", []) if c.get("kind") == "FieldDecl"]
            elif where == ["rant", "types"] and n.get("kind") == "EnumDecl":
                found[n["name"]] = "enum class " + n["name"] + " : " + n["fixedUnderlyingType"]["qualType"]
    walk(json.loads(run.stdout), [])
    return found, fields


def find_clang():
    for c in (os.environ.get("RANT_MATH_CLANG"), shutil.which("clang"),
              os.path.join(os.environ.get("ProgramFiles", ""), "LLVM", "bin", "clang.exe")):
        if c and os.path.isfile(c):
            return c
    raise RuntimeError("bindgen reads the headers through clang: put it on PATH or name it in RANT_MATH_CLANG")


def clean_doc(comment):
    # Only what also holds in a binding: the sentences that speak of C's pointers and counts go.
    words = " ".join(comment.replace("*", " ").split())
    sentences = re.split(r"(?<=[.:])\s+", words)
    keep = [s for s in sentences if not re.search(
        r"\bNULL\b|\bcount\b|\bcounts\b|rant_|RANT_|ERR_|\bOK\b|\bout\b|out_count|\bpointer\b|holds the room|\bdst\b|\bview\b", s)]
    return " ".join(keep)


# Each backend writes these itself: the text its errors carry and the free of its own tree.
HAND = {"rant_math_result_text", "rant_kdtree_destroy"}
# The library's table of itself, which a binding reads by hand where it needs it.
HAND |= {f"rant_math_{w}_{k}" for w in ("function", "method", "operator") for k in ("count", "at")}

# Lists of a type a binding holds as its own kind.
LIST_KINDS = {"RantDetection2DView": "det2d", "RantDetection3DView": "det3d"}

# Views a call writes through: changed in place, unless a tag makes the view a new value.
WRITTEN_KINDS = {"RantImageView": "image", "RantPointCloudView": "cloud", "RantGridView": "grid"}


def tag(p, name):
    """The words of tag name on a parameter or function, None when it has none."""
    for n, words in p.tags:
        if n == name:
            return words
    return None


def tags(p, name):
    return [words for n, words in p.tags if n == name]


def classify(f, m):
    if f.c in HAND:
        f.hand = True
        f.vector = False
    if f.c not in HAND:
        auto(f, m)
    if f.ret == "RantMathResult":
        f.errors = "throw"
    elif f.ret == "int":
        f.errors = "optional" if f.outputs else "bool"


@dataclass
class Walk:
    """Where the read of one function's parameters stands, and what its rules share."""
    f: Func
    i: int = 0
    by_c: dict = field(default_factory=dict)      # each C parameter's input or output, for the tags that name one
    rooms: dict = field(default_factory=dict)     # a count parameter that a room output names, to the input it becomes
    pending: list = field(default_factory=list)   # lists whose count parameter is still to come
    last_list: object = None

    def param(self, k=0):
        j = self.i + k
        return self.f.params[j] if j < len(self.f.params) else None

    @property
    def p(self):
        return self.param()

    @property
    def name(self):
        return renamed(self.p)

    @property
    def nxt(self):
        return self.param(1)

    @property
    def counted(self):
        return is_count(self.nxt)


def is_count(p):
    return p is not None and p.type == "uint32_t" and not p.pointer


def fixed_out(w):
    p, f = w.p, w.f
    if not p.dim:
        return None
    out = Output(p.name, "fixed", p.type, p.dim)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    return 1


def matrix_out(w):
    p, f = w.p, w.f
    if p.type != "RantMatrixView" or p.pointer or tag(p, "out") is None:
        return None
    rows, cols = tag(p, "out").split()
    out = Output(p.name, "matrix", rows=rows, cols=cols)
    f.outputs.append(out)
    f.args.append(Arg("matrix_out", out))
    return 1


def matrix_in(w):
    p, f = w.p, w.f
    if p.type != "RantMatrixView" or p.pointer:
        return None
    inp = w.by_c[p.name] = Input(w.name, "matrix")
    f.inputs.append(inp)
    f.args.append(Arg("matrix", inp))
    return 1


def list_view(w):
    p, f = w.p, w.f
    if p.type not in LIST_VIEWS or p.pointer:
        return None
    elem, word = LIST_VIEWS[p.type]
    inp = w.by_c[p.name] = Input(word, "list", elem)
    f.inputs.append(inp)
    f.args.append(Arg("view", inp, p.type))
    w.last_list = inp
    return 1


def mesh_view(w):
    p, f = w.p, w.f
    if p.type != "RantMeshView" or p.pointer:
        return None
    inp = w.by_c[p.name] = Input("mesh", "mesh", "RantMesh")
    f.inputs.append(inp)
    f.args.append(Arg("mesh", inp))
    w.last_list = inp
    return 1


def prism_view(w):
    p, f = w.p, w.f
    if p.type != "RantPrismView" or p.pointer:
        return None
    inp = w.by_c[p.name] = Input("prism", "prism", "RantPrism")
    f.inputs.append(inp)
    f.args.append(Arg("prism", inp))
    return 1


def object_ref(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and p.type in OBJECT_KINDS):
        return None
    inp = w.by_c[p.name] = Input(w.name, OBJECT_KINDS[p.type], p.type)
    f.inputs.append(inp)
    f.args.append(Arg(OBJECT_KINDS[p.type], inp))
    return 1


def detection_list(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and p.type in LIST_KINDS and w.counted):
        return None
    inp = w.by_c[p.name] = Input(w.name, LIST_KINDS[p.type], p.type)
    f.inputs.append(inp)
    f.args += [Arg("det_ptr", inp), Arg("det_count", inp)]
    w.last_list = inp
    return 2


def list_same(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and tag(p, "same") is not None):
        return None
    inp = w.by_c[p.name] = Input(w.name, "list", p.type)
    f.inputs.append(inp)
    f.args.append(Arg("list_ptr", inp))
    return 1


def list_degree(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and w.counted and tag(w.nxt, "degree") is not None):
        return None
    inp = w.by_c[p.name] = Input(w.name, "list", p.type)
    f.inputs.append(inp)
    f.args += [Arg("list_ptr", inp), Arg("degree", inp)]
    w.last_list = inp
    return 2


def list_counted(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and w.counted and tag(p, "one") is None):
        return None
    inp = w.by_c[p.name] = Input(w.name, "list", p.type)
    f.inputs.append(inp)
    f.args.append(Arg("list_ptr", inp))
    f.args.append(Arg("list_count", inp))
    w.last_list = inp
    return 2


def string_in(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const and p.type == "char"):
        return None
    inp = w.by_c[p.name] = Input(w.name, "string", "char")
    f.inputs.append(inp)
    f.args.append(Arg("string", inp))
    return 1


def list_with_same_out(w):
    p, nxt, f = w.p, w.nxt, w.f
    if not (p.pointer and p.const and nxt is not None and nxt.pointer and not nxt.const and is_count(w.param(2))):
        return None
    inp = Input(w.name, "list", p.type)
    out = Output(nxt.name, "same", nxt.type, inp)
    f.inputs.append(inp)
    f.outputs.append(out)
    f.args += [Arg("list_ptr", inp), Arg("out", out), Arg("list_count", inp)]
    w.last_list = inp
    return 3


def list_pair(w):
    p, nxt, f = w.p, w.nxt, w.f
    if not (p.pointer and p.const and nxt is not None and nxt.pointer and nxt.const and is_count(w.param(2))):
        return None
    a = Input(w.name, "list", p.type)
    b = Input(renamed(nxt), "list", nxt.type)
    f.inputs += [a, b]
    f.args += [Arg("list_ptr", a), Arg("list_ptr", b), Arg("list_count", a, b)]
    w.last_list = a
    return 3


def ref_in(w):
    p, f = w.p, w.f
    if not (p.pointer and p.const):
        return None
    kind = "ref" if tag(p, "optional") is None else "opt_ref"
    inp = w.by_c[p.name] = Input(w.name, kind, p.type)
    f.inputs.append(inp)
    f.args.append(Arg(kind, inp))
    return 1


def handle_out(w):
    p, f = w.p, w.f
    if p.pointer != 2:
        return None
    out = Output(p.name, "handle", p.type)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    f.vector = False
    return 1


def image_new(w):
    p, f = w.p, w.f
    if not (p.pointer and p.type in WRITTEN_KINDS and tag(p, "image") is not None):
        return None
    size = dict(word.split("=") for word in tag(p, "image").split())
    out = Output(p.name, "image", p.type, rows=size.get("height", "src.height"),
                 cols=size.get("width", "src.width"), size=size.get("format", "src.format"))
    f.outputs.append(out)
    f.args.append(Arg("image_out", out))
    for words in tags(p, "takes"):                       # what C reads from the new view
        t, n = words.split()
        f.inputs.append(Input(n, "value", t))
    return 1


def image_copy(w):
    p, f = w.p, w.f
    if not (p.pointer and p.type in WRITTEN_KINDS and tag(p, "copy") is not None):
        return None
    out = Output(p.name, "image_copy", p.type)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    return 1


def mesh_new(w):
    p, f = w.p, w.f
    if not (p.pointer and p.type in WRITTEN_KINDS and tag(p, "mesh") is not None):
        return None
    out = Output("mesh", "mesh_new", "RantMesh")
    f.outputs.append(out)
    f.args.append(Arg("mesh_out", out))
    return 4                                             # its corners, their count, triangles, their count


def view_mutated(w):
    p, f = w.p, w.f
    if not (p.pointer and p.type in WRITTEN_KINDS):
        return None
    kind = WRITTEN_KINDS[p.type] + "_mut"
    inp = w.by_c[p.name] = Input(w.name, kind, p.type)
    f.inputs.append(inp)
    f.outputs.append(Output(p.name, "mutated", p.type, inp))
    f.args.append(Arg(kind, inp))
    return 1


def cloud_select(w):
    p, f = w.p, w.f
    if not (p.pointer and tag(p, "select") is not None):
        return None
    out = Output(p.name, "cloud_select", "RantPointCloudView", w.by_c[tag(p, "select")])
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    w.pending.append(out)
    return 1


def room_out(w):
    p, f = w.p, w.f
    if not (p.pointer and tag(p, "room") is not None):
        return None
    count = w.rooms.setdefault(tag(p, "room"), Input(tag(p, "room"), "value", "uint32_t"))
    out = w.by_c[p.name] = Output(p.name, "room", p.type, count)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    w.pending.append(out)
    return 1


def query_out(w):
    p, f = w.p, w.f
    if not (p.pointer and tag(p, "query") is not None):
        return None
    shares = tag(p, "query")
    out = w.by_c[p.name] = Output(p.name, "query", p.type, w.by_c[shares] if shares else None)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    w.pending.append(out)
    return 1


def count_out(w):
    p, f = w.p, w.f
    if not (p.pointer and p.type == "uint32_t" and w.pending):
        return None
    if p.name in w.rooms:
        f.inputs.append(w.rooms[p.name])
    f.args.append(Arg("count_ptr", w.pending[0]))
    w.pending = []
    return 1


def size_out(w):
    p, f = w.p, w.f
    if not (p.pointer and tag(p, "size") is not None):
        return None
    out = Output(p.name, "expr", p.type, tag(p, "size"))
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    return 1


def same_out(w):
    p, f = w.p, w.f
    if not (p.pointer and tag(p, "same") is not None):
        return None
    out = Output(p.name, "same", p.type, w.by_c[tag(p, "same")])
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    return 1


def query_with_count(w):
    p, nxt, f = w.p, w.nxt, w.f
    if not (p.pointer and nxt is not None and nxt.type == "uint32_t" and nxt.pointer):
        return None
    out = Output(p.name, "query", p.type, per=int(tag(p, "per") or 1))
    f.outputs.append(out)
    f.args += [Arg("out", out), Arg("count_ptr", out)]
    return 2


def sized_out(w):
    p, nxt, f = w.p, w.nxt, w.f
    if not (p.pointer and w.counted):
        return None
    count = Input(renamed(nxt), "value", "uint32_t")
    out = Output(p.name, "sized", p.type, count)
    f.inputs.append(count)
    f.outputs.append(out)
    f.args += [Arg("out", out), Arg("value", count)]
    return 2


def plain_out(w):
    p, f = w.p, w.f
    if not p.pointer:
        return None
    out = Output(p.name, "out", p.type)
    f.outputs.append(out)
    f.args.append(Arg("out", out))
    return 1


def value_in(w):
    p, f = w.p, w.f
    inp = w.by_c[p.name] = Input(w.name, "value", p.type)
    f.inputs.append(inp)
    f.args.append(Arg("value", inp))
    return 1


# A rule returns None when its C shape is not at w.i, else applies itself and returns how many parameters
# it took. The order is the grammar: the first rule that matches wins.
RULES = [
    fixed_out,      # T out[N]
    matrix_out,     # RantMatrixView with (out rows cols)
    matrix_in,      # RantMatrixView by value
    list_view,      # a view of a list by value
    mesh_view,      # a binding's Mesh, its corners read in C
    prism_view,     # RantPrismView by value
    object_ref,     # const pointer to a type a binding holds as its own kind
    detection_list,  # const detections pointer, uint32_t count
    list_same,      # const T *a with (same p): a list as long as another input
    list_degree,    # const T *a, uint32_t degree
    list_counted,   # const T *a, uint32_t count
    string_in,      # const char *s
    list_with_same_out,  # const T *in, U *out, uint32_t count
    list_pair,      # const T *a, const T *b, uint32_t count
    ref_in,         # const T *one, NULL too with (optional)
    handle_out,     # T **out: a handle the binding frees
    image_new,      # a written view with (image format= width= height=)
    image_copy,     # a written view with (copy)
    mesh_new,       # a written view with (mesh), four parameters
    view_mutated,   # a written view the call changes in place
    cloud_select,   # T *out with (select cloud): the points of a cloud a mask keeps
    room_out,       # T *out with (room count): as many as the count asks for
    query_out,      # T *out with (query): as many as the call finds
    count_out,      # uint32_t *count after lists of room or query outs
    size_out,       # T *out with (size words)
    same_out,       # T *out with (same p): one for each item of an input
    query_with_count,  # T *out, uint32_t *count
    sized_out,      # T *out, uint32_t count
    plain_out,      # T *out
    value_in,       # anything left, by value
]


def auto(f, m):
    """Read the C parameters left to right, by their types and the tags beside them."""
    w = Walk(f)
    while w.i < len(f.params):
        for rule in RULES:
            used = rule(w)
            if used is not None:
                break
        w.i += used
    ret = normal_type(f.ret)
    if ret in ("RantShape", "RantShape2D"):
        f.outputs.insert(0, Output("return", "shape_new", ret))     # a binding's own Shape holds the data
        f.vector = False
    elif ret == "RantPrismView":
        f.outputs.insert(0, Output("return", "prism", "RantPrism"))
    elif ret == "RantPointCloudView":
        f.outputs.insert(0, Output("return", "cloud_layout", ret))
    elif ret == "RantPointFieldView":
        f.outputs.insert(0, Output("return", "field", ret))
    elif f.ret == "const char *":
        f.outputs.insert(0, Output("return", "return", "string"))
    elif ret not in ("void", "int", "RantMathResult"):
        last = tag(f, "last")                                        # (last name): the return comes after the outs
        if last is None:
            f.outputs.insert(0, Output("return", "return", ret))
        else:
            f.outputs.append(Output(last, "return", ret))
    if not f.outputs and f.ret == "void":
        raise ValueError("no answer from " + f.c)


def normal_type(t):
    return t.replace("const ", "").replace("*", "").strip()


# What a first input of these kinds is in every binding.
RECEIVER_KINDS = {
    "shape": "Shape", "shape2d": "Shape2D", "image": "Image", "image_mut": "Image", "cloud": "PointCloud",
    "cloud_mut": "PointCloud", "grid": "Grid", "grid_mut": "Grid", "scan": "LaserScan", "tree": "KdTree",
    "mesh": "Mesh", "prism": "Prism",
}

# Types whose functions start with another word than their name.
PREFIXES = {"CameraIntrinsics": "camera", "KdTree": "kdtree"}

# A method spelled as an operator by its name, with a value of a struct on the right.
OPERATOR_VERBS = {"add": "+", "subtract": "-", "compose": "*", "rotate": "*", "apply": "*"}


def short(t):
    return t[4:] if t.startswith("Rant") else t


def prefix(t):
    return PREFIXES.get(t) or re.sub(r"(?<=[a-z])(?=[A-Z])", "_", t).lower()


def is_vector(t):
    # Float, Double and Int vectors move between each other by casts in Rant's types, not by methods.
    return re.fullmatch(r"(Float|Double|Int)\d", t) is not None


def receiver(f, m):
    """The type a function's first input is in the bindings, and the field of it the call reads."""
    if not f.inputs:
        return None, ""
    first = f.inputs[0]
    if first.kind in ("value", "ref") and first.type in m.structs and first.type not in HIDDEN_STRUCTS:
        return first.type[4:], ""
    if first.kind in RECEIVER_KINDS:
        return RECEIVER_KINDS[first.kind], ""
    view = next((a.b for a in f.args if a.kind == "view" and a.a is first), None)
    if view:                                                        # RantPolygonView is a Polygon's points
        return view[4:-4], LIST_VIEWS[view][1]
    return None, ""


def result_type(f):
    if len(f.outputs) == 1 and f.outputs[0].kind in ("return", "out", "shape_new") and f.outputs[0].type.startswith("Rant"):
        return f.outputs[0].type[4:]
    return None


def find_methods(m):
    """rant_<type>_<verb> on a first input of that type is <type>.<verb>, and <other>_from_<type> is
    <type>.to_<other>. A call that makes a type from other values is a static of it."""
    methods = []
    for f in m.functions:
        if f.hand:
            continue
        t, field_name = receiver(f, m)
        made = result_type(f)
        if t is not None and f.name.startswith(prefix(t) + "_"):
            verb = f.name[len(prefix(t)) + 1:]
            # A from_ call makes the type, so it reads as GeoPoint.from_enu(origin, enu) even with one first.
            methods.append(Method(t, verb, f, static=verb.startswith("from_"), field=field_name))
        elif t is not None and made and f.name == prefix(made) + "_from_" + prefix(t):
            if not (is_vector(t) and is_vector(made)):
                methods.append(Method(t, "to_" + prefix(made), f, field=field_name))
        elif made and made != t and f.name.startswith(prefix(made) + "_"):
            later = [k for k, i in enumerate(f.inputs) if k and i.kind in ("value", "ref") and i.type == "Rant" + made]
            if later:                                                # rant_twist_transform(pose, twist): twist.transform(pose)
                methods.append(Method(made, f.name[len(prefix(made)) + 1:], f, self_index=later[0]))
            else:
                methods.append(Method(made, f.name[len(prefix(made)) + 1:], f, static=True))
    seen = set()
    for me in methods:
        if (me.type, me.name) in seen:
            raise ValueError(f"two methods {me.type}.{me.name}")
        seen.add((me.type, me.name))
    return methods


def find_operators(m):
    ops = []
    for me in m.methods:
        f = me.func
        if me.static or f.errors != "none" or len(f.outputs) != 1:
            continue
        if me.name == "negate" and len(f.inputs) == 1 and f.outputs[0].type == f.inputs[0].type:
            ops.append(Operator("neg", me.type, "", f, "call"))
            continue
        if len(f.inputs) != 2 or f.inputs[1].kind != "value":
            continue
        right = f.inputs[1].type
        if me.name == "scale" and right == "double" and f.outputs[0].type == f.inputs[0].type:
            ops += [Operator("*", me.type, "double", f, "scale"), Operator("*", "double", me.type, f, "scale_left")]
            continue
        if me.name == "divide" and right == "double" and f.outputs[0].type == f.inputs[0].type:
            ops.append(Operator("/", me.type, "double", f, "call"))
            continue
        op = OPERATOR_VERBS.get(me.name) or ("*" if me.name == "multiply" and me.type.startswith("Matrix") else None)
        if op and right in m.structs:
            ops.append(Operator(op, me.type, right[4:], f, "call"))
    seen = set()
    for o in ops:
        if (o.op, o.left, o.right) in seen:
            raise ValueError(f"two operators {o.left} {o.op} {o.right}")
        seen.add((o.op, o.left, o.right))
    return ops


def read_model(root, types_path, clang=None, types_hpp=None):
    src = os.path.join(root, "src")
    order = re.findall(r'#include "(\w+\.h)"', open(os.path.join(src, "rant_math.h"), encoding="utf-8").read())
    clang = clang or find_clang()
    structs, enums, functions = read_headers(clang, root, types_path, order)
    # Rant's src/common/stdtypes.h has its C++ types at bindings/cpp/rant_types.hpp.
    types_hpp = types_hpp or os.path.join(os.path.dirname(types_path), "..", "..", "bindings", "cpp", "rant_types.hpp")
    cpp_types, cpp_fields = read_cpp_types(clang, types_hpp)
    stub = os.path.join(os.path.dirname(types_path), "..", "..", "bindings", "python", "rant_types", "__init__.pyi")
    rant_stub = open(stub, encoding="utf-8").read() if os.path.exists(stub) else None
    version = open(os.path.join(root, "VERSION"), encoding="utf-8").read().strip()
    model = Model(structs, enums, functions, version, cpp_types=cpp_types, cpp_fields=cpp_fields, rant_stub=rant_stub)
    for f in functions:
        classify(f, model)
    model.methods = find_methods(model)
    model.operators = find_operators(model)
    return model


def signature_words(f):
    ins = ", ".join(f"{i.kind}:{i.type} {i.name}" if i.type else f"{i.kind} {i.name}" for i in f.inputs)
    outs = ", ".join(f"{o.kind}:{o.type} {o.name}" for o in f.outputs)
    return f"{f.name}({ins}) -> [{f.errors}] {outs}"
