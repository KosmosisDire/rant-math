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
    if f.ret != "void" or f.c not in HAND:
        auto(f, m)
    if f.ret == "RantMathResult":
        f.errors = "throw"
    elif f.ret == "int":
        f.errors = "optional" if f.outputs else "bool"


def auto(f, m):
    """Read the C parameters left to right, by their types and the tags beside them."""
    ps = f.params
    i = 0
    by_c = {}       # each C parameter's input or output, for the tags that name one
    rooms = {}      # a count parameter that a room output names, to the input it becomes
    pending = []    # lists whose count parameter is still to come
    last_list = None
    while i < len(ps):
        p = ps[i]
        name = RENAMES.get(p.name, p.name)
        nxt = ps[i + 1] if i + 1 < len(ps) else None
        counted = nxt is not None and nxt.type == "uint32_t" and not nxt.pointer
        if p.dim:                                                    # T out[N]
            out = Output(p.name, "fixed", p.type, p.dim)
            f.outputs.append(out)
            f.args.append(Arg("out", out))
        elif p.type == "RantMatrixView" and not p.pointer:
            shape = tag(p, "out")
            if shape is None:
                inp = by_c[p.name] = Input(name, "matrix")
                f.inputs.append(inp)
                f.args.append(Arg("matrix", inp))
            else:                                                    # (out rows cols)
                rows, cols = shape.split()
                out = Output(p.name, "matrix", rows=rows, cols=cols)
                f.outputs.append(out)
                f.args.append(Arg("matrix_out", out))
        elif p.type in LIST_VIEWS and not p.pointer:                 # a view of a list
            elem, word = LIST_VIEWS[p.type]
            inp = by_c[p.name] = Input(word, "list", elem)
            f.inputs.append(inp)
            f.args.append(Arg("view", inp, p.type))
            last_list = inp
        elif p.type == "RantMeshView" and not p.pointer:            # a binding's Mesh, its corners read in C
            inp = by_c[p.name] = Input("mesh", "mesh", "RantMesh")
            f.inputs.append(inp)
            f.args.append(Arg("mesh", inp))
            last_list = inp
        elif p.type == "RantPrismView" and not p.pointer:
            inp = by_c[p.name] = Input("prism", "prism", "RantPrism")
            f.inputs.append(inp)
            f.args.append(Arg("prism", inp))
        elif p.pointer and p.const and p.type in OBJECT_KINDS:
            inp = by_c[p.name] = Input(name, OBJECT_KINDS[p.type], p.type)
            f.inputs.append(inp)
            f.args.append(Arg(OBJECT_KINDS[p.type], inp))
        elif p.pointer and p.const and p.type in LIST_KINDS and counted:
            inp = by_c[p.name] = Input(name, LIST_KINDS[p.type], p.type)
            f.inputs.append(inp)
            f.args += [Arg("det_ptr", inp), Arg("det_count", inp)]
            last_list = inp
            i += 1
        elif p.pointer and p.const and tag(p, "same") is not None:  # a list as long as another input
            inp = by_c[p.name] = Input(name, "list", p.type)
            f.inputs.append(inp)
            f.args.append(Arg("list_ptr", inp))
        elif p.pointer and p.const and counted and tag(nxt, "degree") is not None:
            inp = by_c[p.name] = Input(name, "list", p.type)        # const T *a, uint32_t degree
            f.inputs.append(inp)
            f.args += [Arg("list_ptr", inp), Arg("degree", inp)]
            last_list = inp
            i += 1
        elif p.pointer and p.const and counted and "count" in nxt.name:
            inp = by_c[p.name] = Input(name, "list", p.type)        # const T *a, uint32_t count
            f.inputs.append(inp)
            f.args.append(Arg("list_ptr", inp))
            f.args.append(Arg("list_count", inp))
            last_list = inp
            i += 1
        elif p.pointer and p.const and p.type == "char":
            inp = by_c[p.name] = Input(name, "string", "char")
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
            kind = "ref" if tag(p, "optional") is None else "opt_ref"  # const T *one, NULL too with (optional)
            inp = by_c[p.name] = Input(name, kind, p.type)
            f.inputs.append(inp)
            f.args.append(Arg(kind, inp))
        elif p.pointer == 2:                                         # T **out: a handle the binding frees
            out = Output(p.name, "handle", p.type)
            f.outputs.append(out)
            f.args.append(Arg("out", out))
            f.vector = False
        elif p.pointer and p.type in WRITTEN_KINDS:
            if tag(p, "image") is not None:                          # (image format= width= height=)
                size = dict(w.split("=") for w in tag(p, "image").split())
                out = Output(p.name, "image", p.type, rows=size.get("height", "src.height"),
                             cols=size.get("width", "src.width"), size=size.get("format", "src.format"))
                f.outputs.append(out)
                f.args.append(Arg("image_out", out))
                for words in tags(p, "takes"):                       # what C reads from the new view
                    t, n = words.split()
                    f.inputs.append(Input(n, "value", t))
            elif tag(p, "copy") is not None:
                out = Output(p.name, "image_copy", p.type)
                f.outputs.append(out)
                f.args.append(Arg("out", out))
            elif tag(p, "mesh") is not None:                         # its corners, their count, triangles, their count
                out = Output("mesh", "mesh_new", "RantMesh")
                f.outputs.append(out)
                f.args.append(Arg("mesh_out", out))
                i += 4
                continue
            else:
                kind = WRITTEN_KINDS[p.type] + "_mut"
                inp = by_c[p.name] = Input(name, kind, p.type)
                f.inputs.append(inp)
                f.outputs.append(Output(p.name, "mutated", p.type, inp))
                f.args.append(Arg(kind, inp))
        elif p.pointer:
            if tag(p, "select") is not None:                         # the points of a cloud a mask keeps
                out = Output(p.name, "cloud_select", "RantPointCloudView", by_c[tag(p, "select")])
                f.outputs.append(out)
                f.args.append(Arg("out", out))
                pending.append(out)
            elif tag(p, "room") is not None:                         # as many as the count asks for
                count = rooms.setdefault(tag(p, "room"), Input(tag(p, "room"), "value", "uint32_t"))
                out = by_c[p.name] = Output(p.name, "room", p.type, count)
                f.outputs.append(out)
                f.args.append(Arg("out", out))
                pending.append(out)
            elif tag(p, "query") is not None:                        # as many as the call finds
                shares = tag(p, "query")
                out = by_c[p.name] = Output(p.name, "query", p.type, by_c[shares] if shares else None)
                f.outputs.append(out)
                f.args.append(Arg("out", out))
                pending.append(out)
            elif p.type == "uint32_t" and pending:                   # the count of the lists before it
                if p.name in rooms:
                    f.inputs.append(rooms[p.name])
                f.args.append(Arg("count_ptr", pending[0]))
                pending = []
            elif tag(p, "size") is not None:                         # (size words a backend reads)
                out = Output(p.name, "expr", p.type, tag(p, "size"))
                f.outputs.append(out)
                f.args.append(Arg("out", out))
            elif tag(p, "same") is not None:                         # one for each item of an input
                out = Output(p.name, "same", p.type, by_c[tag(p, "same")])
                f.outputs.append(out)
                f.args.append(Arg("out", out))
            elif nxt is not None and nxt.type == "uint32_t" and nxt.pointer:
                out = Output(p.name, "query", p.type, per=int(tag(p, "per") or 1))
                f.outputs.append(out)                                # T *out, uint32_t *count
                f.args += [Arg("out", out), Arg("count_ptr", out)]
                i += 2
                continue
            elif counted:
                count = Input(RENAMES.get(nxt.name, nxt.name), "value", "uint32_t")
                out = Output(p.name, "sized", p.type, count)         # T *out, uint32_t count
                f.inputs.append(count)
                f.outputs.append(out)
                f.args += [Arg("out", out), Arg("value", count)]
                i += 2
                continue
            elif p.type == "uint8_t" or p.name in ("normals", "labels"):
                out = Output(p.name, "same", p.type, last_list)      # one for each of the points before
                f.outputs.append(out)
                f.args.append(Arg("out", out))
            else:
                out = Output(p.name, "out", p.type)                  # T *out
                f.outputs.append(out)
                f.args.append(Arg("out", out))
        else:
            inp = by_c[p.name] = Input(name, "value", p.type)
            f.inputs.append(inp)
            f.args.append(Arg("value", inp))
        i += 1
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


def read_model(root, types_path, clang=None):
    src = os.path.join(root, "src")
    order = re.findall(r'#include "(\w+\.h)"', open(os.path.join(src, "rant_math.h"), encoding="utf-8").read())
    structs, enums, functions = read_headers(clang or find_clang(), root, types_path, order)
    version = open(os.path.join(root, "VERSION"), encoding="utf-8").read().strip()
    model = Model(structs, enums, functions, version)
    for f in functions:
        classify(f, model)
    return model


def signature_words(f):
    ins = ", ".join(f"{i.kind}:{i.type} {i.name}" if i.type else f"{i.kind} {i.name}" for i in f.inputs)
    outs = ", ".join(f"{o.kind}:{o.type} {o.name}" for o in f.outputs)
    return f"{f.name}({ins}) -> [{f.errors}] {outs}"
