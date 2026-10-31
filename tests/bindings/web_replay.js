// Every case of vectors.json through a JavaScript binding, again with typed arrays where a call takes
// them, then what vectors cannot show. The TypeScript package and the QuickJS glue both answer to it.
let failures = 0;
function check(ok, what) {
    if (!ok) {
        log("FAILED " + what);
        failures++;
    }
}

let doc, types, enums, math, MathError, log;
const ALIAS = new Set(["RantMatrix3x3", "RantMatrix4x4", "RantCovariance3", "RantCovariance6"]);
const IMAGE_FORMATS = { 0: "Mono8", 1: "Mono16", 2: "Rgb8", 3: "Rgba8", 4: "Bgr8", 5: "Yuyv", 6: "Nv12", 7: "Monof32", 16: "Jpeg", 17: "Png" };
const SCALARS = { 1: "I8", 2: "U8", 3: "I16", 4: "U16", 5: "I32", 6: "U32", 7: "F32", 8: "F64", 9: "I64", 10: "U64" };
const MODELS = { 0: "None", 1: "BrownConrady", 2: "Fisheye", 3: "Rational" };

const num = (v) => (v === "nan" ? NaN : v === "inf" ? Infinity : v === "-inf" ? -Infinity : v);
const unhex = (s) => Uint8Array.from(s.match(/../g) ?? [], (h) => parseInt(h, 16));
const hex = (d) => Array.from(d, (b) => b.toString(16).padStart(2, "0")).join("");
const camel = (n) => n.replace(/_(\w)/g, (_, c) => c.toUpperCase());

function make(t, v) {
    if (ALIAS.has(t)) return v.m;
    if (enums.has(t)) return v;
    if (types[t]) {
        const out = {};
        for (const f of types[t]) {
            let x = v[f.name];
            if (t === "RantCameraIntrinsics" && f.name === "model") x = MODELS[x];
            else if (f.dims) x = x.map(num);
            else x = make(f.type, x);
            out[f.name] = x;
        }
        return out;
    }
    return num(v);
}

const list = (t, v) => v.map((x) => make(t, x));
const prism = (v) => ({ pose: make("RantPose", v.pose), base: { points: list("RantDouble2", v.base.points) }, height: num(v.height) });
const image = (v) => ({ width: v.width, height: v.height, stride: v.stride, format: IMAGE_FORMATS[v.format], data: unhex(v.data) });
const cloud = (v) => ({ fields: v.fields.map((f) => ({ ...f, kind: SCALARS[f.kind] })), point_step: v.point_step, data: unhex(v.data) });
const grid = (v) => ({
    origin: { pose: make("RantPose", v.origin) }, resolution: v.resolution, cells_x: v.cells_x, cells_y: v.cells_y, cells_z: v.cells_z,
    fields: v.fields.map((f) => ({ name: f.name, kind: SCALARS[f.kind], data: unhex(f.data) })),
});

const SHAPE = {
    aligned_box: "RantAlignedBox", oriented_box: "RantOrientedBox", sphere: "RantSphere", capsule: "RantCapsule", cylinder: "RantCylinder",
    cone: "RantCone", segment: "RantSegment", aligned_box2d: "RantAlignedBox2D", oriented_box2d: "RantOrientedBox2D", circle: "RantCircle",
    capsule2d: "RantCapsule2D", segment2d: "RantSegment2D",
};

function shape(v, two) {
    const { kind, value } = v;
    const build = math[camel((two ? "shape2d_from_" : "shape_from_") + kind)];
    if (kind === "mesh") return build(list("RantDouble3", value.vertices), value.triangles);
    if (kind === "prism") return build(prism(value));
    if (kind === "polygon" || kind === "polyline") return build(list("RantDouble3", value));
    if (kind === "polygon2d" || kind === "polyline2d") return build(list("RantDouble2", value));
    return build(make(SHAPE[kind], value));
}

const FLAT = { RantDouble2: 2, RantDouble3: 3, RantPose: 7, RantPose2D: 3 };

function flat(v) {
    if (v === null || v === undefined) return [];
    if (typeof v === "string") return [num(v)];
    if (typeof v !== "object") return [v];
    if (Array.isArray(v) || ArrayBuffer.isView(v)) return Array.from(v).flatMap(flat);
    return Object.entries(v).flatMap(([, x]) => flat(x));
}

function input(kind, v, ctx, typed) {
    const colon = kind.indexOf(":");
    const k = colon < 0 ? kind : kind.slice(0, colon);
    const t = colon < 0 ? "" : kind.slice(colon + 1);
    switch (k) {
    case "value": case "ref": return make(t, v);
    case "opt_ref": return v === null ? null : make(t, v);
    case "list":
        if (typed && FLAT[t]) return Float64Array.from(flat(v));
        if (t === "double") return typed ? Float64Array.from(v.map(num)) : v.map(num);
        if (t === "uint32_t" || t === "uint8_t") return typed ? (t === "uint32_t" ? Uint32Array.from(v) : Uint8Array.from(v)) : v;
        return list(t, v);
    case "string": return v;
    case "prism": return prism(v);
    case "shape": return shape(v, false);
    case "shape2d": return shape(v, true);
    case "image": case "image_mut": return image(v);
    case "cloud": case "cloud_mut": return cloud(v);
    case "grid": case "grid_mut": return grid(v);
    case "scan": return { angle_min: v.angle_min, angle_step: v.angle_step, range_min: v.range_min, range_max: v.range_max, echoes: v.echoes, ranges: v.ranges.map(num) };
    case "det2d": return v.map((d) => ({ bounds: make("RantOrientedBox2D", d.bounds), label: d.label, score: d.score }));
    case "det3d": return v.map((d) => ({ bounds: make("RantOrientedBox", d.bounds), label: d.label, score: d.score }));
    case "matrix": return { rows: v.rows, cols: v.cols, data: v.data.map(num) };
    case "fit_options": {
        if (v === null) return null;
        const o = { weights: v.weights, tolerance: v.tolerance, tries: v.tries, seed: v.seed };
        if (v.inliers) o.inliers = new Uint8Array(ctx.points);
        ctx.fit = o;
        return o;
    }
    case "align_options": {
        if (v === null) return null;
        const o = { max_distance: v.max_distance, iterations: v.iterations, normals: v.normals ? list("RantDouble3", v.normals) : null };
        if (v.inliers) o.inliers = new Uint8Array(ctx.points);
        ctx.align = o;
        ctx.wantError = v.error;
        return o;
    }
    case "tree": return math.kdtreeCreate(list("RantDouble3", v.points));
    default: throw new Error("no input " + k);
    }
}

// An answer as vectors.json keeps it.
function answer(v) {
    if (v === undefined || v === null) return null;
    if (typeof v === "number") return Number.isNaN(v) ? "nan" : v === Infinity ? "inf" : v === -Infinity ? "-inf" : v;
    if (typeof v === "boolean" || typeof v === "string") return v;
    if (v instanceof Uint8Array) return hex(v);
    if (ArrayBuffer.isView(v)) return Array.from(v, answer);
    if (Array.isArray(v)) return v.map(answer);
    if ("format" in v && "data" in v) return { width: v.width, height: v.height, stride: v.stride, format: enumNumber(IMAGE_FORMATS, v.format), data: hex(v.data) };
    if ("point_step" in v) return { fields: v.fields.map(answer), point_step: v.point_step, data: hex(v.data) };
    if ("offset" in v && "kind" in v) return { name: v.name, offset: v.offset, kind: enumNumber(SCALARS, v.kind), count: v.count };
    if ("cells_x" in v) {
        return {
            origin: answer(v.origin.pose), resolution: v.resolution, cells_x: v.cells_x, cells_y: v.cells_y, cells_z: v.cells_z,
            fields: v.fields.map((f) => ({ name: f.name, kind: enumNumber(SCALARS, f.kind), data: hex(f.data) })),
        };
    }
    const out = {};
    for (const [k, x] of Object.entries(v)) {
        if (k === "model") out[k] = enumNumber(MODELS, x);
        else out[k] = answer(x);
    }
    return out;
}

function enumNumber(table, name) {
    for (const [k, v] of Object.entries(table)) if (v === name) return Number(k);
    return name;
}

function shaped(entry, v) {
    if (entry.out.length === 1 && ALIAS.has(entry.out[0])) return { m: v };
    if (entry.out.length > 1) return v.map((x, i) => (ALIAS.has(entry.out[i]) ? { m: x } : x));
    return v;
}

function close(want, got, tol) {
    if (typeof want === "number" && typeof got === "number") {
        return want === got || Math.abs(want - got) <= tol * Math.max(1, Math.abs(want), Math.abs(got));
    }
    if (Array.isArray(want) && Array.isArray(got)) return want.length === got.length && want.every((x, i) => close(x, got[i], tol));
    if (want && got && typeof want === "object" && typeof got === "object") {
        const keys = Object.keys(want);
        return keys.length === Object.keys(got).length && keys.every((k) => close(want[k], got[k], tol));
    }
    return want === got;
}

function runCase(entry, c, typed) {
    const ctx = { points: Array.isArray(c.a[0]) ? c.a[0].length : 0 };
    const args = entry.in.map((k, i) => input(k, c.a[i], ctx, typed));
    let value;
    try {
        value = math[camel(entry.f)](...args);
    } catch (e) {
        if (e instanceof MathError) return { e: e.code };
        throw e;
    }
    const mut = entry.in.findIndex((k) => k.endsWith("_mut"));
    if (mut >= 0) value = args[mut];
    if (entry.in.includes("tree")) args[entry.in.indexOf("tree")].free();
    const got = { v: value };
    if (ctx.fit?.inliers) got.inliers = hex(ctx.fit.inliers);
    if (ctx.align) {
        if (ctx.align.inliers) got.inliers = hex(ctx.align.inliers);
        if (ctx.wantError) got.error = answer(ctx.align.error);
    }
    return got;
}

function replay() {
    let done = 0, again = 0;
    for (const entry of doc.functions) {
        for (const c of entry.cases) {
            const { a, ...want } = c;
            const got = runCase(entry, c, false);
            if ("v" in got) got.v = shaped(entry, answer(got.v));
            if (!close(want, got, entry.tol)) check(false, `${entry.f} ${JSON.stringify(a).slice(0, 240)}\n  want ${JSON.stringify(want).slice(0, 400)}\n  got  ${JSON.stringify(got).slice(0, 400)}`);
            done++;
            if (entry.in.some((k) => k.startsWith("list:") && (FLAT[k.slice(5)] || ["double", "uint32_t", "uint8_t"].includes(k.slice(5))))) {
                const typed = runCase(entry, c, true);
                if ("e" in want || "e" in typed) {
                    check(want.e === typed.e, `typed ${entry.f} ${JSON.stringify(a).slice(0, 200)}: want ${JSON.stringify(want).slice(0, 100)} got ${JSON.stringify(typed).slice(0, 100)}`);
                } else {
                    const x = flat(want.v), y = flat(answerFlat(typed.v));
                    const same = x.length === y.length && x.every((p, i) => p === y[i] || (Number.isNaN(p) && Number.isNaN(y[i])) || Math.abs(p - y[i]) <= entry.tol * Math.max(1, Math.abs(p), Math.abs(y[i])));
                    if (!hasText(want.v)) check(same, `typed ${entry.f} ${JSON.stringify(a).slice(0, 200)}\n  want ${String(x).slice(0, 300)}\n  got  ${String(y).slice(0, 300)}`);
                }
                again++;
            }
        }
    }
    log(`replayed ${done} cases of ${doc.functions.length} functions, ${again} again with typed arrays`);
}

const answerFlat = (v) => answer(v);
function hasText(v) {
    if (typeof v === "string") return !["nan", "inf", "-inf"].includes(v);
    if (v && typeof v === "object") return Object.values(v).some(hasText);
    return false;
}

function throws(kind, f, what) {
    try {
        f();
        check(false, what + ": no throw");
    } catch (e) {
        check(e instanceof MathError && e.kind === kind, `${what}: ${e}`);
        check(e.message.length > 0, what + ": no words");
    }
}

const d3 = (x, y, z) => ({ x, y, z });
const IDENTITY = { x: 0, y: 0, z: 0, w: 1 };

function handTests() {
    // results
    const plane = { position: d3(0, 0, 0), normal: d3(0, 0, 1) };
    check(math.planeRaycast(plane, { origin: d3(0, 0, 5), direction: d3(0, 0, 1) }) === null, "a miss is null");
    check(math.planeRaycast(plane, { origin: d3(0, 0, 5), direction: d3(0, 0, -1) }).distance === 5, "a hit");
    check(math.sphereContains({ center: d3(0, 0, 0), radius: 1 }, d3(0, 0, 0)) === true, "a bool");
    const [axis, angle] = math.quaternionToAxisAngle(math.quaternionFromAxisAngle(d3(0, 0, 1), 0.5));
    check(Math.abs(angle - 0.5) < 1e-12 && Math.abs(axis.z - 1) < 1e-12, "a pair");
    throws("Degenerate", () => math.matrix3x3Inverse([[0, 0, 0], [0, 0, 0], [0, 0, 0]]), "a singular matrix");
    check(math.matrix3x3Identity()[1][1] === 1, "a matrix comes back as rows");
    check(math.matrix3x3Identity()[0] instanceof Float32Array, "as Rant's value holds it");

    // anything with the fields counts, as a THREE.Vector3 does
    class Vector3 { constructor(x, y, z) { this.x = x; this.y = y; this.z = z; } length() { return Math.hypot(this.x, this.y, this.z); } }
    check(math.double3Length(new Vector3(3, 4, 0)) === 5, "a class by its fields");
    check(math.polylineLength([new Vector3(0, 0, 0), new Vector3(2, 0, 0)]) === 2, "a list of them");

    // typed arrays in give typed arrays out
    const moved = math.poseApplyPoints({ position: d3(1, 0, 0), orientation: IDENTITY }, Float64Array.of(0, 0, 0, 1, 0, 0));
    check(moved instanceof Float64Array && moved[3] === 2, "typed in, typed out");
    check(Array.isArray(math.poseApplyPoints({ position: d3(1, 0, 0), orientation: IDENTITY }, [d3(0, 0, 0)])), "plain in, plain out");
    throws("Argument", () => math.polylineLength(Float64Array.of(0, 0)), "a typed list that is not in threes");

    // mismatches
    throws("Argument", () => math.pointsFitPose([d3(0, 0, 0)], []), "lists that differ");
    throws("Argument", () => math.polynomialAt([], 1), "no coefficients");
    throws("Argument", () => math.meshArea([d3(0, 0, 0)], [0, 1]), "triangles not in threes");

    // a shape holds its own data
    const pts = [d3(0, 0, 0), d3(2, 0, 0)];
    const line = math.shapeFromPolyline(pts);
    pts[1].x = 99;
    pts.length = 0;
    check(Math.abs(math.shapeDistance(line, d3(1, 1, 0)) - 1) < 1e-12, "a shape keeps its list");
    check(JSON.stringify(math.shapeFromSphere({ center: d3(0, 0, 0), radius: 1 })).includes("sphere"), "a plain shape stays plain");

    // options write back
    const three = [d3(0, 0, 0), d3(1, 0, 0), d3(0, 1, 0)];
    const inliers = new Uint8Array(3);
    const fitted = math.pointsFitPlane(three, { inliers });
    check(inliers.every((x) => x === 1) && Math.abs(Math.abs(fitted.normal.z) - 1) < 1e-12, "inliers come back");
    throws("Argument", () => math.pointsFitPlane(three, { inliers: new Uint8Array(2) }), "inliers of the wrong size");
    const more = [...three, d3(0, 0, 1)];
    const o = {};
    math.pointsAlign(more, more, { position: d3(0, 0, 0), orientation: IDENTITY }, o);
    check(o.error < 1e-9, "the error comes back");

    // the kd tree
    const tree = math.kdtreeCreate(more);
    const [indices, distances] = math.kdtreeNearest(tree, d3(0, 0, 0.1), 2);
    check(indices instanceof Uint32Array && indices[0] === 0 && Math.abs(distances[0] - 0.1) < 1e-12, "nearest");
    check(math.kdtreeWithin(tree, d3(0, 0, 0), 0.5)[0].length === 1, "within");
    tree.free();
    tree.free();
    throws("Argument", () => math.kdtreeNearest(tree, d3(0, 0, 0), 1), "a freed tree");

    // images and clouds, in place and new
    const img = { width: 2, height: 1, stride: 0, format: "Rgb8", data: Uint8Array.of(1, 2, 3, 4, 5, 6) };
    check(math.imageColorAt(img, 1, 0).g === 5, "a pixel");
    check(math.imageConvert(img, "Mono8").data.length === 2, "a new image");
    check(math.imageRotate(img, -1).height === 2, "turned");
    math.imageDraw(img, math.shape2dFromCircle({ center: { x: 0, y: 0 }, radius: 5 }), { r: 7, g: 8, b: 9, a: 255 }, 0);
    check(img.data[0] === 7, "drawn in place");
    throws("Format", () => math.imageCheck({ width: 2, height: 2, stride: 0, format: "Jpeg", data: Uint8Array.of(1) }), "a jpeg");
    const c = math.pointCloudLayout("Xyz");
    c.data = new Uint8Array(24);
    math.pointCloudWritePoints(c, [d3(1, 2, 3), d3(4, 5, 6)]);
    check(Math.abs(math.pointCloudReadPoints(c)[1].y - 5) < 1e-6, "a cloud round trip");
    check(math.pointCloudField(c, "z").offset === 8 && math.pointCloudField(c, "q") === null, "a field");
    check(math.pointCloudSelect(c, [0, 1]).data.length === 12, "select");
    throws("Argument", () => math.pointCloudSelect(c, [1]), "a short mask");

    // the heap may grow during a call, far past the scratch memory
    const big = new Float64Array(3 * 2000000);
    for (let i = 0; i < big.length; i++) big[i] = i;
    const copied = math.poseApplyPoints({ position: d3(1, 0, 0), orientation: IDENTITY }, big);
    check(copied[3] === 4 && copied.length === big.length, "a list larger than the first heap");
    check(math.double3Length(d3(3, 4, 0)) === 5, "and calls after it");

    // a name in place of a number, and a bad one
    check(math.axesOf("Unity").x === -2, "an enum by name");
    throws("Argument", () => math.axesOf("Nothing"), "an enum that does not exist");
    const q = math.quaternionFromEuler(0.1, 0.2, 0.3, "FixedXyz");
    check(Math.abs(q.w) > 0.5, "euler by name");
}


/** Runs all of it over `binding`, the object holding every call. Returns how many failed. */
export function run(binding, vectors, say) {
    math = binding;
    MathError = binding.MathError;
    doc = vectors;
    types = doc.types;
    enums = new Set(doc.enums);
    log = say;
    failures = 0;
    replay();
    handTests();
    return failures;
}
