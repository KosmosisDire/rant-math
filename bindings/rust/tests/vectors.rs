//! The Rust binding: every case of vectors.json through its call, then what vectors cannot show.

use rant_math::rant_types::*;
use rant_math::*;
use serde_json::{Map, Value};

trait Json: Sized {
    fn get(v: &Value) -> Self;
    fn put(&self) -> Value;
    fn get_list(v: &Value) -> Vec<Self> {
        v.as_array().unwrap().iter().map(Self::get).collect()
    }
    fn put_list(v: &[Self]) -> Value {
        Value::Array(v.iter().map(Self::put).collect())
    }
}

fn number(v: &Value) -> f64 {
    match v {
        Value::String(s) => match s.as_str() {
            "nan" => f64::NAN,
            "inf" => f64::INFINITY,
            _ => f64::NEG_INFINITY,
        },
        _ => v.as_f64().unwrap(),
    }
}

fn put_number(x: f64) -> Value {
    if x.is_nan() {
        Value::from("nan")
    } else if x.is_infinite() {
        Value::from(if x > 0.0 { "inf" } else { "-inf" })
    } else {
        Value::from(x)
    }
}

impl Json for f64 {
    fn get(v: &Value) -> f64 {
        number(v)
    }
    fn put(&self) -> Value {
        put_number(*self)
    }
}

impl Json for f32 {
    fn get(v: &Value) -> f32 {
        number(v) as f32
    }
    fn put(&self) -> Value {
        put_number(*self as f64)
    }
}

macro_rules! whole {
    ($($t:ty),*) => {$(
        impl Json for $t {
            fn get(v: &Value) -> $t { v.as_f64().unwrap() as $t }
            fn put(&self) -> Value { Value::from(*self as f64) }
        }
    )*};
}
whole!(i8, i32, u32, u64, usize);

fn hex(d: &[u8]) -> std::string::String {
    d.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
}

impl Json for u8 {
    fn get(v: &Value) -> u8 {
        v.as_f64().unwrap() as u8
    }
    fn put(&self) -> Value {
        Value::from(*self as f64)
    }
    fn get_list(v: &Value) -> Vec<u8> {
        v.as_array().unwrap().iter().map(u8::get).collect()
    }
    fn put_list(v: &[u8]) -> Value {
        Value::from(hex(v))
    }
}

impl Json for bool {
    fn get(v: &Value) -> bool {
        v.as_bool().unwrap()
    }
    fn put(&self) -> Value {
        Value::from(*self)
    }
}

impl<T: Json> Json for Vec<T> {
    fn get(v: &Value) -> Vec<T> {
        T::get_list(v)
    }
    fn put(&self) -> Value {
        T::put_list(self)
    }
}

impl<T: Json + Copy, const N: usize> Json for [T; N] {
    fn get(v: &Value) -> [T; N] {
        let list = T::get_list(v);
        std::array::from_fn(|i| list[i])
    }
    fn put(&self) -> Value {
        T::put_list(self)
    }
}

impl<T: Json> Json for Option<T> {
    fn get(v: &Value) -> Option<T> {
        if v.is_null() { None } else { Some(T::get(v)) }
    }
    fn put(&self) -> Value {
        self.as_ref().map_or(Value::Null, T::put)
    }
}

impl<A: Json, B: Json> Json for (A, B) {
    fn get(_: &Value) -> Self {
        unreachable!()
    }
    fn put(&self) -> Value {
        Value::Array(vec![self.0.put(), self.1.put()])
    }
}

impl<A: Json, B: Json, C: Json> Json for (A, B, C) {
    fn get(_: &Value) -> Self {
        unreachable!()
    }
    fn put(&self) -> Value {
        Value::Array(vec![self.0.put(), self.1.put(), self.2.put()])
    }
}

impl Json for Matrix {
    fn get(v: &Value) -> Matrix {
        Matrix { rows: Json::get(&v["rows"]), cols: Json::get(&v["cols"]), data: Json::get(&v["data"]) }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("rows".into(), self.rows.put());
        m.insert("cols".into(), self.cols.put());
        m.insert("data".into(), self.data.put());
        Value::Object(m)
    }
}

impl Json for ScalarKind {
    fn get(v: &Value) -> ScalarKind {
        match v.as_i64().unwrap() {
            1 => ScalarKind::I8,
            2 => ScalarKind::U8,
            3 => ScalarKind::I16,
            4 => ScalarKind::U16,
            5 => ScalarKind::I32,
            6 => ScalarKind::U32,
            7 => ScalarKind::F32,
            8 => ScalarKind::F64,
            9 => ScalarKind::I64,
            _ => ScalarKind::U64,
        }
    }
    fn put(&self) -> Value {
        Value::from(*self as i64)
    }
}

impl Json for Image {
    fn get(v: &Value) -> Image {
        Image {
            width: Json::get(&v["width"]),
            height: Json::get(&v["height"]),
            stride: Json::get(&v["stride"]),
            format: Json::get(&v["format"]),
            data: unhex(v["data"].as_str().unwrap()),
        }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("width".into(), self.width.put());
        m.insert("height".into(), self.height.put());
        m.insert("stride".into(), self.stride.put());
        m.insert("format".into(), self.format.put());
        m.insert("data".into(), Value::from(hex(&self.data)));
        Value::Object(m)
    }
}

impl Json for PointField {
    fn get(v: &Value) -> PointField {
        PointField {
            name: v["name"].as_str().unwrap().to_string(),
            offset: Json::get(&v["offset"]),
            kind: Json::get(&v["kind"]),
            count: Json::get(&v["count"]),
        }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("name".into(), Value::from(self.name.clone()));
        m.insert("offset".into(), self.offset.put());
        m.insert("kind".into(), self.kind.put());
        m.insert("count".into(), self.count.put());
        Value::Object(m)
    }
}

impl Json for PointCloud {
    fn get(v: &Value) -> PointCloud {
        PointCloud {
            fields: Json::get(&v["fields"]),
            point_step: Json::get(&v["point_step"]),
            data: unhex(v["data"].as_str().unwrap()),
            ..PointCloud::default()
        }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("fields".into(), self.fields.put());
        m.insert("point_step".into(), self.point_step.put());
        m.insert("data".into(), Value::from(hex(&self.data)));
        Value::Object(m)
    }
}

impl Json for GridField {
    fn get(v: &Value) -> GridField {
        GridField { name: v["name"].as_str().unwrap().to_string(), kind: Json::get(&v["kind"]), data: unhex(v["data"].as_str().unwrap()) }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("name".into(), Value::from(self.name.clone()));
        m.insert("kind".into(), self.kind.put());
        m.insert("data".into(), Value::from(hex(&self.data)));
        Value::Object(m)
    }
}

impl Json for Grid {
    fn get(v: &Value) -> Grid {
        Grid {
            origin: Transform { pose: Json::get(&v["origin"]), ..Transform::default() },
            resolution: Json::get(&v["resolution"]),
            cells_x: Json::get(&v["cells_x"]),
            cells_y: Json::get(&v["cells_y"]),
            cells_z: Json::get(&v["cells_z"]),
            fields: Json::get(&v["fields"]),
        }
    }
    fn put(&self) -> Value {
        let mut m = Map::new();
        m.insert("origin".into(), self.origin.pose.put());
        m.insert("resolution".into(), self.resolution.put());
        m.insert("cells_x".into(), self.cells_x.put());
        m.insert("cells_y".into(), self.cells_y.put());
        m.insert("cells_z".into(), self.cells_z.put());
        m.insert("fields".into(), self.fields.put());
        Value::Object(m)
    }
}

impl Json for Prism {
    fn get(v: &Value) -> Prism {
        Prism { pose: Json::get(&v["pose"]), base: Polygon2D { points: Json::get(&v["base"]["points"]) }, height: Json::get(&v["height"]) }
    }
    fn put(&self) -> Value {
        let mut base = Map::new();
        base.insert("points".into(), self.base.points.put());
        let mut m = Map::new();
        m.insert("pose".into(), self.pose.put());
        m.insert("base".into(), Value::Object(base));
        m.insert("height".into(), self.height.put());
        Value::Object(m)
    }
}

fn get_prism(v: &Value) -> Prism {
    Json::get(v)
}
fn get_image(v: &Value) -> Image {
    Json::get(v)
}
fn get_cloud(v: &Value) -> PointCloud {
    Json::get(v)
}
fn get_grid(v: &Value) -> Grid {
    Json::get(v)
}
fn get_matrix(v: &Value) -> Matrix {
    Json::get(v)
}

fn get_scan(v: &Value) -> LaserScan {
    LaserScan {
        angle_min: Json::get(&v["angle_min"]),
        angle_step: Json::get(&v["angle_step"]),
        range_min: Json::get(&v["range_min"]),
        range_max: Json::get(&v["range_max"]),
        echoes: Json::get(&v["echoes"]),
        ranges: Json::get(&v["ranges"]),
        ..LaserScan::default()
    }
}

fn get_det2d(v: &Value) -> Vec<Detection2D> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|d| Detection2D { bounds: Json::get(&d["bounds"]), label: d["label"].as_str().unwrap().to_string(), score: Json::get(&d["score"]), ..Detection2D::default() })
        .collect()
}

fn get_det3d(v: &Value) -> Vec<Detection3D> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|d| Detection3D { bounds: Json::get(&d["bounds"]), label: d["label"].as_str().unwrap().to_string(), score: Json::get(&d["score"]), ..Detection3D::default() })
        .collect()
}

// What a borrowed shape borrows, kept beside it for the call.
struct ShapeData {
    v: Value,
    prism: Option<Prism>,
    points: Vec<Double3>,
    triangles: Vec<u32>,
}

impl ShapeData {
    fn new(v: &Value) -> ShapeData {
        let mut d = ShapeData { v: v.clone(), prism: None, points: vec![], triangles: vec![] };
        let value = &v["value"];
        match v["kind"].as_str().unwrap() {
            "prism" => d.prism = Some(Json::get(value)),
            "polyline" | "polygon" => d.points = Json::get(value),
            "mesh" => {
                d.points = Json::get(&value["vertices"]);
                d.triangles = Json::get(&value["triangles"]);
            }
            _ => {}
        }
        d
    }
    fn shape(&self) -> Shape<'_> {
        let value = &self.v["value"];
        match self.v["kind"].as_str().unwrap() {
            "aligned_box" => shape_from_aligned_box(Json::get(value)),
            "oriented_box" => shape_from_oriented_box(Json::get(value)),
            "sphere" => shape_from_sphere(Json::get(value)),
            "capsule" => shape_from_capsule(Json::get(value)),
            "cylinder" => shape_from_cylinder(Json::get(value)),
            "cone" => shape_from_cone(Json::get(value)),
            "prism" => shape_from_prism(self.prism.as_ref().unwrap()),
            "segment" => shape_from_segment(Json::get(value)),
            "polyline" => shape_from_polyline(&self.points),
            "polygon" => shape_from_polygon(&self.points),
            "mesh" => shape_from_mesh(&self.points, &self.triangles),
            k => panic!("no shape {k}"),
        }
    }
}

struct Shape2DData {
    v: Value,
    points: Vec<Double2>,
}

impl Shape2DData {
    fn new(v: &Value) -> Shape2DData {
        let mut d = Shape2DData { v: v.clone(), points: vec![] };
        if let "polyline2d" | "polygon2d" = v["kind"].as_str().unwrap() {
            d.points = Json::get(&v["value"]);
        }
        d
    }
    fn shape(&self) -> Shape2D<'_> {
        let value = &self.v["value"];
        match self.v["kind"].as_str().unwrap() {
            "aligned_box2d" => shape2d_from_aligned_box2d(Json::get(value)),
            "oriented_box2d" => shape2d_from_oriented_box2d(Json::get(value)),
            "circle" => shape2d_from_circle(Json::get(value)),
            "capsule2d" => shape2d_from_capsule2d(Json::get(value)),
            "polygon2d" => shape2d_from_polygon2d(&self.points),
            "segment2d" => shape2d_from_segment2d(Json::get(value)),
            "polyline2d" => shape2d_from_polyline2d(&self.points),
            k => panic!("no 2D shape {k}"),
        }
    }
}

struct FitData {
    some: bool,
    weights: Option<Vec<f64>>,
    tolerance: f64,
    tries: u32,
    seed: u64,
    inliers: Option<Vec<u8>>,
}

impl FitData {
    fn new(v: &Value, points: usize) -> FitData {
        if v.is_null() {
            return FitData { some: false, weights: None, tolerance: 0.0, tries: 0, seed: 0, inliers: None };
        }
        FitData {
            some: true,
            weights: Json::get(&v["weights"]),
            tolerance: Json::get(&v["tolerance"]),
            tries: Json::get(&v["tries"]),
            seed: Json::get(&v["seed"]),
            inliers: if v["inliers"].as_bool().unwrap() { Some(vec![0; points]) } else { None },
        }
    }
    fn options(&mut self) -> Option<FitOptions<'_>> {
        if !self.some {
            return None;
        }
        Some(FitOptions {
            weights: self.weights.as_deref(),
            tolerance: self.tolerance,
            tries: self.tries,
            seed: self.seed,
            inliers: self.inliers.as_deref_mut(),
        })
    }
    fn extras(&self, extra: &mut Map<std::string::String, Value>) {
        if let Some(i) = &self.inliers {
            extra.insert("inliers".into(), Value::from(hex(i)));
        }
    }
}

struct AlignData {
    some: bool,
    max_distance: f64,
    iterations: u32,
    normals: Option<Vec<Double3>>,
    inliers: Option<Vec<u8>>,
    error: Option<f64>,
}

impl AlignData {
    fn new(v: &Value, source: usize) -> AlignData {
        if v.is_null() {
            return AlignData { some: false, max_distance: 0.0, iterations: 0, normals: None, inliers: None, error: None };
        }
        AlignData {
            some: true,
            max_distance: Json::get(&v["max_distance"]),
            iterations: Json::get(&v["iterations"]),
            normals: Json::get(&v["normals"]),
            inliers: if v["inliers"].as_bool().unwrap() { Some(vec![0; source]) } else { None },
            error: if v["error"].as_bool().unwrap() { Some(0.0) } else { None },
        }
    }
    fn options(&mut self) -> Option<AlignOptions<'_>> {
        if !self.some {
            return None;
        }
        Some(AlignOptions {
            max_distance: self.max_distance,
            iterations: self.iterations,
            normals: self.normals.as_deref(),
            inliers: self.inliers.as_deref_mut(),
            error: self.error.as_mut(),
        })
    }
    fn extras(&self, extra: &mut Map<std::string::String, Value>) {
        if let Some(i) = &self.inliers {
            extra.insert("inliers".into(), Value::from(hex(i)));
        }
        if let Some(e) = self.error {
            extra.insert("error".into(), put_number(e));
        }
    }
}

include!("generated/thunks.rs");

fn close(want: &Value, got: &Value, tol: f64) -> bool {
    match (want, got) {
        (Value::Number(a), Value::Number(b)) => {
            let (a, b) = (a.as_f64().unwrap(), b.as_f64().unwrap());
            a == b || (a - b).abs() <= tol * 1f64.max(a.abs()).max(b.abs())
        }
        (Value::Array(a), Value::Array(b)) => a.len() == b.len() && a.iter().zip(b).all(|(x, y)| close(x, y, tol)),
        (Value::Object(a), Value::Object(b)) => a.len() == b.len() && a.iter().all(|(k, x)| b.get(k).is_some_and(|y| close(x, y, tol))),
        _ => want == got,
    }
}

#[test]
fn every_case_of_every_function() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tests/bindings/vectors.json");
    let doc: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut cases = 0;
    let mut failures = 0;
    let functions = doc["functions"].as_array().unwrap();
    for entry in functions {
        let name = entry["f"].as_str().unwrap();
        let tol = entry["tol"].as_f64().unwrap();
        for case in entry["cases"].as_array().unwrap() {
            let mut extra = Map::new();
            let mut got = Map::new();
            match call(name, case["a"].as_array().unwrap(), &mut extra) {
                Ok(v) => {
                    got.insert("v".into(), v);
                    got.extend(extra);
                }
                Err(e) => {
                    got.insert("e".into(), Value::from(e.code() as f64));
                }
            }
            let mut want = case.as_object().unwrap().clone();
            want.remove("a");
            if !close(&Value::Object(want.clone()), &Value::Object(got.clone()), tol) {
                println!("{name} {}\n  want {}\n  got  {}", case["a"], Value::Object(want), Value::Object(got));
                failures += 1;
            }
            cases += 1;
        }
    }
    println!("replayed {cases} cases of {} functions", functions.len());
    assert_eq!(failures, 0);
}

fn d3(x: f64, y: f64, z: f64) -> Double3 {
    Double3 { x, y, z }
}

#[test]
fn results() {
    let plane = Plane { position: d3(0.0, 0.0, 0.0), normal: d3(0.0, 0.0, 1.0) };
    assert!(plane_raycast(plane, Ray { origin: d3(0.0, 0.0, 5.0), direction: d3(0.0, 0.0, 1.0) }).is_none());
    assert_eq!(plane_raycast(plane, Ray { origin: d3(0.0, 0.0, 5.0), direction: d3(0.0, 0.0, -1.0) }).unwrap().distance, 5.0);
    assert!(sphere_contains(Sphere { center: d3(0.0, 0.0, 0.0), radius: 1.0 }, d3(0.0, 0.0, 0.0)));
    let (axis, angle) = quaternion_to_axis_angle(quaternion_from_axis_angle(d3(0.0, 0.0, 1.0), 0.5));
    assert!((axis.z - 1.0).abs() < 1e-12 && (angle - 0.5).abs() < 1e-12);
    assert_eq!(matrix3x3_inverse(Matrix3x3::default()), Err(MathError::Degenerate));
    assert!(!MathError::Degenerate.to_string().is_empty());
    assert_eq!(aligned_box_corners(AlignedBox { min: d3(0.0, 0.0, 0.0), max: d3(1.0, 1.0, 1.0) }).len(), 8);
}

#[test]
fn lists_and_options() {
    let three = [d3(0.0, 0.0, 0.0), d3(1.0, 0.0, 0.0), d3(0.0, 1.0, 0.0)];
    assert_eq!(points_fit_pose(&three, &three[..2], None), Err(MathError::Argument));
    let mut marks = [0u8; 3];
    let plane = points_fit_plane(&three, Some(&mut FitOptions { inliers: Some(&mut marks), ..Default::default() })).unwrap();
    assert_eq!(marks, [1, 1, 1]);
    assert!((plane.normal.z.abs() - 1.0).abs() < 1e-12);
    let mut short = [0u8; 2];
    assert_eq!(points_fit_plane(&three, Some(&mut FitOptions { inliers: Some(&mut short), ..Default::default() })), Err(MathError::Argument));
    let more = [d3(0.0, 0.0, 0.0), d3(1.0, 0.0, 0.0), d3(0.0, 1.0, 0.0), d3(0.0, 0.0, 1.0)];
    let mut error = -1.0;
    points_align(&more, &more, Pose::identity(), Some(&mut AlignOptions { error: Some(&mut error), ..Default::default() })).unwrap();
    assert!((0.0..1e-9).contains(&error));

    let tree = kdtree_create(&more).unwrap();
    let (indices, distances) = kdtree_nearest(&tree, d3(0.0, 0.0, 0.1), 2).unwrap();
    assert_eq!(indices[0], 0);
    assert!((distances[0] - 0.1).abs() < 1e-12);
    assert_eq!(kdtree_within(&tree, d3(0.0, 0.0, 0.0), 0.5).unwrap().0.len(), 1);
    let shared = std::sync::Arc::new(tree);
    let handles: Vec<_> = (0..4)
        .map(|_| {
            let t = shared.clone();
            std::thread::spawn(move || kdtree_nearest(&t, d3(0.0, 0.0, 0.0), 1).unwrap().0[0])
        })
        .collect();
    for h in handles {
        assert_eq!(h.join().unwrap(), 0);
    }
}

#[test]
#[should_panic(expected = "threes")]
fn triangles_not_in_threes_panic() {
    mesh_area(&[d3(0.0, 0.0, 0.0)], &[0, 1]);
}

#[test]
fn shapes_borrow() {
    let points = vec![d3(0.0, 0.0, 0.0), d3(2.0, 0.0, 0.0)];
    let line = shape_from_polyline(&points);
    let copy = line;
    assert!((shape_distance(&copy, d3(1.0, 1.0, 0.0)) - 1.0).abs() < 1e-12);
    let (vertices, triangles) = shape_mesh(&shape_from_sphere(Sphere { center: d3(0.0, 0.0, 0.0), radius: 1.0 }), 8).unwrap();
    assert!(!vertices.is_empty() && !triangles.is_empty() && triangles.len() % 3 == 0);
}

#[test]
fn images_clouds_grids() {
    let mut image = Image { width: 2, height: 1, stride: 0, format: ImageFormat::Rgb8, data: vec![1, 2, 3, 4, 5, 6] };
    assert_eq!(image_color_at(&image, 1, 0).unwrap().g, 5);
    let view = ImageRef { width: 2, height: 1, stride: 0, format: ImageFormat::Rgb8, data: &image.data };
    assert_eq!(image_color_at(view, 0, 0).unwrap().r, 1);
    assert_eq!(image_convert(&image, ImageFormat::Mono8).unwrap().data.len(), 2);
    assert_eq!(image_rotate(&image, -1).unwrap().height, 2);
    let part = image_crop(&image, 1, 0, 1, 1).unwrap();
    image.data[3] = 99;
    assert_eq!(part.data[0], 4);
    let circle = shape2d_from_circle(Circle { center: Double2 { x: 0.0, y: 0.0 }, radius: 5.0 });
    image_draw(&mut image, &circle, Color { r: 7, g: 8, b: 9, a: 255 }, 0.0).unwrap();
    assert_eq!(image.data[0], 7);
    let mut pixels = [0u8; 3];
    let target = ImageMut { width: 1, height: 1, stride: 0, format: ImageFormat::Rgb8, data: &mut pixels };
    image_draw(target, &circle, Color { r: 1, g: 2, b: 3, a: 255 }, 0.0).unwrap();
    assert_eq!(pixels, [1, 2, 3]);

    let mut cloud = point_cloud_layout(PointCloudLayout::Xyz);
    cloud.data = vec![0; 24];
    point_cloud_write_points(&mut cloud, &[d3(1.0, 2.0, 3.0), d3(4.0, 5.0, 6.0)]).unwrap();
    assert!((point_cloud_read_points(&cloud).unwrap()[1].y - 5.0).abs() < 1e-6);
    assert_eq!(point_cloud_field(&cloud, "z").unwrap().offset, 8);
    assert!(point_cloud_field(&cloud, "q").is_none());
    assert_eq!(point_cloud_select(&cloud, &[0, 1]).unwrap().data.len(), 12);
    assert_eq!(point_cloud_select(&cloud, &[1]), Err(MathError::Argument));

    let mut grid = Grid {
        origin: Transform::identity(),
        resolution: 1.0,
        cells_x: 2,
        cells_y: 2,
        cells_z: 1,
        fields: vec![GridField { name: "cost".into(), kind: ScalarKind::U8, data: vec![0, 1, 2, 3] }],
    };
    grid_write_value(&mut grid, "cost", Int3 { x: 1, y: 1, z: 0 }, 9.0).unwrap();
    assert_eq!(grid.fields[0].data[3], 9);
    assert_eq!(grid_read_value(&grid, "cost", Int3 { x: 1, y: 1, z: 0 }).unwrap(), 9.0);
}

#[cfg(feature = "nalgebra")]
#[test]
fn nalgebra_conversions() {
    let v = d3(1.0, 2.0, 3.0);
    assert_eq!(v.to_nalgebra().to_rant(), v);
    let q = quaternion_from_axis_angle(d3(0.0, 0.0, 1.0), 0.5);
    let p = Pose { position: v, orientation: q };
    let by_math = pose_apply(p, d3(4.0, 5.0, 6.0));
    let by_na = p.to_nalgebra() * nalgebra::Point3::new(4.0, 5.0, 6.0);
    assert!((by_math.x - by_na.x).abs() < 1e-9 && (by_math.y - by_na.y).abs() < 1e-9);
    let back = p.to_nalgebra().to_rant();
    assert!((back.orientation.w - q.w).abs() < 1e-12 && back.position == v);
    let m = Matrix { rows: 2, cols: 3, data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0] };
    assert_eq!(m.to_nalgebra()[(1, 0)], 4.0);
    assert_eq!(m.to_nalgebra().to_rant(), m);
}
