//! Rant Math for a program that scripts in QuickJS: the same names and plain values as the TypeScript
//! binding, for JavaScript that has no wasm. `install` gives the object that holds every call.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use rquickjs::function::Rest;
use rquickjs::{Array, Ctx, Exception, Function, Object, Result as JsResult, TypedArray, Value};

use crate::*;

mod glue {
    include!("quickjs_glue.rs");
}

/// Every call by name in JavaScript with what it takes, as a module that exports `NAMES`, for
/// completion and hints.
pub const NAMES: &str = include_str!("quickjs_names.js");

const PRELUDE: &str = include_str!("quickjs_prelude.js");

/// What a program's context holds behind each kd tree it made, freed with the context.
#[derive(Default)]
pub(crate) struct TreeList {
    next: u32,
    trees: HashMap<u32, Rc<KdTree>>,
}

impl TreeList {
    fn insert(&mut self, tree: KdTree) -> u32 {
        self.next += 1;
        self.trees.insert(self.next, Rc::new(tree));
        self.next
    }
}

type Trees = Rc<RefCell<TreeList>>;

/// The object holding every call of Rant Math in this context.
pub fn install<'js>(ctx: &Ctx<'js>) -> JsResult<Object<'js>> {
    let trees: Trees = Rc::default();
    let free = glue_free_index();
    let call = Function::new(ctx.clone(), move |ctx: Ctx<'js>, fail: Function<'js>, index: u32, args: Rest<Value<'js>>| -> JsResult<Value<'js>> {
        if index == free {
            if let Some(id) = args.0.first().and_then(|v| v.as_number()) {
                trees.borrow_mut().trees.remove(&(id as u32));
            }
            return Ok(Value::new_undefined(ctx.clone()));
        }
        // A bad name or a freed tree is known only deep in the reading, so it surfaces here.
        glue::dispatch(&ctx, &fail, &trees, index, &args.0).map_err(|e| match e {
            rquickjs::Error::Unknown => math_error(&ctx, &fail, MathError::Argument),
            other => other,
        })
    })?;
    let make: Function = ctx.eval(PRELUDE)?;
    make.call((call,))
}

/// The number the prelude frees a tree by, one past every call.
fn glue_free_index() -> u32 {
    let source = include_str!("quickjs_prelude.js");
    let at = source.find("if (this.id) call(fail, ").expect("the prelude frees a tree") + "if (this.id) call(fail, ".len();
    source[at..].split(',').next().expect("its number").trim().parse().expect("a number")
}

pub(crate) trait Js<'js>: Sized {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self>;
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>>;
}

fn at<'js>(args: &[Value<'js>], i: usize) -> Value<'js> {
    args.get(i).cloned().unwrap_or_else(|| Value::new_undefined(args_ctx(args)))
}

fn args_ctx<'a, 'js>(args: &'a [Value<'js>]) -> Ctx<'js> {
    args.first().expect("a call with an argument").ctx().clone()
}

fn fail_argument<'js>(_: &Ctx<'js>) -> rquickjs::Error {
    rquickjs::Error::Unknown
}

fn math_error<'js>(ctx: &Ctx<'js>, fail: &Function<'js>, e: MathError) -> rquickjs::Error {
    match fail.call::<_, Value>((e.code(), e.to_string())) {
        Ok(error) => ctx.throw(error),
        Err(other) => other,
    }
}

fn field<'js, T: Js<'js>>(ctx: &Ctx<'js>, v: &Value<'js>, name: &str) -> JsResult<T> {
    let Some(o) = v.as_object() else { return Err(Exception::throw_type(ctx, &format!("a value with a field {name}"))) };
    let inner: Value = o.get(name)?;
    T::get(ctx, &inner).map_err(|e| if inner.is_undefined() { Exception::throw_type(ctx, &format!("a value with a field {name}")) } else { e })
}

fn maybe<'js, T: Js<'js>>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Option<T>> {
    if v.is_undefined() || v.is_null() { Ok(None) } else { T::get(ctx, v).map(Some) }
}

fn text<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<std::string::String> {
    match v.as_string() {
        Some(s) => s.to_string(),
        None => Err(Exception::throw_type(ctx, "text")),
    }
}

macro_rules! numeric {
    ($($t:ty),*) => {$(
        impl<'js> Js<'js> for $t {
            fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<$t> {
                match v.as_number() {
                    Some(n) => Ok(n as $t),
                    None => Err(Exception::throw_type(ctx, "a number")),
                }
            }
            fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
                Ok(Value::new_number(ctx.clone(), *self as f64))
            }
        }
    )*};
}
numeric!(f64, f32, i8, i32, u8, u32, u64, usize);

impl<'js> Js<'js> for bool {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<bool> {
        v.as_bool().ok_or_else(|| Exception::throw_type(ctx, "true or false"))
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        Ok(Value::new_bool(ctx.clone(), *self))
    }
}

// Numbers from any list: an array or a typed array of any kind.
fn numbers<'js, T: Js<'js>>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Vec<T>> {
    let Some(o) = v.as_object() else { return Err(Exception::throw_type(ctx, "a list")) };
    let length: Option<f64> = o.get::<_, Value>("length")?.as_number();
    let Some(length) = length else { return Err(Exception::throw_type(ctx, "a list")) };
    let mut out = Vec::with_capacity(length as usize);
    for i in 0..length as u32 {
        let item: Value = o.get(i)?;
        out.push(T::get(ctx, &item)?);
    }
    Ok(out)
}

// A number a typed array can hold.
trait Typed: Copy {
    fn array<'js>(ctx: &Ctx<'js>, v: &[Self]) -> JsResult<Value<'js>>;
}

macro_rules! typed {
    ($($t:ty),*) => {$(
        impl Typed for $t {
            fn array<'js>(ctx: &Ctx<'js>, v: &[Self]) -> JsResult<Value<'js>> {
                Ok(TypedArray::<$t>::new_copy(ctx.clone(), v)?.into_value())
            }
        }
    )*};
}
typed!(f64, f32, u8, u32);

fn numbers_out<'js, T: Typed>(ctx: &Ctx<'js>, v: &[T]) -> JsResult<Value<'js>> {
    T::array(ctx, v)
}

// A list of structs, or a Float64Array of flat doubles each when a struct is only doubles.
fn list<'js, T: Js<'js> + Copy>(ctx: &Ctx<'js>, v: &Value<'js>, flat: usize) -> JsResult<(Vec<T>, bool)> {
    if flat > 0 {
        if let Ok(typed) = TypedArray::<f64>::from_value(v.clone()) {
            let Some(raw) = (unsafe { typed.as_bytes() }) else { return Err(Exception::throw_type(ctx, "a detached list")) };
            if raw.len() % (flat * 8) != 0 {
                return Err(Exception::throw_type(ctx, "a list that is not in whole points"));
            }
            debug_assert_eq!(std::mem::size_of::<T>(), flat * 8);
            let mut out = Vec::<T>::with_capacity(raw.len() / (flat * 8));
            unsafe {
                std::ptr::copy_nonoverlapping(raw.as_ptr(), out.as_mut_ptr() as *mut u8, raw.len());
                out.set_len(raw.len() / (flat * 8));
            }
            return Ok((out, true));
        }
    }
    Ok((numbers(ctx, v)?, false))
}

fn list_out<'js, T: Js<'js> + Copy>(ctx: &Ctx<'js>, v: &[T], typed: bool, flat: usize) -> JsResult<Value<'js>> {
    if typed && flat > 0 {
        debug_assert_eq!(std::mem::size_of::<T>(), flat * 8);
        let doubles = unsafe { std::slice::from_raw_parts(v.as_ptr() as *const f64, v.len() * flat) };
        return numbers_out(ctx, doubles);
    }
    let out = Array::new(ctx.clone())?;
    for (i, x) in v.iter().enumerate() {
        out.set(i, x.put(ctx)?)?;
    }
    Ok(out.into_value())
}

fn pair<'js>(ctx: &Ctx<'js>, items: Vec<Value<'js>>) -> JsResult<Value<'js>> {
    let out = Array::new(ctx.clone())?;
    for (i, x) in items.into_iter().enumerate() {
        out.set(i, x)?;
    }
    Ok(out.into_value())
}

fn rows<'js, T, const N: usize>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<[[T; N]; N]>
where
    T: Js<'js> + Copy + Default,
{
    let list: Vec<Value> = numbers(ctx, v)?;
    if list.len() != N {
        return Err(math_argument(ctx));
    }
    let mut out = [[T::default(); N]; N];
    for (r, row) in list.iter().enumerate() {
        let cells: Vec<T> = numbers(ctx, row)?;
        if cells.len() != N {
            return Err(math_argument(ctx));
        }
        out[r].copy_from_slice(&cells);
    }
    Ok(out)
}

fn rows_out<'js, T: Typed, const N: usize>(ctx: &Ctx<'js>, m: &[[T; N]; N]) -> JsResult<Value<'js>> {
    let out = Array::new(ctx.clone())?;
    for (r, row) in m.iter().enumerate() {
        out.set(r, numbers_out(ctx, row)?)?;
    }
    Ok(out.into_value())
}

impl<'js> Js<'js> for Value<'js> {
    fn get(_: &Ctx<'js>, v: &Value<'js>) -> JsResult<Value<'js>> {
        Ok(v.clone())
    }
    fn put(&self, _: &Ctx<'js>) -> JsResult<Value<'js>> {
        Ok(self.clone())
    }
}

fn math_argument<'js>(ctx: &Ctx<'js>) -> rquickjs::Error {
    Exception::throw_range(ctx, "a list of the wrong size")
}

fn coeffs<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<[f64; 8]> {
    let list: Vec<f64> = numbers(ctx, &v.as_object().expect("a camera").get::<_, Value>("coeffs")?)?;
    if list.len() > 8 {
        return Err(math_argument(ctx));
    }
    let mut out = [0.0; 8];
    out[..list.len()].copy_from_slice(&list);
    Ok(out)
}

fn bytes_of<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Vec<u8>> {
    if let Ok(typed) = TypedArray::<u8>::from_value(v.clone()) {
        return Ok(unsafe { typed.as_bytes() }.map(<[u8]>::to_vec).unwrap_or_default());
    }
    numbers(ctx, v)
}

// Writes what a call changed back over the bytes the value came with.
fn bytes_back<'js>(v: &Value<'js>, bytes: &[u8]) -> JsResult<()> {
    if let Ok(typed) = TypedArray::<u8>::from_value(v.clone()) {
        if let Some(raw) = typed.as_raw() {
            let n = raw.len().min(bytes.len());
            unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), raw.as_ptr() as *mut u8, n) };
        }
    }
    Ok(())
}

fn prism_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Prism> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a prism"))?;
    let base: Value = o.get("base")?;
    let points: Value = base.as_object().ok_or_else(|| Exception::throw_type(ctx, "a prism with a base"))?.get("points")?;
    Ok(Prism { pose: field(ctx, v, "pose")?, base: Polygon2D { points: list(ctx, &points, 2)?.0 }, height: field(ctx, v, "height")? })
}

impl<'js> Js<'js> for Prism {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Prism> {
        prism_in(ctx, v)
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let base = Object::new(ctx.clone())?;
        base.set("points", list_out(ctx, &self.base.points, false, 2)?)?;
        let o = Object::new(ctx.clone())?;
        o.set("pose", self.pose.put(ctx)?)?;
        o.set("base", base)?;
        o.set("height", self.height)?;
        Ok(o.into_value())
    }
}

fn image_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Image> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "an image"))?;
    Ok(Image {
        width: field(ctx, v, "width")?,
        height: field(ctx, v, "height")?,
        stride: field(ctx, v, "stride").unwrap_or(0),
        format: field(ctx, v, "format")?,
        data: bytes_of(ctx, &o.get::<_, Value>("data")?)?,
    })
}

fn image_back<'js>(v: &Value<'js>, image: &Image) -> JsResult<()> {
    bytes_back(&v.as_object().expect("an image").get::<_, Value>("data")?, &image.data)
}

impl<'js> Js<'js> for Image {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Image> {
        image_in(ctx, v)
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let o = Object::new(ctx.clone())?;
        o.set("width", self.width)?;
        o.set("height", self.height)?;
        o.set("stride", self.stride)?;
        o.set("format", self.format.put(ctx)?)?;
        o.set("data", numbers_out(ctx, &self.data)?)?;
        Ok(o.into_value())
    }
}

impl<'js> Js<'js> for PointField {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<PointField> {
        Ok(PointField {
            name: text(ctx, &v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a field"))?.get::<_, Value>("name")?)?,
            offset: field(ctx, v, "offset")?,
            kind: field(ctx, v, "kind")?,
            count: field(ctx, v, "count")?,
        })
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let o = Object::new(ctx.clone())?;
        o.set("name", self.name.as_str())?;
        o.set("offset", self.offset)?;
        o.set("kind", self.kind.put(ctx)?)?;
        o.set("count", self.count)?;
        Ok(o.into_value())
    }
}

impl<'js> Js<'js> for Option<PointField> {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self> {
        maybe(ctx, v)
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        match self {
            Some(f) => f.put(ctx),
            None => Ok(Value::new_null(ctx.clone())),
        }
    }
}

fn cloud_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<PointCloud> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a cloud"))?;
    Ok(PointCloud {
        fields: numbers(ctx, &o.get::<_, Value>("fields")?)?,
        point_step: field(ctx, v, "point_step")?,
        data: bytes_of(ctx, &o.get::<_, Value>("data")?)?,
        ..PointCloud::default()
    })
}

fn cloud_back<'js>(v: &Value<'js>, cloud: &PointCloud) -> JsResult<()> {
    bytes_back(&v.as_object().expect("a cloud").get::<_, Value>("data")?, &cloud.data)
}

impl<'js> Js<'js> for PointCloud {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<PointCloud> {
        cloud_in(ctx, v)
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let origin = Object::new(ctx.clone())?;
        origin.set("pose", self.origin.pose.put(ctx)?)?;
        origin.set("parent", "")?;
        let o = Object::new(ctx.clone())?;
        o.set("origin", origin)?;
        o.set("width", self.width)?;
        o.set("fields", list_fields(ctx, &self.fields)?)?;
        o.set("point_step", self.point_step)?;
        o.set("dense", self.dense)?;
        o.set("data", numbers_out(ctx, &self.data)?)?;
        Ok(o.into_value())
    }
}

fn list_fields<'js>(ctx: &Ctx<'js>, fields: &[PointField]) -> JsResult<Value<'js>> {
    let out = Array::new(ctx.clone())?;
    for (i, f) in fields.iter().enumerate() {
        out.set(i, f.put(ctx)?)?;
    }
    Ok(out.into_value())
}

impl<'js> Js<'js> for GridField {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<GridField> {
        let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a field"))?;
        Ok(GridField {
            name: text(ctx, &o.get::<_, Value>("name")?)?,
            kind: field(ctx, v, "kind")?,
            data: bytes_of(ctx, &o.get::<_, Value>("data")?)?,
        })
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let o = Object::new(ctx.clone())?;
        o.set("name", self.name.as_str())?;
        o.set("kind", self.kind.put(ctx)?)?;
        o.set("data", numbers_out(ctx, &self.data)?)?;
        Ok(o.into_value())
    }
}

fn grid_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Grid> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a grid"))?;
    let origin: Value = o.get("origin")?;
    Ok(Grid {
        origin: Transform { pose: field(ctx, &origin, "pose")?, ..Transform::default() },
        resolution: field(ctx, v, "resolution")?,
        cells_x: field(ctx, v, "cells_x")?,
        cells_y: field(ctx, v, "cells_y")?,
        cells_z: field(ctx, v, "cells_z")?,
        fields: numbers(ctx, &o.get::<_, Value>("fields")?)?,
    })
}

fn grid_back<'js>(v: &Value<'js>, grid: &Grid) -> JsResult<()> {
    let fields: Object = v.as_object().expect("a grid").get("fields")?;
    for (i, f) in grid.fields.iter().enumerate() {
        let one: Object = fields.get(i as u32)?;
        bytes_back(&one.get::<_, Value>("data")?, &f.data)?;
    }
    Ok(())
}

fn scan_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<LaserScan> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a scan"))?;
    Ok(LaserScan {
        angle_min: field(ctx, v, "angle_min")?,
        angle_step: field(ctx, v, "angle_step")?,
        range_min: field(ctx, v, "range_min")?,
        range_max: field(ctx, v, "range_max")?,
        echoes: field(ctx, v, "echoes")?,
        ranges: numbers(ctx, &o.get::<_, Value>("ranges")?)?,
        ..LaserScan::default()
    })
}

impl<'js> Js<'js> for Detection2D {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Detection2D> {
        let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a detection"))?;
        Ok(Detection2D { bounds: field(ctx, v, "bounds")?, label: text(ctx, &o.get::<_, Value>("label")?)?, score: field(ctx, v, "score")?, ..Detection2D::default() })
    }
    fn put(&self, _: &Ctx<'js>) -> JsResult<Value<'js>> {
        unreachable!("no call answers with a detection")
    }
}

impl<'js> Js<'js> for Detection3D {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Detection3D> {
        let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a detection"))?;
        Ok(Detection3D { bounds: field(ctx, v, "bounds")?, label: text(ctx, &o.get::<_, Value>("label")?)?, score: field(ctx, v, "score")?, ..Detection3D::default() })
    }
    fn put(&self, _: &Ctx<'js>) -> JsResult<Value<'js>> {
        unreachable!("no call answers with a detection")
    }
}

fn detections2d_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Vec<Detection2D>> {
    numbers(ctx, v)
}

fn detections3d_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Vec<Detection3D>> {
    numbers(ctx, v)
}

fn matrix_in<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Matrix> {
    let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a matrix"))?;
    Ok(Matrix { rows: field(ctx, v, "rows")?, cols: field(ctx, v, "cols")?, data: numbers(ctx, &o.get::<_, Value>("data")?)? })
}

impl<'js> Js<'js> for Matrix {
    fn get(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Matrix> {
        matrix_in(ctx, v)
    }
    fn put(&self, ctx: &Ctx<'js>) -> JsResult<Value<'js>> {
        let o = Object::new(ctx.clone())?;
        o.set("rows", self.rows)?;
        o.set("cols", self.cols)?;
        o.set("data", numbers_out(ctx, &self.data)?)?;
        Ok(o.into_value())
    }
}

// What a shape borrows, read from its plain description for one call.
struct ShapeHold<'js> {
    kind: std::string::String,
    value: Value<'js>,
    prism: Option<Prism>,
    points: Vec<Double3>,
    triangles: Vec<u32>,
}

impl<'js> ShapeHold<'js> {
    fn new(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self> {
        let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a shape"))?;
        let kind = text(ctx, &o.get::<_, Value>("kind")?)?;
        let value: Value = o.get("value")?;
        let mut hold = ShapeHold { kind, value: value.clone(), prism: None, points: vec![], triangles: vec![] };
        match hold.kind.as_str() {
            "polyline" | "polygon" => hold.points = list(ctx, &value, 3)?.0,
            "mesh" => {
                let m = value.as_object().ok_or_else(|| Exception::throw_type(ctx, "a mesh"))?;
                hold.points = list(ctx, &m.get::<_, Value>("vertices")?, 3)?.0;
                hold.triangles = numbers(ctx, &m.get::<_, Value>("triangles")?)?;
            }
            "prism" => {
                let p = value.as_object().ok_or_else(|| Exception::throw_type(ctx, "a prism"))?;
                let base: Vec<Double2> = list(ctx, &p.get::<_, Value>("base")?, 2)?.0;
                hold.prism = Some(Prism { pose: field(ctx, &value, "pose")?, base: Polygon2D { points: base }, height: field(ctx, &value, "height")? });
            }
            _ => {}
        }
        Ok(hold)
    }
    fn shape(&self, ctx: &Ctx<'js>) -> JsResult<Shape<'_>> {
        let v = &self.value;
        Ok(match self.kind.as_str() {
            "aligned_box" => shape_from_aligned_box(Js::get(ctx, v)?),
            "oriented_box" => shape_from_oriented_box(Js::get(ctx, v)?),
            "sphere" => shape_from_sphere(Js::get(ctx, v)?),
            "capsule" => shape_from_capsule(Js::get(ctx, v)?),
            "cylinder" => shape_from_cylinder(Js::get(ctx, v)?),
            "cone" => shape_from_cone(Js::get(ctx, v)?),
            "segment" => shape_from_segment(Js::get(ctx, v)?),
            "polyline" => shape_from_polyline(&self.points),
            "polygon" => shape_from_polygon(&self.points),
            "mesh" => shape_from_mesh(&self.points, &self.triangles),
            "prism" => shape_from_prism(self.prism.as_ref().expect("read in new")),
            _ => return Err(fail_argument(ctx)),
        })
    }
}

struct Shape2DHold<'js> {
    kind: std::string::String,
    value: Value<'js>,
    points: Vec<Double2>,
}

impl<'js> Shape2DHold<'js> {
    fn new(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self> {
        let o = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "a shape"))?;
        let kind = text(ctx, &o.get::<_, Value>("kind")?)?;
        let value: Value = o.get("value")?;
        let points = if matches!(kind.as_str(), "polyline2d" | "polygon2d") { list(ctx, &value, 2)?.0 } else { vec![] };
        Ok(Shape2DHold { kind, value, points })
    }
    fn shape(&self, ctx: &Ctx<'js>) -> JsResult<Shape2D<'_>> {
        let v = &self.value;
        Ok(match self.kind.as_str() {
            "aligned_box2d" => shape2d_from_aligned_box2d(Js::get(ctx, v)?),
            "oriented_box2d" => shape2d_from_oriented_box2d(Js::get(ctx, v)?),
            "circle" => shape2d_from_circle(Js::get(ctx, v)?),
            "capsule2d" => shape2d_from_capsule2d(Js::get(ctx, v)?),
            "segment2d" => shape2d_from_segment2d(Js::get(ctx, v)?),
            "polyline2d" => shape2d_from_polyline2d(&self.points),
            "polygon2d" => shape2d_from_polygon2d(&self.points),
            _ => return Err(fail_argument(ctx)),
        })
    }
}

struct FitHold {
    some: bool,
    weights: Option<Vec<f64>>,
    tolerance: f64,
    tries: u32,
    seed: u64,
    inliers: Option<Vec<u8>>,
}

fn opt_field<'js, T: Js<'js> + Default>(ctx: &Ctx<'js>, v: &Value<'js>, name: &str) -> JsResult<T> {
    let inner: Value = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "options"))?.get(name)?;
    if inner.is_undefined() || inner.is_null() { Ok(T::default()) } else { T::get(ctx, &inner) }
}

fn opt_numbers<'js, T: Js<'js>>(ctx: &Ctx<'js>, v: &Value<'js>, name: &str) -> JsResult<Option<Vec<T>>> {
    let inner: Value = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "options"))?.get(name)?;
    if inner.is_undefined() || inner.is_null() { Ok(None) } else { numbers(ctx, &inner).map(Some) }
}

impl FitHold {
    fn new<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self> {
        if v.is_undefined() || v.is_null() {
            return Ok(FitHold { some: false, weights: None, tolerance: 0.0, tries: 0, seed: 0, inliers: None });
        }
        Ok(FitHold {
            some: true,
            weights: opt_numbers(ctx, v, "weights")?,
            tolerance: opt_field(ctx, v, "tolerance")?,
            tries: opt_field(ctx, v, "tries")?,
            seed: opt_field(ctx, v, "seed")?,
            inliers: opt_numbers(ctx, v, "inliers")?,
        })
    }
    fn options(&mut self) -> Option<FitOptions<'_>> {
        if !self.some {
            return None;
        }
        Some(FitOptions { weights: self.weights.as_deref(), tolerance: self.tolerance, tries: self.tries, seed: self.seed, inliers: self.inliers.as_deref_mut() })
    }
    fn done<'js>(&self, v: &Value<'js>) -> JsResult<()> {
        if let Some(i) = &self.inliers {
            bytes_back(&v.as_object().expect("options").get::<_, Value>("inliers")?, i)?;
        }
        Ok(())
    }
}

struct AlignHold {
    some: bool,
    max_distance: f64,
    iterations: u32,
    normals: Option<Vec<Double3>>,
    inliers: Option<Vec<u8>>,
    error: f64,
}

impl AlignHold {
    fn new<'js>(ctx: &Ctx<'js>, v: &Value<'js>) -> JsResult<Self> {
        if v.is_undefined() || v.is_null() {
            return Ok(AlignHold { some: false, max_distance: 0.0, iterations: 0, normals: None, inliers: None, error: 0.0 });
        }
        let normals: Value = v.as_object().ok_or_else(|| Exception::throw_type(ctx, "options"))?.get("normals")?;
        Ok(AlignHold {
            some: true,
            max_distance: opt_field(ctx, v, "max_distance")?,
            iterations: opt_field(ctx, v, "iterations")?,
            normals: if normals.is_undefined() || normals.is_null() { None } else { Some(list(ctx, &normals, 3)?.0) },
            inliers: opt_numbers(ctx, v, "inliers")?,
            error: 0.0,
        })
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
            error: Some(&mut self.error),
        })
    }
    fn done<'js>(&self, v: &Value<'js>) -> JsResult<()> {
        if !self.some {
            return Ok(());
        }
        let o = v.as_object().expect("options");
        if let Some(i) = &self.inliers {
            bytes_back(&o.get::<_, Value>("inliers")?, i)?;
        }
        o.set("error", self.error)
    }
}

fn tree_of<'js>(ctx: &Ctx<'js>, trees: &Trees, v: &Value<'js>) -> JsResult<Rc<KdTree>> {
    let id: u32 = field(ctx, v, "id")?;
    match trees.borrow().trees.get(&id) {
        Some(t) => Ok(t.clone()),
        None => Err(fail_argument(ctx)),
    }
}
