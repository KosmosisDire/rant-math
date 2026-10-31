//! Rant Math for Rust: every call of the C library as `rant_math::<name>` on the standard types of
//! `rant_types`. A call that can fail returns a `Result`, one that can miss an `Option`. A list's
//! wrong shape that a plain answer has no way to report, such as triangles not in threes, panics.

#![allow(clippy::too_many_arguments, clippy::missing_safety_doc)]

mod api;
mod ffi;
mod own;

use std::ffi::{CString, c_char, c_int};
use std::fmt;
use std::marker::PhantomData;

pub use api::*;
pub use own::*;
pub use rant_types;
use rant_types::*;

/// What a call that failed reports, the C result it came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MathError {
    /// A list too short, or a value out of range.
    Argument,
    /// No single answer: a singular matrix, points on one line.
    Degenerate,
    /// Never seen here, a binding always gives the room asked for.
    Space,
    /// An image or cloud laid out in a way this call cannot read.
    Format,
    /// Out of memory.
    Memory,
    /// A point the call cannot map, such as one behind the camera.
    Outside,
}

impl MathError {
    /// The C result this error came from.
    pub fn code(self) -> i32 {
        match self {
            MathError::Argument => -1,
            MathError::Degenerate => -2,
            MathError::Space => -3,
            MathError::Format => -4,
            MathError::Memory => -5,
            MathError::Outside => -6,
        }
    }
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = unsafe { std::ffi::CStr::from_ptr(ffi::rant_math_result_text(self.code())) };
        f.write_str(&text.to_string_lossy())
    }
}

impl std::error::Error for MathError {}

pub(crate) fn check(r: c_int) -> Result<(), MathError> {
    match r {
        0 => Ok(()),
        -1 => Err(MathError::Argument),
        -2 => Err(MathError::Degenerate),
        -3 => Err(MathError::Space),
        -4 => Err(MathError::Format),
        -5 => Err(MathError::Memory),
        _ => Err(MathError::Outside),
    }
}

// The answer to asking a list's size: too small is what the question expects.
pub(crate) fn room(r: c_int) -> Result<(), MathError> {
    if r == -3 { Ok(()) } else { check(r) }
}

pub(crate) fn len32(n: usize) -> u32 {
    u32::try_from(n).expect("a list of over 4 billion items")
}

pub(crate) fn same(a: usize, b: usize) -> Result<u32, MathError> {
    if a == b { Ok(len32(a)) } else { Err(MathError::Argument) }
}

pub(crate) fn cstring(s: &str) -> CString {
    CString::new(s).expect("a name holds no NUL")
}

pub(crate) fn mesh_raw(vertices: &[Double3], triangles: &[u32]) -> ffi::RawMeshView {
    assert!(triangles.len() % 3 == 0, "triangles come in threes");
    ffi::RawMeshView {
        vertices: vertices.as_ptr(),
        vertex_count: len32(vertices.len()),
        triangles: triangles.as_ptr(),
        triangle_count: len32(triangles.len() / 3),
    }
}

pub(crate) fn prism_raw(p: &Prism) -> ffi::RawPrismView {
    ffi::RawPrismView {
        pose: p.pose,
        base: ffi::RawPolygon2DView { points: p.base.points.as_ptr(), count: len32(p.base.points.len()) },
        height: p.height,
    }
}

pub(crate) fn prism_of(v: &ffi::RawPrismView) -> Prism {
    let points = unsafe { std::slice::from_raw_parts(v.base.points, v.base.count as usize) };
    Prism { pose: v.pose, base: Polygon2D { points: points.to_vec() }, height: v.height }
}

/// A dense matrix of doubles, rows one after another.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Matrix {
    pub rows: u32,
    pub cols: u32,
    pub data: Vec<f64>,
}

impl Matrix {
    /// A matrix of zeros.
    pub fn new(rows: u32, cols: u32) -> Matrix {
        Matrix { rows, cols, data: vec![0.0; rows as usize * cols as usize] }
    }
    /// The number at a row and column.
    pub fn at(&self, row: u32, col: u32) -> f64 {
        self.data[row as usize * self.cols as usize + col as usize]
    }
    pub(crate) fn raw_mut(&mut self) -> ffi::RawMatrixView {
        ffi::RawMatrixView { data: self.data.as_mut_ptr(), rows: self.rows, cols: self.cols }
    }
}

pub(crate) fn matrix_raw(m: &Matrix) -> Result<ffi::RawMatrixView, MathError> {
    if m.data.len() != m.rows as usize * m.cols as usize {
        return Err(MathError::Argument);
    }
    Ok(ffi::RawMatrixView { data: m.data.as_ptr() as *mut f64, rows: m.rows, cols: m.cols })
}

/// The pixels of an Image, or of any buffer laid out as one, read only and with no copy.
#[derive(Clone, Copy, Debug)]
pub struct ImageRef<'a> {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub format: ImageFormat,
    pub data: &'a [u8],
}

impl<'a> From<&'a Image> for ImageRef<'a> {
    fn from(i: &'a Image) -> ImageRef<'a> {
        ImageRef { width: i.width, height: i.height, stride: i.stride, format: i.format, data: &i.data }
    }
}

impl ImageRef<'_> {
    pub(crate) fn raw(&self) -> ffi::RawImageView {
        ffi::RawImageView {
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format as c_int,
            data: self.data.as_ptr() as *mut u8,
            size: self.data.len(),
        }
    }
}

/// The pixels of an Image, or of any buffer laid out as one, for a call that draws on them.
#[derive(Debug)]
pub struct ImageMut<'a> {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub format: ImageFormat,
    pub data: &'a mut [u8],
}

impl<'a> From<&'a mut Image> for ImageMut<'a> {
    fn from(i: &'a mut Image) -> ImageMut<'a> {
        ImageMut { width: i.width, height: i.height, stride: i.stride, format: i.format, data: &mut i.data }
    }
}

impl ImageMut<'_> {
    pub(crate) fn raw(&mut self) -> ffi::RawImageView {
        ffi::RawImageView {
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format as c_int,
            data: self.data.as_mut_ptr(),
            size: self.data.len(),
        }
    }
}

pub(crate) fn new_image(format: ImageFormat, width: u32, height: u32) -> Image {
    let size = unsafe { ffi::rant_image_bytes(format as c_int, width, height) };
    Image { width, height, stride: 0, format, data: vec![0; size] }
}

pub(crate) fn copy_image(part: &ffi::RawImageView) -> Result<Image, MathError> {
    let mut out = new_image(api::image_format_of(part.format), part.width, part.height);
    let mut to = ImageMut::from(&mut out).raw();
    check(unsafe { ffi::rant_image_convert(part, &mut to) })?;
    Ok(out)
}

pub(crate) fn scan_raw(s: &LaserScan) -> ffi::RawLaserScanView {
    ffi::RawLaserScanView {
        angle_min: s.angle_min as f64,
        angle_step: s.angle_step as f64,
        range_min: s.range_min as f64,
        range_max: s.range_max as f64,
        echoes: s.echoes,
        ranges: s.ranges.as_ptr(),
        range_count: len32(s.ranges.len()),
    }
}

pub(crate) fn detection2d_raw(d: &Detection2D) -> ffi::RawDetection2DView {
    ffi::RawDetection2DView { bounds: d.bounds, label: d.label.as_ptr() as *const c_char, label_len: d.label.len(), score: d.score as f64 }
}

pub(crate) fn detection3d_raw(d: &Detection3D) -> ffi::RawDetection3DView {
    ffi::RawDetection3DView { bounds: d.bounds, label: d.label.as_ptr() as *const c_char, label_len: d.label.len(), score: d.score as f64 }
}

pub(crate) fn field_of(f: &ffi::RawPointFieldView) -> PointField {
    let name = unsafe { std::slice::from_raw_parts(f.name as *const u8, f.name_len) };
    PointField {
        name: std::string::String::from_utf8_lossy(name).into_owned(),
        offset: f.offset,
        kind: api::scalar_kind_of(f.kind),
        count: f.count,
    }
}

pub(crate) fn cloud_like(v: &ffi::RawPointCloudView) -> PointCloud {
    let fields = unsafe { std::slice::from_raw_parts(v.fields, v.field_count as usize) };
    PointCloud { fields: fields.iter().map(field_of).collect(), point_step: v.point_step, ..PointCloud::default() }
}

// The C view of a cloud, its fields held beside it.
pub(crate) struct CloudRaw {
    pub raw: ffi::RawPointCloudView,
    _fields: Vec<ffi::RawPointFieldView>,
}

impl CloudRaw {
    fn with(cloud: &PointCloud, data: *mut u8) -> CloudRaw {
        let fields: Vec<ffi::RawPointFieldView> = cloud
            .fields
            .iter()
            .map(|f| ffi::RawPointFieldView {
                name: f.name.as_ptr() as *const c_char,
                name_len: f.name.len(),
                offset: f.offset,
                kind: f.kind as c_int,
                count: f.count,
            })
            .collect();
        let raw = ffi::RawPointCloudView {
            fields: fields.as_ptr(),
            field_count: len32(fields.len()),
            point_step: cloud.point_step,
            data,
            size: cloud.data.len(),
        };
        CloudRaw { raw, _fields: fields }
    }
    pub fn new(cloud: &PointCloud) -> CloudRaw {
        Self::with(cloud, cloud.data.as_ptr() as *mut u8)
    }
    pub fn new_mut(cloud: &mut PointCloud) -> CloudRaw {
        let data = cloud.data.as_mut_ptr();
        Self::with(cloud, data)
    }
}

// The C view of a grid, its fields held beside it.
pub(crate) struct GridRaw {
    pub raw: ffi::RawGridView,
    _fields: Vec<ffi::RawGridFieldView>,
}

impl GridRaw {
    pub fn new(grid: &Grid) -> GridRaw {
        let fields: Vec<ffi::RawGridFieldView> = grid
            .fields
            .iter()
            .map(|f| ffi::RawGridFieldView {
                name: f.name.as_ptr() as *const c_char,
                name_len: f.name.len(),
                kind: f.kind as c_int,
                data: f.data.as_ptr() as *mut u8,
                size: f.data.len(),
            })
            .collect();
        let raw = ffi::RawGridView {
            origin: grid.origin.pose,
            resolution: grid.resolution,
            cells_x: grid.cells_x,
            cells_y: grid.cells_y,
            cells_z: grid.cells_z,
            fields: fields.as_ptr(),
            field_count: len32(fields.len()),
        };
        GridRaw { raw, _fields: fields }
    }
    pub fn new_mut(grid: &mut Grid) -> GridRaw {
        // Each field's bytes are written through the pointer new takes from the same exclusive borrow.
        Self::new(grid)
    }
}

/// How a fit treats its points. All default, every point counts alike.
#[derive(Debug, Default)]
pub struct FitOptions<'a> {
    /// One per point: how many times its squared error counts.
    pub weights: Option<&'a [f64]>,
    /// Above 0, RANSAC: a point farther than this from a model is an outlier of it.
    pub tolerance: f64,
    /// The most models RANSAC tries, 0 for 1000.
    pub tries: u32,
    /// RANSAC's random draws: the same seed gives the same answer.
    pub seed: u64,
    /// One per point: gets 1 within tolerance of the answer, else 0.
    pub inliers: Option<&'a mut [u8]>,
}

// A C options struct, or none, that a call points at.
pub(crate) struct OptionsRaw<T> {
    raw: Option<T>,
}

impl<T> OptionsRaw<T> {
    pub fn as_ptr(&mut self) -> *const T {
        self.raw.as_ref().map_or(std::ptr::null(), |x| x as *const T)
    }
}

pub(crate) fn fit_raw(options: Option<&mut FitOptions<'_>>, points: usize) -> Result<OptionsRaw<ffi::RawFitOptions>, MathError> {
    let Some(o) = options else { return Ok(OptionsRaw { raw: None }) };
    if o.weights.is_some_and(|w| w.len() != points) || o.inliers.as_ref().is_some_and(|i| i.len() != points) {
        return Err(MathError::Argument);
    }
    Ok(OptionsRaw {
        raw: Some(ffi::RawFitOptions {
            weights: o.weights.map_or(std::ptr::null(), |w| w.as_ptr()),
            tolerance: o.tolerance,
            tries: o.tries,
            seed: o.seed,
            inliers: o.inliers.as_mut().map_or(std::ptr::null_mut(), |i| i.as_mut_ptr()),
        }),
    })
}

/// How points_align pairs points. All default, each pairs with its nearest.
#[derive(Debug, Default)]
pub struct AlignOptions<'a> {
    /// Above 0, a source point with no target point this near is left out.
    pub max_distance: f64,
    /// The most rounds, 0 for 50.
    pub iterations: u32,
    /// One per target point: each pair counts by its distance along the normal.
    pub normals: Option<&'a [Double3]>,
    /// One per source point: gets 1 when the answer pairs it, else 0.
    pub inliers: Option<&'a mut [u8]>,
    /// Gets the RMS distance of the pairs of the answer.
    pub error: Option<&'a mut f64>,
}

pub(crate) fn align_raw(options: Option<&mut AlignOptions<'_>>, source: usize, target: usize) -> Result<OptionsRaw<ffi::RawAlignOptions>, MathError> {
    let Some(o) = options else { return Ok(OptionsRaw { raw: None }) };
    if o.normals.is_some_and(|n| n.len() != target) || o.inliers.as_ref().is_some_and(|i| i.len() != source) {
        return Err(MathError::Argument);
    }
    Ok(OptionsRaw {
        raw: Some(ffi::RawAlignOptions {
            max_distance: o.max_distance,
            iterations: o.iterations,
            normals: o.normals.map_or(std::ptr::null(), |n| n.as_ptr()),
            inliers: o.inliers.as_mut().map_or(std::ptr::null_mut(), |i| i.as_mut_ptr()),
            error: o.error.as_mut().map_or(std::ptr::null_mut(), |e| &mut **e as *mut f64),
        }),
    })
}

/// A kd tree over a copy of some points, freed when dropped. Queries never change it, so threads
/// may share one.
#[derive(Debug)]
pub struct KdTree {
    pub(crate) raw: *mut ffi::RawKdTree,
}

unsafe impl Send for KdTree {}
unsafe impl Sync for KdTree {}

impl Drop for KdTree {
    fn drop(&mut self) {
        unsafe { ffi::rant_kdtree_destroy(self.raw) };
    }
}

/// Any shape behind one type, made by a shape_from_ call. It borrows any list it was made from.
#[derive(Clone, Copy)]
pub struct Shape<'a> {
    pub(crate) raw: ffi::RawShape,
    pub(crate) data: PhantomData<&'a ()>,
}

/// The same in 2D.
#[derive(Clone, Copy)]
pub struct Shape2D<'a> {
    pub(crate) raw: ffi::RawShape2D,
    pub(crate) data: PhantomData<&'a ()>,
}

#[cfg(feature = "nalgebra")]
mod nalgebra_conversions;
#[cfg(feature = "nalgebra")]
pub use nalgebra_conversions::*;
