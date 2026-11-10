//! The library's table of itself: every call with what it takes and gives, and each type's methods and
//! operators, read from the C library so every binding sees one list.

use std::ffi::{CStr, c_char, c_void};

use crate::ffi;

/// One thing a call takes or gives: `kind` is how a binding passes it, `type_name` a standard type, a C
/// number type or empty.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Arg {
    pub name: &'static str,
    pub kind: &'static str,
    pub type_name: &'static str,
}

/// A call by its name without `rant_`. `errors` is none, throw, optional or bool.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Function {
    pub name: &'static str,
    pub doc: &'static str,
    pub inputs: Vec<Arg>,
    pub outputs: Vec<Arg>,
    pub errors: &'static str,
}

/// A call as a method of `type_name`. `self_index` is the input that is the value, none for a static.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Method {
    pub type_name: &'static str,
    pub name: &'static str,
    pub function: usize,
    pub self_index: Option<usize>,
    pub field: &'static str,
}

/// `left op right`, as `Pose * Double3`: `op` is + - * / or neg.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Operator {
    pub op: &'static str,
    pub left: &'static str,
    pub right: &'static str,
    pub function: usize,
    pub how: &'static str,
}

#[repr(C)]
struct RawArg {
    name: *const c_char,
    kind: *const c_char,
    type_name: *const c_char,
}

#[repr(C)]
struct RawFunction {
    name: *const c_char,
    doc: *const c_char,
    inputs: *const RawArg,
    input_count: u32,
    outputs: *const RawArg,
    output_count: u32,
    errors: *const c_char,
}

#[repr(C)]
struct RawMethod {
    type_name: *const c_char,
    name: *const c_char,
    function: u32,
    self_index: u32,
    field: *const c_char,
}

#[repr(C)]
struct RawOperator {
    op: *const c_char,
    left: *const c_char,
    right: *const c_char,
    function: u32,
    how: *const c_char,
}

// The table's strings are the library's own constants, so they live as long as the program.
fn text(p: *const c_char) -> &'static str {
    unsafe { CStr::from_ptr(p) }.to_str().unwrap_or("")
}

fn args(p: *const RawArg, n: u32) -> Vec<Arg> {
    (0..n as usize)
        .map(|i| {
            let a = unsafe { &*p.add(i) };
            Arg { name: text(a.name), kind: text(a.kind), type_name: text(a.type_name) }
        })
        .collect()
}

fn rows<R, T>(count: u32, at: unsafe extern "C" fn(u32) -> *const c_void, read: impl Fn(&R) -> T) -> Vec<T> {
    (0..count).map(|i| read(unsafe { &*(at(i) as *const R) })).collect()
}

/// Every call of the library, in the order of its headers.
pub fn functions() -> Vec<Function> {
    rows(unsafe { ffi::rant_math_function_count() }, ffi::rant_math_function_at, |f: &RawFunction| Function {
        name: text(f.name),
        doc: text(f.doc),
        inputs: args(f.inputs, f.input_count),
        outputs: args(f.outputs, f.output_count),
        errors: text(f.errors),
    })
}

/// Every method of a standard type or of one of math's own.
pub fn methods() -> Vec<Method> {
    rows(unsafe { ffi::rant_math_method_count() }, ffi::rant_math_method_at, |m: &RawMethod| Method {
        type_name: text(m.type_name),
        name: text(m.name),
        function: m.function as usize,
        self_index: (m.self_index != u32::MAX).then_some(m.self_index as usize),
        field: text(m.field),
    })
}

/// Every operator, each the call it stands for.
pub fn operators() -> Vec<Operator> {
    rows(unsafe { ffi::rant_math_operator_count() }, ffi::rant_math_operator_at, |o: &RawOperator| Operator {
        op: text(o.op),
        left: text(o.left),
        right: text(o.right),
        function: o.function as usize,
        how: text(o.how),
    })
}
