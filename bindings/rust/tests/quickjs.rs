//! The QuickJS glue: the TypeScript binding's own replay of vectors.json, run through it.
#![cfg(feature = "quickjs")]

use rquickjs::{CatchResultExt, Context, Function, Module, Object, Runtime};

#[test]
fn every_case_through_quickjs() {
    let rt = Runtime::new().unwrap();
    let ctx = Context::full(&rt).unwrap();
    ctx.with(|ctx| {
        let math = rant_math::quickjs::install(&ctx).catch(&ctx).unwrap();
        let (module, done) = Module::declare(ctx.clone(), "replay.js", include_str!("../../../tests/bindings/web_replay.js")).unwrap().eval().catch(&ctx).unwrap();
        done.finish::<()>().catch(&ctx).unwrap();
        let exports: Object = module.namespace().unwrap();
        let run: Function = exports.get("run").unwrap();
        let text = include_str!("../../../tests/bindings/vectors.json");
        let parse: Function = ctx.eval("JSON.parse").unwrap();
        let vectors: rquickjs::Value = parse.call((text,)).unwrap();
        let say = Function::new(ctx.clone(), |line: String| println!("{line}")).unwrap();
        let failures: i32 = run.call((math, vectors, say)).catch(&ctx).unwrap();
        assert_eq!(failures, 0);
    });
}

#[test]
fn names_read_as_a_module() {
    let rt = Runtime::new().unwrap();
    let ctx = Context::full(&rt).unwrap();
    ctx.with(|ctx| {
        let (module, done) = Module::declare(ctx.clone(), "names.js", rant_math::quickjs::NAMES).unwrap().eval().catch(&ctx).unwrap();
        done.finish::<()>().catch(&ctx).unwrap();
        let names: Vec<Object> = module.namespace().unwrap().get("NAMES").unwrap();
        assert!(names.len() > 400);
        let math = rant_math::quickjs::install(&ctx).catch(&ctx).unwrap();
        for n in &names {
            let name: String = n.get("name").unwrap();
            assert!(math.get::<_, rquickjs::Value>(name.as_str()).unwrap().is_function(), "{name} is listed and not there");
        }
    });
}
