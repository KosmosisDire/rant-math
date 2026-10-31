// Compiles the packed C library into the crate, so a Rust program needs no DLL.

use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let csrc = crate_dir.join("csrc");
    if !csrc.join("rant_math.c").exists() {
        panic!("rant-math: csrc/rant_math.c is missing. A release tag holds it, and in a checkout copy rant_math.h and rant_math.c from the CMake build's dist folder");
    }
    println!("cargo:rerun-if-changed={}", csrc.display());
    cc::Build::new().file(csrc.join("rant_math.c")).include(&csrc).warnings(false).compile("rant_math");
    if std::env::var("CARGO_CFG_UNIX").is_ok() {
        println!("cargo:rustc-link-lib=m");
    }
}
