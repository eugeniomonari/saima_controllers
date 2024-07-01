use cbindgen::{Builder, Language};
use std::env;


fn main() {
let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    let mut header = String::new();
    header.push_str("/* This is an auto-generated file made from optimization engine: ");
    header.push_str("https://crates.io/crates/optimization_engine */\n\n");
    header.push_str("#pragma once\n\n");

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::C)
        .with_header(&header)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("optimizer1_bindings.h");

    Builder::new()
        .with_crate(&crate_dir)
        .with_language(Language::Cxx)
        .with_header(&header)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("optimizer1_bindings.hpp");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=optimizer1_bindings.h");
    println!("cargo:rerun-if-changed=optimizer1_bindings.hpp");

}