fn main() {
    if std::env::var("CARGO_CFG_TARGET_ARCH") == Ok("arm".to_string()) {
        println!("cargo:rustc-link-arg-examples=--nmagic");
        println!("cargo:rustc-link-arg-examples=-Tlink.x");
        println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
    }
}
