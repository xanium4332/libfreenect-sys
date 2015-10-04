extern crate pkg_config;
extern crate cmake;

fn main() {
    // Try and find system libfreenect library. If not, attempt to build from source.
    let found = pkg_config::Config::new().atleast_version("0.5").find("libfreenect").is_ok();

    if found {
        return;
    } else {
        build_from_source();
    }
}

// Perform complete libfreenect build from source.
fn build_from_source() {
    let _ = cmake::Config::new("libfreenect")
        .define("BUILD_CPP", "OFF")
        .define("BUILD_C_SYNC", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_FAKENECT", "OFF")
        .define("BUILD_REDIST_PACKAGE", "ON")
        .register_dep("libusb-sys")
        .build();
}
