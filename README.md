# libfreenect Rust Bindings [in development]
The `libfreenect-sys` crate provides declarations and linkage for the
`libfreenect` C library. Following the `*-sys` package conventions, the
`libfreenect-sys` crate does not define higher-level abstractions over the
native `libfreenect` library functions.

This crate currently exposes an interface compatible with libfreenect-0.2.
Notable missing features from 0.5 include audio support and chunk-based
processing callbacks.
