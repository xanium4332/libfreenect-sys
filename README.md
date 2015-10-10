# libfreenect Rust Bindings [in development]
The `libfreenect-sys` crate provides declarations and linkage for the
`libfreenect` C library. Following the `*-sys` package conventions, the
`libfreenect-sys` crate does not define higher-level abstractions over the
native `libfreenect` library functions.

This crate currently exposes an interface compatible with libfreenect-0.5.
However, it is currently missing declarations for audio- and registration-based
functions.

## Dependencies
The system-wide `libfreenect` library is detected using `pkg_config`. If the
library is present and version >= 0.5.2, this crate will merely link to the
system library.

If no system library is found, this crate will compile and statically link
against the in-tree included `libfreenect`. In this case a dependency on the
`libusb-sys` crate is required in order to provide a working `libusb`
implementation.
