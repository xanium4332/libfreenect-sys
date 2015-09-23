#![allow(non_camel_case_types)]

extern crate libc;
use libc::{
    c_void,
    c_char,
    c_int,
    int8_t,
    int16_t,
    int32_t,
    uint32_t,
    c_double,
};

pub const FREENECT_COUNTS_PER_G: c_int = 819; // Ticks per G for accelerometer as set per http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf

/// Maximum value that a uint16_t pixel will take on in the buffer of any of the FREENECT_DEPTH_MM or FREENECT_DEPTH_REGISTERED frame callbacks
pub const FREENECT_DEPTH_MM_MAX_VALUE: c_int = 10000;
/// Value indicating that this pixel has no data, when using FREENECT_DEPTH_MM or FREENECT_DEPTH_REGISTERED depth modes
pub const FREENECT_DEPTH_MM_NO_VALUE: c_int = 0;
/// Maximum value that a uint16_t pixel will take on in the buffer of any of the FREENECT_DEPTH_11BIT, FREENECT_DEPTH_10BIT, FREENECT_DEPTH_11BIT_PACKED, or FREENECT_DEPTH_10BIT_PACKED frame callbacks
pub const FREENECT_DEPTH_RAW_MAX_VALUE: c_int = 2048;
/// Value indicating that this pixel has no data, when using FREENECT_DEPTH_11BIT, FREENECT_DEPTH_10BIT, FREENECT_DEPTH_11BIT_PACKED, or FREENECT_DEPTH_10BIT_PACKED
pub const FREENECT_DEPTH_RAW_NO_VALUE: c_int = 2047;

/// Flags representing devices to open when freenect_open_device() is called.
/// In particular, this allows libfreenect to grab only a subset of the devices
/// in the Kinect, so you could (for instance) use libfreenect to handle audio
/// and motor support while letting OpenNI have access to the cameras.
/// If a device is not supported on a particular platform, its flag will be ignored.
#[repr(C)] pub enum freenect_device_flags {
    FREENECT_DEVICE_MOTOR  = 0x01,
    FREENECT_DEVICE_CAMERA = 0x02,
    FREENECT_DEVICE_AUDIO  = 0x04,
}

/// A struct used in enumeration to give access to serial numbers, so you can
/// open a particular device by serial rather than depending on index.  This
/// is most useful if you have more than one Kinect.
#[repr(C)] pub struct freenect_device_attributes {
    pub next: *mut freenect_device_attributes, // Next device in the linked list
    pub camera_serial: *const c_char, // Serial number of this device's camera subdevice
}

/// Enumeration of available resolutions.
/// Not all available resolutions are actually supported for all video formats.
/// Frame modes may not perfectly match resolutions.  For instance,
/// FREENECT_RESOLUTION_MEDIUM is 640x488 for the IR camera.
#[repr(C)] pub enum freenect_resolution {
    FREENECT_RESOLUTION_LOW    = 0, // QVGA - 320x240
    FREENECT_RESOLUTION_MEDIUM = 1, // VGA  - 640x480
    FREENECT_RESOLUTION_HIGH   = 2, // SXGA - 1280x1024
    FREENECT_RESOLUTION_DUMMY  = 2147483647, // Dummy value to force enum to be 32 bits wide
}

/// Enumeration of video frame information states.
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
#[repr(C)] pub enum freenect_video_format {
    FREENECT_VIDEO_RGB             = 0, // Decompressed RGB mode (demosaicing done by libfreenect)
    FREENECT_VIDEO_BAYER           = 1, // Bayer compressed mode (raw information from camera)
    FREENECT_VIDEO_IR_8BIT         = 2, // 8-bit IR mode
    FREENECT_VIDEO_IR_10BIT        = 3, // 10-bit IR mode
    FREENECT_VIDEO_IR_10BIT_PACKED = 4, // 10-bit packed IR mode
    FREENECT_VIDEO_YUV_RGB         = 5, // YUV RGB mode
    FREENECT_VIDEO_YUV_RAW         = 6, // YUV Raw mode
    FREENECT_VIDEO_DUMMY           = 2147483647, // Dummy value to force enum to be 32 bits wide
}

/// Enumeration of depth frame states
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
#[repr(C)] pub enum freenect_depth_format {
    FREENECT_DEPTH_11BIT        = 0, // 11 bit depth information in one uint16_t/pixel
    FREENECT_DEPTH_10BIT        = 1, // 10 bit depth information in one uint16_t/pixel
    FREENECT_DEPTH_11BIT_PACKED = 2, // 11 bit packed depth information
    FREENECT_DEPTH_10BIT_PACKED = 3, // 10 bit packed depth information
    FREENECT_DEPTH_REGISTERED   = 4, // processed depth data in mm, aligned to 640x480 RGB
    FREENECT_DEPTH_MM           = 5, // depth to each pixel in mm, but left unaligned to RGB image
    FREENECT_DEPTH_DUMMY        = 2147483647, // Dummy value to force enum to be 32 bits wide
}

/// Enumeration of flags to toggle features with freenect_set_flag()
#[repr(C)] pub enum freenect_flag {
    // values written to the CMOS register
    FREENECT_AUTO_EXPOSURE      = 1 << 14,
    FREENECT_AUTO_WHITE_BALANCE = 1 << 1,
    FREENECT_RAW_COLOR          = 1 << 4,
    // registers to be written with 0 or 1
    FREENECT_MIRROR_DEPTH       = 0x0017,
    FREENECT_MIRROR_VIDEO       = 0x0047,
}

/// Possible values for setting each `freenect_flag`
#[repr(C)] pub enum freenect_flag_value {
    FREENECT_OFF = 0,
    FREENECT_ON  = 1,
}

/// Structure to give information about the width, height, bitrate,
/// framerate, and buffer size of a frame in a particular mode, as
/// well as the total number of bytes needed to hold a single frame.
#[repr(C)] pub struct freenect_frame_mode {
    pub reserved: uint32_t,              // unique ID used internally.  The meaning of values may change without notice.  Don't touch or depend on the contents of this field.  We mean it.
    pub resolution: freenect_resolution, // Resolution this freenect_frame_mode describes, should you want to find it again with freenect_find_*_frame_mode().
    pub dummy: int32_t,                  // The video or depth format that this freenect_frame_mode describes.  The caller should know which of video_format or depth_format to use, since they called freenect_get_*_frame_mode()
    pub bytes: int32_t,                  // Total buffer size in bytes to hold a single frame of data.  Should be equivalent to width * height * (data_bits_per_pixel+padding_bits_per_pixel) / 8
    pub width: int16_t,                  // Width of the frame, in pixels
    pub height: int16_t,                 // Height of the frame, in pixels
    pub data_bits_per_pixel: int8_t,     // Number of bits of information needed for each pixel
    pub padding_bits_per_pixel: int8_t,  // Number of bits of padding for alignment used for each pixel
    pub framerate: int8_t,               // Approximate expected frame rate, in Hz
    pub is_valid: int8_t,                // If 0, this freenect_frame_mode is invalid and does not describe a supported mode.  Otherwise, the frame_mode is valid.
}

/// Enumeration of LED states
/// See http://openkinect.org/wiki/Protocol_Documentation#Setting_LED for more information.
#[repr(C)] pub enum freenect_led_option {
    LED_OFF              = 0, // Turn LED off
    LED_GREEN            = 1, // Turn LED to Green
    LED_RED              = 2, // Turn LED to Red
    LED_YELLOW           = 3, // Turn LED to Yellow
    LED_BLINK_GREEN      = 4, // Make LED blink Green
    // 5 is same as 4, LED blink Green
    LED_BLINK_RED_YELLOW = 6, // Make LED blink Red/Yellow
}

/// Enumeration of tilt motor status
#[repr(C)] pub enum freenect_tilt_status_code {
    TILT_STATUS_STOPPED = 0x00, // Tilt motor is stopped
    TILT_STATUS_LIMIT   = 0x01, // Tilt motor has reached movement limit
    TILT_STATUS_MOVING  = 0x04, // Tilt motor is currently moving to new position
}

/// Data from the tilt motor and accelerometer
#[repr(C)] pub struct freenect_raw_tilt_state {
    pub accelerometer_x: int16_t,					// Raw accelerometer data for X-axis, see FREENECT_COUNTS_PER_G for conversion
    pub accelerometer_y: int16_t,					// Raw accelerometer data for Y-axis, see FREENECT_COUNTS_PER_G for conversion
    pub accelerometer_z: int16_t,					// Raw accelerometer data for Z-axis, see FREENECT_COUNTS_PER_G for conversion
    pub tilt_angle:      int8_t, 					// Raw tilt motor angle encoder information
    pub tilt_status: 	 freenect_tilt_status_code, // State of the tilt motor (stopped, moving, etc...)
}

pub enum freenect_context {} // Holds information about the usb context.

pub enum freenect_device {} // Holds device information.

// usb backend specific section
pub enum freenect_usb_context {} // Holds libusb-1.0 context

/// Enumeration of message logging levels
#[repr(C)] pub enum freenect_loglevel {
    FREENECT_LOG_FATAL = 0,     // Log for crashing/non-recoverable errors
    FREENECT_LOG_ERROR,         // Log for major errors
    FREENECT_LOG_WARNING,       // Log for warning messages
    FREENECT_LOG_NOTICE,        // Log for important messages
    FREENECT_LOG_INFO,          // Log for normal messages
    FREENECT_LOG_DEBUG,         // Log for useful development messages
    FREENECT_LOG_SPEW,          // Log for slightly less useful messages
    FREENECT_LOG_FLOOD,         // Log EVERYTHING. May slow performance.
}

/// Typedef for logging callback functions
pub type freenect_log_cb   = extern fn(dev: *mut freenect_context, level: freenect_loglevel, msg: *const c_char);

/// Typedef for depth image received event callbacks
pub type freenect_depth_cb = extern fn(dev: *mut freenect_device, depth: *mut c_void, timestamp: uint32_t);
/// Typedef for video image received event callbacks
pub type freenect_video_cb = extern fn(dev: *mut freenect_device, video: *mut c_void, timestamp: uint32_t);

#[link(name = "freenect")]
extern "C" {
    pub fn freenect_init(ctx: *mut *mut freenect_context, usb_ctx: *mut freenect_usb_context) -> c_int;
    pub fn freenect_shutdown(ctx: *mut freenect_context) -> c_int;
    pub fn freenect_set_log_level(ctx: *mut freenect_context, level: freenect_loglevel);
    pub fn freenect_set_log_callback(ctx: *mut freenect_context, cb: freenect_log_cb);
    pub fn freenect_process_events(ctx: *mut freenect_context) -> c_int;
    pub fn freenect_process_events_timeout(ctx: *mut freenect_context, timeout: libc::timeval) -> c_int;
    pub fn freenect_num_devices(ctx: *mut freenect_context) -> c_int;
    pub fn freenect_list_device_attributes(ctx: *mut freenect_context, attribute_list: *mut *mut freenect_device_attributes) -> c_int;
    pub fn freenect_free_device_attributes(attribute_list: *mut freenect_device_attributes);
    pub fn freenect_supported_subdevices() -> c_int;
    pub fn freenect_select_subdevices(ctx: *mut freenect_context, subdevs: u32); // Not enum type as bitfield can have multiple choices
    pub fn freenect_enabled_subdevices(ctx: *mut freenect_context) -> freenect_device_flags;
    pub fn freenect_open_device(ctx: *mut freenect_context, dev: *mut *mut freenect_device, index: c_int) -> c_int;
    pub fn freenect_open_device_by_camera_serial(ctx: *mut freenect_context, dev: *mut *mut freenect_device, camera_serial: *const c_char) -> c_int;
    pub fn freenect_close_device(dev: *mut freenect_device) -> c_int;
    pub fn freenect_set_user(dev: *mut freenect_device, user: *mut c_void);
    pub fn freenect_get_user(dev: *mut freenect_device) -> *mut c_void;
    pub fn freenect_set_depth_callback(dev: *mut freenect_device, cb: freenect_depth_cb);
    pub fn freenect_set_video_callback(dev: *mut freenect_device, cb: freenect_video_cb);
    pub fn freenect_set_depth_buffer(dev: *mut freenect_device, buf: *mut c_void) -> c_int;
    pub fn freenect_set_video_buffer(dev: *mut freenect_device, buf: *mut c_void) -> c_int;
    pub fn freenect_start_depth(dev: *mut freenect_device) -> c_int;
    pub fn freenect_start_video(dev: *mut freenect_device) -> c_int;
    pub fn freenect_stop_depth(dev: *mut freenect_device) -> c_int;
    pub fn freenect_stop_video(dev: *mut freenect_device) -> c_int;
    pub fn freenect_update_tilt_state(dev: *mut freenect_device) -> c_int;
    pub fn freenect_get_tilt_state(dev: *mut freenect_device) -> *mut freenect_raw_tilt_state;
    pub fn freenect_get_tilt_degs(state: *mut freenect_raw_tilt_state) -> c_double;
    pub fn freenect_set_tilt_degs(dev: *mut freenect_device, angle: c_double) -> c_int;
    pub fn freenect_get_tilt_status(state: *mut freenect_raw_tilt_state) -> freenect_tilt_status_code;
    pub fn freenect_set_led(dev: *mut freenect_device, option: freenect_led_option) -> c_int;
    pub fn freenect_get_mks_accel(state: *mut freenect_raw_tilt_state, x: *mut c_double, y: *mut c_double, z: *mut c_double);
    pub fn freenect_get_video_mode_count() -> c_int;
    pub fn freenect_get_video_mode(mode_num: c_int) -> freenect_frame_mode;
    pub fn freenect_get_current_video_mode(dev: *mut freenect_device) -> freenect_frame_mode;
    pub fn freenect_find_video_mode(res: freenect_resolution, fmt: freenect_video_format) -> freenect_frame_mode;
    pub fn freenect_set_video_mode(dev: *mut freenect_device, mode: freenect_frame_mode) -> c_int;
    pub fn freenect_get_depth_mode_count() -> c_int;
    pub fn freenect_get_depth_mode(mode_num: c_int) -> freenect_frame_mode;
    pub fn freenect_get_current_depth_mode(dev: *mut freenect_device) -> freenect_frame_mode;
    pub fn freenect_find_depth_mode(res: freenect_resolution, fmt: freenect_depth_format) -> freenect_frame_mode;
    pub fn freenect_set_depth_mode(dev: *mut freenect_device, mode: freenect_frame_mode) -> c_int;
    pub fn freenect_set_flag(dev: *mut freenect_device, flag: freenect_flag, value: freenect_flag_value) -> c_int;
}
