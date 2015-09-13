#![allow(non_camel_case_types)]

extern crate libc;
use libc::{c_int, c_uint, c_char, c_uchar, c_void, c_double, int32_t, uint32_t, int16_t, uint16_t, int8_t};

pub const FREENECT_COUNTS_PER_G: 		c_int = 829;
pub const FREENECT_DEPTH_MM_MAX_VALUE: 	c_int = 10000;
pub const FREENECT_DEPTH_MM_NO_VALUE: 	c_int = 0;
pub const FREENECT_DEPTH_RAW_MAX_VALUE: c_int = 2048;
pub const FREENECT_DEPTH_RAW_NO_VALUE: 	c_int = 2047;

#[repr(C)] pub enum freenect_device_flags {
	FREENECT_DEVICE_MOTOR  = 0x01,
	FREENECT_DEVICE_CAMERA = 0x02,
	FREENECT_DEVICE_AUDIO  = 0x04,
}

#[repr(C)] pub struct freenect_device_attributes {
	pub next: *mut freenect_device_attributes,
	pub camera_serial: *const c_char,
}

#[repr(C)] pub enum freenect_resolution {
	FREENECT_RESOLUTION_LOW    = 0, // QVGA - 320x240
	FREENECT_RESOLUTION_MEDIUM = 1, // VGA  - 640x480
	FREENECT_RESOLUTION_HIGH   = 2, // SXGA - 1280x1024
	FREENECT_RESOLUTION_DUMMY  = 2147483647, // Dummy value to force enum to be 32 bits wide
}

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

#[repr(C)] pub enum freenect_depth_format {
	FREENECT_DEPTH_11BIT        = 0, // 11 bit depth information in one uint16_t/pixel
	FREENECT_DEPTH_10BIT        = 1, // 10 bit depth information in one uint16_t/pixel
	FREENECT_DEPTH_11BIT_PACKED = 2, // 11 bit packed depth information
	FREENECT_DEPTH_10BIT_PACKED = 3, // 10 bit packed depth information
	FREENECT_DEPTH_REGISTERED   = 4, // processed depth data in mm, aligned to 640x480 RGB
	FREENECT_DEPTH_MM           = 5, // depth to each pixel in mm, but left unaligned to RGB image
	FREENECT_DEPTH_DUMMY        = 2147483647, // Dummy value to force enum to be 32 bits wide
}

#[repr(C)] pub enum freenect_flag {
	// values written to the CMOS register
	FREENECT_AUTO_EXPOSURE      = 1 << 14,
	FREENECT_AUTO_WHITE_BALANCE = 1 << 1,
	FREENECT_RAW_COLOR          = 1 << 4,
	// registers to be written with 0 or 1
	FREENECT_MIRROR_DEPTH       = 0x0017,
	FREENECT_MIRROR_VIDEO       = 0x0047,
}

#[repr(C)] pub enum freenect_flag_value {
	FREENECT_OFF = 0,
	FREENECT_ON  = 1,
}

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

#[repr(C)] pub enum freenect_led_option {
	LED_OFF              = 0, // Turn LED off
	LED_GREEN            = 1, // Turn LED to Green
	LED_RED              = 2, // Turn LED to Red
	LED_YELLOW           = 3, // Turn LED to Yellow
	LED_BLINK_GREEN      = 4, // Make LED blink Green
	// 5 is same as 4, LED blink Green
	LED_BLINK_RED_YELLOW = 6, // Make LED blink Red/Yellow
}

#[repr(C)] pub enum freenect_tilt_status_code {
	TILT_STATUS_STOPPED = 0x00, // Tilt motor is stopped
	TILT_STATUS_LIMIT   = 0x01, // Tilt motor has reached movement limit
	TILT_STATUS_MOVING  = 0x04, // Tilt motor is currently moving to new position
}

#[repr(C)] pub struct freenect_raw_tilt_state {
	pub accelerometer_x: int16_t                  , // Raw accelerometer data for X-axis, see FREENECT_COUNTS_PER_G for conversion
	pub accelerometer_y: int16_t                  , // Raw accelerometer data for Y-axis, see FREENECT_COUNTS_PER_G for conversion
	pub accelerometer_z: int16_t                  , // Raw accelerometer data for Z-axis, see FREENECT_COUNTS_PER_G for conversion
	pub tilt_angle:      int8_t                   , // Raw tilt motor angle encoder information
	pub tilt_status: 	 freenect_tilt_status_code, // State of the tilt motor (stopped, moving, etc...)
}

pub enum freenect_context {}

pub enum freenect_device {}

pub enum freenect_usb_context {}

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

pub type freenect_log_cb   = extern fn(dev: *mut freenect_context, level: freenect_loglevel, msg: *const c_char);
pub type freenect_depth_cb = extern fn(dev: *mut freenect_device, depth: *mut c_void, timestamp: uint32_t);
pub type freenect_video_cb = extern fn(dev: *mut freenect_device, video: *mut c_void, timestamp: uint32_t);
pub type freenect_chunk_cb = extern fn(buffer: *mut c_void, pkt_data: *mut c_void, pkt_num: c_int, datalen: c_int, user_data: *mut c_void);

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
	pub fn freenect_set_depth_chunk_callback(dev: *mut freenect_device, cb: freenect_chunk_cb);
	pub fn freenect_set_video_chunk_callback(dev: *mut freenect_device, cb: freenect_chunk_cb);
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
	pub fn freenect_get_ir_brightness(dev: *mut freenect_device) -> c_int;
	pub fn freenect_set_ir_brightness(dev: *mut freenect_device, brightness: uint16_t) -> c_int;
	pub fn freenect_set_fw_address_nui(ctx: *mut freenect_context, fw_ptr: *mut c_uchar, num_bytes: c_uint);
	pub fn freenect_set_fw_address_k4w(ctx: *mut freenect_context, fw_ptr: *mut c_uchar, num_bytes: c_uint);
}
