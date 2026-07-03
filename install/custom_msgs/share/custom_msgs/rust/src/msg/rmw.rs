#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__MapData() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__MapData__init(msg: *mut MapData) -> bool;
    fn custom_msgs__msg__MapData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MapData>, size: usize) -> bool;
    fn custom_msgs__msg__MapData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MapData>);
    fn custom_msgs__msg__MapData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MapData>, out_seq: *mut rosidl_runtime_rs::Sequence<MapData>) -> bool;
}

// Corresponds to custom_msgs__msg__MapData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,

    /// 地图基础信息
    pub width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub res_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub res_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub origin_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub origin_y: f64,

    /// 核心地图数据 (一维展开存储)
    pub occupancy_array: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub esdf_array: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for MapData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__MapData__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__MapData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MapData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__MapData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__MapData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__MapData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MapData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MapData where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/MapData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__MapData() }
  }
}


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__VehicleState() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__VehicleState__init(msg: *mut VehicleState) -> bool;
    fn custom_msgs__msg__VehicleState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VehicleState>, size: usize) -> bool;
    fn custom_msgs__msg__VehicleState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VehicleState>);
    fn custom_msgs__msg__VehicleState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VehicleState>, out_seq: *mut rosidl_runtime_rs::Sequence<VehicleState>) -> bool;
}

// Corresponds to custom_msgs__msg__VehicleState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,

    /// 当前车体物理坐标 (米) - 地图坐标系
    pub pos_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_y: f64,

    /// 当前底盘在地图坐标系下的绝对航向角 (弧度)
    pub yaw: f64,

    /// 当前车体线速度 (米/秒) - 地图坐标系
    pub vel_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vel_y: f64,

}



impl Default for VehicleState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__VehicleState__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__VehicleState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VehicleState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__VehicleState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VehicleState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VehicleState where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/VehicleState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__VehicleState() }
  }
}


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Goal() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__Goal__init(msg: *mut Goal) -> bool;
    fn custom_msgs__msg__Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Goal>, size: usize) -> bool;
    fn custom_msgs__msg__Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Goal>);
    fn custom_msgs__msg__Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<Goal>) -> bool;
}

// Corresponds to custom_msgs__msg__Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Goal {
    /// 该消息包无用
    /// 终点物理坐标 (米)
    pub target_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_y: f64,

}



impl Default for Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__Goal__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Goal where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Goal() }
  }
}


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Result() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__Result__init(msg: *mut Result) -> bool;
    fn custom_msgs__msg__Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Result>, size: usize) -> bool;
    fn custom_msgs__msg__Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Result>);
    fn custom_msgs__msg__Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Result>) -> bool;
}

// Corresponds to custom_msgs__msg__Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 标准消息头，包含时间戳 (stamp) 和坐标系 (frame_id)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,

    /// 轨迹参考点及速度
    pub res_pose_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub res_pose_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub res_vel_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub res_vel_y: f64,

    /// 角度偏差与偏航角速度 (弧度制)
    pub yaw_diff: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vyaw: f64,

}



impl Default for Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__Result__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Result where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__Result() }
  }
}


#[link(name = "custom_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__ChassisInfo() -> *const std::ffi::c_void;
}

#[link(name = "custom_msgs__rosidl_generator_c")]
extern "C" {
    fn custom_msgs__msg__ChassisInfo__init(msg: *mut ChassisInfo) -> bool;
    fn custom_msgs__msg__ChassisInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChassisInfo>, size: usize) -> bool;
    fn custom_msgs__msg__ChassisInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChassisInfo>);
    fn custom_msgs__msg__ChassisInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChassisInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<ChassisInfo>) -> bool;
}

// Corresponds to custom_msgs__msg__ChassisInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChassisInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub trigger_relocation: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub trigger_target: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gimbal_yaw: f64,

}



impl Default for ChassisInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_msgs__msg__ChassisInfo__init(&mut msg as *mut _) {
        panic!("Call to custom_msgs__msg__ChassisInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChassisInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__ChassisInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__ChassisInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_msgs__msg__ChassisInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChassisInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChassisInfo where Self: Sized {
  const TYPE_NAME: &'static str = "custom_msgs/msg/ChassisInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_msgs__msg__ChassisInfo() }
  }
}


