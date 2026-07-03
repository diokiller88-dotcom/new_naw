#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to custom_msgs__msg__MapData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    pub occupancy_array: Vec<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub esdf_array: Vec<f64>,

}



impl Default for MapData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MapData::default())
  }
}

impl rosidl_runtime_rs::Message for MapData {
  type RmwMsg = super::msg::rmw::MapData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_valid: msg.is_valid,
        width: msg.width,
        height: msg.height,
        res_x: msg.res_x,
        res_y: msg.res_y,
        origin_x: msg.origin_x,
        origin_y: msg.origin_y,
        occupancy_array: msg.occupancy_array.into(),
        esdf_array: msg.esdf_array.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_valid: msg.is_valid,
      width: msg.width,
      height: msg.height,
      res_x: msg.res_x,
      res_y: msg.res_y,
      origin_x: msg.origin_x,
      origin_y: msg.origin_y,
        occupancy_array: msg.occupancy_array.as_slice().into(),
        esdf_array: msg.esdf_array.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_valid: msg.is_valid,
      width: msg.width,
      height: msg.height,
      res_x: msg.res_x,
      res_y: msg.res_y,
      origin_x: msg.origin_x,
      origin_y: msg.origin_y,
      occupancy_array: msg.occupancy_array
          .into_iter()
          .collect(),
      esdf_array: msg.esdf_array
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to custom_msgs__msg__VehicleState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VehicleState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VehicleState::default())
  }
}

impl rosidl_runtime_rs::Message for VehicleState {
  type RmwMsg = super::msg::rmw::VehicleState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_valid: msg.is_valid,
        pos_x: msg.pos_x,
        pos_y: msg.pos_y,
        yaw: msg.yaw,
        vel_x: msg.vel_x,
        vel_y: msg.vel_y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_valid: msg.is_valid,
      pos_x: msg.pos_x,
      pos_y: msg.pos_y,
      yaw: msg.yaw,
      vel_x: msg.vel_x,
      vel_y: msg.vel_y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_valid: msg.is_valid,
      pos_x: msg.pos_x,
      pos_y: msg.pos_y,
      yaw: msg.yaw,
      vel_x: msg.vel_x,
      vel_y: msg.vel_y,
    }
  }
}


// Corresponds to custom_msgs__msg__Goal

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Goal::default())
  }
}

impl rosidl_runtime_rs::Message for Goal {
  type RmwMsg = super::msg::rmw::Goal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        target_x: msg.target_x,
        target_y: msg.target_y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      target_x: msg.target_x,
      target_y: msg.target_y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      target_x: msg.target_x,
      target_y: msg.target_y,
    }
  }
}


// Corresponds to custom_msgs__msg__Result
/// 标准消息头，包含时间戳 (stamp) 和坐标系 (frame_id)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Result::default())
  }
}

impl rosidl_runtime_rs::Message for Result {
  type RmwMsg = super::msg::rmw::Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_valid: msg.is_valid,
        res_pose_x: msg.res_pose_x,
        res_pose_y: msg.res_pose_y,
        res_vel_x: msg.res_vel_x,
        res_vel_y: msg.res_vel_y,
        yaw_diff: msg.yaw_diff,
        vyaw: msg.vyaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_valid: msg.is_valid,
      res_pose_x: msg.res_pose_x,
      res_pose_y: msg.res_pose_y,
      res_vel_x: msg.res_vel_x,
      res_vel_y: msg.res_vel_y,
      yaw_diff: msg.yaw_diff,
      vyaw: msg.vyaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_valid: msg.is_valid,
      res_pose_x: msg.res_pose_x,
      res_pose_y: msg.res_pose_y,
      res_vel_x: msg.res_vel_x,
      res_vel_y: msg.res_vel_y,
      yaw_diff: msg.yaw_diff,
      vyaw: msg.vyaw,
    }
  }
}


// Corresponds to custom_msgs__msg__ChassisInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChassisInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ChassisInfo::default())
  }
}

impl rosidl_runtime_rs::Message for ChassisInfo {
  type RmwMsg = super::msg::rmw::ChassisInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_valid: msg.is_valid,
        trigger_relocation: msg.trigger_relocation,
        trigger_target: msg.trigger_target,
        speed: msg.speed,
        target_x: msg.target_x,
        target_y: msg.target_y,
        gimbal_yaw: msg.gimbal_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_valid: msg.is_valid,
      trigger_relocation: msg.trigger_relocation,
      trigger_target: msg.trigger_target,
      speed: msg.speed,
      target_x: msg.target_x,
      target_y: msg.target_y,
      gimbal_yaw: msg.gimbal_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_valid: msg.is_valid,
      trigger_relocation: msg.trigger_relocation,
      trigger_target: msg.trigger_target,
      speed: msg.speed,
      target_x: msg.target_x,
      target_y: msg.target_y,
      gimbal_yaw: msg.gimbal_yaw,
    }
  }
}


