/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <std_srvs/Trigger.h>

#include <mrs_modules_msgs/Bestpos.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/gps_conversions.h>

#include <std_msgs/Float64.h>

#include <geometry_msgs/QuaternionStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/GPSRAW.h>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

namespace mrs_uav_px4_api
{

/* class MrsUavPx4Api //{ */

class MrsUavPx4Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~MrsUavPx4Api(){};

  void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiStatus       getStatus();
  mrs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::HwApiActuatorCmd::ConstPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd::ConstPtr msg);

  void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool& request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  mrs_msgs::HwApiCapabilities _capabilities_;

  std::string _uav_name_;
  std::string _body_frame_name_;
  std::string _world_frame_name_;

  double _mavros_timeout_;

  bool _simulation_;

  double      _sim_rtk_utm_x_;
  double      _sim_rtk_utm_y_;
  std::string _sim_rtk_utm_zone_;
  double      _sim_rtk_amsl_;

  double _ref_sin_lat;
  double _ref_cos_lat;
  double _ref_lat;
  double _ref_lon;
  double _ref_utm_x;
  double _ref_utm_y;
  bool _ref_latlon_init = false;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong> sch_mavros_command_long_;
  mrs_lib::ServiceClientHandler<mavros_msgs::SetMode>     sch_mavros_mode_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_ground_truth_;
  void                                          callbackGroundTruth(const nav_msgs::Odometry::ConstPtr msg);

  mrs_lib::SubscribeHandler<mrs_modules_msgs::Bestpos> sh_rtk_;
  void                                                 callbackRTK(const mrs_modules_msgs::Bestpos::ConstPtr msg);

  mrs_lib::SubscribeHandler<mavros_msgs::State> sh_mavros_state_;

  void   timeoutMavrosState(const std::string& topic, const ros::Time& last_msg);
  double RCChannelToRange(const double& rc_value);
  void   callbackMavrosState(const mavros_msgs::State::ConstPtr msg);

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odometry_local_;
  void                                          callbackOdometryLocal(const nav_msgs::Odometry::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_mavros_gps_;
  void                                              callbackNavsatFix(const sensor_msgs::NavSatFix::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_mavros_distance_sensor_;
  void                                          callbackDistanceSensor(const sensor_msgs::Range::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_mavros_imu_;
  void                                        callbackImu(const sensor_msgs::Imu::ConstPtr msg);

  mrs_lib::SubscribeHandler<std_msgs::Float64> sh_mavros_magnetometer_heading_;
  void                                         callbackMagnetometer(const std_msgs::Float64::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::MagneticField> sh_mavros_magnetic_field_;
  void                                                  callbackMagneticField(const sensor_msgs::MagneticField::ConstPtr msg);

  mrs_lib::SubscribeHandler<mavros_msgs::RCIn> sh_mavros_rc_;
  void                                         callbackRC(const mavros_msgs::RCIn::ConstPtr msg);

  mrs_lib::SubscribeHandler<mavros_msgs::Altitude> sh_mavros_altitude_;
  void                                             callbackAltitude(const mavros_msgs::Altitude::ConstPtr msg);

  mrs_lib::SubscribeHandler<mavros_msgs::GPSRAW> sh_gps_status_raw_;
  void                                           callbackGpsStatusRaw(const mavros_msgs::GPSRAW::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_mavros_battery_;
  void                                                 callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mavros_msgs::AttitudeTarget>  ph_mavros_attitude_target_;
  mrs_lib::PublisherHandler<mavros_msgs::ActuatorControl> ph_mavros_actuator_control_;

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavPx4Api::initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  _uav_name_         = common_handlers->getUavName();
  _body_frame_name_  = common_handlers->getBodyFrameName();
  _world_frame_name_ = common_handlers->getWorldFrameName();

  _capabilities_.api_name = "Px4Api";

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  param_loader.loadParam("mavros_timeout", _mavros_timeout_);

  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("simulated_rtk/utm_x", _sim_rtk_utm_x_);
  param_loader.loadParam("simulated_rtk/utm_y", _sim_rtk_utm_y_);
  param_loader.loadParam("simulated_rtk/utm_zone", _sim_rtk_utm_zone_);
  param_loader.loadParam("simulated_rtk/amsl", _sim_rtk_amsl_);

  param_loader.loadParam("inputs/control_group", (bool&)_capabilities_.accepts_control_group_cmd);
  param_loader.loadParam("inputs/attitude_rate", (bool&)_capabilities_.accepts_attitude_rate_cmd);
  param_loader.loadParam("inputs/attitude", (bool&)_capabilities_.accepts_attitude_cmd);

  param_loader.loadParam("outputs/distance_sensor", (bool&)_capabilities_.produces_distance_sensor);
  param_loader.loadParam("outputs/gnss", (bool&)_capabilities_.produces_gnss);
  param_loader.loadParam("outputs/gnss_status", (bool&)_capabilities_.produces_gnss_status);
  param_loader.loadParam("outputs/rtk", (bool&)_capabilities_.produces_rtk);
  param_loader.loadParam("outputs/ground_truth", (bool&)_capabilities_.produces_ground_truth);
  param_loader.loadParam("outputs/imu", (bool&)_capabilities_.produces_imu);
  param_loader.loadParam("outputs/altitude", (bool&)_capabilities_.produces_altitude);
  param_loader.loadParam("outputs/magnetometer_heading", (bool&)_capabilities_.produces_magnetometer_heading);
  param_loader.loadParam("outputs/magnetic_field", (bool&)_capabilities_.produces_magnetic_field);
  param_loader.loadParam("outputs/rc_channels", (bool&)_capabilities_.produces_rc_channels);
  param_loader.loadParam("outputs/battery_state", (bool&)_capabilities_.produces_battery_state);
  param_loader.loadParam("outputs/position", (bool&)_capabilities_.produces_position);
  param_loader.loadParam("outputs/orientation", (bool&)_capabilities_.produces_orientation);
  param_loader.loadParam("outputs/velocity", (bool&)_capabilities_.produces_velocity);
  param_loader.loadParam("outputs/angular_velocity", (bool&)_capabilities_.produces_angular_velocity);
  param_loader.loadParam("outputs/odometry", (bool&)_capabilities_.produces_odometry);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavPx4Api]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sch_mavros_command_long_ = mrs_lib::ServiceClientHandler<mavros_msgs::CommandLong>(nh_, "mavros_cmd_out");
  sch_mavros_mode_         = mrs_lib::ServiceClientHandler<mavros_msgs::SetMode>(nh_, "mavros_set_mode_out");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsHwPx4Api";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  if (_simulation_) {
    sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "ground_truth_in", &MrsUavPx4Api::callbackGroundTruth, this);
  }

  if (!_simulation_) {
    sh_rtk_ = mrs_lib::SubscribeHandler<mrs_modules_msgs::Bestpos>(shopts, "rtk_in", &MrsUavPx4Api::callbackRTK, this);
  }

  sh_mavros_state_ = mrs_lib::SubscribeHandler<mavros_msgs::State>(shopts, "mavros_state_in", ros::Duration(0.05), &MrsUavPx4Api::timeoutMavrosState, this,
                                                                   &MrsUavPx4Api::callbackMavrosState, this);

  sh_mavros_odometry_local_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_local_position_in", &MrsUavPx4Api::callbackOdometryLocal, this);

  sh_mavros_gps_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "mavros_global_position_in", &MrsUavPx4Api::callbackNavsatFix, this);

  sh_mavros_distance_sensor_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "mavros_garmin_in", &MrsUavPx4Api::callbackDistanceSensor, this);

  sh_mavros_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "mavros_imu_in", &MrsUavPx4Api::callbackImu, this);

  sh_mavros_magnetometer_heading_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "mavros_magnetometer_in", &MrsUavPx4Api::callbackMagnetometer, this);

  sh_mavros_magnetic_field_ =
      mrs_lib::SubscribeHandler<sensor_msgs::MagneticField>(shopts, "mavros_magnetic_field_in", &MrsUavPx4Api::callbackMagneticField, this);

  sh_mavros_rc_ = mrs_lib::SubscribeHandler<mavros_msgs::RCIn>(shopts, "mavros_rc_in", &MrsUavPx4Api::callbackRC, this);

  sh_mavros_altitude_ = mrs_lib::SubscribeHandler<mavros_msgs::Altitude>(shopts, "mavros_altitude_in", &MrsUavPx4Api::callbackAltitude, this);

  sh_gps_status_raw_ = mrs_lib::SubscribeHandler<mavros_msgs::GPSRAW>(shopts, "mavros_gps_status_raw_in", &MrsUavPx4Api::callbackGpsStatusRaw, this);

  sh_mavros_battery_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "mavros_battery_in", &MrsUavPx4Api::callbackBattery, this);

  // | ----------------------- publishers ----------------------- |

  ph_mavros_attitude_target_  = mrs_lib::PublisherHandler<mavros_msgs::AttitudeTarget>(nh_, "mavros_attitude_setpoint_out", 1);
  ph_mavros_actuator_control_ = mrs_lib::PublisherHandler<mavros_msgs::ActuatorControl>(nh_, "mavros_actuator_control_out", 1);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavPx4Api]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

mrs_msgs::HwApiStatus MrsUavPx4Api::getStatus() {

  mrs_msgs::HwApiStatus status;

  status.stamp = ros::Time::now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::HwApiCapabilities MrsUavPx4Api::getCapabilities() {

  _capabilities_.stamp = ros::Time::now();

  return _capabilities_;
}

//}

/* callbackControlActuatorCmd() //{ */

bool MrsUavPx4Api::callbackActuatorCmd([[maybe_unused]] const mrs_msgs::HwApiActuatorCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavPx4Api::callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting control group cmd");

  if (!_capabilities_.accepts_control_group_cmd) {
    ROS_ERROR("[MrsUavPx4Api]: the control group input is not enabled in the config file");
    return false;
  }

  mavros_msgs::ActuatorControl msg_out;

  msg_out.header.frame_id = "base_link";
  msg_out.header.stamp    = msg->stamp;

  msg_out.controls[0] = msg->roll;
  msg_out.controls[1] = -msg->pitch;
  msg_out.controls[2] = -msg->yaw;
  msg_out.controls[3] = msg->throttle;

  ph_mavros_actuator_control_.publish(msg_out);

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting attitude rate cmd");

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: attitude rate input is not enabled in the config file");
    return false;
  }

  mavros_msgs::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp    = msg->stamp;

  attitude_target.body_rate = msg->body_rate;
  attitude_target.thrust    = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting attitude cmd");

  if (!_capabilities_.accepts_attitude_cmd) {
    ROS_ERROR_THROTTLE(1.0, "[MrsUavPx4Api]: attitude input is not enabled in the config file");
    return false;
  }

  mavros_msgs::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp    = msg->stamp;

  attitude_target.orientation.x = msg->orientation.x;
  attitude_target.orientation.y = msg->orientation.y;
  attitude_target.orientation.z = msg->orientation.z;
  attitude_target.orientation.w = msg->orientation.w;

  attitude_target.thrust = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE | attitude_target.IGNORE_ROLL_RATE | attitude_target.IGNORE_PITCH_RATE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgRateCmd([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgRateCmd([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting velocity+hdg rate cmd");

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUavPx4Api::callbackPositionCmd([[maybe_unused]] const mrs_msgs::HwApiPositionCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUavPx4Api::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::TrackerCommand::ConstPtr msg) {
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackArming([[maybe_unused]] const bool& request) {

  std::stringstream ss;

  mavros_msgs::CommandLong srv_out;

  // when REALWORLD AND ARM:=TRUE
  if (!_simulation_ && request) {

    ss << "can not arm by service when not in simulation! You should arm the drone by the RC controller only!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());

    return {srv_out.response.success, ss.str()};
  }

  srv_out.request.broadcast    = false;
  srv_out.request.command      = 400;  // the code for arming
  srv_out.request.confirmation = true;

  srv_out.request.param1 = request ? 1 : 0;      // arm or disarm?
  srv_out.request.param2 = request ? 0 : 21196;  // 21196 allows to disarm even in mid-flight
  srv_out.request.param3 = 0;
  srv_out.request.param4 = 0;
  srv_out.request.param5 = 0;
  srv_out.request.param6 = 0;
  srv_out.request.param7 = 0;

  ROS_INFO("[Px4Api]: calling for %s", request ? "arming" : "disarming");

  if (sch_mavros_command_long_.call(srv_out)) {

    if (srv_out.response.success) {

      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.result << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Px4Api]: " << ss.str());
  }

  return {srv_out.response.success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackOffboard(void) {

  mavros_msgs::SetMode srv;

  srv.request.base_mode   = 0;
  srv.request.custom_mode = "OFFBOARD";

  bool res = sch_mavros_mode_.call(srv);

  std::stringstream ss;

  if (!res) {

    ss << "Service call for offboard failed!";

    ROS_ERROR_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());
    return {false, ss.str()};

  } else {

    if (srv.response.mode_sent != 1) {

      ss << "service call for offboard failed, returned " << srv.response.mode_sent;

      ROS_WARN_THROTTLE(1.0, "[Px4Api]: %s", ss.str().c_str());

      return {false, ss.str()};

    } else {

      ss << "switched to offboard mode";

      return {true, ss.str()};
    }
  }
}

//}

// | ------------------- additional methods ------------------- |

/* timeoutMavrosState() //{ */

void MrsUavPx4Api::timeoutMavrosState([[maybe_unused]] const std::string& topic, const ros::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mavros_state_.hasMsg()) {
    return;
  }

  ros::Duration time = ros::Time::now() - last_msg;

  if (time.toSec() > _mavros_timeout_) {

    {
      std::scoped_lock lock(mutex_status_);

      connected_ = false;
      offboard_  = false;
      armed_     = false;
      mode_      = "";
    }

    ROS_WARN_THROTTLE(1.0, "[MrsUavPx4Api]: Have not received Mavros state for more than '%.3f s'", time.toSec());

  } else {

    ROS_WARN_THROTTLE(1.0, "[MrsUavPx4Api]: Not recieving Mavros state message for '%.3f s'! Setup the PixHawk SD card!!", time.toSec());
    ROS_WARN_THROTTLE(1.0, "[MrsUavPx4Api]: This could be also caused by the not being PixHawk booted properly due to, e.g., antispark connector jerkyness.");
    ROS_WARN_THROTTLE(1.0, "[MrsUavPx4Api]: The Mavros state should be supplied at 100 Hz to provided fast refresh rate on the state of the OFFBOARD mode.");
    ROS_WARN_THROTTLE(1.0, "[MrsUavPx4Api]: If missing, the UAV could be disarmed by safety routines while not knowing it has switched to the MANUAL mode.");
  }
}

//}

/* RCChannelToRange() //{ */

double MrsUavPx4Api::RCChannelToRange(const double& rc_value) {

  double tmp_0_to_1 = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));

  if (tmp_0_to_1 > 1.0) {
    tmp_0_to_1 = 1.0;
  } else if (tmp_0_to_1 < 0.0) {
    tmp_0_to_1 = 0.0;
  }

  return tmp_0_to_1;
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackMavrosState() */

void MrsUavPx4Api::callbackMavrosState(const mavros_msgs::State::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Mavros state");

  {
    std::scoped_lock lock(mutex_status_);

    offboard_  = msg->mode == "OFFBOARD";
    armed_     = msg->armed;
    connected_ = true;
    mode_      = msg->mode;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = ros::Time::now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);
}

//}

/* callbackOdometryLocal() //{ */

void MrsUavPx4Api::callbackOdometryLocal(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Mavros's local odometry");

  auto odom = msg;

  // | -------------------- publish position -------------------- |

  geometry_msgs::PointStamped position;

  position.header.stamp    = odom->header.stamp;
  position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
  position.point           = odom->pose.pose.position;

  double lat, lon, correct_x, correct_y;

  if (_capabilities_.produces_position) {
    
    if (_capabilities_.produces_gnss and _ref_latlon_init) {

      // The px4 Azimuthal Equidistant Projection of WGS84 is inconsistent with the UTM conversion is MRS system,
      // therefore, we convert it back to WGS84 frame and then convert correctly using mrs_lib.
      
      // BEGIN PX4 CODE
      const double x_rad = (double)odom->pose.pose.position.y / 6371000.0;
      const double y_rad = (double)odom->pose.pose.position.x / 6371000.0;
      const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

      if (fabs(c) > 0) {
        const double sin_c = sin(c);
        const double cos_c = cos(c);

        const double lat_rad = asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
        const double lon_rad = (_ref_lon + atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c - x_rad * _ref_sin_lat * sin_c));

        lat = RAD2DEG(lat_rad);
        lon = RAD2DEG(lon_rad);

      } else {
        lat = RAD2DEG(_ref_lat);
        lon = RAD2DEG(_ref_lon);
      }
      // END PX4 CODE

      mrs_lib::UTM(lat, lon, &correct_x, &correct_y);

      correct_x = correct_x - _ref_utm_x;
      correct_y = correct_y - _ref_utm_y;

      position.point.x = correct_x;
      position.point.y = correct_y;

      ROS_DEBUG("[MrsUavPx4Api]: position_px4_x: %f, position_px4_y: %f", position.point.x, position.point.y);
      ROS_DEBUG("[MrsUavPx4Api]: correct_mrs_x: %f, correct_mrs_y: %f", correct_x, correct_y);

    }

    common_handlers_->publishers.publishPosition(position);
  }

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {

    geometry_msgs::QuaternionStamped orientation;

    orientation.header.stamp    = odom->header.stamp;
    orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    orientation.quaternion      = odom->pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {

    geometry_msgs::Vector3Stamped velocity;

    velocity.header.stamp    = odom->header.stamp;
    velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    velocity.vector          = odom->twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom->header.stamp;
    angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    angular_velocity.vector          = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {

    if (_capabilities_.produces_gnss and _ref_latlon_init) {

      auto odom_new = *odom;
      odom_new.pose.pose.position.x = correct_x;
      odom_new.pose.pose.position.y = correct_y;
      common_handlers_->publishers.publishOdometry(odom_new);

    } else {

      common_handlers_->publishers.publishOdometry(*odom);
    }
  }
}

//}

/* callbackNavsatFix() //{ */

void MrsUavPx4Api::callbackNavsatFix(const sensor_msgs::NavSatFix::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting NavSat fix");

  if (_capabilities_.produces_gnss) {

    common_handlers_->publishers.publishGNSS(*msg);

    if (!_ref_latlon_init) {

      _ref_lat     = DEG2RAD(msg->latitude);
      _ref_lon     = DEG2RAD(msg->longitude);
      _ref_sin_lat = sin(_ref_lat);
      _ref_cos_lat = cos(_ref_lat);
      mrs_lib::UTM(msg->latitude, msg->longitude, &_ref_utm_x, &_ref_utm_y);
      _ref_latlon_init = true;
    }

  }
}

//}

/* callbackDistanceSensor() //{ */

void MrsUavPx4Api::callbackDistanceSensor(const sensor_msgs::Range::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting distnace sensor");

  if (_capabilities_.produces_distance_sensor) {

    common_handlers_->publishers.publishDistanceSensor(*msg);
  }
}

//}

/* callbackImu() //{ */

void MrsUavPx4Api::callbackImu(const sensor_msgs::Imu::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting IMU");

  if (_capabilities_.produces_imu) {

    sensor_msgs::Imu new_imu_msg = *msg;
    new_imu_msg.header.frame_id  = _uav_name_ + "/" + _body_frame_name_;

    common_handlers_->publishers.publishIMU(new_imu_msg);
  }
}

//}

/* callbackCompass() //{ */

void MrsUavPx4Api::callbackMagnetometer(const std_msgs::Float64::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting magnetometer heading");

  if (_capabilities_.produces_magnetometer_heading) {

    mrs_msgs::Float64Stamped mag_out;
    mag_out.header.stamp    = ros::Time::now();
    mag_out.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    mag_out.value           = msg->data;

    common_handlers_->publishers.publishMagnetometerHeading(mag_out);
  }
}

//}

/* callbackMagneticField() //{ */

void MrsUavPx4Api::callbackMagneticField(const sensor_msgs::MagneticField::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting magnetic field");

  if (_capabilities_.produces_magnetic_field) {

    common_handlers_->publishers.publishMagneticField(*msg);
  }
}

//}

/* callbackRC() //{ */

void MrsUavPx4Api::callbackRC(const mavros_msgs::RCIn::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting RC");

  if (_capabilities_.produces_rc_channels) {

    mrs_msgs::HwApiRcChannels rc_out;

    rc_out.stamp = msg->header.stamp;

    for (size_t i = 0; i < msg->channels.size(); i++) {
      rc_out.channels.push_back(RCChannelToRange(msg->channels[i]));
    }

    common_handlers_->publishers.publishRcChannels(rc_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUavPx4Api::callbackAltitude(const mavros_msgs::Altitude::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Altitude");

  if (_capabilities_.produces_altitude) {

    mrs_msgs::HwApiAltitude altitude_out;

    altitude_out.stamp = msg->header.stamp;
    altitude_out.amsl  = msg->amsl;

    common_handlers_->publishers.publishAltitude(altitude_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUavPx4Api::callbackGpsStatusRaw(const mavros_msgs::GPSRAW::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting Gps Status Raw");

  if (_capabilities_.produces_gnss_status) {

    mrs_msgs::GpsInfo gps_info_out;

    gps_info_out.stamp = msg->header.stamp;  // [GPS_FIX_TYPE] GPS fix type
    gps_info_out.fix_type = msg->fix_type;  // [GPS_FIX_TYPE] GPS fix type

    gps_info_out.lat                = double(msg->lat) / 10000000;  // [deg] Latitude (WGS84, EGM96 ellipsoid)
    gps_info_out.lon                = double(msg->lon) / 10000000;  // [deg] Longitude (WGS84, EGM96 ellipsoid)
    gps_info_out.alt                = float(msg->alt) / 1000;       // [m]  (MSL). Positive for up. Not WGS84 altitude.
    gps_info_out.eph                = msg->eph;                     // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.epv                = msg->epv;                     // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.vel                = float(msg->vel) / 100;        // [m/s] GPS ground speed. If unknown, set to: UINT16_MAX
    gps_info_out.cog                = float(msg->cog) / 100;        // [deg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees.
    gps_info_out.satellites_visible = msg->satellites_visible;      // Number of satellites visible. If unknown, set to 255

    gps_info_out.alt_ellipsoid = float(msg->alt_ellipsoid) / 1000;  // [m] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    gps_info_out.h_acc         = float(msg->h_acc) / 1000;          // [m] Position uncertainty. Positive for up.
    gps_info_out.v_acc         = float(msg->v_acc) / 1000;          // [m] Altitude uncertainty. Positive for up.
    gps_info_out.vel_acc       = float(msg->vel_acc) / 1000;        // [m/s] Speed uncertainty. Positive for up.
    gps_info_out.hdg_acc       = float(msg->hdg_acc) / 1000;        // [deg] Heading / track uncertainty
    gps_info_out.yaw           = float(msg->yaw) / 100;             // [deg] Yaw in earth frame from north.
    gps_info_out.dgps_num_sats = msg->dgps_numch;                   // Number of DGPS satellites
    gps_info_out.dgps_age      = float(msg->dgps_age) / 1000;       // [s] Age of DGPS info
    gps_info_out.baseline_dist = 0;                                 // [m] distance to the basestation, not supported by the GPSRAW message

    common_handlers_->publishers.publishGNSSStatus(gps_info_out);
  }
}

//}

/* callbackBattery() //{ */

void MrsUavPx4Api::callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting battery");

  if (_capabilities_.produces_battery_state) {

    common_handlers_->publishers.publishBatteryState(*msg);
  }
}

//}

/* callbackGroundTruth() //{ */

void MrsUavPx4Api::callbackGroundTruth(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting ground truth");

  auto odom = msg;

  // | ------------------ publish ground truth ------------------ |

  if (_capabilities_.produces_ground_truth) {

    nav_msgs::Odometry gt = *msg;

    // if frame_id is "/world", "world", "/map" or "map" gazebo reports velocitites in global world frame so we need to transform them to body frame
    if (msg->header.frame_id == "/world" || msg->header.frame_id == "world" || msg->header.frame_id == "/map" || msg->header.frame_id == "map") {

      ROS_INFO_ONCE("[MrsUavPx4Api]: transforming Gazebo ground truth velocities from world to body frame");

      Eigen::Matrix3d R = mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

      Eigen::Vector3d lin_vel_world(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
      Eigen::Vector3d lin_vel_body = R.inverse() * lin_vel_world;

      Eigen::Vector3d angular_vel_world(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
      Eigen::Vector3d angular_vel_body = R.inverse() * angular_vel_world;

      gt.twist.twist.linear.x = lin_vel_body[0];
      gt.twist.twist.linear.y = lin_vel_body[1];
      gt.twist.twist.linear.z = lin_vel_body[2];

      gt.twist.twist.angular.x = angular_vel_body[0];
      gt.twist.twist.angular.y = angular_vel_body[1];
      gt.twist.twist.angular.z = angular_vel_body[2];
    }

    common_handlers_->publishers.publishGroundTruth(gt);
  }

  if (_capabilities_.produces_rtk) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(msg->pose.pose.position.y + _sim_rtk_utm_y_, msg->pose.pose.position.x + _sim_rtk_utm_x_, _sim_rtk_utm_zone_, lat, lon);

    sensor_msgs::NavSatFix gnss;

    gnss.header.stamp = msg->header.stamp;

    gnss.latitude  = lat;
    gnss.longitude = lon;
    gnss.altitude  = msg->pose.pose.position.z + _sim_rtk_amsl_;

    mrs_msgs::RtkGps rtk;

    rtk.header.stamp    = msg->header.stamp;
    rtk.header.frame_id = "gps";

    rtk.gps.latitude      = lat;
    rtk.gps.longitude     = lon;
    rtk.gps.altitude      = msg->pose.pose.position.z + _sim_rtk_amsl_;
    rtk.gps.covariance[0] = std::pow(0.1, 2);
    rtk.gps.covariance[4] = std::pow(0.1, 2);
    rtk.gps.covariance[8] = std::pow(0.1, 2);

    rtk.fix_type.fix_type = rtk.fix_type.RTK_FIX;

    rtk.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

    common_handlers_->publishers.publishRTK(rtk);
  }
}

//}

/* callbackRTK() //{ */

void MrsUavPx4Api::callbackRTK(const mrs_modules_msgs::Bestpos::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsUavPx4Api]: getting rtk");

  mrs_msgs::RtkGps rtk_msg_out;

  rtk_msg_out.gps.latitude  = msg->latitude;
  rtk_msg_out.gps.longitude = msg->longitude;
  rtk_msg_out.gps.altitude  = msg->height;

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

  if (msg->position_type == "L1_INT") {
    rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FIX;

  } else if (msg->position_type == "L1_FLOAT") {
    rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FLOAT;

  } else if (msg->position_type == "PSRDIFF") {
    rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.DGPS;

  } else if (msg->position_type == "SINGLE") {
    rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.SPS;

  } else if (msg->position_type == "NONE") {
    rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.NO_FIX;
  }

  common_handlers_->publishers.publishRTK(rtk_msg_out);
}

//}

}  // namespace mrs_uav_px4_api

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_px4_api::MrsUavPx4Api, mrs_uav_hw_api::MrsUavHwApi)
