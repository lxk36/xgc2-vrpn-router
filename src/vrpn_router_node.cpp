#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDefaultMaxOutputRateHz = 50.0;

std::string normalizeNamespace(std::string value) {
  if (value.empty()) {
    return value;
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string joinTopic(const std::string& ns, const std::string& suffix) {
  const std::string normalized = normalizeNamespace(ns);
  if (normalized.empty() || normalized == "/") {
    return suffix.front() == '/' ? suffix : "/" + suffix;
  }
  if (suffix.empty()) {
    return normalized;
  }
  return normalized + (suffix.front() == '/' ? suffix : "/" + suffix);
}

std::string requiredString(const XmlRpc::XmlRpcValue& route, const char* key, int index) {
  if (!route.hasMember(key) || route[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("routes[" + std::to_string(index) + "]." + key + " must be a string");
  }
  return static_cast<std::string>(route[key]);
}

bool hasString(const XmlRpc::XmlRpcValue& value, const char* key) {
  return value.hasMember(key) && value[key].getType() == XmlRpc::XmlRpcValue::TypeString;
}

bool hasNumber(const XmlRpc::XmlRpcValue& value, const char* key) {
  return value.hasMember(key) &&
         (value[key].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
          value[key].getType() == XmlRpc::XmlRpcValue::TypeInt);
}

double toDouble(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("expected numeric XML-RPC value");
}

double optionalDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
  return hasNumber(value, key) ? toDouble(value[key]) : fallback;
}

bool optionalBool(const XmlRpc::XmlRpcValue& value, const char* key, bool fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::runtime_error(std::string(key) + " must be a boolean");
  }
  return static_cast<bool>(value[key]);
}

tf2::Vector3 readVector3(const XmlRpc::XmlRpcValue& value, const char* key, const tf2::Vector3& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& vector = value[key];
  if (vector.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map with x/y/z");
  }
  return tf2::Vector3(
      optionalDouble(vector, "x", fallback.x()),
      optionalDouble(vector, "y", fallback.y()),
      optionalDouble(vector, "z", fallback.z()));
}

tf2::Quaternion readQuaternion(const XmlRpc::XmlRpcValue& value, const char* key, const tf2::Quaternion& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& rotation = value[key];
  if (rotation.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map");
  }

  tf2::Quaternion q;
  if (hasNumber(rotation, "w") || hasNumber(rotation, "qx")) {
    q.setValue(
        optionalDouble(rotation, "x", optionalDouble(rotation, "qx", 0.0)),
        optionalDouble(rotation, "y", optionalDouble(rotation, "qy", 0.0)),
        optionalDouble(rotation, "z", optionalDouble(rotation, "qz", 0.0)),
        optionalDouble(rotation, "w", optionalDouble(rotation, "qw", 1.0)));
  } else {
    q.setRPY(
        optionalDouble(rotation, "roll", 0.0),
        optionalDouble(rotation, "pitch", 0.0),
        optionalDouble(rotation, "yaw", 0.0));
  }
  q.normalize();
  return q;
}

tf2::Transform readTransform(const XmlRpc::XmlRpcValue& value, const char* key) {
  tf2::Transform transform;
  transform.setIdentity();
  if (!value.hasMember(key)) {
    return transform;
  }

  const auto& se3 = value[key];
  if (se3.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map");
  }

  transform.setOrigin(readVector3(se3, "translation", tf2::Vector3(0.0, 0.0, 0.0)));
  transform.setRotation(readQuaternion(se3, "rotation", tf2::Quaternion(0.0, 0.0, 0.0, 1.0)));
  return transform;
}

tf2::Transform poseToTransform(const geometry_msgs::Pose& pose) {
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (q.length2() <= 1e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  }
  q.normalize();
  return tf2::Transform(
      q,
      tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Pose transformToPose(const tf2::Transform& transform) {
  geometry_msgs::Pose pose;
  const auto origin = transform.getOrigin();
  const auto rotation = transform.getRotation();
  pose.position.x = origin.x();
  pose.position.y = origin.y();
  pose.position.z = origin.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

double poseDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double quaternionAngleDeg(const geometry_msgs::Quaternion& a, const geometry_msgs::Quaternion& b) {
  tf2::Quaternion qa(a.x, a.y, a.z, a.w);
  tf2::Quaternion qb(b.x, b.y, b.z, b.w);
  if (qa.length2() <= 1e-12 || qb.length2() <= 1e-12) {
    return 0.0;
  }
  qa.normalize();
  qb.normalize();
  const double dot = std::abs(qa.dot(qb));
  const double clamped = std::max(-1.0, std::min(1.0, dot));
  return 2.0 * std::acos(clamped) * 180.0 / M_PI;
}

std::string toString(double value) {
  std::ostringstream oss;
  oss.precision(4);
  oss << std::fixed << value;
  return oss.str();
}

void addKv(diagnostic_msgs::DiagnosticStatus& status, const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

struct RouteConfig {
  std::string name;
  std::string tracker;
  std::string input_topic;
  std::string mavros_ns;
  std::string output_topic;
  std::string reference_topic;
  std::string output_frame_id;
  double max_output_rate_hz = kDefaultMaxOutputRateHz;
  double min_input_rate_hz = 30.0;
  double max_input_rate_hz = 150.0;
  double stale_timeout_s = 0.3;
  double stuck_timeout_s = 0.5;
  double stuck_position_epsilon_m = 0.001;
  double stuck_angle_epsilon_deg = 0.2;
  double max_translation_jump_m = 1.0;
  double max_rotation_jump_deg = 45.0;
  double max_reference_delta_m = 2.0;
  bool drop_on_jump = true;
  tf2::Transform tracker_to_body = tf2::Transform::getIdentity();
  tf2::Vector3 field_offset = tf2::Vector3(0.0, 0.0, 0.0);
};

class PoseRoute {
 public:
  PoseRoute(ros::NodeHandle& nh, RouteConfig config, int queue_size)
      : config_(std::move(config)),
        publish_interval_(config_.max_output_rate_hz > 0.0 ? 1.0 / config_.max_output_rate_hz : 0.0) {
    publisher_ = nh.advertise<geometry_msgs::PoseStamped>(config_.output_topic, queue_size);
    subscriber_ = nh.subscribe<geometry_msgs::PoseStamped>(
        config_.input_topic, queue_size, &PoseRoute::poseCallback, this);
    if (!config_.reference_topic.empty()) {
      reference_subscriber_ = nh.subscribe<geometry_msgs::PoseStamped>(
          config_.reference_topic, queue_size, &PoseRoute::referenceCallback, this);
    }

    ROS_INFO_STREAM("vrpn route " << config_.name << ": " << config_.input_topic << " -> "
                                  << config_.output_topic << ", max_output_rate="
                                  << config_.max_output_rate_hz << " Hz");
  }

  diagnostic_msgs::DiagnosticStatus diagnosticStatus(const ros::Time& now) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "vrpn_router/" + config_.name;
    status.hardware_id = config_.tracker;
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";

    const double input_rate = estimateInputRate();
    const double age = last_receive_time_.isZero() ? -1.0 : (now - last_receive_time_).toSec();
    const double reference_delta = has_reference_pose_ && has_last_output_
                                       ? poseDistance(last_output_pose_.pose, reference_pose_.pose)
                                       : -1.0;

    std::vector<std::string> problems;
    if (last_receive_time_.isZero() || age > config_.stale_timeout_s) {
      problems.push_back("vrpn_timeout");
    }
    if (input_rate > 0.0 && input_rate < config_.min_input_rate_hz) {
      problems.push_back("input_rate_low");
    }
    if (input_rate > config_.max_input_rate_hz) {
      problems.push_back("input_rate_high");
    }
    if (stuck_) {
      problems.push_back("vrpn_stuck");
    }
    if (last_jump_detected_) {
      problems.push_back("vrpn_jump");
    }
    if (reference_delta >= 0.0 && reference_delta > config_.max_reference_delta_m) {
      problems.push_back("reference_delta_high");
    }

    if (!problems.empty()) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = problems.front();
      for (size_t i = 1; i < problems.size(); ++i) {
        status.message += "," + problems[i];
      }
    }

    addKv(status, "input_topic", config_.input_topic);
    addKv(status, "output_topic", config_.output_topic);
    addKv(status, "input_rate_hz", toString(input_rate));
    addKv(status, "published_count", std::to_string(published_count_));
    addKv(status, "received_count", std::to_string(received_count_));
    addKv(status, "dropped_by_rate_count", std::to_string(dropped_by_rate_count_));
    addKv(status, "dropped_by_jump_count", std::to_string(dropped_by_jump_count_));
    addKv(status, "age_s", age < 0.0 ? "none" : toString(age));
    addKv(status, "last_jump_translation_m", toString(last_jump_translation_m_));
    addKv(status, "last_jump_rotation_deg", toString(last_jump_rotation_deg_));
    addKv(status, "reference_delta_m", reference_delta < 0.0 ? "none" : toString(reference_delta));
    return status;
  }

 private:
  void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    reference_pose_ = *msg;
    has_reference_pose_ = true;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    updateInputRate(now);
    ++received_count_;

    geometry_msgs::PoseStamped out = transformPose(*msg);
    updateStuckState(out, now);

    const bool jump = detectJump(out);
    if (jump && config_.drop_on_jump) {
      ++dropped_by_jump_count_;
      return;
    }

    if (!shouldPublish(now)) {
      ++dropped_by_rate_count_;
      return;
    }

    publisher_.publish(out);
    last_output_pose_ = out;
    has_last_output_ = true;
    last_publish_time_ = now;
    next_publish_time_ = publish_interval_ > 0.0 ? now + ros::Duration(publish_interval_) : now;
    ++published_count_;
  }

  geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& input) const {
    const tf2::Transform world_tracker = poseToTransform(input.pose);
    tf2::Transform world_body = world_tracker * config_.tracker_to_body;
    world_body.setOrigin(world_body.getOrigin() + config_.field_offset);

    geometry_msgs::PoseStamped out;
    out.header = input.header;
    if (!config_.output_frame_id.empty()) {
      out.header.frame_id = config_.output_frame_id;
    }
    out.pose = transformToPose(world_body);
    return out;
  }

  void updateInputRate(const ros::Time& now) {
    if (!last_receive_time_.isZero()) {
      const double dt = (now - last_receive_time_).toSec();
      if (dt > 0.0) {
        input_intervals_.push_back(dt);
        while (input_intervals_.size() > 50) {
          input_intervals_.pop_front();
        }
      }
    }
    last_receive_time_ = now;
  }

  double estimateInputRate() const {
    if (input_intervals_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const double dt : input_intervals_) {
      sum += dt;
    }
    return sum > 0.0 ? static_cast<double>(input_intervals_.size()) / sum : 0.0;
  }

  bool shouldPublish(const ros::Time& now) {
    if (publish_interval_ <= 0.0 || next_publish_time_.isZero() || now >= next_publish_time_) {
      return true;
    }
    return false;
  }

  bool detectJump(const geometry_msgs::PoseStamped& out) {
    last_jump_detected_ = false;
    if (!has_last_output_) {
      return false;
    }
    last_jump_translation_m_ = poseDistance(out.pose, last_output_pose_.pose);
    last_jump_rotation_deg_ = quaternionAngleDeg(out.pose.orientation, last_output_pose_.pose.orientation);
    last_jump_detected_ = last_jump_translation_m_ > config_.max_translation_jump_m ||
                          last_jump_rotation_deg_ > config_.max_rotation_jump_deg;
    return last_jump_detected_;
  }

  void updateStuckState(const geometry_msgs::PoseStamped& out, const ros::Time& now) {
    if (!has_stuck_reference_) {
      stuck_reference_pose_ = out;
      stuck_reference_since_ = now;
      has_stuck_reference_ = true;
      stuck_ = false;
      return;
    }

    const double dp = poseDistance(out.pose, stuck_reference_pose_.pose);
    const double da = quaternionAngleDeg(out.pose.orientation, stuck_reference_pose_.pose.orientation);
    if (dp > config_.stuck_position_epsilon_m || da > config_.stuck_angle_epsilon_deg) {
      stuck_reference_pose_ = out;
      stuck_reference_since_ = now;
      stuck_ = false;
      return;
    }

    stuck_ = (now - stuck_reference_since_).toSec() > config_.stuck_timeout_s;
  }

  RouteConfig config_;
  ros::Subscriber subscriber_;
  ros::Subscriber reference_subscriber_;
  ros::Publisher publisher_;
  ros::Time last_receive_time_;
  ros::Time last_publish_time_;
  ros::Time next_publish_time_;
  ros::Time stuck_reference_since_;
  geometry_msgs::PoseStamped last_output_pose_;
  geometry_msgs::PoseStamped stuck_reference_pose_;
  geometry_msgs::PoseStamped reference_pose_;
  std::deque<double> input_intervals_;
  double publish_interval_ = 1.0 / kDefaultMaxOutputRateHz;
  double last_jump_translation_m_ = 0.0;
  double last_jump_rotation_deg_ = 0.0;
  uint64_t received_count_ = 0;
  uint64_t published_count_ = 0;
  uint64_t dropped_by_rate_count_ = 0;
  uint64_t dropped_by_jump_count_ = 0;
  bool has_last_output_ = false;
  bool has_stuck_reference_ = false;
  bool has_reference_pose_ = false;
  bool stuck_ = false;
  bool last_jump_detected_ = false;
};

RouteConfig loadRoute(
    const XmlRpc::XmlRpcValue& route,
    int index,
    const std::string& vrpn_pose_suffix,
    const std::string& mavros_pose_suffix,
    const std::string& output_frame_id,
    const RouteConfig& defaults) {
  if (route.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error("routes[" + std::to_string(index) + "] must be a map");
  }

  RouteConfig config = defaults;
  config.tracker = requiredString(route, "tracker", index);
  config.mavros_ns = requiredString(route, "mavros_ns", index);
  config.name = hasString(route, "name") ? static_cast<std::string>(route["name"]) : config.tracker;

  config.input_topic = hasString(route, "input_topic")
                           ? static_cast<std::string>(route["input_topic"])
                           : joinTopic("/vrpn_client_node/" + config.tracker, vrpn_pose_suffix);

  config.output_topic = hasString(route, "output_topic")
                            ? static_cast<std::string>(route["output_topic"])
                            : joinTopic(config.mavros_ns, mavros_pose_suffix);

  config.reference_topic = hasString(route, "reference_topic")
                               ? static_cast<std::string>(route["reference_topic"])
                               : joinTopic(config.mavros_ns, "local_position/pose");

  config.output_frame_id = hasString(route, "output_frame_id")
                               ? static_cast<std::string>(route["output_frame_id"])
                               : output_frame_id;

  config.max_output_rate_hz = optionalDouble(route, "max_output_rate_hz", config.max_output_rate_hz);
  config.min_input_rate_hz = optionalDouble(route, "min_input_rate_hz", config.min_input_rate_hz);
  config.max_input_rate_hz = optionalDouble(route, "max_input_rate_hz", config.max_input_rate_hz);
  config.stale_timeout_s = optionalDouble(route, "stale_timeout_s", config.stale_timeout_s);
  config.stuck_timeout_s = optionalDouble(route, "stuck_timeout_s", config.stuck_timeout_s);
  config.stuck_position_epsilon_m =
      optionalDouble(route, "stuck_position_epsilon_m", config.stuck_position_epsilon_m);
  config.stuck_angle_epsilon_deg =
      optionalDouble(route, "stuck_angle_epsilon_deg", config.stuck_angle_epsilon_deg);
  config.max_translation_jump_m =
      optionalDouble(route, "max_translation_jump_m", config.max_translation_jump_m);
  config.max_rotation_jump_deg =
      optionalDouble(route, "max_rotation_jump_deg", config.max_rotation_jump_deg);
  config.max_reference_delta_m =
      optionalDouble(route, "max_reference_delta_m", config.max_reference_delta_m);
  config.drop_on_jump = optionalBool(route, "drop_on_jump", config.drop_on_jump);
  config.tracker_to_body = readTransform(route, "tracker_to_body");
  config.field_offset = readVector3(route, "field_offset", config.field_offset);
  return config;
}

std::vector<RouteConfig> loadRoutes(ros::NodeHandle& pnh) {
  XmlRpc::XmlRpcValue routes_param;
  if (!pnh.getParam("routes", routes_param)) {
    throw std::runtime_error("private parameter 'routes' is required");
  }
  if (routes_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("private parameter 'routes' must be a list");
  }

  std::string vrpn_pose_suffix;
  std::string mavros_pose_suffix;
  std::string output_frame_id;
  pnh.param<std::string>("vrpn_pose_suffix", vrpn_pose_suffix, "pose");
  pnh.param<std::string>("mavros_pose_suffix", mavros_pose_suffix, "vision_pose/pose");
  pnh.param<std::string>("output_frame_id", output_frame_id, "");

  RouteConfig defaults;
  pnh.param<double>("max_output_rate_hz", defaults.max_output_rate_hz, defaults.max_output_rate_hz);
  pnh.param<double>("min_input_rate_hz", defaults.min_input_rate_hz, defaults.min_input_rate_hz);
  pnh.param<double>("max_input_rate_hz", defaults.max_input_rate_hz, defaults.max_input_rate_hz);
  pnh.param<double>("stale_timeout_s", defaults.stale_timeout_s, defaults.stale_timeout_s);
  pnh.param<double>("stuck_timeout_s", defaults.stuck_timeout_s, defaults.stuck_timeout_s);
  pnh.param<double>(
      "stuck_position_epsilon_m", defaults.stuck_position_epsilon_m, defaults.stuck_position_epsilon_m);
  pnh.param<double>(
      "stuck_angle_epsilon_deg", defaults.stuck_angle_epsilon_deg, defaults.stuck_angle_epsilon_deg);
  pnh.param<double>("max_translation_jump_m", defaults.max_translation_jump_m, defaults.max_translation_jump_m);
  pnh.param<double>("max_rotation_jump_deg", defaults.max_rotation_jump_deg, defaults.max_rotation_jump_deg);
  pnh.param<double>("max_reference_delta_m", defaults.max_reference_delta_m, defaults.max_reference_delta_m);
  pnh.param<bool>("drop_on_jump", defaults.drop_on_jump, defaults.drop_on_jump);

  XmlRpc::XmlRpcValue global_se3;
  if (pnh.getParam("tracker_to_body", global_se3)) {
    XmlRpc::XmlRpcValue wrapper;
    wrapper["tracker_to_body"] = global_se3;
    defaults.tracker_to_body = readTransform(wrapper, "tracker_to_body");
  }

  XmlRpc::XmlRpcValue global_offset;
  if (pnh.getParam("field_offset", global_offset)) {
    XmlRpc::XmlRpcValue wrapper;
    wrapper["field_offset"] = global_offset;
    defaults.field_offset = readVector3(wrapper, "field_offset", defaults.field_offset);
  }

  std::vector<RouteConfig> routes;
  routes.reserve(routes_param.size());
  for (int i = 0; i < routes_param.size(); ++i) {
    routes.push_back(loadRoute(routes_param[i], i, vrpn_pose_suffix, mavros_pose_suffix, output_frame_id, defaults));
  }
  return routes;
}

class DiagnosticsPublisher {
 public:
  DiagnosticsPublisher(ros::NodeHandle& nh, const std::vector<std::unique_ptr<PoseRoute>>& routes)
      : routes_(routes) {
    publisher_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
  }

  void publish(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    for (const auto& route : routes_) {
      array.status.push_back(route->diagnosticStatus(array.header.stamp));
    }
    publisher_.publish(array);
  }

 private:
  const std::vector<std::unique_ptr<PoseRoute>>& routes_;
  ros::Publisher publisher_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_router");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int queue_size = 10;
  double diagnostics_rate_hz = 1.0;
  pnh.param<int>("queue_size", queue_size, queue_size);
  pnh.param<double>("diagnostics_rate_hz", diagnostics_rate_hz, diagnostics_rate_hz);

  try {
    const auto route_configs = loadRoutes(pnh);
    if (route_configs.empty()) {
      throw std::runtime_error("at least one route is required");
    }

    std::vector<std::unique_ptr<PoseRoute>> routes;
    routes.reserve(route_configs.size());
    for (const auto& config : route_configs) {
      routes.emplace_back(std::make_unique<PoseRoute>(nh, config, queue_size));
    }

    DiagnosticsPublisher diagnostics(nh, routes);
    ros::Timer diagnostics_timer =
        nh.createTimer(ros::Duration(1.0 / std::max(0.1, diagnostics_rate_hz)),
                       &DiagnosticsPublisher::publish,
                       &diagnostics);

    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL_STREAM("failed to start vrpn_router: " << ex.what());
    return 1;
  }

  return 0;
}
