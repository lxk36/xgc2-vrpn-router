#pragma once

#include "vrpn_router/topic_utils.h"
#include "vrpn_router/xmlrpc_config.h"

#include <ros/node_handle.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace vrpn_router {

constexpr double kDefaultMaxOutputRateHz = 50.0;

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
  core::Transform tracker_to_body;
  core::Vec3 field_offset;
};

inline void validateRouteConfig(const RouteConfig& config) {
  if (config.tracker.empty()) {
    throw std::runtime_error("route name '" + config.name + "' tracker must not be empty");
  }
  if (config.mavros_ns.empty()) {
    throw std::runtime_error("route '" + config.name + "' mavros_ns must not be empty");
  }
  if (config.input_topic.empty()) {
    throw std::runtime_error("route '" + config.name + "' input_topic must not be empty");
  }
  if (config.output_topic.empty()) {
    throw std::runtime_error("route '" + config.name + "' output_topic must not be empty");
  }
  if (config.max_input_rate_hz < config.min_input_rate_hz) {
    throw std::runtime_error("route '" + config.name + "' max_input_rate_hz must be >= min_input_rate_hz");
  }
}

inline RouteConfig loadRoute(const XmlRpc::XmlRpcValue& route, int index, const std::string& vrpn_pose_suffix,
                             const std::string& mavros_pose_suffix, const std::string& output_frame_id,
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

  config.output_topic = hasString(route, "output_topic") ? static_cast<std::string>(route["output_topic"])
                                                         : joinTopic(config.mavros_ns, mavros_pose_suffix);

  config.reference_topic = hasString(route, "reference_topic") ? static_cast<std::string>(route["reference_topic"])
                                                               : joinTopic(config.mavros_ns, "local_position/pose");

  config.output_frame_id =
      hasString(route, "output_frame_id") ? static_cast<std::string>(route["output_frame_id"]) : output_frame_id;

  config.max_output_rate_hz = optionalPositiveDouble(route, "max_output_rate_hz", config.max_output_rate_hz);
  config.min_input_rate_hz = optionalPositiveDouble(route, "min_input_rate_hz", config.min_input_rate_hz);
  config.max_input_rate_hz = optionalPositiveDouble(route, "max_input_rate_hz", config.max_input_rate_hz);
  config.stale_timeout_s = optionalPositiveDouble(route, "stale_timeout_s", config.stale_timeout_s);
  config.stuck_timeout_s = optionalPositiveDouble(route, "stuck_timeout_s", config.stuck_timeout_s);
  config.stuck_position_epsilon_m =
      optionalNonNegativeDouble(route, "stuck_position_epsilon_m", config.stuck_position_epsilon_m);
  config.stuck_angle_epsilon_deg =
      optionalNonNegativeDouble(route, "stuck_angle_epsilon_deg", config.stuck_angle_epsilon_deg);
  config.max_translation_jump_m =
      optionalNonNegativeDouble(route, "max_translation_jump_m", config.max_translation_jump_m);
  config.max_rotation_jump_deg =
      optionalNonNegativeDouble(route, "max_rotation_jump_deg", config.max_rotation_jump_deg);
  config.max_reference_delta_m =
      optionalNonNegativeDouble(route, "max_reference_delta_m", config.max_reference_delta_m);
  config.tracker_to_body = readTransform(route, "tracker_to_body", config.tracker_to_body);
  config.field_offset = readVector3(route, "field_offset", config.field_offset);
  validateRouteConfig(config);
  return config;
}

inline std::vector<RouteConfig> loadRoutes(ros::NodeHandle& pnh) {
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
  pnh.param<double>("stuck_position_epsilon_m", defaults.stuck_position_epsilon_m, defaults.stuck_position_epsilon_m);
  pnh.param<double>("stuck_angle_epsilon_deg", defaults.stuck_angle_epsilon_deg, defaults.stuck_angle_epsilon_deg);
  pnh.param<double>("max_translation_jump_m", defaults.max_translation_jump_m, defaults.max_translation_jump_m);
  pnh.param<double>("max_rotation_jump_deg", defaults.max_rotation_jump_deg, defaults.max_rotation_jump_deg);
  pnh.param<double>("max_reference_delta_m", defaults.max_reference_delta_m, defaults.max_reference_delta_m);

  XmlRpc::XmlRpcValue global_se3;
  if (pnh.getParam("tracker_to_body", global_se3)) {
    XmlRpc::XmlRpcValue wrapper;
    wrapper["tracker_to_body"] = global_se3;
    defaults.tracker_to_body = readTransform(wrapper, "tracker_to_body", defaults.tracker_to_body);
  }

  XmlRpc::XmlRpcValue global_offset;
  if (pnh.getParam("field_offset", global_offset)) {
    XmlRpc::XmlRpcValue wrapper;
    wrapper["field_offset"] = global_offset;
    defaults.field_offset = readVector3(wrapper, "field_offset", defaults.field_offset);
  }

  std::vector<RouteConfig> routes;
  routes.reserve(static_cast<std::size_t>(routes_param.size()));
  for (int i = 0; i < routes_param.size(); ++i) {
    routes.push_back(loadRoute(routes_param[i], i, vrpn_pose_suffix, mavros_pose_suffix, output_frame_id, defaults));
  }
  return routes;
}

} // namespace vrpn_router
