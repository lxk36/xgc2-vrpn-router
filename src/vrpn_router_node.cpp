#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

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

struct RouteConfig {
  std::string name;
  std::string tracker;
  std::string input_topic;
  std::string mavros_ns;
  std::string output_topic;
  std::string output_frame_id;
};

class PoseRoute {
 public:
  PoseRoute(ros::NodeHandle& nh, RouteConfig config, int queue_size)
      : config_(std::move(config)) {
    publisher_ = nh.advertise<geometry_msgs::PoseStamped>(config_.output_topic, queue_size);
    subscriber_ = nh.subscribe<geometry_msgs::PoseStamped>(
        config_.input_topic, queue_size, &PoseRoute::poseCallback, this);

    ROS_INFO_STREAM("vrpn route " << config_.name << ": " << config_.input_topic << " -> "
                                  << config_.output_topic);
  }

 private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped out = *msg;
    if (!config_.output_frame_id.empty()) {
      out.header.frame_id = config_.output_frame_id;
    }
    publisher_.publish(out);
  }

  RouteConfig config_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};

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

  std::vector<RouteConfig> routes;
  routes.reserve(routes_param.size());

  for (int i = 0; i < routes_param.size(); ++i) {
    const auto& route = routes_param[i];
    if (route.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("routes[" + std::to_string(i) + "] must be a map");
    }

    RouteConfig config;
    config.tracker = requiredString(route, "tracker", i);
    config.mavros_ns = requiredString(route, "mavros_ns", i);
    config.name = route.hasMember("name") && route["name"].getType() == XmlRpc::XmlRpcValue::TypeString
                      ? static_cast<std::string>(route["name"])
                      : config.tracker;

    config.input_topic =
        route.hasMember("input_topic") && route["input_topic"].getType() == XmlRpc::XmlRpcValue::TypeString
            ? static_cast<std::string>(route["input_topic"])
            : joinTopic("/vrpn_client_node/" + config.tracker, vrpn_pose_suffix);

    config.output_topic =
        route.hasMember("output_topic") && route["output_topic"].getType() == XmlRpc::XmlRpcValue::TypeString
            ? static_cast<std::string>(route["output_topic"])
            : joinTopic(config.mavros_ns, mavros_pose_suffix);

    config.output_frame_id =
        route.hasMember("output_frame_id") &&
                route["output_frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString
            ? static_cast<std::string>(route["output_frame_id"])
            : output_frame_id;

    routes.push_back(std::move(config));
  }

  return routes;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_router");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int queue_size = 10;
  pnh.param<int>("queue_size", queue_size, queue_size);

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

    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL_STREAM("failed to start vrpn_router: " << ex.what());
    return 1;
  }

  return 0;
}
