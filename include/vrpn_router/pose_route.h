#pragma once

#include "vrpn_router/core/health_monitor.h"
#include "vrpn_router/core/pose_transformer.h"
#include "vrpn_router/core/rate_limiter.h"
#include "vrpn_router/diagnostics_utils.h"
#include "vrpn_router/ros_pose_adapter.h"
#include "vrpn_router/route_config.h"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <string>
#include <utility>

namespace vrpn_router {

class PoseRoute {
 public:
  PoseRoute(ros::NodeHandle& nh, RouteConfig config, int queue_size)
      : config_(std::move(config)),
        rate_limiter_(config_.max_output_rate_hz),
        health_(toHealthConfig(config_)) {
    transformer_.setTrackerToBody(config_.tracker_to_body);
    transformer_.setFieldOffset(config_.field_offset);

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
    const core::HealthSnapshot snapshot = health_.snapshot(now.toSec());
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "vrpn_router/" + config_.name;
    status.hardware_id = config_.tracker;
    status.level = snapshot.problems.empty() ? diagnostic_msgs::DiagnosticStatus::OK
                                             : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = snapshot.problems.empty() ? "OK" : joinProblems(snapshot.problems);

    addKv(status, "input_topic", config_.input_topic);
    addKv(status, "output_topic", config_.output_topic);
    addKv(status, "input_rate_hz", toString(snapshot.input_rate_hz));
    addKv(status, "published_count", std::to_string(snapshot.published_count));
    addKv(status, "received_count", std::to_string(snapshot.received_count));
    addKv(status, "dropped_by_rate_count", std::to_string(snapshot.dropped_by_rate_count));
    addKv(status, "age_s", snapshot.age_s < 0.0 ? "none" : toString(snapshot.age_s));
    addKv(status, "last_jump_translation_m", toString(snapshot.last_jump_translation_m));
    addKv(status, "last_jump_rotation_deg", toString(snapshot.last_jump_rotation_deg));
    addKv(status, "reference_delta_m", snapshot.reference_delta_m < 0.0 ? "none" : toString(snapshot.reference_delta_m));
    return status;
  }

 private:
  static core::HealthConfig toHealthConfig(const RouteConfig& config) {
    core::HealthConfig health;
    health.min_input_rate_hz = config.min_input_rate_hz;
    health.max_input_rate_hz = config.max_input_rate_hz;
    health.stale_timeout_s = config.stale_timeout_s;
    health.stuck_timeout_s = config.stuck_timeout_s;
    health.stuck_position_epsilon_m = config.stuck_position_epsilon_m;
    health.stuck_angle_epsilon_deg = config.stuck_angle_epsilon_deg;
    health.max_translation_jump_m = config.max_translation_jump_m;
    health.max_rotation_jump_deg = config.max_rotation_jump_deg;
    health.max_reference_delta_m = config.max_reference_delta_m;
    return health;
  }

  static std::string joinProblems(const std::vector<std::string>& problems) {
    std::string message = problems.front();
    for (size_t i = 1; i < problems.size(); ++i) {
      message += "," + problems[i];
    }
    return message;
  }

  void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    health_.onReference(toCorePose(msg->pose));
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const double now_s = now.toSec();
    health_.onInput(now_s);

    const core::Pose transformed = transformer_.apply(toCorePose(msg->pose));
    health_.updateStuckState(transformed, now_s);
    health_.detectJump(transformed, now_s);

    if (!rate_limiter_.shouldPublish(now_s)) {
      health_.onRateDrop();
      return;
    }

    geometry_msgs::PoseStamped out;
    out.header = msg->header;
    if (!config_.output_frame_id.empty()) {
      out.header.frame_id = config_.output_frame_id;
    }
    out.pose = toRosPose(transformed);

    publisher_.publish(out);
    rate_limiter_.markPublished(now_s);
    health_.onPublish(transformed);
  }

  RouteConfig config_;
  core::PoseTransformer transformer_;
  core::RateLimiter rate_limiter_;
  core::HealthMonitor health_;
  ros::Subscriber subscriber_;
  ros::Subscriber reference_subscriber_;
  ros::Publisher publisher_;
};

}  // namespace vrpn_router
