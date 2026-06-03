#pragma once

#include "vrpn_router/core/types.h"

#include <deque>
#include <string>
#include <vector>

namespace vrpn_router::core {

struct HealthConfig {
  double min_input_rate_hz = 30.0;
  double max_input_rate_hz = 150.0;
  double stale_timeout_s = 0.3;
  double stuck_timeout_s = 0.5;
  double stuck_position_epsilon_m = 0.001;
  double stuck_angle_epsilon_deg = 0.2;
  double max_translation_jump_m = 1.0;
  double max_rotation_jump_deg = 45.0;
  double max_reference_delta_m = 2.0;
};

struct HealthSnapshot {
  std::vector<std::string> problems;
  double input_rate_hz = 0.0;
  double age_s = -1.0;
  double last_jump_translation_m = 0.0;
  double last_jump_rotation_deg = 0.0;
  double reference_delta_m = -1.0;
  unsigned long long received_count = 0;
  unsigned long long published_count = 0;
  unsigned long long dropped_by_rate_count = 0;
};

class HealthMonitor {
 public:
  explicit HealthMonitor(HealthConfig config = {}) : config_(config) {}

  void onInput(double now_s) {
    if (has_last_receive_time_) {
      const double dt = now_s - last_receive_time_s_;
      if (dt > 0.0) {
        input_intervals_.push_back(dt);
        while (input_intervals_.size() > 50) {
          input_intervals_.pop_front();
        }
      }
    }
    last_receive_time_s_ = now_s;
    has_last_receive_time_ = true;
    ++received_count_;
  }

  void onRateDrop() { ++dropped_by_rate_count_; }

  void onPublish(const Pose& pose) {
    last_output_pose_ = pose;
    has_last_output_ = true;
    ++published_count_;
  }

  void onReference(const Pose& pose) {
    reference_pose_ = pose;
    has_reference_pose_ = true;
  }

  void updateStuckState(const Pose& pose, double now_s) {
    if (!has_stuck_reference_) {
      stuck_reference_pose_ = pose;
      stuck_reference_since_s_ = now_s;
      has_stuck_reference_ = true;
      stuck_ = false;
      return;
    }

    const double dp = distance(pose, stuck_reference_pose_);
    const double da = angleDeg(pose.orientation, stuck_reference_pose_.orientation);
    if (dp > config_.stuck_position_epsilon_m || da > config_.stuck_angle_epsilon_deg) {
      stuck_reference_pose_ = pose;
      stuck_reference_since_s_ = now_s;
      stuck_ = false;
      return;
    }

    stuck_ = (now_s - stuck_reference_since_s_) > config_.stuck_timeout_s;
  }

  void detectJump(const Pose& pose) {
    last_jump_detected_ = false;
    if (!has_last_output_) {
      return;
    }
    last_jump_translation_m_ = distance(pose, last_output_pose_);
    last_jump_rotation_deg_ = angleDeg(pose.orientation, last_output_pose_.orientation);
    last_jump_detected_ = last_jump_translation_m_ > config_.max_translation_jump_m ||
                          last_jump_rotation_deg_ > config_.max_rotation_jump_deg;
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

  HealthSnapshot snapshot(double now_s) const {
    HealthSnapshot snapshot;
    snapshot.input_rate_hz = estimateInputRate();
    snapshot.age_s = has_last_receive_time_ ? now_s - last_receive_time_s_ : -1.0;
    snapshot.last_jump_translation_m = last_jump_translation_m_;
    snapshot.last_jump_rotation_deg = last_jump_rotation_deg_;
    snapshot.reference_delta_m =
        has_reference_pose_ && has_last_output_ ? distance(last_output_pose_, reference_pose_) : -1.0;
    snapshot.received_count = received_count_;
    snapshot.published_count = published_count_;
    snapshot.dropped_by_rate_count = dropped_by_rate_count_;

    if (!has_last_receive_time_ || snapshot.age_s > config_.stale_timeout_s) {
      snapshot.problems.push_back("vrpn_timeout");
    }
    if (snapshot.input_rate_hz > 0.0 && snapshot.input_rate_hz < config_.min_input_rate_hz) {
      snapshot.problems.push_back("input_rate_low");
    }
    if (snapshot.input_rate_hz > config_.max_input_rate_hz) {
      snapshot.problems.push_back("input_rate_high");
    }
    if (stuck_) {
      snapshot.problems.push_back("vrpn_stuck");
    }
    if (last_jump_detected_) {
      snapshot.problems.push_back("vrpn_jump");
    }
    if (snapshot.reference_delta_m >= 0.0 && snapshot.reference_delta_m > config_.max_reference_delta_m) {
      snapshot.problems.push_back("reference_delta_high");
    }
    return snapshot;
  }

 private:
  HealthConfig config_;
  Pose last_output_pose_;
  Pose stuck_reference_pose_;
  Pose reference_pose_;
  std::deque<double> input_intervals_;
  double last_receive_time_s_ = 0.0;
  double stuck_reference_since_s_ = 0.0;
  double last_jump_translation_m_ = 0.0;
  double last_jump_rotation_deg_ = 0.0;
  unsigned long long received_count_ = 0;
  unsigned long long published_count_ = 0;
  unsigned long long dropped_by_rate_count_ = 0;
  bool has_last_receive_time_ = false;
  bool has_last_output_ = false;
  bool has_stuck_reference_ = false;
  bool has_reference_pose_ = false;
  bool stuck_ = false;
  bool last_jump_detected_ = false;
};

}  // namespace vrpn_router::core
