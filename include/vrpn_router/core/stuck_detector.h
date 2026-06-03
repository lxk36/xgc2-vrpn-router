#pragma once

#include "vrpn_router/core/types.h"

#include <algorithm>
#include <deque>

namespace vrpn_router::core {

class StuckDetector {
 public:
  void observe(const Pose& pose, double now_s, double position_epsilon_m, double angle_epsilon_deg, double timeout_s) {
    samples_.push_back({now_s, pose});
    prune(now_s, timeout_s);

    if (samples_.empty() || (samples_.back().time_s - samples_.front().time_s) < timeout_s) {
      stuck_ = false;
      return;
    }

    const Pose& latest = samples_.back().pose;
    stuck_ = std::all_of(samples_.begin(), samples_.end(), [&](const Sample& sample) {
      return distance(sample.pose, latest) <= position_epsilon_m &&
             angleDeg(sample.pose.orientation, latest.orientation) <= angle_epsilon_deg;
    });
  }

  bool stuck() const { return stuck_; }

 private:
  struct Sample {
    double time_s = 0.0;
    Pose pose;
  };

  void prune(double now_s, double window_s) {
    const double window_start_s = now_s - window_s;
    while (samples_.size() > 1 && samples_[1].time_s <= window_start_s) {
      samples_.pop_front();
    }
  }

  std::deque<Sample> samples_;
  bool stuck_ = false;
};

}  // namespace vrpn_router::core
