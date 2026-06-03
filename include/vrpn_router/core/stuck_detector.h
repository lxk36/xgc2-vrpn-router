#pragma once

#include "vrpn_router/core/types.h"

namespace vrpn_router::core {

class StuckDetector {
 public:
  void observe(const Pose& pose, double now_s, double position_epsilon_m, double angle_epsilon_deg, double timeout_s) {
    if (!has_reference_) {
      reference_pose_ = pose;
      reference_since_s_ = now_s;
      has_reference_ = true;
      stuck_ = false;
      return;
    }

    const double dp = distance(pose, reference_pose_);
    const double da = angleDeg(pose.orientation, reference_pose_.orientation);
    if (dp > position_epsilon_m || da > angle_epsilon_deg) {
      reference_pose_ = pose;
      reference_since_s_ = now_s;
      stuck_ = false;
      return;
    }

    stuck_ = (now_s - reference_since_s_) > timeout_s;
  }

  bool stuck() const { return stuck_; }

 private:
  Pose reference_pose_;
  double reference_since_s_ = 0.0;
  bool has_reference_ = false;
  bool stuck_ = false;
};

}  // namespace vrpn_router::core
