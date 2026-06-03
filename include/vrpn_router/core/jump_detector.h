#pragma once

#include "vrpn_router/core/types.h"

namespace vrpn_router::core {

class JumpDetector {
 public:
  void setReferenceOutput(const Pose& pose) {
    reference_pose_ = pose;
    has_reference_ = true;
  }

  void evaluate(const Pose& pose, double max_translation_m, double max_rotation_deg) {
    detected_ = false;
    if (!has_reference_) {
      return;
    }

    last_translation_m_ = distance(pose, reference_pose_);
    last_rotation_deg_ = angleDeg(pose.orientation, reference_pose_.orientation);
    detected_ = last_translation_m_ > max_translation_m || last_rotation_deg_ > max_rotation_deg;
  }

  bool detected() const { return detected_; }
  double lastTranslationM() const { return last_translation_m_; }
  double lastRotationDeg() const { return last_rotation_deg_; }

 private:
  Pose reference_pose_;
  double last_translation_m_ = 0.0;
  double last_rotation_deg_ = 0.0;
  bool has_reference_ = false;
  bool detected_ = false;
};

}  // namespace vrpn_router::core
