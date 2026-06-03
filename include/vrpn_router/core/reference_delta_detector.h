#pragma once

#include "vrpn_router/core/types.h"

namespace vrpn_router::core {

class ReferenceDeltaDetector {
 public:
  void setReference(const Pose& pose) {
    reference_pose_ = pose;
    has_reference_ = true;
  }

  void setOutput(const Pose& pose) {
    output_pose_ = pose;
    has_output_ = true;
  }

  double deltaM() const {
    return has_reference_ && has_output_ ? distance(output_pose_, reference_pose_) : -1.0;
  }

  bool aboveMax(double max_delta_m) const {
    const double delta = deltaM();
    return delta >= 0.0 && delta > max_delta_m;
  }

 private:
  Pose reference_pose_;
  Pose output_pose_;
  bool has_reference_ = false;
  bool has_output_ = false;
};

}  // namespace vrpn_router::core
