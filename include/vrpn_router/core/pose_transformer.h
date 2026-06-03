#pragma once

#include "vrpn_router/core/types.h"

namespace vrpn_router::core {

class PoseTransformer {
 public:
  void setTrackerToBody(const Transform& transform) { tracker_to_body_ = transform; }
  void setFieldOffset(const Vec3& offset) { field_offset_ = offset; }

  Pose apply(const Pose& world_tracker) const {
    Pose world_body;
    world_body.position =
        world_tracker.position + rotate(world_tracker.orientation, tracker_to_body_.translation) + field_offset_;
    world_body.orientation = multiply(world_tracker.orientation, tracker_to_body_.rotation);
    return world_body;
  }

 private:
  Transform tracker_to_body_;
  Vec3 field_offset_;
};

}  // namespace vrpn_router::core
