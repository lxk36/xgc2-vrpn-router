#pragma once

#include <limits>

namespace vrpn_router::core {

class RateLimiter {
 public:
  explicit RateLimiter(double max_rate_hz = 50.0) { setMaxRate(max_rate_hz); }

  void setMaxRate(double max_rate_hz) {
    interval_s_ = max_rate_hz > 0.0 ? 1.0 / max_rate_hz : 0.0;
    next_publish_s_ = -std::numeric_limits<double>::infinity();
  }

  bool shouldPublish(double now_s) const {
    return interval_s_ <= 0.0 || now_s >= next_publish_s_;
  }

  void markPublished(double now_s) {
    next_publish_s_ = interval_s_ > 0.0 ? now_s + interval_s_ : now_s;
  }

 private:
  double interval_s_ = 0.02;
  double next_publish_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace vrpn_router::core
