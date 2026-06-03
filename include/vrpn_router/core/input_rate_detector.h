#pragma once

#include <cstddef>
#include <deque>

namespace vrpn_router::core {

class InputRateDetector {
 public:
  explicit InputRateDetector(std::size_t window_size = 50) : window_size_(window_size) {}

  void record(double now_s) {
    if (has_last_time_) {
      const double dt = now_s - last_time_s_;
      if (dt > 0.0) {
        intervals_s_.push_back(dt);
        while (intervals_s_.size() > window_size_) {
          intervals_s_.pop_front();
        }
      }
    }
    last_time_s_ = now_s;
    has_last_time_ = true;
  }

  double rateHz() const {
    if (intervals_s_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const double dt : intervals_s_) {
      sum += dt;
    }
    return sum > 0.0 ? static_cast<double>(intervals_s_.size()) / sum : 0.0;
  }

  bool belowMin(double min_rate_hz) const {
    const double rate = rateHz();
    return rate > 0.0 && rate < min_rate_hz;
  }

  bool aboveMax(double max_rate_hz) const { return rateHz() > max_rate_hz; }

 private:
  std::deque<double> intervals_s_;
  std::size_t window_size_;
  double last_time_s_ = 0.0;
  bool has_last_time_ = false;
};

}  // namespace vrpn_router::core
