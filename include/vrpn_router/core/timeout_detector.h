#pragma once

namespace vrpn_router::core {

class TimeoutDetector {
 public:
  void recordInput(double now_s) {
    last_input_s_ = now_s;
    has_input_ = true;
  }

  double ageS(double now_s) const { return has_input_ ? now_s - last_input_s_ : -1.0; }

  bool timedOut(double now_s, double timeout_s) const {
    return !has_input_ || ageS(now_s) > timeout_s;
  }

 private:
  double last_input_s_ = 0.0;
  bool has_input_ = false;
};

}  // namespace vrpn_router::core
