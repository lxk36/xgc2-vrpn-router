#pragma once

#include "vrpn_router/core/input_rate_detector.h"
#include "vrpn_router/core/jump_detector.h"
#include "vrpn_router/core/reference_delta_detector.h"
#include "vrpn_router/core/stuck_detector.h"
#include "vrpn_router/core/timeout_detector.h"
#include "vrpn_router/core/types.h"

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
  double jump_report_hold_s = 0.5;
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
    input_rate_.record(now_s);
    timeout_.recordInput(now_s);
    ++received_count_;
  }

  void onRateDrop() { ++dropped_by_rate_count_; }

  void onPublish(const Pose& pose) {
    jump_.setReferenceOutput(pose);
    reference_delta_.setOutput(pose);
    ++published_count_;
  }

  void onReference(const Pose& pose) {
    reference_delta_.setReference(pose);
  }

  void updateStuckState(const Pose& pose, double now_s) {
    stuck_.observe(
        pose,
        now_s,
        config_.stuck_position_epsilon_m,
        config_.stuck_angle_epsilon_deg,
        config_.stuck_timeout_s);
  }

  void detectJump(const Pose& pose, double now_s) {
    jump_.evaluate(pose, config_.max_translation_jump_m, config_.max_rotation_jump_deg);
    if (jump_.detected()) {
      jump_report_until_s_ = now_s + config_.jump_report_hold_s;
    }
  }

  double estimateInputRate() const { return input_rate_.rateHz(); }

  HealthSnapshot snapshot(double now_s) const {
    HealthSnapshot snapshot;
    snapshot.input_rate_hz = estimateInputRate();
    snapshot.age_s = timeout_.ageS(now_s);
    snapshot.last_jump_translation_m = jump_.lastTranslationM();
    snapshot.last_jump_rotation_deg = jump_.lastRotationDeg();
    snapshot.reference_delta_m = reference_delta_.deltaM();
    snapshot.received_count = received_count_;
    snapshot.published_count = published_count_;
    snapshot.dropped_by_rate_count = dropped_by_rate_count_;

    if (timeout_.timedOut(now_s, config_.stale_timeout_s)) {
      snapshot.problems.push_back("vrpn_timeout");
    }
    if (input_rate_.belowMin(config_.min_input_rate_hz)) {
      snapshot.problems.push_back("input_rate_low");
    }
    if (input_rate_.aboveMax(config_.max_input_rate_hz)) {
      snapshot.problems.push_back("input_rate_high");
    }
    if (stuck_.stuck()) {
      snapshot.problems.push_back("vrpn_stuck");
    }
    if (jump_.detected() || now_s <= jump_report_until_s_) {
      snapshot.problems.push_back("vrpn_jump");
    }
    if (reference_delta_.aboveMax(config_.max_reference_delta_m)) {
      snapshot.problems.push_back("reference_delta_high");
    }
    return snapshot;
  }

 private:
  HealthConfig config_;
  InputRateDetector input_rate_;
  JumpDetector jump_;
  ReferenceDeltaDetector reference_delta_;
  StuckDetector stuck_;
  TimeoutDetector timeout_;
  unsigned long long received_count_ = 0;
  unsigned long long published_count_ = 0;
  unsigned long long dropped_by_rate_count_ = 0;
  double jump_report_until_s_ = -1.0;
};

}  // namespace vrpn_router::core
