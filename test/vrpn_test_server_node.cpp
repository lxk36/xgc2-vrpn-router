#include <sys/time.h>

#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

namespace {

std::atomic_bool g_shutdown_requested{false};

void requestShutdownFromSignal(int) {
  g_shutdown_requested.store(true, std::memory_order_relaxed);
}

void installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = requestShutdownFromSignal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGTERM, &action, nullptr);
}

struct TrackerCase {
  std::string name;
  std::array<double, 3> position;
  double yaw_before_rad{0.0};
  double yaw_after_rad{0.0};
};

void fillYawQuaternion(double yaw_rad, vrpn_float64 quaternion[4]) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  q.normalize();
  quaternion[0] = q.x();
  quaternion[1] = q.y();
  quaternion[2] = q.z();
  quaternion[3] = q.w();
}

} // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_router_test_vrpn_server");
  ros::NodeHandle pnh("~");
  installSignalHandlers();

  int port = 3984;
  double publish_rate_hz = 120.0;
  double jump_after_s = 1.0;
  pnh.param<int>("port", port, port);
  pnh.param<double>("publish_rate_hz", publish_rate_hz, publish_rate_hz);
  pnh.param<double>("jump_after_s", jump_after_s, jump_after_s);

  constexpr double kPi = 3.14159265358979323846;
  std::vector<TrackerCase> cases{
      {"uav1", {1.0, 0.0, 0.0}, 0.0, 0.0},
      {"uav2", {0.0, 2.0, 0.0}, 0.0, kPi},
      {"ugv1", {0.0, 0.0, 3.0}, 0.0, 0.0},
  };

  vrpn_Connection* connection = vrpn_create_server_connection(port);
  if (connection == nullptr) {
    ROS_FATAL_STREAM("failed to create test VRPN server on port " << port);
    return 1;
  }

  std::vector<std::unique_ptr<vrpn_Tracker_Server>> trackers;
  trackers.reserve(cases.size());
  for (const auto& item : cases) {
    trackers.emplace_back(new vrpn_Tracker_Server(item.name.c_str(), connection, 1));
  }

  ROS_INFO_STREAM("test VRPN server publishing " << trackers.size() << " trackers on port " << port);

  const ros::WallTime started = ros::WallTime::now();
  ros::WallRate rate(publish_rate_hz);
  while (ros::ok() && !g_shutdown_requested.load(std::memory_order_relaxed)) {
    struct timeval timestamp {};
    gettimeofday(&timestamp, nullptr);
    const double elapsed_s = (ros::WallTime::now() - started).toSec();
    const bool after_jump = elapsed_s >= jump_after_s;

    for (size_t i = 0; i < cases.size(); ++i) {
      const TrackerCase& item = cases[i];
      const double jitter = 1.0e-7 * std::sin(elapsed_s * 13.0 + static_cast<double>(i));
      const vrpn_float64 position[3] = {
          item.position[0] + jitter,
          item.position[1] - jitter,
          item.position[2] + 0.5 * jitter,
      };
      vrpn_float64 quaternion[4]{};
      fillYawQuaternion(after_jump ? item.yaw_after_rad : item.yaw_before_rad, quaternion);

      trackers[i]->report_pose(0, timestamp, position, quaternion);
      trackers[i]->mainloop();
    }

    connection->mainloop();
    rate.sleep();
  }

  trackers.clear();
  connection->removeReference();
  return 0;
}
