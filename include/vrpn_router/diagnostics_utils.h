#pragma once

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <sstream>
#include <string>

namespace vrpn_router {

inline std::string toString(double value) {
  std::ostringstream oss;
  oss.precision(4);
  oss << std::fixed << value;
  return oss.str();
}

inline void addKv(diagnostic_msgs::DiagnosticStatus& status, const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

} // namespace vrpn_router
