#pragma once

#include <string>

namespace vrpn_router {

inline std::string normalizeNamespace(std::string value) {
  if (value.empty()) {
    return value;
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

inline std::string joinTopic(const std::string& ns, const std::string& suffix) {
  const std::string normalized = normalizeNamespace(ns);
  if (normalized.empty() || normalized == "/") {
    return suffix.front() == '/' ? suffix : "/" + suffix;
  }
  if (suffix.empty()) {
    return normalized;
  }
  return normalized + (suffix.front() == '/' ? suffix : "/" + suffix);
}

}  // namespace vrpn_router
