#pragma once

#include "vrpn_router/core/types.h"
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace vrpn_router {

inline bool hasString(const XmlRpc::XmlRpcValue& value, const char* key) {
  return value.hasMember(key) && value[key].getType() == XmlRpc::XmlRpcValue::TypeString;
}

inline bool hasNumber(const XmlRpc::XmlRpcValue& value, const char* key) {
  return value.hasMember(key) &&
         (value[key].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
          value[key].getType() == XmlRpc::XmlRpcValue::TypeInt);
}

inline double toDouble(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("expected numeric XML-RPC value");
}

inline double optionalDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
  return hasNumber(value, key) ? toDouble(value[key]) : fallback;
}

inline std::string requiredString(const XmlRpc::XmlRpcValue& route, const char* key, int index) {
  if (!route.hasMember(key) || route[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("routes[" + std::to_string(index) + "]." + key + " must be a string");
  }
  return static_cast<std::string>(route[key]);
}

inline core::Vec3 readVector3(
    const XmlRpc::XmlRpcValue& value,
    const char* key,
    const core::Vec3& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& vector = value[key];
  if (vector.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map with x/y/z");
  }
  return {
      optionalDouble(vector, "x", fallback.x),
      optionalDouble(vector, "y", fallback.y),
      optionalDouble(vector, "z", fallback.z)};
}

inline core::Quat readQuaternion(
    const XmlRpc::XmlRpcValue& value,
    const char* key,
    const core::Quat& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& rotation = value[key];
  if (rotation.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map");
  }

  core::Quat q;
  if (hasNumber(rotation, "w") || hasNumber(rotation, "qx")) {
    q = {
        optionalDouble(rotation, "x", optionalDouble(rotation, "qx", 0.0)),
        optionalDouble(rotation, "y", optionalDouble(rotation, "qy", 0.0)),
        optionalDouble(rotation, "z", optionalDouble(rotation, "qz", 0.0)),
        optionalDouble(rotation, "w", optionalDouble(rotation, "qw", 1.0))};
  } else {
    const double roll = optionalDouble(rotation, "roll", 0.0);
    const double pitch = optionalDouble(rotation, "pitch", 0.0);
    const double yaw = optionalDouble(rotation, "yaw", 0.0);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    q = {
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy};
  }
  return core::normalized(q);
}

inline core::Transform readTransform(
    const XmlRpc::XmlRpcValue& value,
    const char* key,
    const core::Transform& fallback = {}) {
  if (!value.hasMember(key)) {
    return fallback;
  }

  const auto& se3 = value[key];
  if (se3.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map");
  }

  core::Transform transform = fallback;
  transform.translation = readVector3(se3, "translation", fallback.translation);
  transform.rotation = readQuaternion(se3, "rotation", fallback.rotation);
  return transform;
}

}  // namespace vrpn_router
