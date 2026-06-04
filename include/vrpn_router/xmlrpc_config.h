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
  return value.hasMember(key) && (value[key].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                                  value[key].getType() == XmlRpc::XmlRpcValue::TypeInt);
}

inline double toDouble(const XmlRpc::XmlRpcValue& value) {
  double number = 0.0;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    number = static_cast<double>(value);
  } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    number = static_cast<int>(value);
  } else {
    throw std::runtime_error("expected numeric XML-RPC value");
  }
  if (!std::isfinite(number)) {
    throw std::runtime_error("numeric XML-RPC value must be finite");
  }
  return number;
}

inline double optionalDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
  return hasNumber(value, key) ? toDouble(value[key]) : fallback;
}

inline double optionalPositiveDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
  const double result = optionalDouble(value, key, fallback);
  if (!std::isfinite(result) || result <= 0.0) {
    throw std::runtime_error(std::string(key) + " must be finite and positive");
  }
  return result;
}

inline double optionalNonNegativeDouble(const XmlRpc::XmlRpcValue& value, const char* key, double fallback) {
  const double result = optionalDouble(value, key, fallback);
  if (!std::isfinite(result) || result < 0.0) {
    throw std::runtime_error(std::string(key) + " must be finite and non-negative");
  }
  return result;
}

inline std::string requiredString(const XmlRpc::XmlRpcValue& route, const char* key, int index) {
  if (!route.hasMember(key) || route[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("routes[" + std::to_string(index) + "]." + key + " must be a string");
  }
  return static_cast<std::string>(route[key]);
}

inline bool hasQuaternionComponent(const XmlRpc::XmlRpcValue& value) {
  return hasNumber(value, "x") || hasNumber(value, "y") || hasNumber(value, "z") || hasNumber(value, "w") ||
         hasNumber(value, "qx") || hasNumber(value, "qy") || hasNumber(value, "qz") || hasNumber(value, "qw");
}

inline core::Quat checkedNormalized(core::Quat q, const char* key) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 1.0e-12) {
    throw std::runtime_error(std::string(key) + " quaternion must not be zero");
  }
  return core::normalized(q);
}

inline core::Vec3 readVector3(const XmlRpc::XmlRpcValue& value, const char* key, const core::Vec3& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& vector = value[key];
  if (vector.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map with x/y/z");
  }
  return {optionalDouble(vector, "x", fallback.x), optionalDouble(vector, "y", fallback.y),
          optionalDouble(vector, "z", fallback.z)};
}

inline core::Quat readQuaternion(const XmlRpc::XmlRpcValue& value, const char* key, const core::Quat& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  const auto& rotation = value[key];
  if (rotation.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error(std::string(key) + " must be a map");
  }

  core::Quat q;
  if (hasQuaternionComponent(rotation)) {
    q = {optionalDouble(rotation, "x", optionalDouble(rotation, "qx", 0.0)),
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
    q = {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
         cr * cp * cy + sr * sp * sy};
  }
  return checkedNormalized(q, key);
}

inline core::Transform readTransform(const XmlRpc::XmlRpcValue& value, const char* key,
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

} // namespace vrpn_router
