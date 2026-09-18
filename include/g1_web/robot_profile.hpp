#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace g1_web {

enum class CapabilityKey {
  kTelemetry,
  kLocomotion,
  kJointDebug,
  kJointTeach,
  kAudio,
  kCameraRgb,
  kCameraDepth,
  kLidar,
  kSlam,
  kHeadControl,
};

enum class VerificationLevel {
  kUnsupported,
  kImplemented,
  kMockVerified,
  kReadonlyVerified,
  kControlVerified,
  kDisabled,
};

struct CapabilityDescriptor {
  CapabilityKey key{CapabilityKey::kTelemetry};
  bool implemented{false};
  bool available{false};
  VerificationLevel verification_level{VerificationLevel::kUnsupported};
  std::map<std::string, std::string> parameters;
  std::string reason;
};

struct RobotIdentity {
  std::string vendor;
  std::string family;
  std::string product_id;
  std::string variant;
  std::string display_name;
  std::string morphology;
};

struct JointDescriptor {
  std::string name;
  std::string name_zh;
  std::string urdf_joint_name;
  std::size_t motor_slot{0};
  std::string display_group;
};

struct JointSchema {
  std::size_t motor_slot_count{0};
  std::vector<JointDescriptor> joints;
};

struct RobotModelVariant {
  std::uint32_t selector_value{0};
  std::string name;
  std::string urdf_file;
};

struct RobotProfile {
  RobotIdentity identity;
  std::string idl_family;
  std::string control_api_family;
  std::string model_asset_root;
  std::string model_package;
  JointSchema joint_schema;
  std::vector<RobotModelVariant> model_variants;
  std::map<std::string, std::string> static_constraints;
  // Product/code baseline only; runtime effective availability is separate.
  std::vector<CapabilityDescriptor> capabilities;
};

inline const RobotModelVariant* FindModelVariant(
    const RobotProfile& profile, std::uint32_t selector_value) {
  for (const auto& model : profile.model_variants) {
    if (model.selector_value == selector_value) return &model;
  }
  return nullptr;
}

}  // namespace g1_web
