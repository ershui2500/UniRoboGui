#include "g1_web/a2_joint_debug_policy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <json/json.h>

namespace g1_web {
namespace {

class A2JointDebugPolicy final : public IJointDebugPolicy {
 public:
  A2JointDebugPolicy(const RobotProfile& profile, const A2JointGains& gains)
      : profile_(profile), gains_(gains) {}

  const RobotProfile& profile() const override { return profile_; }

  std::vector<std::string> GroupsForMode(
      const std::string& mode) const override {
    if (mode != "full_body") return {};
    std::vector<std::string> groups;
    for (const auto& joint : profile_.joint_schema.joints) {
      if (std::find(groups.begin(), groups.end(), joint.display_group) ==
          groups.end()) {
        groups.push_back(joint.display_group);
      }
    }
    return groups;
  }

  std::vector<std::string> TeachGroups() const override {
    return GroupsForMode("full_body");
  }

  std::string TeachMode() const override { return "full_body"; }

  JointDebugTransport TransportForMode(const std::string&) const override {
    return JointDebugTransport::kLowCmd;
  }

  std::optional<std::size_t> ArmSdkWeightMotorSlot() const override {
    return std::nullopt;
  }

  JointDebugGain NormalGain(const JointDescriptor& joint) const override {
    return gains_.normal.at(joint.motor_slot);
  }

  JointDebugGain RecordingGain(const JointDescriptor& joint) const override {
    return gains_.recording.at(joint.motor_slot);
  }

  bool UpperBodyFsmAllowed(std::uint32_t) const override { return false; }

  std::vector<std::string> LowCmdBlockingServices() const override {
    return {"ai_sport"};
  }

  bool UsesMotionSwitcherForLowCmd() const override { return true; }
  bool SupportsMotionModeSwitch() const override { return false; }

  const RobotModelVariant* ResolveModelVariant(
      std::uint32_t mode_machine) const override {
    return FindModelVariant(profile_, mode_machine);
  }

  bool LegacyTeachActionCompatible(std::uint32_t,
                                   std::size_t) const override {
    return false;
  }

  bool SupportsTeachRemoteControl() const override { return true; }
  TeachRemoteSource TeachRemoteDataSource() const override {
    return TeachRemoteSource::kWirelessController;
  }
  std::string TeachRemoteSecondaryKeyLabel() const override { return "F3"; }

 private:
  const RobotProfile& profile_;
  A2JointGains gains_;
};

}  // namespace

bool LoadA2JointGains(const std::string& path, A2JointGains& gains) {
  std::ifstream input(path);
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  if (!input || !Json::parseFromStream(builder, input, &root, &error) ||
      !root.isObject() || !root["joints"].isArray() ||
      root["joints"].size() != 12) return false;
  A2JointGains parsed;
  parsed.remote_protocol_confirmed =
      root["remote_protocol"].isString() &&
      root["remote_protocol"].asString() == "unitree_raw40_keys_v1";
  for (Json::ArrayIndex i = 0; i < 12; ++i) {
    const auto& item = root["joints"][i];
    if (!item.isObject()) return false;
    for (const char* key : {"kp", "kd", "record_kp", "record_kd"}) {
      if (!item[key].isNumeric()) return false;
      const double value = item[key].asDouble();
      if (!std::isfinite(value) || value < 0 ||
          value > std::numeric_limits<float>::max()) return false;
    }
    parsed.normal[i] = {item["kp"].asFloat(), item["kd"].asFloat(), false};
    parsed.recording[i] = {item["record_kp"].asFloat(), item["record_kd"].asFloat(), false};
  }
  gains = parsed;
  return true;
}

std::unique_ptr<IJointDebugPolicy> CreateA2JointDebugPolicy(
    const RobotProfile& profile, const A2JointGains& gains) {
  return std::make_unique<A2JointDebugPolicy>(profile, gains);
}

}  // namespace g1_web
