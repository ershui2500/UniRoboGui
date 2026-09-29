#include "g1_web/go2_joint_debug_policy.hpp"

#include <algorithm>
#include <cmath>

namespace g1_web {
namespace {

class Go2JointDebugPolicy final : public IJointDebugPolicy {
 public:
  explicit Go2JointDebugPolicy(const RobotProfile& profile) : profile_(profile) {}

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
    return JointDebugTransport::kGo2LowCmd;
  }

  std::optional<std::size_t> ArmSdkWeightMotorSlot() const override {
    return std::nullopt;
  }

  JointDebugGain NormalGain(const JointDescriptor&) const override {
    return {60.0F, 5.0F, false};
  }

  JointDebugGain RecordingGain(const JointDescriptor&) const override {
    return {0.0F, 1.0F, false};
  }

  bool UpperBodyFsmAllowed(std::uint32_t) const override { return false; }

  std::vector<std::string> LowCmdBlockingServices() const override {
    return {"mcf", "sport_mode", "ai_sport", "advanced_sport"};
  }

  bool UsesMotionSwitcherForLowCmd() const override { return true; }

  bool LowLevelEntryPoseReached(
      const std::vector<float>& positions) const override {
    if (positions.size() != 12) return false;
    for (const auto index : {1U, 4U, 7U, 10U}) {
      if (!std::isfinite(positions[index]) || positions[index] < 1.1F)
        return false;
    }
    for (const auto index : {2U, 5U, 8U, 11U}) {
      if (!std::isfinite(positions[index]) || positions[index] > -2.4F)
        return false;
    }
    return true;
  }

  bool HighLevelRecoveryPoseReached(
      const std::vector<float>& positions) const override {
    if (positions.size() != 12) return false;
    for (const auto index : {1U, 4U, 7U, 10U}) {
      if (!std::isfinite(positions[index]) || positions[index] > 1.0F)
        return false;
    }
    for (const auto index : {2U, 5U, 8U, 11U}) {
      if (!std::isfinite(positions[index]) || positions[index] < -2.1F)
        return false;
    }
    return true;
  }

  const RobotModelVariant* ResolveModelVariant(
      std::uint32_t mode_machine) const override {
    return FindModelVariant(profile_, mode_machine);
  }

  bool LegacyTeachActionCompatible(std::uint32_t,
                                   std::size_t) const override {
    return false;
  }

  bool SupportsTeachRemoteControl() const override { return true; }

  std::string TeachRemoteSecondaryKeyLabel() const override { return "F2"; }

 private:
  const RobotProfile& profile_;
};

}  // namespace

std::unique_ptr<IJointDebugPolicy> CreateGo2JointDebugPolicy(
    const RobotProfile& profile) {
  return std::make_unique<Go2JointDebugPolicy>(profile);
}

}  // namespace g1_web
