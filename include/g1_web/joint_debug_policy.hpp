#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "g1_web/robot_profile.hpp"

namespace g1_web {

enum class JointDebugTransport { kArmSdk, kLowCmd };

struct JointDebugGain {
  float kp{0.0F};
  float kd{0.0F};
  bool hold_record_start_pose{false};
};

class IJointDebugPolicy {
 public:
  virtual ~IJointDebugPolicy() = default;

  virtual const RobotProfile& profile() const = 0;
  virtual std::vector<std::string> GroupsForMode(
      const std::string& mode) const = 0;
  virtual std::vector<std::string> TeachGroups() const = 0;
  virtual JointDebugTransport TransportForMode(
      const std::string& mode) const = 0;
  virtual std::optional<std::size_t> ArmSdkWeightMotorSlot() const = 0;
  virtual JointDebugGain NormalGain(const JointDescriptor& joint) const = 0;
  virtual JointDebugGain RecordingGain(const JointDescriptor& joint) const = 0;
  virtual JointDebugGain NormalGainForMode(
      const JointDescriptor& joint, const std::string&) const {
    return NormalGain(joint);
  }
  virtual JointDebugGain RecordingGainForMode(
      const JointDescriptor& joint, const std::string&) const {
    return RecordingGain(joint);
  }
  virtual bool UpperBodyFsmAllowed(std::uint32_t fsm_id) const = 0;
  virtual bool UpperBodyActiveFsmAllowed(std::uint32_t fsm_id) const {
    return UpperBodyFsmAllowed(fsm_id);
  }
  virtual const RobotModelVariant* ResolveModelVariant(
      std::uint32_t mode_machine) const = 0;
  virtual bool LegacyTeachActionCompatible(std::uint32_t mode_machine,
                                           std::size_t frame_width) const = 0;
  virtual bool ArmSdkJoint(const JointDescriptor& joint) const {
    const auto groups = GroupsForMode("upper_body");
    return std::find(groups.begin(), groups.end(), joint.display_group) !=
           groups.end();
  }
  virtual bool ArmSdkWeightUsesModePr() const { return false; }
  virtual float ArmSdkPeriodSeconds() const { return 0.02F; }
  virtual float ArmSdkWeightRatePerSecond() const { return 0.5F; }
  virtual bool SupportsTeachRemoteControl() const { return true; }
  virtual bool SupportsArmActions() const { return false; }
  virtual bool SupportsCustomArmActions() const { return false; }
  virtual bool SupportsArmActionInterrupts() const { return false; }
  virtual bool IsKnownArmAction(std::int32_t) const { return false; }
  virtual std::string MockArmActionList() const { return "[]"; }

  virtual std::int32_t InitializeArmActions(std::string& action_list) {
    action_list.clear();
    return -1;
  }
  virtual void ShutdownArmActions() {}
  virtual std::int32_t ExecuteArmAction(std::int32_t) { return -1; }
  virtual std::int32_t ExecuteArmAction(const std::string&) { return -1; }
  virtual std::int32_t StopCustomArmAction() { return -1; }
};

}  // namespace g1_web
