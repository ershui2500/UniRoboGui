#include "g1_web/g1_joint_debug_policy.hpp"

#include <algorithm>
#include <initializer_list>
#include <utility>

#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>

namespace g1_web {
namespace {

bool Contains(std::initializer_list<std::size_t> slots, std::size_t slot) {
  return std::find(slots.begin(), slots.end(), slot) != slots.end();
}

class G1JointDebugPolicy final : public IJointDebugPolicy {
 public:
  explicit G1JointDebugPolicy(const RobotProfile& profile) : profile_(profile) {}

  const RobotProfile& profile() const override { return profile_; }

  std::vector<std::string> GroupsForMode(
      const std::string& mode) const override {
    if (mode == "upper_body") return {"waist", "left_arm", "right_arm"};
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
    return GroupsForMode("upper_body");
  }

  JointDebugTransport TransportForMode(
      const std::string& mode) const override {
    return mode == "full_body" ? JointDebugTransport::kLowCmd
                               : JointDebugTransport::kArmSdk;
  }

  std::optional<std::size_t> ArmSdkWeightMotorSlot() const override {
    return 29;
  }

  JointDebugGain NormalGain(const JointDescriptor& joint) const override {
    const auto slot = joint.motor_slot;
    if (Contains({19, 20, 21, 26, 27, 28}, slot)) return {40.0F, 1.5F, false};
    if (Contains({4, 10, 15, 16, 17, 18, 22, 23, 24, 25}, slot))
      return {80.0F, 3.0F, false};
    return {300.0F, 3.0F, false};
  }

  JointDebugGain RecordingGain(const JointDescriptor& joint) const override {
    const auto slot = joint.motor_slot;
    if (slot == 12) return {0.0F, 10.0F, false};
    if (slot == 13 || slot == 14) {
      auto gain = NormalGain(joint);
      gain.hold_record_start_pose = true;
      return gain;
    }
    if (Contains({19, 20, 21, 26, 27, 28}, slot)) return {0.0F, 0.5F, false};
    return {0.0F, 1.5F, false};
  }

  bool UpperBodyFsmAllowed(std::uint32_t fsm_id) const override {
    return fsm_id == 500 || fsm_id == 501 || fsm_id == 801 || fsm_id == 802;
  }

  const RobotModelVariant* ResolveModelVariant(
      std::uint32_t mode_machine) const override {
    return FindModelVariant(profile_, mode_machine);
  }

  bool SupportsArmActions() const override { return true; }
  bool SupportsCustomArmActions() const override { return true; }

  bool IsKnownArmAction(std::int32_t action_id) const override {
    switch (action_id) {
      case 11:
      case 12:
      case 15:
      case 17:
      case 18:
      case 19:
      case 20:
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
      case 99:
        return true;
      default:
        return false;
    }
  }

  std::string MockArmActionList() const override {
    return R"([[{"id":99,"name":"release_arm"},{"id":11,"name":"blow_kiss_with_both_hands"},{"id":17,"name":"clamp"},{"id":18,"name":"high_five"},{"id":19,"name":"hug"},{"id":22,"name":"refuse"},{"id":25,"name":"wave_under_head"},{"id":26,"name":"wave_above_head"},{"id":27,"name":"shake_hand"}],[{"name":"Waist_Drum_Dance","time":9.5},{"name":"Spin_discs","time":6.9},{"name":"Scratch_head","time":8.1}]])";
  }

  std::int32_t InitializeArmActions(std::string& action_list) override {
    arm_action_client_ =
        std::make_unique<unitree::robot::g1::G1ArmActionClient>();
    arm_action_client_->SetTimeout(10.0F);
    arm_action_client_->Init();
    return arm_action_client_->GetActionList(action_list);
  }

  void ShutdownArmActions() override { arm_action_client_.reset(); }

  std::int32_t ExecuteArmAction(std::int32_t action_id) override {
    return arm_action_client_ ? arm_action_client_->ExecuteAction(action_id)
                              : -1;
  }

  std::int32_t ExecuteArmAction(const std::string& action_name) override {
    return arm_action_client_ ? arm_action_client_->ExecuteAction(action_name)
                              : -1;
  }

  std::int32_t StopCustomArmAction() override {
    return arm_action_client_ ? arm_action_client_->StopCustomAction() : -1;
  }

  bool LegacyTeachActionCompatible(std::uint32_t mode_machine,
                                   std::size_t frame_width) const override {
    if (profile_.identity.product_id != "g1" ||
        profile_.identity.variant != "29dof" ||
        ResolveModelVariant(mode_machine) == nullptr) {
      return false;
    }
    const auto groups = TeachGroups();
    const auto joint_count = std::count_if(
        profile_.joint_schema.joints.begin(), profile_.joint_schema.joints.end(),
        [&](const auto& joint) {
          return std::find(groups.begin(), groups.end(), joint.display_group) !=
                 groups.end();
        });
    return frame_width == static_cast<std::size_t>(joint_count);
  }

 private:
  const RobotProfile& profile_;
  std::unique_ptr<unitree::robot::g1::G1ArmActionClient> arm_action_client_;
};

}  // namespace

std::unique_ptr<IJointDebugPolicy> CreateG1JointDebugPolicy(
    const RobotProfile& profile) {
  return std::make_unique<G1JointDebugPolicy>(profile);
}

}  // namespace g1_web
