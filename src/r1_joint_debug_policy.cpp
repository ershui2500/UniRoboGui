#include "g1_web/r1_joint_debug_policy.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <unitree/robot/client/client.hpp>

namespace g1_web {
namespace {

constexpr char kR1ArmActionService[] = "arm";
constexpr char kR1ArmActionApiVersion[] = "1.0.0.0";
constexpr std::int32_t kR1ExecuteArmActionApi = 7106;
constexpr std::int32_t kR1GetArmActionListApi = 7107;
constexpr std::int32_t kR1ExecuteCustomArmActionApi = 7108;
constexpr std::int32_t kR1StopCustomArmActionApi = 7113;

class R1ArmActionClient final : public unitree::robot::Client {
 public:
  R1ArmActionClient() : Client(kR1ArmActionService, false) {}

  void Init() {
    SetApiVersion(kR1ArmActionApiVersion);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kR1ExecuteArmActionApi);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kR1GetArmActionListApi);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kR1ExecuteCustomArmActionApi);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(kR1StopCustomArmActionApi);
  }

  std::int32_t GetActionList(std::string& data) {
    std::string parameter;
    return Call(kR1GetArmActionListApi, parameter, data);
  }

  std::int32_t ExecuteAction(std::int32_t action_id) {
    std::string data;
    const auto parameter =
        R"({"action_id":)" + std::to_string(action_id) + "}";
    return Call(kR1ExecuteArmActionApi, parameter, data);
  }

  std::int32_t ExecuteAction(const std::string& action_name) {
    std::string data;
    const auto parameter =
        R"({"action_name":")" + action_name + R"("})";
    return Call(kR1ExecuteCustomArmActionApi, parameter, data);
  }

  std::int32_t StopCustomAction() {
    std::string parameter;
    std::string data;
    return Call(kR1StopCustomArmActionApi, parameter, data);
  }
};

class R1JointDebugPolicy final : public IJointDebugPolicy {
 public:
  explicit R1JointDebugPolicy(const RobotProfile& profile) : profile_(profile) {}

  const RobotProfile& profile() const override { return profile_; }

  std::vector<std::string> GroupsForMode(
      const std::string& mode) const override {
    if (mode == "upper_body")
      return {"waist", "head", "left_arm", "right_arm"};
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
    return {"waist", "head", "left_arm", "right_arm"};
  }

  JointDebugTransport TransportForMode(const std::string& mode) const override {
    return mode == "full_body" ? JointDebugTransport::kLowCmd
                               : JointDebugTransport::kArmSdk;
  }

  std::optional<std::size_t> ArmSdkWeightMotorSlot() const override {
    return std::nullopt;
  }

  JointDebugGain NormalGain(const JointDescriptor& joint) const override {
    const auto slot = joint.motor_slot;
    if (slot <= 11) return {200.0F, 3.0F, false};
    if (slot == 12 || slot == 13) return {300.0F, 5.0F, false};
    if (slot == 19 || slot == 26) return {50.0F, 2.0F, false};
    if ((slot >= 15 && slot <= 18) || (slot >= 22 && slot <= 25))
      return {100.0F, 2.0F, false};
    if (slot == 29) return {50.0F, 2.0F, false};
    if (slot == 30) return {10.0F, 0.1F, false};
    return {};
  }

  JointDebugGain RecordingGain(const JointDescriptor& joint) const override {
    return NormalGain(joint);
  }

  JointDebugGain NormalGainForMode(
      const JointDescriptor& joint, const std::string& mode) const override {
    if (mode == "full_body") return NormalGain(joint);
    switch (joint.motor_slot) {
      case 15:
      case 16:
      case 22:
      case 23:
        return {50.0F, 2.0F, false};
      case 17:
      case 18:
      case 24:
      case 25:
        return {40.0F, 2.0F, false};
      case 19:
      case 26:
        return {30.0F, 2.0F, false};
      case 13:
        return {50.0F, 3.0F, false};
      case 29:
      case 30:
        return {15.0F, 1.0F, false};
      default:
        return NormalGain(joint);
    }
  }

  JointDebugGain RecordingGainForMode(
      const JointDescriptor& joint, const std::string& mode) const override {
    if (mode == "upper_body" && joint.motor_slot == 30)
      return {0.0F, 0.1F, false};
    if (mode == "upper_body" &&
        (joint.motor_slot == 19 || joint.motor_slot == 26))
      return {0.0F, 0.5F, false};
    if (mode == "upper_body" && joint.motor_slot == 13)
      return {0.0F, 1.0F, false};
    if (mode == "upper_body" &&
        (joint.display_group == "head" ||
         joint.display_group == "left_arm" ||
         joint.display_group == "right_arm")) {
      return {0.0F, 1.0F, false};
    }
    return NormalGainForMode(joint, mode);
  }

  bool UpperBodyFsmAllowed(std::uint32_t fsm_id) const override {
    return fsm_id == 811;
  }

  bool UpperBodyActiveFsmAllowed(std::uint32_t fsm_id) const override {
    return fsm_id == 811 || fsm_id == 816;
  }

  bool ArmSdkJoint(const JointDescriptor& joint) const override {
    const auto slot = joint.motor_slot;
    return (slot >= 15 && slot <= 19) ||
           (slot >= 22 && slot <= 26) ||
           slot == 13 || slot == 29 || slot == 30;
  }

  bool ArmSdkWeightUsesModePr() const override { return true; }
  float ArmSdkPeriodSeconds() const override { return 0.01F; }
  float ArmSdkWeightRatePerSecond() const override { return 1.0F; }
  bool SupportsTeachRemoteControl() const override { return true; }

  bool SupportsArmActions() const override { return true; }
  bool SupportsCustomArmActions() const override { return true; }
  bool SupportsArmActionInterrupts() const override { return true; }

  bool IsKnownArmAction(std::int32_t action_id) const override {
    switch (action_id) {
      case 11:
      case 12:
      case 13:
      case 15:
      case 17:
      case 18:
      case 19:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
      case 28:
      case 29:
      case 30:
      case 31:
      case 33:
      case 34:
      case 35:
      case 36:
      case 99:
        return true;
      default:
        return false;
    }
  }

  std::string MockArmActionList() const override {
    return R"([[{"id":99,"name":"release_arm"},{"id":11,"name":"blow_kiss_with_both_hands"},{"id":12,"name":"blow_kiss_with_left_hand"},{"id":13,"name":"blow_kiss_with_right_hand"},{"id":15,"name":"both_hands_up"},{"id":17,"name":"clamp"},{"id":18,"name":"high_five"},{"id":19,"name":"hug"},{"id":22,"name":"refuse"},{"id":23,"name":"right_hand_up"},{"id":24,"name":"ultraman_ray"},{"id":25,"name":"wave_under_head"},{"id":26,"name":"wave_above_head"},{"id":27,"name":"shake_hand"},{"id":28,"name":"box_left_hand_win"},{"id":29,"name":"box_right_hand_win"},{"id":30,"name":"box_both_hand_win"},{"id":31,"name":"extend_right_arm_forward"},{"id":33,"name":"right_hand_on_heart"},{"id":34,"name":"both_hands_up_deviate_right"},{"id":35,"name":"emphasize"},{"id":36,"name":"forward_push"}],[{"name":"r1_demo","time":3.0}]])";
  }

  std::int32_t InitializeArmActions(std::string& action_list) override {
    arm_action_client_ = std::make_unique<R1ArmActionClient>();
    arm_action_client_->SetTimeout(10.0F);
    arm_action_client_->Init();
    arm_interrupt_client_ = std::make_unique<R1ArmActionClient>();
    arm_interrupt_client_->SetTimeout(10.0F);
    arm_interrupt_client_->Init();
    return arm_action_client_->GetActionList(action_list);
  }

  void ShutdownArmActions() override {
    arm_interrupt_client_.reset();
    arm_action_client_.reset();
  }

  std::int32_t ExecuteArmAction(std::int32_t action_id) override {
    if (action_id == 99) {
      return arm_interrupt_client_
                 ? arm_interrupt_client_->ExecuteAction(action_id)
                 : -1;
    }
    return arm_action_client_ ? arm_action_client_->ExecuteAction(action_id)
                              : -1;
  }

  std::int32_t ExecuteArmAction(const std::string& action_name) override {
    return arm_action_client_ ? arm_action_client_->ExecuteAction(action_name)
                              : -1;
  }

  std::int32_t StopCustomArmAction() override {
    return arm_interrupt_client_ ? arm_interrupt_client_->StopCustomAction()
                                 : -1;
  }

  const RobotModelVariant* ResolveModelVariant(
      std::uint32_t mode_machine) const override {
    return FindModelVariant(profile_, mode_machine);
  }

  bool LegacyTeachActionCompatible(std::uint32_t,
                                   std::size_t) const override {
    return false;
  }

 private:
  const RobotProfile& profile_;
  std::unique_ptr<R1ArmActionClient> arm_action_client_;
  std::unique_ptr<R1ArmActionClient> arm_interrupt_client_;
};

}  // namespace

std::unique_ptr<IJointDebugPolicy> CreateR1JointDebugPolicy(
    const RobotProfile& profile) {
  return std::make_unique<R1JointDebugPolicy>(profile);
}

}  // namespace g1_web
