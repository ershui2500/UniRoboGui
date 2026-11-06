#include "g1_web/a2_locomotion_adapter.hpp"

#include <cmath>
#include <mutex>
#include <unitree/robot/a2/sport/sport_client.hpp>
#include "g1_web/snapshot_store.hpp"

namespace g1_web {
namespace {
class A2LocomotionAdapter final : public ILocomotion {
 public:
  explicit A2LocomotionAdapter(bool mock, A2SportCall call = {})
      : mock_(mock), call_(std::move(call)) {}

  bool Initialize(LocomotionInitialization& state, std::string& error) override {
    state = {};
    if (mock_) {
      state = {true, 3, 0};
    } else if (!call_) {
      client_ = std::make_unique<unitree::robot::a2::SportClient>();
      client_->SetTimeout(1.0F);
      client_->Init();
    }
    error.clear();
    return true;
  }
  void Shutdown() override {
    const std::lock_guard<std::mutex> lock(mutex_);
    client_.reset();
  }
  bool IsKnownModeCommand(const std::string& command, std::int32_t argument) const override {
    return argument == 0 && (command == "damp" || command == "stand_up" ||
        command == "stand_down" || command == "balance_stand" ||
        command == "stop_move" || command == "recovery_stand" ||
        command == "walk" || command == "run" || command == "climb" ||
        command == "left_side_gait_on" || command == "left_side_gait_off" ||
        command == "right_side_gait_on" || command == "right_side_gait_off" ||
        command == "hand_stand_on" || command == "hand_stand_off" ||
        command == "biped_stand_on" || command == "biped_stand_off" ||
        command == "front_flip" || command == "back_flip");
  }
  bool IsOperationalFsm(std::uint32_t fsm) const override {
    return fsm >= 3 && fsm <= 5;
  }
  std::string ValidateVelocity(const ControlData& control, float x, float y,
                              float yaw, std::int32_t speed) const override {
    if (speed != 0 && speed != 1) return "invalid_speed_mode";
    if (!IsOperationalFsm(control.fsm_id)) return "motion_fsm_not_allowed";
    const float limits[3][2][3] = {
        {{0.8F, 0.5F, 2.0F}, {1.5F, 0.5F, 2.5F}},
        {{2.5F, 0.3F, 1.5F}, {4.0F, 0.3F, 1.0F}},
        {{0.6F, 0.3F, 1.0F}, {0.6F, 0.3F, 1.0F}}};
    const auto& limit = limits[control.fsm_id - 3][speed];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        std::abs(x) > limit[0] || std::abs(y) > limit[1] ||
        std::abs(yaw) > limit[2] || (x == 0 && y == 0 && yaw == 0)) {
      return "velocity_out_of_range";
    }
    return {};
  }
  std::string ValidateVelocityExecution(const ControlData& control,
                                        std::int32_t speed) const override {
    if (!IsOperationalFsm(control.fsm_id)) return "motion_precondition_changed";
    return speed == 0 || speed == 1 ? "" : "invalid_speed_mode";
  }
  LocomotionCommandResult ExecuteMode(const std::string& command,
                                      std::int32_t argument) override {
    if (!IsKnownModeCommand(command, argument)) return {-1, -1, "unknown_or_disallowed_command"};
    int target = -1;
    if (command == "damp") target = 0;
    else if (command == "stand_down") target = 1;
    else if (command == "stand_up") target = 2;
    else if (command == "balance_stand" || command == "walk" || command == "recovery_stand") target = 3;
    else if (command == "run") target = 4;
    else if (command == "climb") target = 5;
    else if (command == "left_side_gait_on") target = 6;
    else if (command == "right_side_gait_on") target = 7;
    else if (command == "hand_stand_on") target = 8;
    else if (command == "biped_stand_on") target = 9;
    // StopMove does not promise a particular resulting FSM.
    const int gait = command == "run" ? 1 : command == "climb" ? 2 : 0;
    return Result(Call(command, gait), target);
  }
  LocomotionCommandResult ApplyVelocity(float x, float y, float yaw,
                                         std::int32_t speed, bool active) override {
    return ApplyVelocityGuarded(x, y, yaw, speed, active, [] { return std::string{}; });
  }
  LocomotionCommandResult ApplyVelocityGuarded(
      float x, float y, float yaw, std::int32_t speed, bool active,
      const std::function<std::string()>& before_move) override {
    if (!active) return Result(Call("stop_move"), -1);
    auto error = before_move();
    if (!error.empty()) return {-1, -1, error};
    if (speed != 0 && speed != 1) return {-1, -1, "invalid_speed_mode"};
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return {-1, -1, "velocity_out_of_range"};
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto result = CallUnlocked("speed_level", speed);
    if (result != 0) return Result(result, -1);
    // SpeedLevel is a synchronous RPC; freshness and the lease may change during it.
    error = before_move();
    if (!error.empty()) return {-1, -1, error};
    return Result(CallUnlocked("move", 0, x, y, yaw), -1);
  }
 private:
  LocomotionCommandResult Result(std::int32_t code, int target) const {
    return {code, target, code == 0 ? "" : "sdk_api_error"};
  }
  std::int32_t Call(const std::string& command, int argument = 0) {
    const std::lock_guard<std::mutex> lock(mutex_);
    return CallUnlocked(command, argument);
  }
  std::int32_t CallUnlocked(const std::string& command, int argument,
                           float x = 0, float y = 0, float yaw = 0) {
    if (mock_) return 0;
    if (call_) return call_(command, argument, x, y, yaw);
    if (!client_) return -1;
    if (command == "damp") return client_->Damp();
    if (command == "stand_up") return client_->StandUp();
    if (command == "stand_down") return client_->StandDown();
    if (command == "balance_stand") return client_->BalanceStand();
    if (command == "stop_move") return client_->StopMove();
    if (command == "recovery_stand") return client_->RecoveryStand();
    if (command == "walk" || command == "run" || command == "climb") return client_->SwitchGait(argument);
    if (command == "left_side_gait_on" || command == "left_side_gait_off") return client_->LeftSideGait(command == "left_side_gait_on" ? 1 : 0);
    if (command == "right_side_gait_on" || command == "right_side_gait_off") return client_->RightSideGait(command == "right_side_gait_on" ? 1 : 0);
    if (command == "hand_stand_on" || command == "hand_stand_off") return client_->HandStand(command == "hand_stand_on" ? 1 : 0);
    if (command == "biped_stand_on" || command == "biped_stand_off") return client_->BipedStand(command == "biped_stand_on" ? 1 : 0);
    if (command == "front_flip") return client_->FrontFlip();
    if (command == "back_flip") return client_->BackFlip();
    if (command == "speed_level") return client_->SpeedLevel(argument);
    if (command == "move") return client_->Move(x, y, yaw);
    return -1;
  }
  bool mock_;
  A2SportCall call_;
  std::mutex mutex_;
  std::unique_ptr<unitree::robot::a2::SportClient> client_;
};
}  // namespace
std::unique_ptr<ILocomotion> CreateA2LocomotionAdapter(bool mock) {
  return std::make_unique<A2LocomotionAdapter>(mock);
}
std::unique_ptr<ILocomotion> CreateA2LocomotionAdapterForTesting(A2SportCall call) {
  return std::make_unique<A2LocomotionAdapter>(false, std::move(call));
}
}  // namespace g1_web
