#include "g1_web/g1_locomotion_adapter.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <unordered_set>

#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include "g1_web/snapshot_store.hpp"

namespace g1_web {
namespace {

constexpr float kMaximumForwardSpeedMps = 3.0F;
constexpr float kMaximumLateralSpeedMps = 1.0F;
constexpr float kMaximumYawSpeedRadS = 1.5F;
constexpr float kVelocityLeaseSeconds = 0.25F;

const std::unordered_set<std::int32_t>& AllowedFsmIds() {
  static const std::unordered_set<std::int32_t> ids{
      0, 1, 2, 3, 4, 500, 501, 702, 706, 801, 802};
  return ids;
}

bool IsLocomotionFsm(std::uint32_t fsm_id) {
  return fsm_id == 500 || fsm_id == 501 || fsm_id == 801 ||
         fsm_id == 802;
}

bool IsWalkRunFsm(std::uint32_t fsm_id) {
  return fsm_id == 801 || fsm_id == 802;
}

bool IsAllowedSpeedMode(std::int32_t speed_mode) {
  return speed_mode == 0 || speed_mode == 1 || speed_mode == 3;
}

struct MotionLimits {
  float forward_m_s;
  float lateral_m_s;
  float yaw_rad_s;
};

MotionLimits MotionLimitsForMode(std::int32_t speed_mode,
                                 std::uint32_t fsm_id) {
  if (speed_mode == 3) return {3.0F, 1.0F, 1.5F};
  if (IsWalkRunFsm(fsm_id)) {
    if (speed_mode == 1) return {1.0F, 0.6F, 1.3F};
    return {0.5F, 0.4F, 1.1F};
  }
  if (speed_mode == 1) return {1.0F, 0.35F, 0.8F};
  return {0.5F, 0.2F, 0.5F};
}

bool IsSpeedModeAllowedForFsm(std::int32_t speed_mode,
                              std::uint32_t fsm_id) {
  if (IsWalkRunFsm(fsm_id)) return IsAllowedSpeedMode(speed_mode);
  return (fsm_id == 500 || fsm_id == 501) &&
         (speed_mode == 0 || speed_mode == 1);
}

class G1LocomotionAdapter final : public ILocomotion {
 public:
  explicit G1LocomotionAdapter(bool mock) : mock_(mock) {}

  bool Initialize(LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    applied_speed_mode_ = -1;
    if (mock_) {
      error.clear();
      return true;
    }

    try {
      command_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
      command_client_->SetTimeout(10.0F);
      command_client_->Init();

      motion_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
      motion_client_->SetTimeout(1.0F);
      motion_client_->Init();

      int fsm_id = 0;
      int fsm_mode = 0;
      if (command_client_->GetFsmId(fsm_id) == 0 &&
          command_client_->GetFsmMode(fsm_mode) == 0) {
        state.state_valid = true;
        state.fsm_id = static_cast<std::uint32_t>(fsm_id);
        state.fsm_mode = static_cast<std::uint32_t>(fsm_mode);
      }
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "Unknown locomotion initialization error";
    }
    Shutdown();
    return false;
  }

  void Shutdown() override {
    motion_client_.reset();
    command_client_.reset();
    applied_speed_mode_ = -1;
  }

  bool IsKnownModeCommand(const std::string& command,
                          std::int32_t argument) const override {
    if (command == "set_fsm_id") {
      return AllowedFsmIds().count(argument) != 0;
    }
    return command == "damp" || command == "zero_torque" ||
           command == "start" || command == "squat" ||
           command == "sit" || command == "stand_up" ||
           command == "balance_stand" || command == "stop_move";
  }

  bool IsOperationalFsm(std::uint32_t fsm_id) const override {
    return IsLocomotionFsm(fsm_id);
  }

  std::string ValidateVelocity(const ControlData& control,
                               float vx_m_s, float vy_m_s,
                               float vyaw_rad_s,
                               std::int32_t speed_mode) const override {
    if (!IsAllowedSpeedMode(speed_mode)) return "invalid_speed_mode";

    const auto limits = MotionLimitsForMode(speed_mode, control.fsm_id);
    if (std::abs(vx_m_s) > kMaximumForwardSpeedMps ||
        std::abs(vx_m_s) > limits.forward_m_s ||
        std::abs(vy_m_s) > kMaximumLateralSpeedMps ||
        std::abs(vy_m_s) > limits.lateral_m_s ||
        std::abs(vyaw_rad_s) > kMaximumYawSpeedRadS ||
        std::abs(vyaw_rad_s) > limits.yaw_rad_s ||
        (std::abs(vx_m_s) < 0.001F && std::abs(vy_m_s) < 0.001F &&
         std::abs(vyaw_rad_s) < 0.001F)) {
      return "velocity_out_of_range";
    }
    if (!IsLocomotionFsm(control.fsm_id)) return "motion_fsm_not_allowed";
    if (!IsSpeedModeAllowedForFsm(speed_mode, control.fsm_id)) {
      return "speed_mode_requires_walkrun";
    }
    return {};
  }

  std::string ValidateVelocityExecution(
      const ControlData& control, std::int32_t speed_mode) const override {
    if (!IsLocomotionFsm(control.fsm_id)) return "motion_precondition_changed";
    if (!IsSpeedModeAllowedForFsm(speed_mode, control.fsm_id)) {
      return "speed_mode_requires_walkrun";
    }
    return {};
  }

  LocomotionCommandResult ExecuteMode(
      const std::string& command, std::int32_t argument) override {
    LocomotionCommandResult result;
    if (!IsKnownModeCommand(command, argument)) {
      result.error = "unknown_or_disallowed_command";
      return result;
    }

    if (command == "damp") result.target_fsm = 1;
    else if (command == "zero_torque") result.target_fsm = 0;
    else if (command == "start") result.target_fsm = 500;
    else if (command == "squat") result.target_fsm = 2;
    else if (command == "sit") result.target_fsm = 3;
    else if (command == "stand_up") result.target_fsm = 4;
    else if (command == "set_fsm_id") result.target_fsm = argument;

    if (mock_) {
      result.api_result = 0;
      return result;
    }

    try {
      if (command == "damp") result.api_result = command_client_->Damp();
      else if (command == "zero_torque") result.api_result = command_client_->ZeroTorque();
      else if (command == "start") result.api_result = command_client_->Start();
      else if (command == "squat") result.api_result = command_client_->Squat();
      else if (command == "sit") result.api_result = command_client_->Sit();
      else if (command == "stand_up") result.api_result = command_client_->StandUp();
      else if (command == "balance_stand") result.api_result = command_client_->BalanceStand();
      else if (command == "stop_move") result.api_result = command_client_->StopMove();
      else if (command == "set_fsm_id") result.api_result = command_client_->SetFsmId(argument);
    } catch (const std::exception& exception) {
      result.api_result = -1;
      result.error = exception.what();
      return result;
    } catch (...) {
      result.api_result = -1;
      result.error = "unknown_control_exception";
      return result;
    }
    if (result.api_result != 0) result.error = "sdk_api_error";
    return result;
  }

  LocomotionCommandResult ApplyVelocity(
      float vx_m_s, float vy_m_s, float vyaw_rad_s,
      std::int32_t speed_mode, bool active) override {
    LocomotionCommandResult result;
    if (mock_) {
      applied_speed_mode_ = speed_mode;
      result.api_result = 0;
      return result;
    }

    try {
      if (!active) {
        result.api_result = motion_client_->StopMove();
      } else {
        result.api_result = 0;
        if (applied_speed_mode_ != speed_mode) {
          result.api_result = motion_client_->SetSpeedMode(speed_mode);
          if (result.api_result == 0) applied_speed_mode_ = speed_mode;
        }
        if (result.api_result == 0) {
          result.api_result = motion_client_->SetVelocity(
              vx_m_s, vy_m_s, vyaw_rad_s, kVelocityLeaseSeconds);
        }
      }
    } catch (const std::exception& exception) {
      result.api_result = -1;
      result.error = exception.what();
      return result;
    } catch (...) {
      result.api_result = -1;
      result.error = "unknown_motion_exception";
      return result;
    }
    if (result.api_result != 0) result.error = "sdk_api_error";
    return result;
  }

 private:
  bool mock_{false};
  std::int32_t applied_speed_mode_{-1};
  std::unique_ptr<unitree::robot::g1::LocoClient> command_client_;
  std::unique_ptr<unitree::robot::g1::LocoClient> motion_client_;
};

}  // namespace

std::unique_ptr<ILocomotion> CreateG1LocomotionAdapter(bool mock) {
  return std::make_unique<G1LocomotionAdapter>(mock);
}

}  // namespace g1_web
