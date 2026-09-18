#include "g1_web/r1_locomotion_adapter.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>

#include <unitree/robot/r1/loco/r1_loco_client.hpp>

#include "g1_web/snapshot_store.hpp"

namespace g1_web {
namespace {

constexpr float kMaximumForwardSpeedMps = 1.0F;
constexpr float kMaximumLateralSpeedMps = 0.6F;
constexpr float kMaximumYawSpeedRadS = 1.2F;
constexpr float kVelocityLeaseSeconds = 1.0F;

bool IsAllowedSpeedMode(std::int32_t speed_mode) {
  return speed_mode == 0 || speed_mode == 1 || speed_mode == 3;
}

struct MotionLimits {
  float forward_m_s;
  float lateral_m_s;
  float yaw_rad_s;
};

MotionLimits MotionLimitsForMode(std::int32_t speed_mode) {
  if (speed_mode == 3) return {1.0F, 0.6F, 1.2F};
  if (speed_mode == 1) return {0.5F, 0.5F, 1.0F};
  return {0.4F, 0.4F, 0.9F};
}

class R1LocomotionAdapter final : public ILocomotion {
 public:
  explicit R1LocomotionAdapter(bool mock) : mock_(mock) {}

  bool Initialize(LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    if (mock_) {
      state.state_valid = true;
      state.fsm_id = 811;
      state.fsm_mode = 0;
      error.clear();
      return true;
    }
    try {
      client_ = std::make_unique<unitree::robot::r1::LocoClient>();
      client_->Init();
      client_->SetTimeout(10.0F);
      std::string state_error;
      QueryState(state, state_error);
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "unknown_r1_locomotion_initialization_error";
    }
    Shutdown();
    return false;
  }

  bool QueryState(LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    if (mock_) {
      state.state_valid = true;
      state.fsm_id = 811;
      state.fsm_mode = 0;
      error.clear();
      return true;
    }
    if (!client_) {
      error = "r1_state_client_not_initialized";
      return false;
    }
    try {
      const std::lock_guard<std::mutex> lock(client_mutex_);
      int fsm_id = 0;
      if (client_->GetFsmId(fsm_id) != 0) {
        error = "sdk_api_error";
        return false;
      }
      int fsm_mode = -1;
      const auto fsm_mode_result = client_->GetFsmMode(fsm_mode);
      state.state_valid = true;
      state.fsm_id = static_cast<std::uint32_t>(fsm_id);
      state.fsm_mode = fsm_mode_result == 0 && fsm_mode >= 0
                           ? static_cast<std::uint32_t>(fsm_mode)
                           : std::numeric_limits<std::uint32_t>::max();
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "unknown_r1_state_query_error";
    }
    return false;
  }

  void Shutdown() override {
    const std::lock_guard<std::mutex> lock(client_mutex_);
    client_.reset();
  }

  bool IsKnownModeCommand(const std::string& command,
                          std::int32_t) const override {
    return command == "damp" || command == "zero_torque" ||
           command == "start" || command == "stand_up" ||
           command == "lie_to_stand" || command == "stand_to_lie" ||
           command == "stop_move";
  }

  bool IsOperationalFsm(std::uint32_t fsm_id) const override {
    return fsm_id == 811;
  }

  std::string ValidateVelocity(const ControlData& control,
                               float vx_m_s, float vy_m_s,
                               float vyaw_rad_s,
                               std::int32_t speed_mode) const override {
    if (!IsAllowedSpeedMode(speed_mode)) return "invalid_speed_mode";
    const auto limits = MotionLimitsForMode(speed_mode);
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
    return IsOperationalFsm(control.fsm_id) ? std::string{}
                                            : "motion_fsm_not_allowed";
  }

  std::string ValidateVelocityExecution(
      const ControlData& control, std::int32_t speed_mode) const override {
    if (!IsOperationalFsm(control.fsm_id)) {
      return "motion_precondition_changed";
    }
    return IsAllowedSpeedMode(speed_mode) ? std::string{}
                                          : "invalid_speed_mode";
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
    else if (command == "start") result.target_fsm = 811;
    else if (command == "stand_up") result.target_fsm = 4;
    else if (command == "lie_to_stand") result.target_fsm = 701;
    else if (command == "stand_to_lie") result.target_fsm = 702;

    if (mock_) {
      result.api_result = 0;
      return result;
    }
    try {
      const std::lock_guard<std::mutex> lock(client_mutex_);
      if (command == "damp") result.api_result = client_->Damp();
      else if (command == "zero_torque") result.api_result = client_->ZeroTorque();
      else if (command == "start") result.api_result = client_->Start();
      else if (command == "stand_up") result.api_result = client_->StandUp();
      else if (command == "lie_to_stand") result.api_result = client_->SetFsmId(701);
      else if (command == "stand_to_lie") result.api_result = client_->SetFsmId(702);
      else if (command == "stop_move") result.api_result = client_->StopMove();
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
      std::int32_t, bool active) override {
    LocomotionCommandResult result;
    if (mock_) {
      result.api_result = 0;
      return result;
    }
    try {
      const std::lock_guard<std::mutex> lock(client_mutex_);
      if (!active) {
        result.api_result = client_->StopMove();
      } else {
        result.api_result = client_->SetVelocity(
            vx_m_s, vy_m_s, vyaw_rad_s, kVelocityLeaseSeconds);
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
    if (result.api_result == 127) result.api_result = 0;
    if (result.api_result != 0) {
      result.error = "sdk_api_error";
    }
    return result;
  }

 private:
  bool mock_{false};
  std::mutex client_mutex_;
  std::unique_ptr<unitree::robot::r1::LocoClient> client_;
};

}  // namespace

std::unique_ptr<ILocomotion> CreateR1LocomotionAdapter(bool mock) {
  return std::make_unique<R1LocomotionAdapter>(mock);
}

}  // namespace g1_web
