#include "g1_web/go2_locomotion_adapter.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <mutex>

#include <unitree/robot/go2/sport/sport_client.hpp>

#include "g1_web/snapshot_store.hpp"

namespace g1_web {
namespace {

constexpr float kMaximumForwardSpeedMps = 0.5F;
constexpr float kMaximumLateralSpeedMps = 0.3F;
constexpr float kMaximumYawSpeedRadS = 0.6F;

bool IsAllowedSpeedMode(std::int32_t speed_mode) {
  return speed_mode == 0 || speed_mode == 1 || speed_mode == 3;
}

float ForwardLimit(std::int32_t speed_mode) {
  if (speed_mode == 0) return 0.2F;
  if (speed_mode == 1) return 0.35F;
  return kMaximumForwardSpeedMps;
}

float LateralLimit(std::int32_t speed_mode) {
  if (speed_mode == 0) return 0.12F;
  if (speed_mode == 1) return 0.2F;
  return kMaximumLateralSpeedMps;
}

float YawLimit(std::int32_t speed_mode) {
  if (speed_mode == 0) return 0.25F;
  if (speed_mode == 1) return 0.4F;
  return kMaximumYawSpeedRadS;
}

std::string Go2ApiError(std::int32_t result) {
  switch (result) {
    case 3104:
      return "go2_dds_timeout";
    case 4101:
      return "go2_trajectory_point_error";
    case 4201:
      return "go2_action_timeout";
    case 4205:
      return "go2_state_machine_not_ready";
    case 4206:
      return "go2_pose_not_suitable";
    default:
      return "sdk_api_error";
  }
}

std::int32_t TargetFsm(const std::string& command) {
  if (command == "damp") return 1001;
  if (command == "stand_up") return 1002;
  if (command == "stand_down") return 1004;
  if (command == "balance_stand" || command == "recovery_stand" ||
      command == "rise_sit") {
    return 1013;
  }
  if (command == "sit") return 1007;
  if (command == "free_walk") return 100;
  if (command == "static_walk") return 1015;
  if (command == "trot_run") return 1016;
  if (command == "economic_gait") return 1017;
  if (command == "pose_on") return 1091;
  if (command == "classic_walk_on") return 2010;
  if (command == "free_avoid_on") return 2007;
  if (command == "free_bound_on") return 2008;
  if (command == "free_jump_on") return 2009;
  if (command == "hand_stand_on") return 2011;
  if (command == "cross_step_on") return 2016;
  if (command == "walk_upright_on") return 2017;
  if (command == "classic_walk_off" || command == "free_avoid_off" ||
      command == "free_bound_off" ||
      command == "free_jump_off" || command == "hand_stand_off" ||
      command == "cross_step_off" || command == "walk_upright_off") {
    return 100;
  }
  return -1;
}

class Go2LocomotionAdapter final : public ILocomotion {
 public:
  explicit Go2LocomotionAdapter(bool mock) : mock_(mock) {}

  bool Initialize(LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    if (mock_) {
      state.state_valid = true;
      state.fsm_id = 1013;
      state.fsm_mode = 0;
      error.clear();
      return true;
    }
    try {
      auto client = std::make_unique<unitree::robot::go2::SportClient>();
      client->SetTimeout(10.0F);
      client->Init();
      client->WaitLeaseApplied();
      const std::lock_guard<std::mutex> lock(client_mutex_);
      client_ = std::move(client);
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "unknown_go2_locomotion_initialization_error";
    }
    Shutdown();
    return false;
  }

  void Shutdown() override {
    const std::lock_guard<std::mutex> lock(client_mutex_);
    client_.reset();
  }

  bool IsKnownModeCommand(const std::string& command,
                          std::int32_t) const override {
    return command == "damp" || command == "balance_stand" ||
           command == "stop_move" || command == "stand_up" ||
           command == "stand_down" || command == "recovery_stand" ||
           command == "sit" || command == "rise_sit" ||
           command == "free_walk" || command == "static_walk" ||
           command == "trot_run" || command == "economic_gait" ||
           command == "classic_walk_on" || command == "classic_walk_off" ||
           command == "free_avoid_on" || command == "free_avoid_off" ||
           command == "free_bound_on" || command == "free_bound_off" ||
           command == "free_jump_on" || command == "free_jump_off" ||
           command == "hand_stand_on" || command == "hand_stand_off" ||
           command == "walk_upright_on" || command == "walk_upright_off" ||
           command == "cross_step_on" || command == "cross_step_off" ||
           command == "pose_on" || command == "pose_off" ||
           command == "hello" || command == "stretch" ||
           command == "content" || command == "heart" ||
           command == "scrape" || command == "dance1" ||
           command == "dance2" || command == "front_jump" ||
           command == "front_pounce" || command == "front_flip" ||
           command == "left_flip" || command == "back_flip";
  }

  bool IsOperationalFsm(std::uint32_t fsm_id) const override {
    switch (fsm_id) {
      case 100:
      case 1013:
      case 1015:
      case 1016:
      case 1017:
      case 2007:
      case 2008:
      case 2009:
      case 2010:
      case 2011:
      case 2016:
      case 2017:
        return true;
      default:
        return false;
    }
  }

  std::string ValidateVelocity(const ControlData& control,
                               float vx_m_s, float vy_m_s,
                               float vyaw_rad_s,
                               std::int32_t speed_mode) const override {
    if (!IsAllowedSpeedMode(speed_mode)) return "invalid_speed_mode";
    if (std::abs(vx_m_s) > ForwardLimit(speed_mode) ||
        std::abs(vy_m_s) > LateralLimit(speed_mode) ||
        std::abs(vyaw_rad_s) > YawLimit(speed_mode) ||
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
    result.target_fsm = TargetFsm(command);
    if (mock_) {
      result.api_result = 0;
      return result;
    }
    try {
      const std::lock_guard<std::mutex> lock(client_mutex_);
      if (!client_) {
        result.error = "go2_sport_client_not_initialized";
        return result;
      }
      if (command == "damp") result.api_result = client_->Damp();
      else if (command == "balance_stand") {
        result.api_result = client_->BalanceStand();
      } else if (command == "stop_move") {
        result.api_result = client_->StopMove();
      } else if (command == "stand_up") {
        result.api_result = client_->StandUp();
      } else if (command == "stand_down") {
        result.api_result = client_->StandDown();
      } else if (command == "recovery_stand") {
        result.api_result = client_->RecoveryStand();
      } else if (command == "sit") {
        result.api_result = client_->Sit();
      } else if (command == "rise_sit") {
        result.api_result = client_->RiseSit();
      } else if (command == "free_walk") {
        result.api_result = client_->FreeWalk();
      } else if (command == "static_walk") {
        result.api_result = client_->StaticWalk();
      } else if (command == "trot_run") {
        result.api_result = client_->TrotRun();
      } else if (command == "economic_gait") {
        result.api_result = client_->EconomicGait();
      } else if (command == "classic_walk_on") {
        result.api_result = client_->ClassicWalk(true);
      } else if (command == "classic_walk_off") {
        result.api_result = client_->ClassicWalk(false);
      } else if (command == "free_avoid_on") {
        result.api_result = client_->FreeAvoid(true);
      } else if (command == "free_avoid_off") {
        result.api_result = client_->FreeAvoid(false);
      } else if (command == "free_bound_on") {
        result.api_result = client_->FreeBound(true);
      } else if (command == "free_bound_off") {
        result.api_result = client_->FreeBound(false);
      } else if (command == "free_jump_on") {
        result.api_result = client_->FreeJump(true);
      } else if (command == "free_jump_off") {
        result.api_result = client_->FreeJump(false);
      } else if (command == "hand_stand_on") {
        result.api_result = client_->HandStand(true);
      } else if (command == "hand_stand_off") {
        result.api_result = client_->HandStand(false);
      } else if (command == "walk_upright_on") {
        result.api_result = client_->WalkUpright(true);
      } else if (command == "walk_upright_off") {
        result.api_result = client_->WalkUpright(false);
      } else if (command == "cross_step_on") {
        result.api_result = client_->CrossStep(true);
      } else if (command == "cross_step_off") {
        result.api_result = client_->CrossStep(false);
      } else if (command == "pose_on") {
        result.api_result = client_->Pose(true);
      } else if (command == "pose_off") {
        result.api_result = client_->Pose(false);
      } else if (command == "hello") {
        result.api_result = client_->Hello();
      } else if (command == "stretch") {
        result.api_result = client_->Stretch();
      } else if (command == "content") {
        result.api_result = client_->Content();
      } else if (command == "heart") {
        result.api_result = client_->Heart();
      } else if (command == "scrape") {
        result.api_result = client_->Scrape();
      } else if (command == "dance1") {
        result.api_result = client_->Dance1();
      } else if (command == "dance2") {
        result.api_result = client_->Dance2();
      } else if (command == "front_jump") {
        result.api_result = client_->FrontJump();
      } else if (command == "front_pounce") {
        result.api_result = client_->FrontPounce();
      } else if (command == "front_flip") {
        result.api_result = client_->FrontFlip();
      } else if (command == "left_flip") {
        result.api_result = client_->LeftFlip();
      } else if (command == "back_flip") {
        result.api_result = client_->BackFlip();
      }
    } catch (const std::exception& exception) {
      result.api_result = -1;
      result.error = exception.what();
      return result;
    } catch (...) {
      result.api_result = -1;
      result.error = "unknown_go2_control_exception";
      return result;
    }
    if (result.api_result != 0) {
      result.error = Go2ApiError(result.api_result);
    }
    return result;
  }

  LocomotionCommandResult ApplyVelocity(
      float vx_m_s, float vy_m_s, float vyaw_rad_s,
      std::int32_t speed_mode, bool active) override {
    LocomotionCommandResult result;
    if (active && !IsAllowedSpeedMode(speed_mode)) {
      result.error = "invalid_speed_mode";
      return result;
    }
    if (mock_) {
      result.api_result = 0;
      return result;
    }
    try {
      const std::lock_guard<std::mutex> lock(client_mutex_);
      if (!client_) {
        result.error = "go2_sport_client_not_initialized";
        return result;
      }
      // Web speed modes are conservative velocity presets, not firmware gait
      // levels.  Move() already takes the requested velocity directly.  Do
      // not call SpeedLevel() here: on the deployed Go2 firmware the low
      // level (-1) request is rejected even while FSM 100 is movable.
      // V2 documents StopMove() as restoring most motion parameters to their
      // defaults. A dead-man key release only needs to zero velocity, so keep
      // the current gait/mode intact; the explicit stop_move command still
      // uses SportClient::StopMove().
      result.api_result = active ? client_->Move(vx_m_s, vy_m_s, vyaw_rad_s)
                                 : client_->Move(0.0F, 0.0F, 0.0F);
    } catch (const std::exception& exception) {
      result.api_result = -1;
      result.error = exception.what();
      return result;
    } catch (...) {
      result.api_result = -1;
      result.error = "unknown_go2_motion_exception";
      return result;
    }
    if (result.api_result != 0) result.error = Go2ApiError(result.api_result);
    return result;
  }

 private:
  bool mock_{false};
  std::mutex client_mutex_;
  std::unique_ptr<unitree::robot::go2::SportClient> client_;
};

}  // namespace

std::unique_ptr<ILocomotion> CreateGo2LocomotionAdapter(bool mock) {
  return std::make_unique<Go2LocomotionAdapter>(mock);
}

}  // namespace g1_web
