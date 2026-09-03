#include "g1_web/control_service.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <json/json.h>

namespace g1_web {
namespace {

constexpr std::size_t kRecentRequestLimit = 64;
constexpr float kMaximumForwardSpeedMps = 3.0F;
constexpr float kMaximumLateralSpeedMps = 1.0F;
constexpr float kMaximumYawSpeedRadS = 1.5F;
constexpr float kVelocityLeaseSeconds = 0.25F;

const std::unordered_set<std::int32_t>& AllowedArmActionIds() {
  static const std::unordered_set<std::int32_t> ids{
      11, 12, 15, 17, 18, 19, 20, 21,
      22, 23, 24, 25, 26, 27, 99};
  return ids;
}

const std::unordered_set<std::int32_t>& AllowedFsmIds() {
  static const std::unordered_set<std::int32_t> ids{
      0, 1, 2, 3, 4, 500, 501, 702, 706, 801, 802};
  return ids;
}

bool IsFreshSportState(const ControlData& control) {
  if (!control.sport_state_received) {
    return false;
  }
  return SteadyClock::now() - control.sport_state_last_update <=
         std::chrono::seconds(1);
}

bool IsArmActionFsm(std::uint32_t fsm_id) {
  return fsm_id == 500 || fsm_id == 501 || fsm_id == 801 ||
         fsm_id == 802;
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
  if (speed_mode == 3) {
    return {3.0F, 1.0F, 1.5F};
  }
  if (IsWalkRunFsm(fsm_id)) {
    if (speed_mode == 1) {
      return {1.0F, 0.6F, 1.3F};
    }
    return {0.5F, 0.4F, 1.1F};
  }
  if (speed_mode == 1) {
    return {1.0F, 0.35F, 0.8F};
  }
  return {0.5F, 0.2F, 0.5F};
}

bool IsSpeedModeAllowedForFsm(std::int32_t speed_mode,
                              std::uint32_t fsm_id) {
  if (IsWalkRunFsm(fsm_id)) {
    return IsAllowedSpeedMode(speed_mode);
  }
  return (fsm_id == 500 || fsm_id == 501) &&
         (speed_mode == 0 || speed_mode == 1);
}

bool FirmwareActionAvailable(const std::string& raw,
                             std::int32_t action_id) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(raw);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isArray() || root.empty() || !root[0].isArray()) {
    return false;
  }
  for (const auto& action : root[0]) {
    if (action.isObject() && action["id"].isIntegral() &&
        action["id"].asInt() == action_id) {
      return true;
    }
  }
  return false;
}

bool FirmwareTeachActionAvailable(const std::string& raw,
                                  const std::string& action_name) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(raw);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isArray() || root.size() < 2 || !root[1].isArray()) {
    return false;
  }
  for (const auto& action : root[1]) {
    if (action.isObject() && action["name"].isString() &&
        action["name"].asString() == action_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool ControlService::Start(std::string& error) {
  if (running_.load()) {
    error = "Control service is already running";
    return false;
  }

  ControlData control;
  control.mock = mock_;
  control.enabled = true;

  try {
    if (mock_) {
      control.initialized = true;
      control.action_list_api_result = 0;
      control.action_list_raw =
          R"([[{"id":99,"name":"release_arm"},{"id":11,"name":"blow_kiss_with_both_hands"},{"id":17,"name":"clamp"},{"id":18,"name":"high_five"},{"id":19,"name":"hug"},{"id":22,"name":"refuse"},{"id":25,"name":"wave_under_head"},{"id":26,"name":"wave_above_head"},{"id":27,"name":"shake_hand"}],[{"name":"Waist_Drum_Dance","time":9.5},{"name":"Spin_discs","time":6.9},{"name":"Scratch_head","time":8.1}]])";
    } else {
      loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
      loco_client_->SetTimeout(10.0F);
      loco_client_->Init();

      motion_client_ =
          std::make_unique<unitree::robot::g1::LocoClient>();
      motion_client_->SetTimeout(1.0F);
      motion_client_->Init();

      arm_action_client_ =
          std::make_unique<unitree::robot::g1::G1ArmActionClient>();
      arm_action_client_->SetTimeout(10.0F);
      arm_action_client_->Init();

      int fsm_id = 0;
      int fsm_mode = 0;
      const std::int32_t fsm_id_result =
          loco_client_->GetFsmId(fsm_id);
      const std::int32_t fsm_mode_result =
          loco_client_->GetFsmMode(fsm_mode);
      if (fsm_id_result == 0 && fsm_mode_result == 0) {
        unitree_hg::msg::dds_::SportModeState_ state;
        state.fsm_id(static_cast<std::uint32_t>(fsm_id));
        state.fsm_mode(static_cast<std::uint32_t>(fsm_mode));
        store_.UpdateSportMode(state);
      }

      control.action_list_api_result =
          arm_action_client_->GetActionList(control.action_list_raw);
      control.initialized = true;
    }
  } catch (const std::exception& exception) {
    control.initialization_error = exception.what();
  } catch (...) {
    control.initialization_error = "Unknown control initialization error";
  }

  const auto current = store_.GetSnapshot().control;
  control.sport_state_received = current.sport_state_received;
  control.sport_state_last_update = current.sport_state_last_update;
  control.fsm_id = current.fsm_id;
  control.fsm_mode = current.fsm_mode;
  control.task_id = current.task_id;
  control.task_time_s = current.task_time_s;
  store_.SetControlState(control);

  if (!control.initialized) {
    loco_client_.reset();
    motion_client_.reset();
    arm_action_client_.reset();
    error = control.initialization_error;
    return false;
  }

  running_.store(true);
  StartJointDebugImpl(*joint_debug_);
  applied_speed_mode_ = -1;
  worker_ = std::thread([this] { WorkerLoop(); });
  motion_worker_ = std::thread([this] { MotionWorkerLoop(); });
  error.clear();
  return true;
}

void ControlService::Stop() {
  if (joint_debug_) {
    StopJointDebugImpl(*joint_debug_);
  }
  running_.store(false);
  queue_cv_.notify_all();
  motion_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (motion_worker_.joinable()) {
    motion_worker_.join();
  }
  if (motion_active_.exchange(false) && !mock_ && motion_client_) {
    motion_client_->StopMove();
  }
  arm_action_client_.reset();
  motion_client_.reset();
  loco_client_.reset();
  applied_speed_mode_ = -1;
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.clear();
  command_running_ = false;
}

ControlSubmitResult ControlService::Submit(
    const std::string& request_key, const std::string& category,
    const std::string& command, std::int32_t argument,
    bool confirmed, const std::string& action_name) {
  ControlSubmitResult result;
  if (!running_.load()) {
    result.error = "control_not_ready";
    return result;
  }
  if (JointDebugActive()) {
    result.error = "control_busy";
    return result;
  }
  if (!IsValidRequestKey(request_key)) {
    result.error = "invalid_request_key";
    return result;
  }
  if (!IsKnownCommand(category, command, argument)) {
    result.error = "unknown_or_disallowed_command";
    return result;
  }
  if (command == "execute_custom" && !IsValidActionName(action_name)) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }

  Request request;
  request.request_key = request_key;
  request.category = category;
  request.command = command;
  request.argument = argument;
  request.action_name = action_name;

  std::string precondition_error;
  if (!ValidatePreconditions(request, precondition_error)) {
    result.error = precondition_error;
    return result;
  }
  if (command == "damp" || command == "stop_move") {
    SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto duplicate = recent_requests_.find(request_key);
    if (duplicate != recent_requests_.end()) {
      result.accepted = true;
      result.duplicate = true;
      result.request_id = duplicate->second;
      return result;
    }
    if (command_running_ || !queue_.empty()) {
      result.error = "control_busy";
      return result;
    }

    request.request_id = next_request_id_.fetch_add(1);
    recent_requests_[request_key] = request.request_id;
    recent_request_order_.push_back(request_key);
    while (recent_request_order_.size() > kRecentRequestLimit) {
      recent_requests_.erase(recent_request_order_.front());
      recent_request_order_.pop_front();
    }
    queue_.push_back(request);
  }

  ControlCommandData command_data;
  command_data.request_id = request.request_id;
  command_data.request_key = request.request_key;
  command_data.category = request.category;
  command_data.command = request.command;
  command_data.argument = request.argument;
  command_data.action_name = request.action_name;
  command_data.state = "queued";
  command_data.accepted_time_ms = SystemTimeMs();
  store_.UpdateControlCommand(command_data);
  queue_cv_.notify_one();

  result.accepted = true;
  result.request_id = request.request_id;
  return result;
}

MotionSubmitResult ControlService::SubmitVelocity(
    float vx_m_s, float vy_m_s, float vyaw_rad_s,
    std::int32_t speed_mode, bool active) {
  MotionSubmitResult result;
  if (!running_.load()) {
    result.error = "control_not_ready";
    return result;
  }
  if (active && JointDebugActive()) {
    result.error = "control_busy";
    return result;
  }
  if (!std::isfinite(vx_m_s) || !std::isfinite(vy_m_s) ||
      !std::isfinite(vyaw_rad_s)) {
    result.error = "invalid_velocity";
    return result;
  }

  if (active) {
    if (!IsAllowedSpeedMode(speed_mode)) {
      result.error = "invalid_speed_mode";
      return result;
    }
    const auto control = store_.GetSnapshot().control;
    const auto limits = MotionLimitsForMode(speed_mode, control.fsm_id);
    if (std::abs(vx_m_s) > kMaximumForwardSpeedMps ||
        std::abs(vx_m_s) > limits.forward_m_s ||
        std::abs(vy_m_s) > kMaximumLateralSpeedMps ||
        std::abs(vy_m_s) > limits.lateral_m_s ||
        std::abs(vyaw_rad_s) > kMaximumYawSpeedRadS ||
        std::abs(vyaw_rad_s) > limits.yaw_rad_s ||
        (std::abs(vx_m_s) < 0.001F &&
         std::abs(vy_m_s) < 0.001F &&
         std::abs(vyaw_rad_s) < 0.001F)) {
      result.error = "velocity_out_of_range";
      return result;
    }
    if (!IsFreshSportState(control)) {
      result.error = "sport_state_stale";
      return result;
    }
    if (!IsLocomotionFsm(control.fsm_id)) {
      result.error = "motion_fsm_not_allowed";
      return result;
    }
    if (!IsSpeedModeAllowedForFsm(speed_mode, control.fsm_id)) {
      result.error = "speed_mode_requires_walkrun";
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (command_running_ || !queue_.empty()) {
        result.error = "control_busy";
        return result;
      }
    }
  } else {
    vx_m_s = 0.0F;
    vy_m_s = 0.0F;
    vyaw_rad_s = 0.0F;
    speed_mode = 0;
  }

  VelocityRequest request;
  request.sequence = next_motion_sequence_.fetch_add(1);
  request.vx_m_s = vx_m_s;
  request.vy_m_s = vy_m_s;
  request.vyaw_rad_s = vyaw_rad_s;
  request.speed_mode = speed_mode;
  request.active = active;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    pending_velocity_ = request;
    velocity_pending_ = true;
  }
  motion_active_.store(active);

  MotionData motion;
  motion.active = active;
  motion.vx_m_s = vx_m_s;
  motion.vy_m_s = vy_m_s;
  motion.vyaw_rad_s = vyaw_rad_s;
  motion.speed_mode = speed_mode;
  motion.sequence = request.sequence;
  motion.state = "queued";
  motion.updated_time_ms = SystemTimeMs();
  store_.UpdateMotion(motion);
  motion_cv_.notify_one();

  result.accepted = true;
  result.sequence = request.sequence;
  return result;
}

bool ControlService::IsKnownCommand(const std::string& category,
                                    const std::string& command,
                                    std::int32_t argument) {
  if (category == "mode") {
    if (command == "set_fsm_id") {
      return AllowedFsmIds().count(argument) != 0;
    }
    return command == "damp" || command == "zero_torque" ||
           command == "start" || command == "squat" ||
           command == "sit" || command == "stand_up" ||
           command == "balance_stand" || command == "stop_move";
  }
  if (category == "arm_action" && command == "execute") {
    return AllowedArmActionIds().count(argument) != 0;
  }
  if (category == "arm_action" && command == "execute_custom") {
    return argument == 0;
  }
  if (category == "arm_action" && command == "stop_custom") {
    return argument == 0;
  }
  return false;
}

bool ControlService::ValidatePreconditions(
    const Request& request, std::string& error) const {
  if (request.command == "damp" || request.command == "stop_move") {
    error.clear();
    return true;
  }
  if (motion_active_.load()) {
    error = "motion_active";
    return false;
  }

  const auto control = store_.GetSnapshot().control;
  if (!IsFreshSportState(control)) {
    error = "sport_state_stale";
    return false;
  }
  if (request.category == "arm_action") {
    if (!mock_ && request.command == "execute" &&
        (control.action_list_api_result != 0 ||
         !FirmwareActionAvailable(control.action_list_raw,
                                  request.argument))) {
      error = "arm_action_not_available_on_firmware";
      return false;
    }
    if (!mock_ && request.command == "execute_custom" &&
        (control.action_list_api_result != 0 ||
         !FirmwareTeachActionAvailable(control.action_list_raw,
                                       request.action_name))) {
      error = "teach_action_not_available_on_firmware";
      return false;
    }
    const auto snapshot = store_.GetSnapshot();
    if ((request.argument == 20 || request.argument == 21) &&
        snapshot.mode_machine != 5 && snapshot.mode_machine != 6) {
      error = "arm_action_not_supported_by_model";
      return false;
    }
    if (!IsArmActionFsm(control.fsm_id)) {
      error = "arm_action_fsm_not_allowed";
      return false;
    }
    if (control.fsm_mode != 0 && control.fsm_mode != 3) {
      error = "arm_action_robot_not_static";
      return false;
    }
  } else if (control.fsm_mode != 0) {
    error = "robot_not_static";
    return false;
  }

  error.clear();
  return true;
}

void ControlService::WorkerLoop() {
  while (running_.load()) {
    Request request;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock,
                     [this] { return !running_.load() || !queue_.empty(); });
      if (!running_.load()) {
        break;
      }
      request = std::move(queue_.front());
      queue_.pop_front();
      command_running_ = true;
    }

    Execute(std::move(request));

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      command_running_ = false;
    }
  }
}

void ControlService::MotionWorkerLoop() {
  while (running_.load()) {
    VelocityRequest request;
    {
      std::unique_lock<std::mutex> lock(motion_mutex_);
      motion_cv_.wait(lock, [this] {
        return !running_.load() || velocity_pending_;
      });
      if (!running_.load()) {
        break;
      }
      request = pending_velocity_;
      velocity_pending_ = false;
    }

    MotionData motion;
    motion.active = request.active;
    motion.vx_m_s = request.vx_m_s;
    motion.vy_m_s = request.vy_m_s;
    motion.vyaw_rad_s = request.vyaw_rad_s;
    motion.speed_mode = request.speed_mode;
    motion.sequence = request.sequence;
    motion.state = "running";
    motion.updated_time_ms = SystemTimeMs();
    store_.UpdateMotion(motion);

    try {
      if (mock_) {
        applied_speed_mode_ = request.speed_mode;
        motion.api_result = 0;
      } else if (request.active) {
        const auto control = store_.GetSnapshot().control;
        if (!IsFreshSportState(control) ||
            !IsLocomotionFsm(control.fsm_id)) {
          motion.api_result = -1;
          motion.error = "motion_precondition_changed";
        } else if (!IsSpeedModeAllowedForFsm(
                       request.speed_mode, control.fsm_id)) {
          motion.api_result = -1;
          motion.error = "speed_mode_requires_walkrun";
        } else {
          if (applied_speed_mode_ != request.speed_mode) {
            motion.api_result =
                motion_client_->SetSpeedMode(request.speed_mode);
            if (motion.api_result == 0) {
              applied_speed_mode_ = request.speed_mode;
            }
          }
          if (motion.api_result == 0) {
            motion.api_result = motion_client_->SetVelocity(
                request.vx_m_s, request.vy_m_s,
                request.vyaw_rad_s, kVelocityLeaseSeconds);
          }
        }
      } else {
        motion.api_result = motion_client_->StopMove();
      }
    } catch (const std::exception& exception) {
      motion.api_result = -1;
      motion.error = exception.what();
    } catch (...) {
      motion.api_result = -1;
      motion.error = "unknown_motion_exception";
    }

    if (motion.api_result != 0 && motion.error.empty()) {
      motion.error = "sdk_api_error";
    }
    if (motion.api_result == 0 && motion.error.empty()) {
      motion.state = request.active ? "active" : "stopped";
    } else {
      motion.state = "failed";
      motion.active = false;
      motion_active_.store(false);
    }
    if (!request.active) {
      motion_active_.store(false);
    }
    motion.updated_time_ms = SystemTimeMs();
    store_.UpdateMotion(motion);
  }
}

void ControlService::Execute(Request request) {
  ControlCommandData command;
  command.request_id = request.request_id;
  command.request_key = request.request_key;
  command.category = request.category;
  command.command = request.command;
  command.argument = request.argument;
  command.action_name = request.action_name;
  command.state = "running";
  command.accepted_time_ms = SystemTimeMs();
  store_.UpdateControlCommand(command);

  std::cout << "[CONTROL] request=" << request.request_id
            << " category=" << request.category
            << " command=" << request.command
            << " argument=" << request.argument
            << (request.action_name.empty()
                    ? ""
                    : " action_name=" + request.action_name)
            << " started\n";

  try {
    if (mock_) {
      ExecuteMock(request);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      command.api_result = 0;
    } else {
      command.api_result = ExecuteReal(request, command.error);
    }
  } catch (const std::exception& exception) {
    command.api_result = -1;
    command.error = exception.what();
  } catch (...) {
    command.api_result = -1;
    command.error = "unknown_control_exception";
  }

  command.state =
      command.api_result == 0 && command.error.empty() ? "succeeded"
                                                       : "failed";
  command.completed_time_ms = SystemTimeMs();
  store_.UpdateControlCommand(command);
  std::cout << "[CONTROL] request=" << request.request_id
            << " result=" << command.api_result
            << " state=" << command.state
            << (command.error.empty() ? "" : " error=" + command.error)
            << '\n';
}

std::int32_t ControlService::ExecuteReal(const Request& request,
                                         std::string& error) {
  std::int32_t result = -1;
  std::int32_t target_fsm = -1;
  if (request.category == "mode") {
    if (request.command == "damp") {
      result = loco_client_->Damp();
      target_fsm = 1;
    } else if (request.command == "zero_torque") {
      result = loco_client_->ZeroTorque();
      target_fsm = 0;
    } else if (request.command == "start") {
      result = loco_client_->Start();
      target_fsm = 500;
    } else if (request.command == "squat") {
      result = loco_client_->Squat();
      target_fsm = 2;
    } else if (request.command == "sit") {
      result = loco_client_->Sit();
      target_fsm = 3;
    } else if (request.command == "stand_up") {
      result = loco_client_->StandUp();
      target_fsm = 4;
    } else if (request.command == "balance_stand") {
      result = loco_client_->BalanceStand();
    } else if (request.command == "stop_move") {
      result = loco_client_->StopMove();
    } else if (request.command == "set_fsm_id") {
      result = loco_client_->SetFsmId(request.argument);
      target_fsm = request.argument;
    }
  } else if (request.command == "execute") {
    result = arm_action_client_->ExecuteAction(request.argument);
  } else if (request.command == "execute_custom") {
    result = arm_action_client_->ExecuteAction(request.action_name);
  } else if (request.command == "stop_custom") {
    result = arm_action_client_->StopCustomAction();
  }

  if (result != 0) {
    error = "sdk_api_error";
    return result;
  }
  if (target_fsm >= 0 &&
      !WaitForFsm(static_cast<std::uint32_t>(target_fsm),
                  std::chrono::seconds(6))) {
    error = "fsm_confirmation_timeout";
    return result;
  }
  error.clear();
  return result;
}

void ControlService::ExecuteMock(const Request& request) {
  auto current = store_.GetSnapshot().control;
  unitree_hg::msg::dds_::SportModeState_ state;
  state.fsm_id(current.fsm_id);
  state.fsm_mode(0);
  state.task_id(current.task_id);
  state.task_time(current.task_time_s);

  if (request.command == "damp") {
    state.fsm_id(1);
  } else if (request.command == "zero_torque") {
    state.fsm_id(0);
  } else if (request.command == "start") {
    state.fsm_id(500);
  } else if (request.command == "squat") {
    state.fsm_id(2);
  } else if (request.command == "sit") {
    state.fsm_id(3);
  } else if (request.command == "stand_up") {
    state.fsm_id(4);
  } else if (request.command == "set_fsm_id") {
    state.fsm_id(static_cast<std::uint32_t>(request.argument));
  } else if (request.category == "arm_action" &&
             request.command == "execute") {
    state.task_id(static_cast<std::uint32_t>(request.argument));
    state.task_time(0.1F);
  } else if (request.category == "arm_action" &&
             request.command == "execute_custom") {
    state.task_id(100);
    state.task_time(0.1F);
  }
  store_.UpdateSportMode(state);
}

bool ControlService::WaitForFsm(
    std::uint32_t target_fsm_id,
    std::chrono::milliseconds timeout) const {
  const auto deadline = SteadyClock::now() + timeout;
  while (running_.load() && SteadyClock::now() < deadline) {
    const auto control = store_.GetSnapshot().control;
    if (control.sport_state_received &&
        control.fsm_id == target_fsm_id) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool ControlService::IsValidRequestKey(
    const std::string& request_key) {
  if (request_key.size() < 8 || request_key.size() > 128) {
    return false;
  }
  return std::all_of(request_key.begin(), request_key.end(), [](char value) {
    const auto byte = static_cast<unsigned char>(value);
    return std::isalnum(byte) != 0 || value == '-' || value == '_';
  });
}

bool ControlService::IsValidActionName(const std::string& action_name) {
  if (action_name.empty() || action_name.size() > 128) {
    return false;
  }
  return std::none_of(action_name.begin(), action_name.end(), [](char value) {
    const auto byte = static_cast<unsigned char>(value);
    return byte < 0x20 || byte == 0x7F || value == '"' || value == '\\';
  });
}

std::int64_t ControlService::SystemTimeMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace g1_web
