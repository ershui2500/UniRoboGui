#include "g1_web/control_service.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <utility>

#include <json/json.h>

namespace g1_web {
namespace {

constexpr std::size_t kRecentRequestLimit = 64;
constexpr char kLocomotionMotionOwner[] = "locomotion.motion";
constexpr char kLocomotionCommandOwner[] = "locomotion.command";
constexpr char kArmActionOwner[] = "arm_action";

struct ResourceReleaseGuard {
  ResourceManager& manager;
  ControlResource resource;
  std::string owner;
  bool release{false};

  ~ResourceReleaseGuard() {
    if (release) manager.Release(resource, owner);
  }
};

bool IsFreshSportState(const ControlData& control) {
  if (!control.sport_state_received) {
    return false;
  }
  return SteadyClock::now() - control.sport_state_last_update <=
         std::chrono::seconds(1);
}

const CapabilityDescriptor* FindCapability(const RobotProfile& profile,
                                           CapabilityKey key) {
  const auto capability = std::find_if(
      profile.capabilities.begin(), profile.capabilities.end(),
      [key](const auto& item) { return item.key == key; });
  return capability == profile.capabilities.end() ? nullptr : &*capability;
}

bool ModeTransitionAllowed(const RobotProfile& profile,
                           const std::string& command,
                           std::uint32_t source_fsm) {
  const auto* locomotion = FindCapability(profile, CapabilityKey::kLocomotion);
  if (!locomotion) return false;
  const auto policy = locomotion->parameters.find("mode_sources");
  if (policy == locomotion->parameters.end()) return true;

  std::istringstream entries(policy->second);
  std::string entry;
  while (std::getline(entries, entry, ';')) {
    const auto separator = entry.find(':');
    if (separator == std::string::npos || entry.substr(0, separator) != command) {
      continue;
    }
    std::istringstream sources(entry.substr(separator + 1));
    std::string source;
    while (std::getline(sources, source, '|')) {
      if (source == std::to_string(source_fsm)) return true;
    }
    return false;
  }
  return false;
}

bool IsUnknownFsmMode(const RobotProfile& profile, std::uint32_t fsm_mode) {
  const auto* locomotion = FindCapability(profile, CapabilityKey::kLocomotion);
  if (!locomotion) return false;
  const auto unknown = locomotion->parameters.find("fsm_mode_unknown");
  return unknown != locomotion->parameters.end() &&
         unknown->second == std::to_string(fsm_mode);
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

CapabilityKey ControlCapabilityForRequest(const std::string& category,
                                          const std::string& command) {
  if (category == "mode") return CapabilityKey::kLocomotion;
  return command == "execute_custom" || command == "stop_custom"
             ? CapabilityKey::kJointTeach
             : CapabilityKey::kJointDebug;
}

std::string ArmActionApiError(std::int32_t result) {
  switch (result) {
    case 3103:
      return "arm_action_api_unavailable";
    case 3104:
      return "arm_action_timeout";
    case 3204:
      return "arm_action_invalid_parameter";
    case 7399:
      return "arm_action_service_error";
    case 7400:
      return "arm_action_occupied";
    case 7401:
      return "arm_action_holding";
    case 7402:
      return "arm_action_invalid_id";
    case 7403:
      return "arm_action_file_error";
    case 7404:
      return "arm_action_fsm_not_allowed";
    case 7405:
      return "arm_action_name_exists";
    case 7406:
      return "arm_action_low_battery";
    case 7407:
      return "arm_action_motor_error";
    default:
      return "sdk_api_error";
  }
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
    LocomotionInitialization locomotion_state;
    std::string locomotion_error;
    if (!locomotion_) {
      control.initialization_error = "locomotion_adapter_missing";
    } else if (!locomotion_->Initialize(locomotion_state,
                                        locomotion_error)) {
      control.initialization_error = locomotion_error;
    } else {
      if (locomotion_state.state_valid) {
        unitree_hg::msg::dds_::SportModeState_ state;
        state.fsm_id(locomotion_state.fsm_id);
        state.fsm_mode(locomotion_state.fsm_mode);
        store_.UpdateSportMode(state);
      }
      if (mock_) {
        control.initialized = true;
        if (joint_debug_policy_ && joint_debug_policy_->SupportsArmActions()) {
          control.action_list_api_result = 0;
          control.action_list_raw = joint_debug_policy_->MockArmActionList();
        }
      } else {
        if (joint_debug_policy_) {
          control.action_list_api_result =
              joint_debug_policy_->InitializeArmActions(
                  control.action_list_raw);
        }
        control.initialized = true;
      }
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
    if (locomotion_) locomotion_->Shutdown();
    if (joint_debug_policy_) joint_debug_policy_->ShutdownArmActions();
    error = control.initialization_error;
    return false;
  }

  running_.store(true);
  if (joint_debug_) StartJointDebugImpl(*joint_debug_);
  worker_ = std::thread([this] { WorkerLoop(); });
  motion_worker_ = std::thread([this] { MotionWorkerLoop(); });
  if (poll_locomotion_state_) {
    state_worker_ = std::thread([this] { StateWorkerLoop(); });
  }
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
  if (state_worker_.joinable()) {
    state_worker_.join();
  }
  if (motion_active_.exchange(false) && locomotion_) {
    locomotion_->ApplyVelocity(0.0F, 0.0F, 0.0F, 0, false);
  }
  resources_.ReleaseOwner(kLocomotionMotionOwner);
  if (joint_debug_policy_) joint_debug_policy_->ShutdownArmActions();
  if (locomotion_) locomotion_->Shutdown();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
    command_running_ = false;
    arm_interrupt_running_ = false;
  }
  resources_.ReleaseOwner(kLocomotionCommandOwner);
  resources_.ReleaseOwner(kArmActionOwner);
}

ControlSubmitResult ControlService::Submit(
    const std::string& request_key, const std::string& category,
    const std::string& command, std::int32_t argument,
    bool confirmed, const std::string& action_name) {
  ControlSubmitResult result;
  const auto ready = safety_.CheckServiceReady(running_.load());
  if (!ready.allowed) {
    result.error = ready.error;
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
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto capability = safety_.CheckCapability(
      robot_profile_, ControlCapabilityForRequest(category, command), mock_);
  if (!capability.allowed) {
    result.error = capability.error;
    return result;
  }

  Request request;
  request.request_key = request_key;
  request.category = category;
  request.command = command;
  request.argument = argument;
  request.action_name = action_name;
  request.resource = category == "mode" ? ControlResource::Locomotion
                                        : ControlResource::Arm;
  request.resource_owner = category == "mode" ? kLocomotionCommandOwner
                                               : kArmActionOwner;
  const bool arm_interrupt =
      joint_debug_policy_ &&
      joint_debug_policy_->SupportsArmActionInterrupts() &&
      category == "arm_action" &&
      ((command == "execute" && argument == 99) ||
       command == "stop_custom");

  const bool stop_command =
      category == "mode" && (command == "damp" || command == "stop_move");
  if (!(stop_command &&
        resources_.Query(ControlResource::Locomotion) ==
            kLocomotionMotionOwner)) {
    const auto acquired =
        resources_.Acquire(request.resource, request.resource_owner);
    const auto resource = safety_.CheckResource(
        acquired, resources_.Query(ControlResource::Locomotion) ==
                          kLocomotionMotionOwner
                      ? "motion_active"
                      : "control_busy");
    if (!resource.allowed) {
      result.error = resource.error;
      return result;
    }
    request.release_resource = !acquired.already_owned;
  }
  ResourceReleaseGuard submit_resource_guard{
      resources_, request.resource, request.resource_owner,
      request.release_resource};

  std::string precondition_error;
  if (!ValidatePreconditions(request, precondition_error)) {
    result.error = precondition_error;
    return result;
  }
  if (command == "damp" || command == "stop_move") {
    SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);
  }

  bool execute_immediately = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto duplicate = recent_requests_.find(request_key);
    if (duplicate != recent_requests_.end()) {
      result.accepted = true;
      result.duplicate = true;
      result.request_id = duplicate->second;
      return result;
    }
    execute_immediately = arm_interrupt && command_running_ &&
                          !arm_interrupt_running_ && queue_.empty();
    if ((command_running_ || arm_interrupt_running_ || !queue_.empty()) &&
        !execute_immediately) {
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
    if (execute_immediately) arm_interrupt_running_ = true;
    else queue_.push_back(request);
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
  submit_resource_guard.release = false;
  if (execute_immediately) {
    Execute(std::move(request));
    std::lock_guard<std::mutex> lock(queue_mutex_);
    arm_interrupt_running_ = false;
    result.accepted = true;
    result.request_id = command_data.request_id;
    return result;
  }
  queue_cv_.notify_one();

  result.accepted = true;
  result.request_id = request.request_id;
  return result;
}

MotionSubmitResult ControlService::SubmitVelocity(
    float vx_m_s, float vy_m_s, float vyaw_rad_s,
    std::int32_t speed_mode, bool active) {
  MotionSubmitResult result;
  const auto ready = safety_.CheckServiceReady(running_.load());
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  if (active) {
    const auto capability = safety_.CheckCapability(
        robot_profile_, CapabilityKey::kLocomotion, mock_);
    if (!capability.allowed) {
      result.error = capability.error;
      return result;
    }
  }

  ResourceReleaseGuard velocity_resource_guard{
      resources_, ControlResource::Locomotion, kLocomotionMotionOwner, false};
  if (active) {
    const auto acquired =
        resources_.Acquire(ControlResource::Locomotion,
                           kLocomotionMotionOwner);
    const auto resource = safety_.CheckResource(acquired);
    if (!resource.allowed) {
      result.error = resource.error;
      return result;
    }
    velocity_resource_guard.release = !acquired.already_owned;
  }

  const auto reject = [](const std::string& error) {
    MotionSubmitResult rejected;
    rejected.error = error;
    return rejected;
  };

  if (!std::isfinite(vx_m_s) || !std::isfinite(vy_m_s) ||
      !std::isfinite(vyaw_rad_s)) {
    return reject("invalid_velocity");
  }

  if (active) {
    const auto control = store_.GetSnapshot().control;
    const auto freshness = safety_.CheckFreshness(
        IsFreshSportState(control), "sport_state_stale");
    if (!freshness.allowed) return reject(freshness.error);
    result.error = locomotion_->ValidateVelocity(
        control, vx_m_s, vy_m_s, vyaw_rad_s, speed_mode);
    if (!result.error.empty()) return reject(result.error);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (command_running_ || arm_interrupt_running_ || !queue_.empty())
        return reject("control_busy");
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
  request.release_resource =
      !active && resources_.Query(ControlResource::Locomotion) ==
                     kLocomotionMotionOwner;
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
  velocity_resource_guard.release = false;
  motion_cv_.notify_one();

  result.accepted = true;
  result.sequence = request.sequence;
  return result;
}

bool ControlService::IsKnownCommand(const std::string& category,
                                    const std::string& command,
                                    std::int32_t argument) const {
  if (category == "mode") {
    return locomotion_ && locomotion_->IsKnownModeCommand(command, argument);
  }
  if (category == "arm_action" &&
      (!joint_debug_policy_ || !joint_debug_policy_->SupportsArmActions())) {
    return false;
  }
  if (category == "arm_action" && command == "execute") {
    return joint_debug_policy_->IsKnownArmAction(argument);
  }
  if (category == "arm_action" && command == "execute_custom") {
    return joint_debug_policy_->SupportsCustomArmActions() && argument == 0;
  }
  if (category == "arm_action" && command == "stop_custom") {
    return joint_debug_policy_->SupportsCustomArmActions() && argument == 0;
  }
  return false;
}

bool ControlService::ValidatePreconditions(
    const Request& request, std::string& error) const {
  if (request.command == "damp" || request.command == "stop_move") {
    error.clear();
    return true;
  }
  const auto control = store_.GetSnapshot().control;
  const auto freshness = safety_.CheckFreshness(
      IsFreshSportState(control), "sport_state_stale");
  if (!freshness.allowed) {
    error = freshness.error;
    return false;
  }
  if (request.category == "arm_action") {
    const bool arm_interrupt =
        joint_debug_policy_ &&
        joint_debug_policy_->SupportsArmActionInterrupts() &&
        ((request.command == "execute" && request.argument == 99) ||
         request.command == "stop_custom");
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
    if (!arm_interrupt) {
      const auto snapshot = store_.GetSnapshot();
      if ((request.argument == 20 || request.argument == 21) &&
          snapshot.mode_machine != 5 && snapshot.mode_machine != 6) {
        error = "arm_action_not_supported_by_model";
        return false;
      }
      if (!locomotion_ || !locomotion_->IsOperationalFsm(control.fsm_id)) {
        error = "arm_action_fsm_not_allowed";
        return false;
      }
      if (control.fsm_mode != 0 && control.fsm_mode != 3) {
        error = "arm_action_robot_not_static";
        return false;
      }
    }
  } else {
    if (control.fsm_mode != 0 &&
        !IsUnknownFsmMode(robot_profile_, control.fsm_mode)) {
      error = "robot_not_static";
      return false;
    }
    if (request.category == "mode" &&
        !ModeTransitionAllowed(robot_profile_, request.command,
                               control.fsm_id)) {
      error = "mode_transition_not_allowed";
      return false;
    }
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
      if (request.active) {
        const auto control = store_.GetSnapshot().control;
        const auto freshness = safety_.CheckFreshness(
            IsFreshSportState(control), "motion_precondition_changed");
        motion.error = freshness.allowed
                           ? locomotion_->ValidateVelocityExecution(
                                 control, request.speed_mode)
                           : freshness.error;
      }
      if (motion.error.empty()) {
        const auto execution = locomotion_->ApplyVelocity(
            request.vx_m_s, request.vy_m_s, request.vyaw_rad_s,
            request.speed_mode, request.active);
        motion.api_result = execution.api_result;
        motion.error = execution.error;
      } else {
        motion.api_result = -1;
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
      motion.vx_m_s = 0.0F;
      motion.vy_m_s = 0.0F;
      motion.vyaw_rad_s = 0.0F;
      motion.speed_mode = 0;
      motion_active_.store(false);
    }
    if (!request.active) {
      motion_active_.store(false);
    }
    if ((motion.state == "failed" && request.active) ||
        (!request.active && request.release_resource)) {
      safety_.CleanupControlFailure(resources_, ControlResource::Locomotion,
                                    kLocomotionMotionOwner);
    }
    motion.updated_time_ms = SystemTimeMs();
    store_.UpdateMotion(motion);
  }
}

void ControlService::StateWorkerLoop() {
  while (running_.load()) {
    LocomotionInitialization state;
    std::string error;
    if (locomotion_ && locomotion_->QueryState(state, error) &&
        state.state_valid) {
      unitree_hg::msg::dds_::SportModeState_ message;
      message.fsm_id(state.fsm_id);
      message.fsm_mode(state.fsm_mode);
      const auto current = store_.GetSnapshot().control;
      message.task_id(current.task_id);
      message.task_time(current.task_time_s);
      store_.UpdateSportMode(message);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void ControlService::Execute(Request request) {
  ResourceReleaseGuard resource_guard{resources_, request.resource,
                                      request.resource_owner,
                                      request.release_resource};
  ControlCommandData command;
  command.request_id = request.request_id;
  command.request_key = request.request_key;
  command.category = request.category;
  command.command = request.command;
  command.argument = request.argument;
  command.action_name = request.action_name;
  command.state = "running";
  command.accepted_time_ms = SystemTimeMs();
  if (store_.GetSnapshot().control.last_command.request_id <=
      command.request_id) {
    store_.UpdateControlCommand(command);
  }

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
  if (store_.GetSnapshot().control.last_command.request_id <=
      command.request_id) {
    store_.UpdateControlCommand(command);
  }
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
    const auto execution =
        locomotion_->ExecuteMode(request.command, request.argument);
    result = execution.api_result;
    target_fsm = execution.target_fsm;
    error = execution.error;
  } else if (request.command == "execute") {
    result = joint_debug_policy_
                 ? joint_debug_policy_->ExecuteArmAction(request.argument)
                 : -1;
  } else if (request.command == "execute_custom") {
    result = joint_debug_policy_
                 ? joint_debug_policy_->ExecuteArmAction(request.action_name)
                 : -1;
  } else if (request.command == "stop_custom") {
    result = joint_debug_policy_
                 ? joint_debug_policy_->StopCustomArmAction()
                 : -1;
  }

  if (result != 0) {
    if (error.empty()) {
      error = request.category == "arm_action"
                  ? ArmActionApiError(result)
                  : "sdk_api_error";
    }
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

  if (request.category == "mode") {
    const auto execution =
        locomotion_->ExecuteMode(request.command, request.argument);
    if (execution.target_fsm >= 0) {
      state.fsm_id(static_cast<std::uint32_t>(execution.target_fsm));
    }
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
