#include "g1_web/json_serializer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>
#include <vector>

#include <json/json.h>

#ifndef UNI_ROBO_GUI_VERSION
#define UNI_ROBO_GUI_VERSION "dev"
#endif

namespace g1_web {
namespace {

template <typename T>
Json::Value Number(T value) {
  if constexpr (std::is_floating_point<T>::value) {
    if (!std::isfinite(value)) {
      return Json::Value(Json::nullValue);
    }
    return Json::Value(static_cast<double>(value));
  } else if constexpr (std::is_signed<T>::value) {
    return Json::Value(static_cast<Json::Int64>(value));
  } else {
    return Json::Value(static_cast<Json::UInt64>(value));
  }
}

template <typename T, std::size_t Size>
Json::Value Array(const std::array<T, Size>& values) {
  Json::Value result(Json::arrayValue);
  for (const auto& value : values) {
    result.append(Number(value));
  }
  return result;
}

Json::Value Imu(const ImuData& imu) {
  Json::Value result(Json::objectValue);
  result["quaternion_wxyz"] = Array(imu.quaternion);
  result["gyroscope_rad_s"] = Array(imu.gyroscope);
  result["accelerometer_m_s2"] = Array(imu.accelerometer);
  result["rpy_rad"] = Array(imu.rpy);
  result["temperature_raw"] = imu.temperature_raw;
  return result;
}

std::int64_t AgeMs(const SourceState& source,
                   SteadyClock::time_point now) {
  if (!source.received) {
    return -1;
  }
  return std::max<std::int64_t>(
      0, std::chrono::duration_cast<std::chrono::milliseconds>(
             now - source.last_update)
             .count());
}

Json::Value SourceJson(const SourceState& source,
                       SteadyClock::time_point now) {
  const std::int64_t age_ms = AgeMs(source, now);
  const Freshness freshness =
      ClassifyFreshness(source.received, age_ms < 0 ? 3001 : age_ms);
  Json::Value result(Json::objectValue);
  result["received"] = source.received;
  result["status"] = FreshnessName(freshness);
  if (age_ms < 0) {
    result["age_ms"] = Json::Value(Json::nullValue);
  } else {
    result["age_ms"] = Json::Int64(age_ms);
  }
  return result;
}

Json::Value SourcesJson(const RobotSnapshot& snapshot,
                        SteadyClock::time_point now) {
  Json::Value result(Json::objectValue);
  for (std::size_t i = 0;
       i < static_cast<std::size_t>(SourceId::kCount); ++i) {
    const auto source = static_cast<SourceId>(i);
    result[SourceName(source)] = SourceJson(snapshot.sources[i], now);
  }
  return result;
}

Json::Value MotorJson(const MotorData& motor, std::size_t motor_slot,
                      const JointDescriptor* joint) {
  Json::Value result(Json::objectValue);
  result["index"] = static_cast<Json::UInt>(motor_slot);
  if (joint) {
    result["name"] = joint->name;
    result["name_zh"] = joint->name_zh;
  } else {
    result["name"] = "reserved_slot_" + std::to_string(motor_slot);
    result["name_zh"] = "原始槽 " + std::to_string(motor_slot);
  }
  result["mode_raw"] = static_cast<Json::UInt>(motor.mode);
  result["q_rad"] = Number(motor.q);
  result["dq_rad_s"] = Number(motor.dq);
  result["ddq_rad_s2"] = Number(motor.ddq);
  result["tau_est_nm"] = Number(motor.tau_est);
  result["temperature_raw"] = Array(motor.temperature);
  result["voltage_v"] = Number(motor.voltage);
  result["sensor_raw"] = Array(motor.sensor_raw);
  result["state_raw"] = Json::UInt64(motor.state_raw);
  return result;
}

Json::Value CommonRoot(const RobotSnapshot& snapshot,
                       SteadyClock::time_point now) {
  const auto server_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now()
                                   .time_since_epoch())
                               .count();
  Json::Value root(Json::objectValue);
  root["application"] = "UniRoboGui";
  root["application_version"] = UNI_ROBO_GUI_VERSION;
  root["schema_version"] = 1;
  root["sequence"] = Json::UInt64(snapshot.sequence);
  root["server_time_ms"] = Json::Int64(server_time);
  root["dds_initialized"] = snapshot.dds_initialized;
  root["dds_error"] = snapshot.dds_error;
  root["sources"] = SourcesJson(snapshot, now);
  return root;
}

Json::Value VoiceJson(const VoiceData& voice) {
  Json::Value result(Json::objectValue);
  result["initialized"] = voice.initialized;
  result["initialization_error"] = voice.initialization_error;
  result["asr_subscribed"] = voice.asr_subscribed;
  result["asr_control_api_result"] = voice.asr_control_api_result;
  result["asr_control_error"] = voice.asr_control_error;
  result["chat_go_found"] = voice.chat_go_found;
  result["chat_go_service_name"] = voice.chat_go_service_name;
  result["chat_go_closed"] = voice.chat_go_closed;
  result["chat_go_api_result"] = voice.chat_go_api_result;
  result["chat_go_status_raw"] = voice.chat_go_status_raw;
  result["vui_service_found"] = voice.vui_service_found;
  result["vui_service_status_raw"] = voice.vui_service_status_raw;
  result["volume_api_result"] = voice.volume_api_result;
  result["volume_pct"] = voice.volume_pct;
  result["play_state_raw"] = voice.play_state_raw;

  Json::Value asr(Json::objectValue);
  asr["received"] = voice.asr.received;
  asr["index"] = Json::UInt64(voice.asr.index);
  asr["timestamp_raw"] = Json::UInt64(voice.asr.timestamp_raw);
  asr["text"] = voice.asr.text;
  asr["angle_raw"] = voice.asr.angle_raw;
  asr["speaker_id_raw"] = voice.asr.speaker_id_raw;
  asr["sense_raw"] = voice.asr.sense_raw;
  asr["confidence"] = Number(voice.asr.confidence);
  asr["language"] = voice.asr.language;
  asr["is_final"] = voice.asr.is_final;
  asr["raw_json"] = voice.asr.raw_json;
  asr["received_time_ms"] = Json::Int64(voice.asr.received_time_ms);
  result["asr"] = asr;

  Json::Value tts(Json::objectValue);
  tts["request_id"] = Json::UInt64(voice.tts.request_id);
  tts["state"] = voice.tts.state;
  tts["text"] = voice.tts.text;
  tts["speaker_id"] = voice.tts.speaker_id;
  tts["backend"] = voice.tts.backend;
  tts["backend_error"] = voice.tts.backend_error;
  tts["api_result"] = voice.tts.api_result;
  tts["updated_time_ms"] = Json::Int64(voice.tts.updated_time_ms);
  result["tts"] = tts;

  Json::Value llm(Json::objectValue);
  llm["mode"] = voice.llm.mode;
  llm["builtin_api_available"] = voice.llm.builtin_api_available;
  llm["builtin_response_subscribed"] =
      voice.llm.builtin_response_subscribed;
  llm["builtin_request_topic"] = voice.llm.builtin_request_topic;
  llm["builtin_response_topic"] = voice.llm.builtin_response_topic;
  llm["customer_api_available"] = voice.llm.customer_api_available;
  llm["customer_api_configured"] = voice.llm.customer_api_configured;
  llm["customer_api_key_configured"] =
      voice.llm.customer_api_key_configured;
  llm["customer_api_url"] = voice.llm.customer_api_url;
  llm["customer_model"] = voice.llm.customer_model;
  llm["customer_role_prompt"] = voice.llm.customer_role_prompt;
  llm["customer_wake_word"] = voice.llm.customer_wake_word;
  llm["customer_wake_enabled"] = voice.llm.customer_wake_enabled;
  llm["customer_qa_count"] = Json::UInt(voice.llm.customer_qa_count);
  llm["customer_tts_backend"] = voice.llm.customer_tts_backend;
  llm["last_response_source"] = voice.llm.last_response_source;
  llm["request_state"] = voice.llm.request_state;
  llm["request_id"] = Json::Int64(voice.llm.request_id);
  llm["response_status_code"] = voice.llm.response_status_code;
  llm["last_user_message"] = voice.llm.last_user_message;
  llm["last_response"] = voice.llm.last_response;
  llm["last_error"] = voice.llm.last_error;
  llm["updated_time_ms"] = Json::Int64(voice.llm.updated_time_ms);
  result["llm"] = llm;
  return result;
}

Json::Value ControlJson(const ControlData& control) {
  Json::Value result(Json::objectValue);
  result["initialized"] = control.initialized;
  result["enabled"] = control.enabled;
  result["mock"] = control.mock;
  result["initialization_error"] = control.initialization_error;
  result["sport_state_received"] = control.sport_state_received;
  if (control.sport_state_received) {
    const auto age_ms = std::max<std::int64_t>(
        0, std::chrono::duration_cast<std::chrono::milliseconds>(
               SteadyClock::now() - control.sport_state_last_update)
               .count());
    result["sport_state_age_ms"] = Json::Int64(age_ms);
    result["sport_state_status"] =
        age_ms <= 1000 ? "online" : age_ms <= 3000 ? "delayed" : "offline";
  } else {
    result["sport_state_age_ms"] = Json::Value(Json::nullValue);
    result["sport_state_status"] = "offline";
  }
  result["fsm_id"] = Json::UInt(control.fsm_id);
  result["fsm_mode"] = Json::UInt(control.fsm_mode);
  result["task_id"] = Json::UInt(control.task_id);
  result["task_time_s"] = Number(control.task_time_s);
  result["action_list_api_result"] = control.action_list_api_result;
  result["action_list_raw"] = control.action_list_raw;

  Json::Value command(Json::objectValue);
  command["request_id"] = Json::UInt64(control.last_command.request_id);
  command["request_key"] = control.last_command.request_key;
  command["category"] = control.last_command.category;
  command["command"] = control.last_command.command;
  command["argument"] = control.last_command.argument;
  command["action_name"] = control.last_command.action_name;
  command["state"] = control.last_command.state;
  command["api_result"] = control.last_command.api_result;
  command["error"] = control.last_command.error;
  command["accepted_time_ms"] =
      Json::Int64(control.last_command.accepted_time_ms);
  command["completed_time_ms"] =
      Json::Int64(control.last_command.completed_time_ms);
  result["last_command"] = command;

  Json::Value motion(Json::objectValue);
  motion["active"] = control.motion.active;
  motion["vx_m_s"] = Number(control.motion.vx_m_s);
  motion["vy_m_s"] = Number(control.motion.vy_m_s);
  motion["vyaw_rad_s"] = Number(control.motion.vyaw_rad_s);
  motion["speed_mode"] = control.motion.speed_mode;
  motion["sequence"] = Json::UInt64(control.motion.sequence);
  motion["state"] = control.motion.state;
  motion["api_result"] = control.motion.api_result;
  motion["error"] = control.motion.error;
  motion["updated_time_ms"] =
      Json::Int64(control.motion.updated_time_ms);
  result["motion"] = motion;
  return result;
}

const char* CapabilityKeyName(CapabilityKey key) {
  switch (key) {
    case CapabilityKey::kTelemetry:
      return "telemetry";
    case CapabilityKey::kLocomotion:
      return "locomotion";
    case CapabilityKey::kJointDebug:
      return "joint_debug";
    case CapabilityKey::kJointTeach:
      return "joint_teach";
    case CapabilityKey::kAudio:
      return "audio";
    case CapabilityKey::kCameraRgb:
      return "camera_rgb";
    case CapabilityKey::kCameraDepth:
      return "camera_depth";
    case CapabilityKey::kLidar:
      return "lidar";
    case CapabilityKey::kSlam:
      return "slam";
    case CapabilityKey::kHeadControl:
      return "head_control";
  }
  return "unknown";
}

const char* VerificationLevelName(VerificationLevel level) {
  switch (level) {
    case VerificationLevel::kUnsupported:
      return "unsupported";
    case VerificationLevel::kImplemented:
      return "implemented";
    case VerificationLevel::kMockVerified:
      return "mock_verified";
    case VerificationLevel::kReadonlyVerified:
      return "readonly_verified";
    case VerificationLevel::kControlVerified:
      return "control_verified";
    case VerificationLevel::kDisabled:
      return "disabled";
  }
  return "unsupported";
}

std::string WriteJson(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

bool ProfileCapabilityAvailable(const RobotProfile& profile,
                                CapabilityKey key) {
  for (const auto& capability : profile.capabilities) {
    if (capability.key == key) {
      return capability.implemented && capability.available;
    }
  }
  return false;
}

std::string TelemetryRequiredSources(const RobotProfile& profile) {
  for (const auto& capability : profile.capabilities) {
    if (capability.key != CapabilityKey::kTelemetry) continue;
    const auto required = capability.parameters.find("required_sources");
    if (required != capability.parameters.end()) return required->second;
  }
  return "low_state,bms,secondary_imu,mainboard,odometry";
}

bool CsvContains(const std::string& csv, const std::string& value) {
  std::size_t start = 0;
  while (start <= csv.size()) {
    const auto end = csv.find(',', start);
    if (csv.substr(start, end == std::string::npos ? std::string::npos
                                                   : end - start) == value) {
      return true;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return false;
}

}  // namespace

std::string SerializeSnapshot(const RobotProfile& profile,
                              const RobotSnapshot& snapshot) {
  const auto now = SteadyClock::now();
  Json::Value root = CommonRoot(snapshot, now);

  Json::Value robot(Json::objectValue);
  robot["version"] = Array(snapshot.version);
  robot["mode_pr_raw"] = static_cast<Json::UInt>(snapshot.mode_pr);
  robot["mode_machine_raw"] =
      static_cast<Json::UInt>(snapshot.mode_machine);
  const auto* model = FindModelVariant(profile, snapshot.mode_machine);
  robot["model_supported"] = model != nullptr;
  robot["model_source"] = "local";
  robot["model_dof"] = static_cast<Json::UInt>(profile.joint_schema.joints.size());
  if (model) {
    robot["model_name"] = model->name;
    robot["urdf_file"] = model->urdf_file;
  } else {
    robot["model_name"] = Json::Value(Json::nullValue);
    robot["urdf_file"] = Json::Value(Json::nullValue);
  }
  robot["tick"] = Json::UInt64(snapshot.tick);
  root["robot"] = robot;

  Json::Value battery(Json::objectValue);
  battery["version"] =
      std::to_string(snapshot.battery.version_high) + "." +
      std::to_string(snapshot.battery.version_low);
  battery["function_raw"] =
      static_cast<Json::UInt>(snapshot.battery.function_raw);
  battery["soc_pct"] = static_cast<Json::UInt>(snapshot.battery.soc);
  battery["soh_pct"] = static_cast<Json::UInt>(snapshot.battery.soh);
  battery["current_raw"] = Json::Int64(snapshot.battery.current_raw);
  battery["cell_voltage_raw"] =
      Array(snapshot.battery.cell_voltage_raw);
  battery["voltage_raw"] = Array(snapshot.battery.voltage_raw);
  battery["temperature_raw"] = Array(snapshot.battery.temperature_raw);
  battery["cycle"] = snapshot.battery.cycle;
  battery["manufacturer_date_raw"] =
      snapshot.battery.manufacturer_date_raw;
  battery["state_raw"] = Array(snapshot.battery.state_raw);
  root["battery"] = battery;

  Json::Value odometry(Json::objectValue);
  odometry["error_code"] = Json::UInt64(snapshot.odometry.error_code);
  odometry["mode_raw"] =
      static_cast<Json::UInt>(snapshot.odometry.mode_raw);
  odometry["gait_type_raw"] =
      static_cast<Json::UInt>(snapshot.odometry.gait_type_raw);
  odometry["progress"] = Number(snapshot.odometry.progress);
  odometry["position_m"] = Array(snapshot.odometry.position);
  odometry["velocity_m_s"] = Array(snapshot.odometry.velocity);
  odometry["body_height_m"] = Number(snapshot.odometry.body_height);
  odometry["yaw_speed_rad_s"] = Number(snapshot.odometry.yaw_speed);
  odometry["imu"] = Imu(snapshot.odometry.imu);
  root["odometry"] = odometry;

  Json::Value imu(Json::objectValue);
  imu["hip"] = Imu(snapshot.hip_imu);
  imu["torso"] = Imu(snapshot.torso_imu);
  root["imu"] = imu;

  Json::Value joints(Json::arrayValue);
  std::vector<bool> semantic_slots(snapshot.motors.size(), false);
  for (const auto& joint : profile.joint_schema.joints) {
    if (joint.motor_slot >= snapshot.motors.size()) continue;
    semantic_slots[joint.motor_slot] = true;
    joints.append(MotorJson(snapshot.motors[joint.motor_slot],
                            joint.motor_slot, &joint));
  }
  root["joints"] = joints;

  Json::Value reserved(Json::arrayValue);
  const auto raw_motor_slot_count =
      std::min(snapshot.motors.size(), profile.joint_schema.motor_slot_count);
  for (std::size_t motor_slot = 0; motor_slot < raw_motor_slot_count;
       ++motor_slot) {
    if (!semantic_slots[motor_slot]) {
      reserved.append(MotorJson(snapshot.motors[motor_slot], motor_slot,
                                nullptr));
    }
  }
  root["reserved_motor_slots"] = reserved;

  Json::Value mainboard(Json::objectValue);
  mainboard["fan_state_raw"] = Array(snapshot.mainboard.fan_state_raw);
  mainboard["temperature_raw"] =
      Array(snapshot.mainboard.temperature_raw);
  mainboard["value_raw"] = Array(snapshot.mainboard.value_raw);
  mainboard["state_raw"] = Array(snapshot.mainboard.state_raw);
  root["mainboard"] = mainboard;
  root["voice"] = VoiceJson(snapshot.voice);
  root["control"] = ControlJson(snapshot.control);

  return WriteJson(root);
}

std::string SerializeRobotManifest(const RobotProfile& profile,
                                   const RobotSnapshot& snapshot) {
  Json::Value root(Json::objectValue);
  root["schema_version"] = 1;

  Json::Value identity(Json::objectValue);
  identity["vendor"] = profile.identity.vendor;
  identity["family"] = profile.identity.family;
  identity["product_id"] = profile.identity.product_id;
  identity["variant"] = profile.identity.variant;
  identity["display_name"] = profile.identity.display_name;
  identity["morphology"] = profile.identity.morphology;
  root["identity"] = identity;

  Json::Value model(Json::objectValue);
  const auto* selected_model = FindModelVariant(profile, snapshot.mode_machine);
  model["supported"] = selected_model != nullptr;
  model["asset_root"] = profile.model_asset_root;
  model["package"] = profile.model_package;
  model["dof"] = static_cast<Json::UInt>(profile.joint_schema.joints.size());
  if (selected_model) {
    model["model_name"] = selected_model->name;
    model["urdf_file"] = selected_model->urdf_file;
  } else {
    model["model_name"] = Json::Value(Json::nullValue);
    model["urdf_file"] = Json::Value(Json::nullValue);
  }
  root["model"] = model;

  Json::Value joints(Json::arrayValue);
  for (const auto& descriptor : profile.joint_schema.joints) {
    Json::Value joint(Json::objectValue);
    joint["name"] = descriptor.name;
    joint["name_zh"] = descriptor.name_zh;
    joint["urdf_joint_name"] = descriptor.urdf_joint_name;
    joint["display_group"] = descriptor.display_group;
    joint["motor_slot"] = static_cast<Json::UInt>(descriptor.motor_slot);
    joints.append(joint);
  }
  root["joints"] = joints;

  Json::Value capabilities(Json::arrayValue);
  for (const auto& descriptor : profile.capabilities) {
    Json::Value capability(Json::objectValue);
    capability["key"] = CapabilityKeyName(descriptor.key);
    capability["implemented"] = descriptor.implemented;
    capability["available"] = descriptor.available;
    capability["reason"] = descriptor.reason;
    capability["verification_level"] =
        VerificationLevelName(descriptor.verification_level);
    Json::Value parameters(Json::objectValue);
    for (const auto& [key, value] : descriptor.parameters) {
      parameters[key] = value;
    }
    capability["parameters"] = parameters;
    capabilities.append(capability);
  }
  root["capabilities"] = capabilities;

  Json::Value diagnostics(Json::objectValue);
  diagnostics["mode_machine_raw"] =
      static_cast<Json::UInt>(snapshot.mode_machine);
  root["diagnostics"] = diagnostics;

  return WriteJson(root);
}

std::string SerializeHealth(const RobotProfile& profile,
                            const RobotSnapshot& snapshot) {
  const auto now = SteadyClock::now();
  Json::Value root = CommonRoot(snapshot, now);
  const bool llm_ready =
      (snapshot.voice.llm.mode == "builtin" &&
       !snapshot.voice.chat_go_closed &&
       snapshot.voice.llm.builtin_api_available &&
       snapshot.voice.llm.builtin_response_subscribed) ||
      (snapshot.voice.llm.mode == "customer" &&
       snapshot.voice.chat_go_closed &&
       snapshot.voice.llm.customer_api_available &&
       snapshot.voice.llm.customer_api_configured);
  const bool audio_required =
      ProfileCapabilityAvailable(profile, CapabilityKey::kAudio);
  const bool control_required =
      ProfileCapabilityAvailable(profile, CapabilityKey::kLocomotion) ||
      ProfileCapabilityAvailable(profile, CapabilityKey::kJointDebug) ||
      ProfileCapabilityAvailable(profile, CapabilityKey::kJointTeach);
  bool all_online = snapshot.dds_initialized;
  if (audio_required) all_online = all_online && snapshot.voice.initialized && llm_ready;
  if (control_required) {
    all_online = all_online && snapshot.control.initialized && snapshot.control.enabled;
  }

  const auto source_online = [&](const SourceState& source) {
    const auto age_ms = AgeMs(source, now);
    return ClassifyFreshness(source.received, age_ms < 0 ? 3001 : age_ms) ==
           Freshness::kOnline;
  };
  const auto required_sources = TelemetryRequiredSources(profile);
  for (std::size_t index = 0;
       index < static_cast<std::size_t>(SourceId::kCount); ++index) {
    const auto source = static_cast<SourceId>(index);
    if (CsvContains(required_sources, SourceName(source)) &&
        !source_online(snapshot.sources[index])) {
      all_online = false;
    }
  }
  root["status"] = all_online ? "ok" : "degraded";
  root["motion_control_enabled"] =
      control_required && snapshot.control.initialized && snapshot.control.enabled;
  root["voice_tts_enabled"] = audio_required && snapshot.voice.initialized;
  root["chat_go_closed"] = snapshot.voice.chat_go_closed;
  root["llm_mode"] = snapshot.voice.llm.mode;
  root["llm_ready"] = audio_required && llm_ready;
  return WriteJson(root);
}

std::string SerializeVoiceStatus(const RobotSnapshot& snapshot) {
  Json::Value root(Json::objectValue);
  root["schema_version"] = 1;
  root["voice"] = VoiceJson(snapshot.voice);
  return WriteJson(root);
}

std::string SerializeControlStatus(const RobotSnapshot& snapshot) {
  Json::Value root(Json::objectValue);
  root["schema_version"] = 1;
  root["control"] = ControlJson(snapshot.control);
  return WriteJson(root);
}

}  // namespace g1_web
