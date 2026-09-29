#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>

#include "g1_web/camera_service.hpp"
#include "g1_web/control_service.hpp"
#include "g1_web/device_capability.hpp"
#include "g1_web/go2_audio_capability.hpp"
#include "g1_web/go2_device_capability.hpp"
#include "g1_web/go2_joint_debug_policy.hpp"
#include "g1_web/go2_locomotion_adapter.hpp"
#include "g1_web/json_serializer.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/robot_registry.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/static_assets.hpp"
#include "g1_web/voice_service.hpp"

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "GO2 TEST FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

Json::Value Parse(const std::string& text) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(text);
  Require(Json::parseFromStream(builder, stream, &root, &errors),
          "JSON should parse: " + errors);
  return root;
}

const g1_web::CapabilityDescriptor* FindCapability(
    const g1_web::RobotProfile& profile, g1_web::CapabilityKey key) {
  for (const auto& capability : profile.capabilities) {
    if (capability.key == key) return &capability;
  }
  return nullptr;
}

g1_web::RobotProfile TelemetryOnly(g1_web::RobotProfile profile) {
  for (auto& capability : profile.capabilities) {
    if (capability.key != g1_web::CapabilityKey::kTelemetry) {
      capability.available = false;
    }
  }
  return profile;
}

bool WaitFor(const std::function<bool()>& predicate,
             std::chrono::milliseconds timeout =
                 std::chrono::milliseconds(2000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

std::vector<std::string> SplitCsv(const std::string& value) {
  std::vector<std::string> result;
  std::size_t start = 0;
  while (start <= value.size()) {
    const auto end = value.find(',', start);
    const auto item = value.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    if (!item.empty()) result.push_back(item);
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return result;
}

void TestGo2ProfileAndRuntime() {
  const auto* profile = g1_web::RobotRegistry::Find("go2");
  Require(profile != nullptr, "Go2 must be registered");
  Require(profile->identity.vendor == "unitree" &&
              profile->identity.product_id == "go2" &&
              profile->identity.variant == "go2_edu" &&
              profile->identity.morphology == "quadruped",
          "Go2 identity must be an explicit quadruped profile");
  Require(profile->idl_family == "unitree_sdk2_go2" &&
              profile->control_api_family == "unitree_sdk2_go2_v2_sport" &&
              profile->joint_schema.motor_slot_count == 20 &&
              profile->joint_schema.joints.size() == 12 &&
              profile->capabilities.size() == 10,
          "Go2 must declare 20 raw slots and 12 semantic joints");

  static constexpr const char* kUrdfJoints[] = {
      "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint",
  };
  for (std::size_t index = 0; index < 12; ++index) {
    const auto& joint = profile->joint_schema.joints[index];
    Require(joint.motor_slot == index &&
                joint.urdf_joint_name == kUrdfJoints[index],
            "Go2 semantic joint must map exactly to raw slots 0-11");
  }
  Require(profile->static_constraints.at("reserved_motor_slots") == "12-19" &&
              profile->static_constraints.at("lowstate_topic") ==
                  "rt/lowstate" &&
              profile->static_constraints.at("sport_state_topic") ==
                  "rt/sportmodestate" &&
              profile->static_constraints.at("minimum_software_version") ==
                  "1.1.6" &&
              profile->static_constraints.at("model_asset_kind") ==
                  "official_urdf_and_meshes",
          "Go2 transport, version and model constraints must be explicit");

  const auto* locomotion =
      FindCapability(*profile, g1_web::CapabilityKey::kLocomotion);
  const auto* telemetry =
      FindCapability(*profile, g1_web::CapabilityKey::kTelemetry);
  const auto* joint_debug =
      FindCapability(*profile, g1_web::CapabilityKey::kJointDebug);
  const auto* joint_teach =
      FindCapability(*profile, g1_web::CapabilityKey::kJointTeach);
  const auto* audio = FindCapability(*profile, g1_web::CapabilityKey::kAudio);
  const auto* depth =
      FindCapability(*profile, g1_web::CapabilityKey::kCameraDepth);
  const auto* lidar =
      FindCapability(*profile, g1_web::CapabilityKey::kLidar);
  const auto* head =
      FindCapability(*profile, g1_web::CapabilityKey::kHeadControl);
  Require(telemetry &&
              telemetry->parameters.at("fault_service_topic") ==
                  "rt/errorlist" &&
              telemetry->parameters.at("fault_service_encoding") ==
                  "unitree_errorlist_v1" &&
              telemetry->parameters.at("odometry_error_code_role") ==
                  "motion_state_id" &&
              locomotion && locomotion->implemented && locomotion->available &&
              locomotion->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              locomotion->parameters.at("control_client") ==
                  "go2_v2_sport_client" &&
              locomotion->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              locomotion->parameters.at("web_mode_commands").find("free_walk") !=
                  std::string::npos &&
              locomotion->parameters.at("web_mode_commands").find(
                  "classic_walk_off") != std::string::npos &&
              locomotion->parameters.at("interaction_action_commands").find(
                  "hello") != std::string::npos &&
              locomotion->parameters.at("interaction_action_commands").find(
                  "front_flip") != std::string::npos &&
              locomotion->parameters.at("speed_modes") == "0,1,3" &&
              locomotion->parameters.at("allowed_fsm").find("1013") !=
                  std::string::npos,
          "Go2 locomotion must expose the V2 SportClient modes, interaction actions and conservative velocity presets");
  Require(joint_debug && joint_debug->implemented && joint_debug->available &&
              joint_debug->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_debug->parameters.at("control_modes") == "full_body" &&
              joint_debug->parameters.at("control_topic") == "rt/lowcmd" &&
              joint_teach && joint_teach->implemented &&
              joint_teach->available &&
              joint_teach->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_teach->parameters.at("control_mode") == "full_body" &&
              joint_teach->parameters.at("control_topic") == "rt/lowcmd" &&
              joint_teach->parameters.at("remote_binding") == "true" &&
              joint_teach->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              audio && audio->implemented && audio->available &&
              audio->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              audio->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              audio->parameters.at("play_stream") == "true" &&
              audio->parameters.at("volume") == "true" &&
              audio->parameters.at("tts") == "false" &&
              audio->parameters.at("asr") == "false" &&
              audio->parameters.at("builtin_llm") == "false" &&
              audio->parameters.at("play_stream_transport") ==
                  "audiohub_file_player" &&
              depth && !depth->implemented &&
              lidar && lidar->parameters.at("raw_points_topic") ==
                           "rt/utlidar/cloud" &&
              lidar->parameters.at("raw_points_frame") == "utlidar_lidar" &&
              lidar->parameters.at("deskewed_points_topic") ==
                  "rt/utlidar/cloud_deskewed" &&
              lidar->parameters.at("deskewed_points_frame") == "odom" &&
              head && !head->implemented,
          "Go2 must expose guarded full-body Joint Debug while keeping teach/head unsupported");

  g1_web::RuntimeOptions mock_options;
  mock_options.mock = true;
  auto runtime = g1_web::RobotRegistry::CreateRuntime("go2", mock_options);
  Require(runtime.locomotion && runtime.audio && runtime.joint_debug &&
              runtime.devices && runtime.telemetry.sources.size() == 3,
          "Go2 runtime must assemble audio, locomotion, full-body Joint Debug and shared devices");
  const auto audio_features = runtime.audio->Features();
  g1_web::RuntimeOptions real_options;
  const auto real_runtime = g1_web::RobotRegistry::CreateRuntime("go2", real_options);
  const auto* real_audio =
      FindCapability(real_runtime.profile, g1_web::CapabilityKey::kAudio);
  Require(real_audio && real_audio->available &&
              real_audio->reason == "go2_audio_hardware_unverified",
          "Go2 real audio may be exposed only for explicit operator validation until audible hardware proof");
  Require(audio_features.play_stream && audio_features.volume &&
              !audio_features.tts && !audio_features.asr &&
              !audio_features.builtin_llm &&
              !audio_features.incremental_play_stream &&
              !audio_features.play_state_events,
          "Go2 audio runtime must expose only buffered PlayStream and volume");
  Require(runtime.telemetry.sources[0].key ==
                  g1_web::TelemetrySourceKey::kLowState &&
              runtime.telemetry.sources[0].kind ==
                  g1_web::TelemetrySourceKind::kGo2LowState &&
              runtime.telemetry.sources[0].topic == "rt/lowstate" &&
              runtime.telemetry.sources[0].required_for_health &&
              runtime.telemetry.sources[1].key ==
                  g1_web::TelemetrySourceKey::kOdometry &&
              runtime.telemetry.sources[1].kind ==
                  g1_web::TelemetrySourceKind::kGo2SportModeState &&
              runtime.telemetry.sources[1].topic == "rt/sportmodestate" &&
              runtime.telemetry.sources[1].required_for_health &&
              runtime.telemetry.sources[1].supplies_control_state &&
              runtime.telemetry.sources[2].key ==
                  g1_web::TelemetrySourceKey::kFaults &&
              runtime.telemetry.sources[2].kind ==
                  g1_web::TelemetrySourceKind::kGo2ErrorList &&
              runtime.telemetry.sources[2].topic == "rt/errorlist" &&
              !runtime.telemetry.sources[2].required_for_health,
          "Go2 telemetry plan must bind LowState, SportModeState and the optional official fault collection");

  g1_web::RuntimeOptions readonly_options;
  readonly_options.read_only = true;
  auto readonly_runtime =
      g1_web::RobotRegistry::CreateRuntime("go2", readonly_options);
  const auto* readonly_locomotion =
      FindCapability(readonly_runtime.profile,
                     g1_web::CapabilityKey::kLocomotion);
  Require(readonly_locomotion && readonly_locomotion->implemented &&
              !readonly_locomotion->available &&
              readonly_locomotion->reason == "readonly_runtime",
          "Go2 read-only runtime must keep locomotion implemented but disabled");

  const auto* slam =
      FindCapability(runtime.profile, g1_web::CapabilityKey::kSlam);
  Require(slam && !slam->available &&
              slam->reason == "required_attachment_disabled" &&
              !runtime.devices->Perception().attachment_declared,
          "Go2 SLAM must fail closed when the dock/official lidar attachment is absent");

  g1_web::RuntimeOptions attached_options;
  attached_options.mock = true;
  attached_options.enabled_devices = {"dock", "mid360"};
  auto attached =
      g1_web::RobotRegistry::CreateRuntime("go2", attached_options);
  slam = FindCapability(attached.profile, g1_web::CapabilityKey::kSlam);
  Require(slam && slam->available &&
              attached.devices->Perception().attachment_declared &&
              attached.devices->Perception().dependencies.size() == 2 &&
              attached.devices->Perception().dependencies[0].lifecycle ==
                  g1_web::DependencyLifecycle::kExternalRequired &&
              attached.devices->Perception().dependencies[1].lifecycle ==
                  g1_web::DependencyLifecycle::kExternalRequired,
          "dock+one official lidar may open only the static SLAM gate and must keep services external");

  attached_options.enabled_devices = {"dock", "mid360", "xt16"};
  auto ambiguous =
      g1_web::RobotRegistry::CreateRuntime("go2", attached_options);
  Require(!ambiguous.devices->Perception().attachment_declared,
          "ambiguous dual official lidar declaration must fail closed");
}

void TestGo2MockAudioAndVoice() {
  auto audio = g1_web::CreateGo2AudioCapability(true);
  std::string error;
  Require(audio->Start(error) && error.empty() && audio->Available(),
          "Go2 mock audio capability must start");
  const auto features = audio->Features();
  Require(features.play_stream && features.volume && !features.tts &&
              !features.asr && !features.builtin_llm &&
              !features.incremental_play_stream &&
              !features.play_state_events,
          "Go2 mock audio feature contract must match the real adapter");
  std::uint8_t volume = 0;
  Require(audio->GetVolume(volume) == 0 && volume == 90,
          "Go2 mock volume must use Web percent units");
  Require(audio->SetVolume(64) == 0 && audio->GetVolume(volume) == 0 &&
              volume == 60,
          "Go2 volume must round Web percent to the Vui 0-10 scale");
  Require(audio->TtsMaker("unsupported", 0) != 0,
          "Go2 must not expose Unitree native TTS");
  Require(audio->PlayStream("test", "1", {0, 0, 1, 0}) == 0 &&
              audio->PlayStream("test", "2", {0}) != 0,
          "Go2 PlayStream must accept complete s16 PCM only");
  audio->Stop();

  g1_web::SnapshotStore store;
  g1_web::VoiceService voice(
      store, g1_web::CreateGo2AudioCapability(true), true);
  Require(voice.Start(error) && error.empty(),
          "Go2 mock VoiceService must start without RobotState/chat_go");
  auto snapshot = store.GetSnapshot();
  Require(snapshot.voice.initialized && !snapshot.voice.asr_subscribed &&
              !snapshot.voice.chat_go_found && snapshot.voice.chat_go_closed &&
              snapshot.voice.llm.mode == "customer" &&
              !snapshot.voice.llm.builtin_api_available &&
              snapshot.voice.volume_pct == 90,
          "Go2 voice must default to customer LLM with audio-only capabilities");

  const auto builtin = voice.ChatWithLlm("builtin", "hello");
  Require(!builtin.accepted && builtin.error == "llm_mode_mismatch" &&
              builtin.mode == "customer",
          "Go2 must keep builtin chat_go interaction unavailable");

  g1_web::CustomerVoiceConfig wake_config;
  wake_config.wake_enabled = true;
  wake_config.wake_word = "go2";
  wake_config.tts_backend = "kokoro";
  const auto wake_result = voice.SetCustomerVoiceConfig(wake_config);
  Require(!wake_result.accepted &&
              wake_result.error == "audio_asr_unavailable",
          "Go2 must reject customer wake configuration until microphone ASR exists");

  g1_web::CustomerVoiceConfig native_tts_config;
  native_tts_config.tts_backend = "unitree";
  const auto native_tts_result =
      voice.SetCustomerVoiceConfig(native_tts_config);
  Require(!native_tts_result.accepted &&
              native_tts_result.error == "audio_capability_unavailable",
          "Go2 must reject Unitree native TTS in customer configuration");

  const auto unitree_tts = voice.EnqueueTts("hello", -1, "unitree");
  Require(!unitree_tts.accepted &&
              unitree_tts.error == "audio_capability_unavailable",
          "Go2 must reject the unavailable Unitree TTS backend");
  const auto kokoro = voice.EnqueueTts("hello", -1, "kokoro");
  Require(kokoro.accepted,
          "Go2 must accept Kokoro through buffered PlayStream");
  Require(WaitFor([&store] {
            return store.GetSnapshot().voice.tts.state == "succeeded";
          }),
          "Go2 mock Kokoro request must complete");
  voice.Stop();
}

void PopulateGo2LowState(unitree_go::msg::dds_::LowState_& low) {
  low.version()[0] = 0x01010006U;
  low.version()[1] = 2U;
  low.tick(12345U);
  low.bit_flag(0x91U);
  low.imu_state().rpy()[1] = 0.25F;
  low.imu_state().temperature(41);
  for (std::size_t index = 0; index < 20; ++index) {
    auto& motor = low.motor_state()[index];
    motor.mode(static_cast<std::uint8_t>(index < 12 ? 1 : 0));
    motor.q(static_cast<float>(index) * 0.1F);
    motor.dq(static_cast<float>(index) * -0.01F);
    motor.ddq(static_cast<float>(index) * 0.001F);
    motor.tau_est(static_cast<float>(index) * 0.2F);
    motor.temperature(static_cast<std::uint8_t>(40 + index));
    motor.lost(static_cast<std::uint32_t>(index));
    motor.reserve()[0] = index == 2 ? 0x55U : 0U;
    motor.reserve()[1] = 500U + static_cast<std::uint32_t>(index);
  }
  low.motor_state()[2].temperature(250U);
  auto& bms = low.bms_state();
  bms.version_high(1);
  bms.version_low(9);
  bms.status(3);
  bms.soc(76);
  bms.current(-1234);
  bms.cycle(88);
  bms.bq_ntc()[0] = 200;
  bms.bq_ntc()[1] = 32;
  bms.mcu_ntc()[0] = 33;
  bms.mcu_ntc()[1] = 34;
  for (std::size_t index = 0; index < bms.cell_vol().size(); ++index) {
    bms.cell_vol()[index] =
        static_cast<std::uint16_t>(3800U + index);
  }
}

void PopulateGo2Sport(unitree_go::msg::dds_::SportModeState_& sport) {
  sport.error_code(1001U);
  sport.mode(3);
  sport.gait_type(2);
  sport.progress(0.75F);
  sport.position()[0] = 1.25F;
  sport.position()[1] = -0.5F;
  sport.velocity()[0] = 0.4F;
  sport.velocity()[1] = 0.1F;
  sport.body_height(0.32F);
  sport.yaw_speed(0.2F);
  sport.imu_state().rpy()[2] = 0.4F;
}

void TestGo2NormalizationFreshnessAndSerialization() {
  g1_web::SnapshotStore store;
  store.SetDdsStatus(true);
  unitree_go::msg::dds_::LowState_ low;
  PopulateGo2LowState(low);
  store.UpdateGo2LowState(low);

  auto snapshot = store.GetSnapshot();
  Require(snapshot.tick == 12345U &&
              snapshot.low_state_component_state_raw == 0x91U &&
              std::abs(snapshot.hip_imu.rpy[1] - 0.25F) < 1e-6 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kLowState)].received &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kBms)].received,
          "Go2 LowState must update IMU, component bits, lowstate and embedded BMS freshness");
  Require(snapshot.motors[2].state_raw == 0x55U &&
              snapshot.motors[2].sensor_raw[0] == 2U &&
              snapshot.motors[2].sensor_raw[1] == 502U &&
              snapshot.motors[2].temperature[0] == 250 &&
              std::abs(snapshot.motors[11].q - 1.1F) < 1e-6 &&
              snapshot.motors[12].mode == 0 &&
              snapshot.motors[20].mode == 0,
          "Go2 motor normalizer must preserve reserve[0] faults and 20-slot wire layout");
  Require(snapshot.battery.version_high == 1 &&
              snapshot.battery.version_low == 9 &&
              snapshot.battery.function_raw == 3 &&
              snapshot.battery.soc == 76 &&
              snapshot.battery.current_raw == -1234 &&
              snapshot.battery.cycle == 88 &&
              snapshot.battery.cell_voltage_raw[14] == 3814 &&
              snapshot.battery.temperature_raw[0] == 200 &&
              snapshot.battery.temperature_raw[3] == 34 &&
              snapshot.battery.state_raw[0] == 3,
          "Go2 embedded BMS fields must normalize without a separate BMS topic");

  g1_web::RuntimeOptions mock_options;
  mock_options.mock = true;
  auto runtime = g1_web::RobotRegistry::CreateRuntime("go2", mock_options);
  const auto telemetry_profile = TelemetryOnly(runtime.profile);
  auto health = Parse(
      g1_web::SerializeHealth(telemetry_profile, store.GetSnapshot()));
  Require(health["status"].asString() == "degraded" &&
              health["sources"]["low_state"]["status"].asString() ==
                  "online" &&
              health["sources"]["bms"]["status"].asString() == "online" &&
              health["sources"]["odometry"]["status"].asString() ==
                  "offline",
          "Go2 single-source LowState must degrade health while SportModeState is missing");

  unitree_go::msg::dds_::SportModeState_ sport;
  PopulateGo2Sport(sport);
  store.UpdateGo2SportMode(sport);
  store.UpdateGo2FaultList(
      R"({"errors":[[1720000000000,100,128],[1720000000100,302,5],[1720000000200,400,2]]})");
  snapshot = store.GetSnapshot();
  Require(snapshot.odometry.error_code == 1001U &&
              snapshot.odometry.mode_raw == 3 &&
              snapshot.odometry.gait_type_raw == 2 &&
              std::abs(snapshot.odometry.position[0] - 1.25F) < 1e-6 &&
              std::abs(snapshot.odometry.velocity[0] - 0.4F) < 1e-6 &&
              std::abs(snapshot.odometry.body_height - 0.32F) < 1e-6 &&
              snapshot.control.sport_state_received &&
              snapshot.control.fsm_id == 1001U &&
              snapshot.faults.received && snapshot.faults.parse_valid &&
              snapshot.faults.entries.size() == 3 &&
              snapshot.faults.entries[1].source == 302U &&
              snapshot.faults.entries[1].code == 5U,
          "Go2 V2 state and rt/errorlist must preserve their distinct control and fault contracts");
  health = Parse(g1_web::SerializeHealth(telemetry_profile, snapshot));
  Require(health["status"].asString() == "ok" &&
              health["sources"]["odometry"]["status"].asString() == "online",
          "Go2 health must recover when both required sources are fresh");

  g1_web::SnapshotStore sport_only_store;
  sport_only_store.SetDdsStatus(true);
  sport_only_store.UpdateGo2SportMode(sport);
  health = Parse(g1_web::SerializeHealth(
      telemetry_profile, sport_only_store.GetSnapshot()));
  Require(health["status"].asString() == "degraded" &&
              health["sources"]["low_state"]["status"].asString() ==
                  "offline" &&
              health["sources"]["odometry"]["status"].asString() == "online",
          "Go2 single-source SportModeState must degrade health while LowState is missing");

  const auto root =
      Parse(g1_web::SerializeSnapshot(runtime.profile, snapshot));
  Require(root["robot"]["model_supported"].asBool() &&
              root["robot"]["urdf_file"].asString() ==
                  "go2_description.urdf" &&
              root["robot"]["low_state_component_state_raw"].asUInt() ==
                  0x91U &&
              root["fault_service"]["topic"].asString() == "rt/errorlist" &&
              root["fault_service"]["parse_valid"].asBool() &&
              root["fault_service"]["entries"].size() == 3 &&
              root["fault_service"]["entries"][2]["source"].asUInt() ==
                  400U &&
              root["joints"].size() == 12 &&
              root["reserved_motor_slots"].size() == 8,
          "Go2 snapshot must select the official model and expose 12 semantic plus 8 reserved slots");

  store.UpdateGo2FaultList(R"({"errors":[[1,300]]})");
  snapshot = store.GetSnapshot();
  Require(snapshot.faults.received && !snapshot.faults.parse_valid &&
              snapshot.faults.parse_error == "invalid_errorlist_entry" &&
              snapshot.faults.entries.empty(),
          "Go2 fault collection must fail closed on malformed entries instead of retaining stale faults");
  for (Json::ArrayIndex index = 0; index < 12; ++index) {
    Require(root["joints"][index]["index"].asUInt() == index,
            "Go2 semantic JSON mapping must stay raw slots 0-11");
  }
  for (Json::ArrayIndex index = 0; index < 8; ++index) {
    Require(root["reserved_motor_slots"][index]["index"].asUInt() ==
                index + 12,
            "Go2 reserved JSON mapping must stay raw slots 12-19");
  }

  const auto manifest =
      Parse(g1_web::SerializeRobotManifest(runtime.profile, snapshot));
  Require(manifest["identity"]["product_id"].asString() == "go2" &&
              manifest["identity"]["morphology"].asString() == "quadruped" &&
              manifest["model"]["asset_root"].asString() ==
                  "/assets/unitree/go2_description" &&
              manifest["model"]["urdf_file"].asString() ==
                  "go2_description.urdf" &&
              manifest["capabilities"][1]["parameters"]["web_mode_labels"]
                      .asString()
                      .find("stand_up:起立|Stand up|StandUp") !=
                  std::string::npos &&
              manifest["joints"].size() == 12,
          "Go2 Manifest must be entirely Profile-driven");
}

void TestGo2DevicePolicyAndFailClosedPaths() {
  auto policy = g1_web::CreateGo2DeviceCapabilityPolicy();
  const auto& perception = policy->Perception();
  const auto& camera = policy->Camera();
  Require(perception.raw_lidar_points_topic == "rt/utlidar/cloud" &&
              perception.raw_point_source == "utlidar_lidar" &&
              !perception.raw_input_required &&
              !perception.attachment_declared &&
              perception.service_version_status == "unverified",
          "Go2 onboard LiDAR and unverified SLAM conditions must be explicit");
  const auto* rgb =
      g1_web::FindCameraStream(camera, g1_web::CameraStreamRole::kRgb);
  Require(rgb && rgb->transport == g1_web::CameraTransport::kRtpH264Udp &&
              rgb->fixed_source == "udp://230.1.1.1:1720" &&
              rgb->port == 1720 && rgb->width == 1280 &&
              rgb->height == 720 && rgb->fps == 15 &&
              !camera.allow_client_source_override,
          "Go2 RGB must use the fixed official H264 multicast source");
  const auto pipeline = g1_web::BuildFixedRtpH264UdpPipeline(*rgb);
  Require(pipeline.find("address=230.1.1.1") != std::string::npos &&
              pipeline.find("port=1720") != std::string::npos &&
              pipeline.find("encoding-name=H264") != std::string::npos &&
              !g1_web::CameraFrameFresh(true, 2001),
          "Go2 camera pipeline must pin multicast source and reject stale frames");

  g1_web::CameraOptions camera_options;
  camera_options.mock = true;
  g1_web::CameraService camera_service(camera_options, *policy);
  std::string camera_error;
  Require(camera_service.Start(camera_error),
          "Go2 mock camera should start with its RGB-only fixed policy");
  g1_web::CameraRequest camera_request;
  camera_request.request_key = "go2-fixed-rgb-start";
  camera_request.command = "start_v4l2";
  camera_request.confirmed = true;
  Require(camera_service.Submit(camera_request).accepted,
          "Go2 fixed RGB start must not require an unsupported depth stream");
  camera_service.Stop();

  g1_web::PerceptionService readonly_probe(false, false, *policy);
  const auto device_states = readonly_probe.DeviceCapabilities();
  const auto lidar_state = device_states[0];
  const auto slam_state = device_states[1];
  Require(lidar_state.key == g1_web::CapabilityKey::kLidar &&
              lidar_state.hardware_presence ==
                  g1_web::HardwarePresence::kUnknown &&
              slam_state.key == g1_web::CapabilityKey::kSlam &&
              slam_state.hardware_presence ==
                  g1_web::HardwarePresence::kAbsent &&
              slam_state.reason == "required_attachment_disabled",
          "without fresh LiDAR samples or attachment evidence device state must stay fail-closed");

  g1_web::RuntimeOptions real_options;
  auto runtime = g1_web::RobotRegistry::CreateRuntime("go2", real_options);
  auto stale_profile = runtime.profile;
  g1_web::DeviceCapabilityRuntime stale_lidar;
  stale_lidar.key = g1_web::CapabilityKey::kLidar;
  stale_lidar.hardware_presence = g1_web::HardwarePresence::kUnknown;
  stale_lidar.reason = "lidar_topic_stale";
  g1_web::ApplyEffectiveDeviceCapabilities(stale_profile, {stale_lidar});
  const auto* lidar =
      FindCapability(stale_profile, g1_web::CapabilityKey::kLidar);
  Require(lidar && !lidar->available && lidar->reason == "lidar_topic_stale",
          "stale Go2 LiDAR evidence must tighten availability");

  real_options.enabled_devices = {"dock", "xt16"};
  auto attached = g1_web::RobotRegistry::CreateRuntime("go2", real_options);
  Require(attached.devices->Perception().slam_service == "unitree_slam" &&
              attached.devices->Perception().dependencies[0].name ==
                  "xt16_driver" &&
              attached.devices->Perception().dependencies[1].name ==
                  "unitree_slam",
          "Go2 SLAM policy must require the documented external driver and unitree_slam process");
  g1_web::DeviceCapabilityRuntime missing_service;
  missing_service.key = g1_web::CapabilityKey::kSlam;
  missing_service.hardware_presence = g1_web::HardwarePresence::kPresent;
  missing_service.service_available = false;
  missing_service.reason = "slam_dependency_unavailable";
  auto service_profile = attached.profile;
  g1_web::ApplyEffectiveDeviceCapabilities(service_profile, {missing_service});
  const auto* slam =
      FindCapability(service_profile, g1_web::CapabilityKey::kSlam);
  Require(slam && !slam->available &&
              slam->reason == "slam_dependency_unavailable",
          "missing Go2 SLAM service must fail closed");

  g1_web::DeviceCapabilityRuntime unverified_firmware;
  unverified_firmware.key = g1_web::CapabilityKey::kSlam;
  unverified_firmware.hardware_presence = g1_web::HardwarePresence::kPresent;
  unverified_firmware.service_compatible = false;
  unverified_firmware.service_version_status = "unverified";
  unverified_firmware.reason = "firmware_version_unverified";
  auto firmware_profile = attached.profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      firmware_profile, {unverified_firmware});
  slam = FindCapability(firmware_profile, g1_web::CapabilityKey::kSlam);
  Require(slam && !slam->available &&
              slam->reason == "firmware_version_unverified" &&
              slam->parameters.at("required_firmware_version") == "1.1.7",
          "unverified Go2 SLAM firmware must fail closed");
}

void TestGo2MockLocomotion() {
  auto locomotion = g1_web::CreateGo2LocomotionAdapter(true);
  g1_web::LocomotionInitialization initial;
  std::string error;
  Require(locomotion->Initialize(initial, error) && initial.state_valid &&
              initial.fsm_id == 1013U && error.empty(),
          "Go2 Mock V2 locomotion must initialize in documented balance stand");
  g1_web::ControlData control;
  control.fsm_id = 1013U;

  const auto* profile = g1_web::RobotRegistry::Find("go2");
  const auto* capability =
      profile ? FindCapability(*profile, g1_web::CapabilityKey::kLocomotion)
              : nullptr;
  Require(capability != nullptr,
          "Go2 locomotion command catalog must be available");
  for (const auto& command :
       SplitCsv(capability->parameters.at("mode_commands"))) {
    Require(locomotion->IsKnownModeCommand(command, 0),
            "Go2 adapter must recognize declared command: " + command);
    const auto execution = locomotion->ExecuteMode(command, 0);
    Require(execution.api_result == 0 && execution.error.empty(),
            "Go2 mock adapter must execute declared command: " + command);
  }
  const auto mode_sources = capability->parameters.at("mode_sources");
  for (const auto& command :
       SplitCsv(capability->parameters.at("web_mode_commands"))) {
    if (command == "damp" || command == "stop_move") continue;
    Require(mode_sources.find(command + ":") != std::string::npos,
            "Go2 Web mode command must declare a transition source: " +
                command);
  }

  Require(locomotion->ValidateVelocity(control, 0.2F, 0.0F, 0.1F, 0)
              .empty() &&
              locomotion->ValidateVelocity(control, 0.6F, 0.0F, 0.0F, 0) ==
                  "velocity_out_of_range" &&
              locomotion->ValidateVelocity(control, 0.3F, 0.0F, 0.0F, 1)
                  .empty() &&
              locomotion->ValidateVelocity(control, 0.51F, 0.0F, 0.0F, 3) ==
                  "velocity_out_of_range",
          "Go2 Mock velocity validation must expose three conservative Web presets within the hard limit");
  Require(locomotion->ExecuteMode("damp", 0).api_result == 0 &&
              locomotion->ExecuteMode("damp", 0).target_fsm == 1001 &&
              locomotion->ExecuteMode("balance_stand", 0).api_result == 0 &&
              locomotion->ExecuteMode("free_walk", 0).target_fsm == 100 &&
              locomotion->ExecuteMode("pose_off", 0).target_fsm == -1 &&
              locomotion->ExecuteMode("hello", 0).api_result == 0 &&
              locomotion->ExecuteMode("front_flip", 0).api_result == 0 &&
              locomotion->ApplyVelocity(0.2F, 0.0F, 0.1F, 0, true)
                      .api_result == 0 &&
              locomotion->ExecuteMode("special_action", 0).error ==
                  "unknown_or_disallowed_command",
          "Go2 Mock adapter must cover the explicit V2 locomotion and interaction surface");
  locomotion->Shutdown();
}

void TestGo2MockControlSequences() {
  const auto* profile = g1_web::RobotRegistry::Find("go2");
  Require(profile != nullptr, "Go2 profile must exist for control tests");

  g1_web::SnapshotStore store;
  g1_web::ControlService control(
      store, g1_web::CreateGo2LocomotionAdapter(true), *profile, nullptr,
      true);
  std::string error;
  Require(control.Start(error) && error.empty(),
          "Go2 mock control service must start");
  Require(store.GetSnapshot().control.fsm_id == 1013U,
          "Go2 mock control must start in balance stand");

  auto submit_mode = [&](const std::string& key, const std::string& command,
                         std::uint32_t expected_fsm) {
    const auto result = control.Submit(key, "mode", command, 0, true);
    Require(result.accepted,
            "Go2 mode command should be accepted: " + command + " / " +
                result.error);
    Require(WaitFor([&] {
              const auto state = store.GetSnapshot().control;
              return state.last_command.request_id == result.request_id &&
                     state.last_command.state == "succeeded" &&
                     state.fsm_id == expected_fsm;
            }),
            "Go2 mode command should settle at expected FSM: " + command);
  };

  submit_mode("go2-locked-stand", "stand_up", 1002U);
  const auto hello =
      control.Submit("go2-hello-locked", "mode", "hello", 0, true);
  Require(hello.accepted && WaitFor([&] {
            const auto state = store.GetSnapshot().control;
            return state.last_command.request_id == hello.request_id &&
                   state.last_command.state == "succeeded" &&
                   state.fsm_id == 1002U;
          }),
          "Go2 interaction action should be available from a stable locked stand without inventing a target FSM");

  submit_mode("go2-balance-before-free", "balance_stand", 1013U);

  submit_mode("go2-free-walk", "free_walk", 100U);
  const auto free_hello =
      control.Submit("go2-hello-free", "mode", "hello", 0, true);
  Require(free_hello.accepted && WaitFor([&] {
            const auto state = store.GetSnapshot().control;
            return state.last_command.request_id == free_hello.request_id &&
                   state.last_command.state == "succeeded" &&
                   state.fsm_id == 100U;
          }),
          "Go2 interaction action should be available from stationary free walk");
  submit_mode("go2-damp-before-block", "damp", 1001U);
  const auto blocked_hello =
      control.Submit("go2-hello-blocked", "mode", "hello", 0, true);
  Require(!blocked_hello.accepted &&
              blocked_hello.error == "mode_transition_not_allowed",
          "Go2 interaction action must remain blocked in damping state");
  submit_mode("go2-recover-after-block", "recovery_stand", 1013U);
  submit_mode("go2-static-walk", "static_walk", 1015U);
  submit_mode("go2-classic-on", "classic_walk_on", 2010U);
  submit_mode("go2-classic-off", "classic_walk_off", 100U);
  submit_mode("go2-balance", "balance_stand", 1013U);

  const auto motion = control.SubmitVelocity(0.2F, 0.0F, 0.1F, 0, true);
  Require(motion.accepted && WaitFor([&] {
            return store.GetSnapshot().control.motion.state == "active";
          }),
          "Go2 mock velocity command should become active");
  const auto blocked_mode =
      control.Submit("go2-mode-while-moving", "mode", "free_walk", 0, true);
  Require(!blocked_mode.accepted && blocked_mode.error == "motion_active",
          "Go2 mode switching must be blocked while Web motion owns locomotion");
  const auto stop = control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);
  Require(stop.accepted && WaitFor([&] {
            return store.GetSnapshot().control.motion.state == "stopped";
          }),
          "Go2 mock dead-man release should stop velocity deterministically");

  submit_mode("go2-pose-on", "pose_on", 1091U);
  const auto pose_off =
      control.Submit("go2-pose-off", "mode", "pose_off", 0, true);
  Require(pose_off.accepted && WaitFor([&] {
            const auto state = store.GetSnapshot().control;
            return state.last_command.request_id == pose_off.request_id &&
                   state.last_command.state == "succeeded";
          }),
          "Go2 Pose(false) should execute without guessing an undocumented target FSM");
  control.Stop();
}

void TestGo2MockJointDebugger() {
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  const auto teach_store = std::filesystem::temp_directory_path() /
      ("go2_web_joint_teach_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) +
       ".json");
  const auto* profile = g1_web::RobotRegistry::Find("go2");
  Require(profile != nullptr, "Go2 profile must exist for Joint Debug");

  g1_web::SnapshotStore store;
  unitree_go::msg::dds_::LowState_ low;
  for (std::size_t index = 0; index < 12; ++index) {
    low.motor_state()[index].mode(1);
    low.motor_state()[index].q(index % 3 == 1 ? 0.8F
                                  : index % 3 == 2 ? -1.5F
                                                   : 0.0F);
  }
  store.UpdateGo2LowState(low);

  g1_web::ControlService control(
      store, g1_web::CreateGo2LocomotionAdapter(true), *profile,
      g1_web::CreateGo2JointDebugPolicy(*profile), true, web_root.string(),
      teach_store.string());
  std::string error;
  Require(control.Start(error), "Go2 Mock Joint Debug should start: " + error);

  const auto initial = Parse(control.SerializeJointDebugStatus());
  Require(initial["initialized"].asBool() && initial["joints"].size() == 12 &&
              initial["full_body_allowed"].asBool() &&
              !initial["upper_body_allowed"].asBool() &&
              initial["control_topics"]["full_body"].asString() ==
                  "rt/lowcmd" &&
              initial["control_topics"]["upper_body"].isNull() &&
              initial["motion_service_names"].size() == 4 &&
              initial["motion_mode_switch_supported"].asBool() &&
              initial["position_kp"].asFloat() == 60.0F &&
              initial["position_kd"].asFloat() == 5.0F &&
              initial["teach_mode"].asString() == "full_body" &&
              initial["teach_control_topic"].asString() == "rt/lowcmd" &&
              initial["remote_binding_supported"].asBool() &&
              initial["remote_control_ready"].asBool() &&
              initial["remote_binding_options"].size() == 16 &&
              initial["remote_binding_options"][8]["label"].asString() ==
                  "F2+A" &&
              initial["teach_actions"].empty(),
          "Go2 Joint Debug status must expose guarded full-body teaching and Go2 remote bindings");

  const auto upper = control.ApplyJointDebug("upper_body", {{0, 0.0F}}, true);
  Require(!upper.accepted && upper.error == "invalid_control_mode",
          "Go2 must reject the humanoid upper-body debug mode");

  const auto record = control.StartJointTeachRecording("go2_teach", true);
  Require(record.accepted,
          "Go2 must record all 12 joints through the guarded full-body LowCmd path");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const auto recording_stats = control.GetJointDebugTestStats();
  Require(recording_stats.lowcmd_published &&
              recording_stats.last_kp[0] == 0.0F &&
              recording_stats.last_kd[0] == 1.0F,
          "Go2 teaching must use passive Kp=0/Kd=1 recording gains");
  Require(control.FinishJointTeachRecording(true, true).accepted,
          "Go2 must save and release a full-body taught action");
  Require(control.SetJointTeachRemoteBinding("go2_teach", "F3+A").accepted,
          "Go2 must persist the shared second-function-key remote binding");
  const auto taught = Parse(control.SerializeJointDebugStatus());
  Require(taught["teach_actions"].size() == 1 &&
              taught["teach_actions"][0]["frames"].asUInt() >= 2 &&
              taught["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "Go2 taught action must retain 12-joint frames and its remote binding");
  control.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  control.SetMockJointDebugRemoteKeys(0);
  const auto remotely_started = Parse(control.SerializeJointDebugStatus());
  Require(remotely_started["remote_last_action"].asString() == "go2_teach" &&
              remotely_started["remote_last_binding"].asString() == "F3+A",
          "Go2 remote F2+A edge must replay the bound taught action");
  Require(WaitFor([&] {
            return Parse(control.SerializeJointDebugStatus())["dds_state"]
                       .asString() == "idle";
          }),
          "Go2 taught-action playback must finish and release LowCmd");

  control.SetMockJointDebugAiSport(true);
  const auto enter_low = control.SwitchJointDebugMode(true, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Require(enter_low.accepted &&
              store.GetSnapshot().control.fsm_id == 1004U &&
              control.GetJointDebugTestStats().lowcmd_published &&
              !Parse(control.SerializeJointDebugStatus())
                   ["motion_service_active"].asBool(),
          "Go2 enter-low-level workflow must lie down and hold before releasing MCF");
  const auto exit_low = control.SwitchJointDebugMode(false, true);
  Require(exit_low.accepted && store.GetSnapshot().control.fsm_id == 100U &&
              Parse(control.SerializeJointDebugStatus())
                  ["motion_service_active"].asBool(),
          "Go2 exit-low-level workflow must restore MCF, recover stand, then enter FreeWalk");

  std::vector<float> standing_pose(12, 0.0F);
  for (const auto index : {1U, 4U, 7U, 10U}) standing_pose[index] = 0.8F;
  for (const auto index : {2U, 5U, 8U, 11U}) standing_pose[index] = -1.5F;
  std::vector<float> crouched_pose = standing_pose;
  for (const auto index : {1U, 4U, 7U, 10U}) crouched_pose[index] = 1.3F;
  for (const auto index : {2U, 5U, 8U, 11U}) crouched_pose[index] = -2.65F;
  const auto go2_policy = g1_web::CreateGo2JointDebugPolicy(*profile);
  const auto recording_gain =
      go2_policy->RecordingGain(profile->joint_schema.joints.front());
  Require(go2_policy->TeachMode() == "full_body" &&
              go2_policy->TeachGroups() ==
                  std::vector<std::string>({"front_right_leg",
                                            "front_left_leg",
                                            "rear_right_leg",
                                            "rear_left_leg"}) &&
              go2_policy->SupportsTeachRemoteControl() &&
              go2_policy->TeachRemoteSecondaryKeyLabel() == "F2" &&
              recording_gain.kp == 0.0F && recording_gain.kd == 1.0F &&
              go2_policy->HighLevelRecoveryPoseReached(standing_pose) &&
              !go2_policy->HighLevelRecoveryPoseReached(crouched_pose),
          "Go2 teaching policy must use passive full-body LowCmd and the Go2 remote labels");

  for (std::size_t index = 0; index < 12; ++index) {
    low.motor_state()[index].q(index % 3 == 1 ? 1.3F
                                  : index % 3 == 2 ? -2.65F
                                                   : 0.0F);
  }
  store.UpdateGo2LowState(low);
  unitree_hg::msg::dds_::SportModeState_ damping;
  damping.fsm_id(1001);
  store.UpdateSportMode(damping);
  const auto enter_from_damping = control.SwitchJointDebugMode(true, true);
  Require(enter_from_damping.accepted &&
              store.GetSnapshot().control.fsm_id == 1001U &&
              !Parse(control.SerializeJointDebugStatus())
                   ["motion_service_active"].asBool(),
          "an already crouched Go2 in damping must release MCF without issuing another motion command");
  Require(control.StopJointDebug(true).accepted,
          "Go2 automatic crouch hold must stop cleanly in Mock");

  std::vector<std::pair<std::size_t, float>> targets;
  for (Json::ArrayIndex index = 0; index < initial["joints"].size(); ++index) {
    const auto& joint = initial["joints"][index];
    targets.emplace_back(
        static_cast<std::size_t>(index),
        (joint["lower"].asFloat() + joint["upper"].asFloat()) * 0.5F);
  }

  control.SetMockJointDebugAiSport(true);
  const auto publish_count_before_block =
      control.GetJointDebugTestStats().publish_count;
  const auto blocked = control.ApplyJointDebug("full_body", targets, true);
  Require(!blocked.accepted && blocked.error == "debug_mode_required" &&
              control.GetJointDebugTestStats().publish_count ==
                  publish_count_before_block,
          "an active high-level motion service must block Go2 rt/lowcmd");

  control.SetMockJointDebugAiSport(false);
  control.SetMockJointDebugServiceDelay(std::chrono::milliseconds(100));
  const auto loop_starts_before_apply =
      control.GetJointDebugTestStats().loop_starts;
  const auto apply = control.ApplyJointDebug("full_body", targets, true);
  Require(apply.accepted,
          "Go2 full-body Joint Debug should accept a complete in-limit target set: " +
              apply.error);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  Require(control.GetJointDebugTestStats().publish_count >= 5,
          "slow Go2 service checks must not block the 500 Hz rt/lowcmd writer");
  control.SetMockJointDebugServiceDelay(std::chrono::milliseconds(0));
  Require(WaitFor([&] {
            return control.GetJointDebugTestStats().lowcmd_published;
          }),
          "Go2 full-body Joint Debug must drive the mock rt/lowcmd writer");

  const auto stats = control.GetJointDebugTestStats();
  Require(stats.publish_count > 0 &&
              stats.loop_starts == loop_starts_before_apply + 1 &&
              stats.last_kp.size() == 20 && stats.last_kd.size() == 20 &&
              stats.maximum_step_rad <= 0.00051F,
          "Go2 Joint Debug must keep the shared 500 Hz / 0.25 rad/s interpolation bound: starts=" +
              std::to_string(stats.loop_starts) + "/" +
              std::to_string(loop_starts_before_apply + 1) + " step=" +
              std::to_string(stats.maximum_step_rad) + " kp=" +
              std::to_string(stats.last_kp.size()) + " kd=" +
              std::to_string(stats.last_kd.size()));
  for (std::size_t index = 0; index < 12; ++index) {
    Require(stats.last_kp[index] == 60.0F && stats.last_kd[index] == 5.0F,
            "Go2 active leg joints must use the official 60/5 stand-example gains");
  }
  for (std::size_t index = 12; index < 20; ++index) {
    Require(stats.last_kp[index] == 0.0F && stats.last_kd[index] == 0.0F,
            "Go2 reserved motor slots 12-19 must remain untouched by semantic targets");
  }

  const auto motion_conflict =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(!motion_conflict.accepted && motion_conflict.error == "control_busy",
          "active Go2 full-body debug must exclude Web locomotion");

  control.SetMockJointDebugAiSport(true);
  Require(WaitFor([&] {
            const auto status = Parse(control.SerializeJointDebugStatus());
            return status["dds_state"].asString() == "idle" &&
                   status["last_error"].asString() == "debug_mode_required";
          }),
          "Go2 service monitoring must stop rt/lowcmd when high-level motion resumes");
  control.SetMockJointDebugAiSport(false);
  Require(control.ApplyJointDebug("full_body", targets, true).accepted,
          "Go2 Joint Debug should restart after the motion service is released");
  Require(control.StopJointDebug(true).accepted,
          "Go2 full-body debug should release deterministically");
  Require(WaitFor([&] {
            return Parse(control.SerializeJointDebugStatus())["dds_state"].asString() ==
                   "idle";
          }),
          "Go2 Joint Debug should return to idle after release");
  control.Stop();
  std::error_code remove_error;
  std::filesystem::remove(teach_store, remove_error);
}

void TestGo2AssetsAndSharedDom() {
  const auto project_root =
      std::filesystem::path(__FILE__).parent_path().parent_path();
  const auto web_root = project_root / "web";
  std::filesystem::path resolved;
  Require(g1_web::ResolveStaticAsset(
              web_root,
              "/assets/unitree/go2_description/go2_description.urdf",
              resolved),
          "Go2 official URDF must be served by the shared static asset path");
  std::ifstream urdf_stream(resolved);
  const std::string urdf((std::istreambuf_iterator<char>(urdf_stream)),
                         std::istreambuf_iterator<char>());
  Require(urdf.find("package://go2_description/dae/base.dae") !=
                  std::string::npos &&
              urdf.find("<joint name=\"FR_hip_joint\" type=\"revolute\">") !=
                  std::string::npos &&
              urdf.find("<joint name=\"RL_calf_joint\" type=\"revolute\">") !=
                  std::string::npos,
          "Go2 viewer model must be the official mesh-backed URDF, not a generated placeholder");
  Require(g1_web::ResolveStaticAsset(
              web_root,
              "/assets/unitree/go2_description/dae/base.dae",
              resolved),
          "Go2 official DAE meshes must be served locally");
  std::ifstream source_stream(
      web_root / "assets/unitree/go2_description/SOURCE.md");
  const std::string source((std::istreambuf_iterator<char>(source_stream)),
                           std::istreambuf_iterator<char>());
  Require(source.find("ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee") !=
              std::string::npos,
          "Go2 model import must retain the verified official commit");

  std::ifstream app_stream(web_root / "app.js");
  const std::string app((std::istreambuf_iterator<char>(app_stream)),
                        std::istreambuf_iterator<char>());
  std::ifstream index_stream(web_root / "index.html");
  const std::string index((std::istreambuf_iterator<char>(index_stream)),
                          std::istreambuf_iterator<char>());
  std::ifstream viewer_stream(web_root / "robot-viewer.js");
  const std::string viewer((std::istreambuf_iterator<char>(viewer_stream)),
                           std::istreambuf_iterator<char>());
  std::ifstream perception_stream(web_root / "perception.js");
  const std::string perception(
      (std::istreambuf_iterator<char>(perception_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream joint_debug_stream(web_root / "joint-debug.js");
  const std::string joint_debug(
      (std::istreambuf_iterator<char>(joint_debug_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream styles_stream(web_root / "styles.css");
  const std::string styles((std::istreambuf_iterator<char>(styles_stream)),
                           std::istreambuf_iterator<char>());
  Require(app.find("product_id === \"go2\"") == std::string::npos &&
              app.find("product_id === 'go2'") == std::string::npos &&
              app.find("if (go2") == std::string::npos &&
              app.find("audioParameters.asr !== \"false\"") !=
                  std::string::npos &&
              app.find("audioParameters.tts !== \"false\"") !=
                  std::string::npos &&
              app.find("builtin_llm !== \"false\"") != std::string::npos &&
              index.find("id=\"controlTabArm\"") != std::string::npos &&
              index.find("id=\"interactionActionGrid\"") !=
                  std::string::npos &&
              app.find("interaction_action_commands") != std::string::npos &&
              app.find("const profileAllowed = actionIds.has") !=
                  std::string::npos &&
              app.find("parameters.firmware_teach_actions === \"true\"") !=
                  std::string::npos &&
              app.find("createProfileControlButton") != std::string::npos &&
              app.find("decodeGo2Fault") != std::string::npos &&
              app.find("odometry_error_code_role") != std::string::npos &&
              index.find("id=\"diagnosticFaultServiceList\"") !=
                  std::string::npos &&
              index.find("rt/errorlist") != std::string::npos,
          "same Web DOM must fail closed while the manifest selects humanoid or quadruped actions");
  Require(joint_debug.find("front_right_leg") != std::string::npos &&
              joint_debug.find("front_left_leg") != std::string::npos &&
              joint_debug.find("rear_right_leg") != std::string::npos &&
              joint_debug.find("rear_left_leg") != std::string::npos &&
              joint_debug.find("supported.length <= 1") != std::string::npos &&
              joint_debug.find("next?.teach_mode || \"upper_body\"") !=
                  std::string::npos &&
              joint_debug.find("next?.teach_control_topic || \"rt/arm_sdk\"") !=
                  std::string::npos &&
              joint_debug.find("remote_secondary_function_key") !=
                  std::string::npos &&
              joint_debug.find("product_id === \"go2\"") ==
                  std::string::npos &&
              styles.find("data-layout=\"quadruped\"") != std::string::npos,
          "Go2 Joint Debug UI must render full-body teaching and remote labels from capability data without a product-id branch");
  Require(viewer.find("const meshCache = new Map()") != std::string::npos &&
              viewer.find("manager.onLoad = () =>") != std::string::npos &&
              viewer.find("latestData?.sources?.low_state?.status") !=
                  std::string::npos &&
              viewer.find("不使用默认零位冒充") != std::string::npos &&
              app.find("robotWorkspaceVisible") != std::string::npos &&
              perception.find("启动 R1") == std::string::npos &&
              perception.find("cameraName") != std::string::npos,
          "Go2 Web must avoid repeated mesh parsing, hidden workspace churn, and R1-only camera copy");

  std::ifstream adapter_stream(project_root / "src/go2_locomotion_adapter.cpp");
  const std::string adapter((std::istreambuf_iterator<char>(adapter_stream)),
                            std::istreambuf_iterator<char>());
  Require(adapter.find("unitree/robot/go2/sport/sport_client.hpp") !=
                  std::string::npos &&
              adapter.find("client_->SpeedLevel(") == std::string::npos &&
              adapter.find("LowCmd") == std::string::npos &&
              adapter.find("lowcmd") == std::string::npos,
          "Go2 Web motion must use direct V2 Move velocity control without depending on SpeedLevel or bypassing gates with LowCmd");
}

}  // namespace

int main() {
  TestGo2ProfileAndRuntime();
  TestGo2MockAudioAndVoice();
  TestGo2NormalizationFreshnessAndSerialization();
  TestGo2DevicePolicyAndFailClosedPaths();
  TestGo2MockLocomotion();
  TestGo2MockControlSequences();
  TestGo2MockJointDebugger();
  TestGo2AssetsAndSharedDom();
  std::cout << "All Go2 integration tests passed.\n";
  return EXIT_SUCCESS;
}
