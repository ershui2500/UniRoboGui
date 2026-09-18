#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <json/json.h>

#include "g1_web/control_service.hpp"
#include "g1_web/camera_service.hpp"
#include "g1_web/device_capability.hpp"
#include "g1_web/g1_audio_capability.hpp"
#include "g1_web/g1_device_capability.hpp"
#include "g1_web/g1_joint_debug_policy.hpp"
#include "g1_web/g1_locomotion_adapter.hpp"
#include "g1_web/g1_model_catalog.hpp"
#include "g1_web/joint_names.hpp"
#include "g1_web/json_serializer.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/r1_audio_capability.hpp"
#include "g1_web/r1_device_capability.hpp"
#include "g1_web/r1_joint_debug_policy.hpp"
#include "g1_web/r1_locomotion_adapter.hpp"
#include "g1_web/robot_profile.hpp"
#include "g1_web/robot_registry.hpp"
#include "g1_web/resource_manager.hpp"
#include "g1_web/safety_manager.hpp"
#include "g1_web/ros_navigation_bridge.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/static_assets.hpp"
#include "g1_web/unitree_data_source.hpp"
#include "g1_web/voice_service.hpp"

namespace g1_web {
struct VoiceServiceTestAccess {
  static void HandleAudioMessage(VoiceService& voice,
                                 const std::string& message) {
    voice.HandleAudioMessage(message);
  }

  static void ExpireWakeSession(VoiceService& voice) {
    std::lock_guard<std::mutex> lock(voice.llm_mutex_);
    voice.customer_wake_until_ = std::chrono::steady_clock::now() -
                                 std::chrono::seconds(1);
  }
};
}  // namespace g1_web

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "TEST FAILED: " << message << '\n';
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

class UnavailableAudioCapability final : public g1_web::IAudioCapability {
 public:
  g1_web::AudioCapabilityFeatures Features() const override {
    return {true, true, true, true};
  }
  bool Start(std::string& error) override {
    error = "mock_audio_unavailable";
    return false;
  }
  void Stop() override {}
  bool Available() const override { return false; }
  std::int32_t GetVolume(std::uint8_t&) override { return -1; }
  std::int32_t SetVolume(std::uint8_t) override { return -1; }
  std::int32_t TtsMaker(const std::string&, std::int32_t) override {
    return -1;
  }
  std::int32_t PlayStream(std::string, std::string,
                          std::vector<std::uint8_t>) override {
    return -1;
  }
  std::int32_t PlayStop(std::string) override { return -1; }
};

void TestJointNames() {
  static constexpr std::array<const char*, 29> kExpectedNames{{
      "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
      "left_ankle_pitch", "left_ankle_roll", "right_hip_pitch",
      "right_hip_roll", "right_hip_yaw", "right_knee",
      "right_ankle_pitch", "right_ankle_roll", "waist_yaw", "waist_roll",
      "waist_pitch", "left_shoulder_pitch", "left_shoulder_roll",
      "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
      "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch",
      "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
      "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
  }};
  const auto& names = g1_web::JointNames();
  Require(names.size() == kExpectedNames.size(),
          "G1 should expose exactly 29 named motor slots");
  for (std::size_t index = 0; index < kExpectedNames.size(); ++index) {
    Require(names[index].name == kExpectedNames[index],
            "G1 semantic joint must retain motor slot " +
                std::to_string(index));
  }
}

void TestG1ModelCatalog() {
  static constexpr std::array<std::pair<std::uint8_t, const char*>, 11>
      kExpectedModels{{
          {2, "g1_29dof.urdf"},
          {3, "g1_29dof_lock_waist.urdf"},
          {5, "g1_29dof_rev_1_0.urdf"},
          {6, "g1_29dof_lock_waist_rev_1_0.urdf"},
          {11, "g1_29dof_mode_11.urdf"},
          {12, "g1_29dof_mode_12.urdf"},
          {13, "g1_29dof_mode_13.urdf"},
          {14, "g1_29dof_mode_14.urdf"},
          {15, "g1_29dof_mode_15.urdf"},
          {16, "g1_29dof_mode_16.urdf"},
          {18, "g1_29dof_mode_18.urdf"},
      }};
  const auto& models = g1_web::G1ModelCatalog();
  Require(models.size() == kExpectedModels.size(),
          "all supported G1 mode_machine models");
  for (std::size_t index = 0; index < kExpectedModels.size(); ++index) {
    Require(models[index].mode_machine == kExpectedModels[index].first &&
                models[index].urdf_file == kExpectedModels[index].second,
            "G1 mode_machine to URDF mapping must stay stable");
  }
  Require(g1_web::FindG1Model(17) == nullptr,
          "unknown mode_machine must not be guessed");
}

g1_web::RobotProfile BuildG1ProfileForTest() {
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must exist for tests");
  return *profile;
}

void TestRobotProfileDomainModel() {
  const auto profile = BuildG1ProfileForTest();
  Require(profile.identity.vendor == "unitree" &&
              profile.identity.family == "g1" &&
              profile.identity.product_id == "g1" &&
              profile.identity.variant == "29dof" &&
              profile.identity.morphology == "humanoid",
          "G1 profile identity must not derive product_id from mode_machine");
  Require(profile.static_constraints.at("mode_machine_role") ==
              "urdf_selector_not_product_id",
          "mode_machine must remain a G1 model selector, not product identity");

  const auto& joints = g1_web::JointNames();
  Require(profile.joint_schema.motor_slot_count == 35 &&
              profile.joint_schema.joints.size() == joints.size(),
          "G1 profile must own the 29-joint schema and 35 raw motor slots");
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const auto& descriptor = profile.joint_schema.joints[index];
    Require(descriptor.name == joints[index].name &&
                descriptor.name_zh == joints[index].name_zh &&
                descriptor.urdf_joint_name == descriptor.name + "_joint" &&
                descriptor.motor_slot == index &&
                !descriptor.display_group.empty(),
            "G1 profile joint descriptor must preserve semantic, URDF and motor-slot metadata");
  }

  const auto& models = g1_web::G1ModelCatalog();
  Require(profile.model_asset_root == "/assets/unitree/g1_description" &&
              profile.model_package == "g1_description" &&
              profile.model_variants.size() == models.size(),
          "G1 profile must own model asset metadata and every registered variant");
  for (std::size_t index = 0; index < models.size(); ++index) {
    Require(profile.model_variants[index].selector_value ==
                    models[index].mode_machine &&
                profile.model_variants[index].urdf_file ==
                    models[index].urdf_file,
            "G1 profile mode_machine to URDF mapping must remain stable");
  }
  Require(g1_web::FindModelVariant(profile, 17) == nullptr,
          "G1 profile must not guess unknown variants");

  Require(profile.capabilities.size() == 10,
          "G1 profile must include the task-12 lidar capability without adding future products");
  const auto find_capability = [&profile](g1_web::CapabilityKey key) {
    for (const auto& capability : profile.capabilities) {
      if (capability.key == key) return &capability;
    }
    return static_cast<const g1_web::CapabilityDescriptor*>(nullptr);
  };
  const auto* locomotion =
      find_capability(g1_web::CapabilityKey::kLocomotion);
  Require(locomotion != nullptr && locomotion->implemented &&
              locomotion->available &&
              locomotion->verification_level ==
                  g1_web::VerificationLevel::kControlVerified &&
              locomotion->parameters.at("speed_modes") == "0,1,3" &&
              locomotion->parameters.at("allowed_fsm") ==
                  "500,501,801,802" &&
              locomotion->parameters.at("forward_hard_limit_mps") == "3.0" &&
              locomotion->parameters.at("lateral_hard_limit_mps") == "1.0" &&
              locomotion->parameters.at("yaw_hard_limit_radps") == "1.5",
          "G1 locomotion capability must preserve task-01 safety limits and be control-verified");
  const auto* audio = find_capability(g1_web::CapabilityKey::kAudio);
  Require(audio != nullptr && audio->implemented && audio->available &&
              audio->parameters.at("tts_backends") == "kokoro,unitree" &&
              audio->parameters.at("play_stream") == "true" &&
              audio->parameters.at("tts") == "true" &&
              audio->parameters.at("volume") == "true" &&
              audio->parameters.at("asr") == "true",
          "G1 audio capability must report the task-11 adapter feature set");
  const auto* lidar = find_capability(g1_web::CapabilityKey::kLidar);
  Require(lidar != nullptr && lidar->implemented && lidar->available &&
              lidar->verification_level ==
                  g1_web::VerificationLevel::kMockVerified,
          "G1 profile must declare static lidar support separately from runtime device presence");
  const auto* head = find_capability(g1_web::CapabilityKey::kHeadControl);
  Require(head != nullptr && !head->implemented && !head->available &&
              head->verification_level ==
                  g1_web::VerificationLevel::kUnsupported &&
              !head->reason.empty(),
          "unsupported capability must carry availability, reason and verification level");

  auto alternate_api = profile;
  alternate_api.idl_family = "test_alternate_idl";
  alternate_api.control_api_family = "test_alternate_control";
  Require(alternate_api.identity.morphology == profile.identity.morphology &&
              alternate_api.idl_family != profile.idl_family &&
              alternate_api.control_api_family != profile.control_api_family,
          "morphology must not determine IDL or control API family");

  auto effective_locomotion = *locomotion;
  effective_locomotion.available = false;
  effective_locomotion.verification_level =
      g1_web::VerificationLevel::kDisabled;
  effective_locomotion.reason = "runtime_safety_state";
  Require(locomotion->available && !effective_locomotion.available &&
              effective_locomotion.verification_level ==
                  g1_web::VerificationLevel::kDisabled,
          "profile capability baseline must remain distinct from runtime effective availability");
}

void TestR1RobotProfile() {
  const auto* profile = g1_web::RobotRegistry::Find("r1");
  Require(profile != nullptr, "task-13 registry must expose the R1 profile");
  Require(profile->identity.vendor == "unitree" &&
              profile->identity.family == "r1" &&
              profile->identity.product_id == "r1" &&
              profile->identity.variant == "26dof" &&
              profile->identity.display_name == "Unitree R1" &&
              profile->identity.morphology == "humanoid",
          "R1 identity must be independent from mode_machine");
  Require(profile->idl_family == "unitree_sdk2_hg" &&
              profile->control_api_family == "unitree_sdk2_r1" &&
              profile->joint_schema.motor_slot_count == 31 &&
              profile->joint_schema.joints.size() == 26,
          "R1 must use the HG IDL and 26 semantic joints across slots 0 through 30");

  static constexpr std::array<std::size_t, 26> kExpectedSlots{{
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
      15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30,
  }};
  for (std::size_t index = 0; index < kExpectedSlots.size(); ++index) {
    const auto& joint = profile->joint_schema.joints[index];
    Require(joint.motor_slot == kExpectedSlots[index] &&
                joint.urdf_joint_name == joint.name + "_joint" &&
                !joint.display_group.empty(),
            "R1 semantic joint order must preserve the official sparse IDL slot mapping");
  }
  const auto& head_pitch = profile->joint_schema.joints[24];
  const auto& head_yaw = profile->joint_schema.joints[25];
  Require(head_pitch.name == "head_pitch" && head_pitch.motor_slot == 29 &&
              head_pitch.display_group == "head" &&
              head_yaw.name == "head_yaw" && head_yaw.motor_slot == 30 &&
              head_yaw.display_group == "head",
          "R1 head pitch/yaw must be first-class JointSchema entries");
  Require(profile->model_asset_root == "/assets/unitree/r1_description" &&
              profile->model_package == "r1_description" &&
              profile->model_variants.size() == 1 &&
              profile->model_variants[0].selector_value == 1 &&
              profile->model_variants[0].urdf_file == "R1.urdf" &&
              profile->static_constraints.at("reserved_motor_slots") ==
                  "14,20,21,27,28" &&
              profile->static_constraints.at("lowstate_topic") ==
                  "rt/lf/lowstate" &&
              profile->static_constraints.count("telemetry_scope") == 0 &&
              profile->static_constraints.at("model_asset_kind") ==
                  "official_urdf_and_meshes",
          "R1 model, official meshes and read-only transport metadata must match the platform baseline");

  const auto find_capability = [profile](g1_web::CapabilityKey key) {
    return std::find_if(profile->capabilities.begin(), profile->capabilities.end(),
                        [key](const auto& capability) {
                          return capability.key == key;
                        });
  };
  const auto telemetry = find_capability(g1_web::CapabilityKey::kTelemetry);
  Require(telemetry != profile->capabilities.end() && telemetry->implemented &&
              telemetry->available &&
              telemetry->verification_level ==
                  g1_web::VerificationLevel::kReadonlyVerified &&
              telemetry->parameters.at("joint_count") == "26" &&
              telemetry->parameters.at("lowstate_topic") ==
                  "rt/lf/lowstate",
          "R1 task-13 telemetry must reflect real read-only DDS verification");
  const auto locomotion = find_capability(g1_web::CapabilityKey::kLocomotion);
  Require(locomotion != profile->capabilities.end() &&
              locomotion->implemented && locomotion->available &&
              locomotion->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              locomotion->parameters.at("allowed_fsm") == "811" &&
              locomotion->parameters.at("speed_modes") == "0,1,3" &&
              locomotion->parameters.at("speed_presets") ==
                  "0:0.40:0.40:0.90,1:0.50:0.50:1.00,3:1.00:0.60:1.20" &&
              locomotion->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              locomotion->parameters.at("state_refresh_policy") ==
                  "client_poll" &&
              locomotion->parameters.at("web_mode_commands") ==
                  "zero_torque,damp,stand_up,lie_to_stand,stand_to_lie,start" &&
              locomotion->parameters.at("mode_targets") ==
                  "zero_torque:0,damp:1,stand_up:4,lie_to_stand:701,stand_to_lie:702,start:811" &&
              locomotion->parameters.at("mode_sources") ==
                  "zero_torque:1;stand_up:1|811;start:4;lie_to_stand:4|702;stand_to_lie:811" &&
              locomotion->parameters.at("fsm_mode_unknown") == "4294967295" &&
              locomotion->parameters.at("forward_hard_limit_mps") == "1.0" &&
              locomotion->parameters.at("lateral_hard_limit_mps") == "0.6" &&
              locomotion->parameters.at("yaw_hard_limit_radps") == "1.2" &&
              locomotion->parameters.at("velocity_lease_seconds") == "1.0",
          "R1 locomotion must expose only the documented FSM, Web mode commands, hard limits and configured Mock-verified lease policy");
  const auto audio = find_capability(g1_web::CapabilityKey::kAudio);
  Require(audio != profile->capabilities.end() && audio->implemented &&
              audio->available &&
              audio->verification_level ==
                  g1_web::VerificationLevel::kControlVerified &&
              audio->parameters.at("play_stream") == "true" &&
              audio->parameters.at("tts") == "true" &&
              audio->parameters.at("volume") == "true" &&
              audio->parameters.at("asr") == "true",
          "R1 audio must expose the official AudioClient feature set through the shared Voice core");
  const auto joint_debug =
      find_capability(g1_web::CapabilityKey::kJointDebug);
  const auto head_control =
      find_capability(g1_web::CapabilityKey::kHeadControl);
  const auto joint_teach = find_capability(g1_web::CapabilityKey::kJointTeach);
  Require(joint_debug != profile->capabilities.end() &&
              joint_debug->implemented && joint_debug->available &&
              joint_debug->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_debug->parameters.at("upper_body_groups") ==
                  "waist,head,left_arm,right_arm" &&
              joint_debug->parameters.at("upper_body_excluded_joints") ==
                  "waist_roll" &&
              joint_debug->parameters.at("upper_body_topic") == "rt/arm_sdk" &&
              joint_debug->parameters.at("full_body_topic") == "rt/lowcmd" &&
              joint_debug->parameters.at("arm_action_ids") ==
                  "11,12,13,15,17,18,19,22,23,24,25,26,27,28,29,30,31,33,34,35,36,99" &&
              joint_debug->parameters.at("firmware_teach_actions") == "true" &&
              joint_debug->parameters.at("arm_action_interrupts") == "true" &&
              joint_debug->parameters.at("arm_action_service") == "arm" &&
              joint_debug->parameters.at("arm_action_api_ids") ==
                  "7106,7107,7108,7113" &&
              joint_debug->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              head_control != profile->capabilities.end() &&
              head_control->implemented && head_control->available &&
              head_control->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              head_control->parameters.at("control_modes") == "upper_body" &&
              head_control->parameters.at("motor_slots") == "29,30" &&
              head_control->parameters.at("control_topic") == "rt/arm_sdk" &&
              head_control->parameters.at("real_control_policy") ==
                  "operator_validation",
          "R1 Joint Debug and Head Control must expose arm_sdk with full-body LowCmd under operator validation");
  Require(joint_teach != profile->capabilities.end() &&
              joint_teach->implemented && joint_teach->available &&
              joint_teach->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_teach->parameters.at("control_topic") == "rt/arm_sdk" &&
              joint_teach->parameters.at("remote_binding") == "true" &&
              joint_teach->parameters.at("real_control_policy") ==
                  "operator_validation",
          "R1 Joint Teach must include shared remote bindings over arm_sdk");
  const auto camera_rgb =
      find_capability(g1_web::CapabilityKey::kCameraRgb);
  const auto camera_depth =
      find_capability(g1_web::CapabilityKey::kCameraDepth);
  Require(camera_rgb != profile->capabilities.end() &&
              camera_rgb->implemented && camera_rgb->available &&
              camera_rgb->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              camera_rgb->parameters.at("transport") == "rtp_h264_udp" &&
              camera_rgb->parameters.at("port") == "5003" &&
              camera_rgb->parameters.at("img_port") == "5001" &&
              camera_rgb->parameters.at("left_eye_port") == "5002" &&
              camera_rgb->parameters.at("right_eye_port") == "5003" &&
              camera_rgb->parameters.at("output") == "480x360" &&
              camera_depth != profile->capabilities.end() &&
              camera_depth->implemented && camera_depth->available &&
              camera_depth->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              camera_depth->parameters.at("fixed_source") ==
                  "/dev/video-dep" &&
              camera_depth->parameters.at("width") == "544" &&
              camera_depth->parameters.at("height") == "448" &&
              camera_depth->parameters.at("fps") == "10",
          "task-24 R1 camera profile must expose fixed read-only RGB/depth policy without claiming real verification");
  const auto lidar = find_capability(g1_web::CapabilityKey::kLidar);
  const auto slam = find_capability(g1_web::CapabilityKey::kSlam);
  Require(lidar != profile->capabilities.end() && !lidar->implemented &&
              !lidar->available &&
              lidar->verification_level ==
                  g1_web::VerificationLevel::kUnsupported,
          "task-25 must not claim a standalone R1 raw lidar capability without a verified raw topic");
  Require(slam != profile->capabilities.end() && slam->implemented &&
              slam->available &&
              slam->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              slam->parameters.at("required_devices") == "mid360" &&
              slam->parameters.at("raw_input_required") == "false" &&
              slam->parameters.at("map_whitelist") ==
                  "test1.pcd..test10.pcd",
          "task-25 R1 product profile may support SLAM only as a Mid360-gated capability before runtime evidence is applied");
}

void TestRobotRegistry() {
  const auto* g1 = g1_web::RobotRegistry::Find("g1");
  const auto* r1 = g1_web::RobotRegistry::Find("r1");
  Require(g1 != nullptr && g1->identity.product_id == "g1" &&
              g1->identity.display_name == "Unitree G1",
          "registry must resolve the registered G1 profile");
  Require(g1->joint_schema.joints.size() == g1_web::JointNames().size() &&
              g1->model_variants.size() == g1_web::G1ModelCatalog().size() &&
              g1->capabilities.size() == 10,
          "registered G1 profile must retain the task-02 domain baseline");
  Require(r1 != nullptr && r1->identity.product_id == "r1" &&
              r1->joint_schema.joints.size() == 26,
          "registry must resolve the task-13 R1 profile");
  const auto products = g1_web::RobotRegistry::All();
  Require(products.size() == 2 && products[0] == g1 && products[1] == r1,
          "registry product listing must expose G1/R1 without a frontend product table");
  Require(g1_web::RobotRegistry::Find("auto") == nullptr &&
              g1_web::RobotRegistry::Find("G1") == nullptr &&
              g1_web::RobotRegistry::Find("R1") == nullptr,
          "registry must reject every unregistered spelling without guessing");
}

void TestTask22RuntimeAssemblyBoundary() {
  const auto source_file = std::filesystem::path(__FILE__);
  const auto project_root = source_file.parent_path().parent_path();
  std::ifstream main_stream(project_root / "src/main.cpp");
  const std::string main_cpp((std::istreambuf_iterator<char>(main_stream)),
                             std::istreambuf_iterator<char>());
  std::ifstream r1_locomotion_stream(
      project_root / "src/r1_locomotion_adapter.cpp");
  const std::string r1_locomotion_cpp(
      (std::istreambuf_iterator<char>(r1_locomotion_stream)),
      std::istreambuf_iterator<char>());

  Require(main_cpp.find("g1_runtime") == std::string::npos &&
              main_cpp.find("r1_runtime") == std::string::npos &&
              main_cpp.find("CreateG1LocomotionAdapter") == std::string::npos &&
              main_cpp.find("CreateR1LocomotionAdapter") == std::string::npos &&
              main_cpp.find("CreateG1AudioCapability") == std::string::npos &&
              main_cpp.find("CreateR1AudioCapability") == std::string::npos &&
              main_cpp.find("CreateG1JointDebugPolicy") == std::string::npos &&
              main_cpp.find("CreateR1JointDebugPolicy") == std::string::npos &&
              main_cpp.find("CreateG1DeviceCapabilityPolicy") ==
                  std::string::npos &&
              main_cpp.find("--device") != std::string::npos &&
              main_cpp.find("enabled_devices.insert(value)") !=
                  std::string::npos &&
              main_cpp.find("robot_switch_handler = request_robot_switch") !=
                  std::string::npos &&
              main_cpp.find("std::move(robot_switch_handler)") !=
                  std::string::npos &&
              main_cpp.find("RegisteredProductsText()") != std::string::npos &&
              main_cpp.find("支持 g1、r1") == std::string::npos &&
              r1_locomotion_cpp.find("GetFsmMode(fsm_mode)") !=
                  std::string::npos &&
              r1_locomotion_cpp.find(
                  "std::numeric_limits<std::uint32_t>::max()") !=
                  std::string::npos,
          "task-27 product assembly must stay in RobotRegistry and R1 must feed the real FSM mode into the shared G1-style status interlock or fail closed when firmware omits it");

  const auto require_no_product_client_include =
      [](const std::filesystem::path& path) {
        std::ifstream stream(path);
        const std::string text((std::istreambuf_iterator<char>(stream)),
                               std::istreambuf_iterator<char>());
        Require(text.find("unitree/robot/g1/") == std::string::npos &&
                    text.find("unitree/robot/r1/") == std::string::npos,
                "task-22 shared Core must not include G1/R1 product clients");
      };
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(project_root / "include" /
                                                     "g1_web")) {
    if (entry.is_regular_file()) {
      require_no_product_client_include(entry.path());
    }
  }
  for (const auto& entry :
       std::filesystem::directory_iterator(project_root / "src")) {
    if (!entry.is_regular_file() || entry.path().extension() != ".cpp") continue;
    const auto filename = entry.path().filename().string();
    if (filename.rfind("g1_", 0) == 0 || filename.rfind("r1_", 0) == 0) {
      continue;
    }
    require_no_product_client_include(entry.path());
  }
}

void TestRobotRuntimeBundle() {
  g1_web::RuntimeOptions mock_options;
  mock_options.mock = true;

  const auto find_source = [](const g1_web::TelemetrySubscriptionPlan& plan,
                              g1_web::TelemetrySourceKey key) {
    return std::find_if(plan.sources.begin(), plan.sources.end(),
                        [key](const auto& source) { return source.key == key; });
  };

  auto g1 = g1_web::RobotRegistry::CreateRuntime("g1", mock_options);
  const auto g1_odometry =
      find_source(g1.telemetry, g1_web::TelemetrySourceKey::kOdometry);
  Require(g1.profile.identity.product_id == "g1" && g1.locomotion &&
              g1.audio && g1.joint_debug && g1.devices &&
              g1.telemetry.sources.size() == 6 &&
              g1_odometry != g1.telemetry.sources.end() &&
              g1_odometry->kind ==
                  g1_web::TelemetrySourceKind::kGo2SportModeState &&
              g1_odometry->topic == "rt/odommodestate" &&
              g1_odometry->required_for_health,
          "task-23 G1 runtime must preserve its complete telemetry plan");

  auto r1_mock = g1_web::RobotRegistry::CreateRuntime("r1", mock_options);
  const auto r1_low_state =
      find_source(r1_mock.telemetry, g1_web::TelemetrySourceKey::kLowState);
  const auto r1_bms =
      find_source(r1_mock.telemetry, g1_web::TelemetrySourceKey::kBms);
  const auto r1_mainboard =
      find_source(r1_mock.telemetry, g1_web::TelemetrySourceKey::kMainBoard);
  const auto r1_secondary_imu = find_source(
      r1_mock.telemetry, g1_web::TelemetrySourceKey::kSecondaryImu);
  const auto r1_odometry =
      find_source(r1_mock.telemetry, g1_web::TelemetrySourceKey::kOdometry);
  Require(r1_mock.profile.identity.product_id == "r1" &&
              r1_mock.locomotion && r1_mock.audio && r1_mock.joint_debug &&
              r1_mock.devices && r1_mock.telemetry.sources.size() == 5 &&
              r1_low_state != r1_mock.telemetry.sources.end() &&
              r1_low_state->kind ==
                  g1_web::TelemetrySourceKind::kHgLowState &&
              r1_low_state->topic == "rt/lf/lowstate" &&
              r1_low_state->required_for_health &&
              r1_bms != r1_mock.telemetry.sources.end() &&
              r1_bms->kind == g1_web::TelemetrySourceKind::kHgBmsState &&
              r1_bms->topic == "rt/lf/bmsstate" &&
              !r1_bms->required_for_health &&
              r1_mainboard != r1_mock.telemetry.sources.end() &&
              r1_mainboard->kind ==
                  g1_web::TelemetrySourceKind::kHgMainBoardState &&
              r1_mainboard->topic == "rt/lf/mainboardstate" &&
              !r1_mainboard->required_for_health &&
              r1_secondary_imu != r1_mock.telemetry.sources.end() &&
              r1_secondary_imu->kind ==
                  g1_web::TelemetrySourceKind::kHgImuState &&
              r1_secondary_imu->topic == "rt/lf/secondary_imu" &&
              !r1_secondary_imu->required_for_health &&
              r1_odometry != r1_mock.telemetry.sources.end() &&
              r1_odometry->kind ==
                  g1_web::TelemetrySourceKind::kGo2SportModeState &&
              r1_odometry->topic == "rt/odommodestate" &&
              !r1_odometry->required_for_health,
          "task-23 R1 runtime must subscribe the five read-only sources verified by official IDL and the bounded odometry probe");

  const auto r1_telemetry = std::find_if(
      r1_mock.profile.capabilities.begin(), r1_mock.profile.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kTelemetry;
      });
  Require(r1_telemetry != r1_mock.profile.capabilities.end() &&
              r1_telemetry->parameters.at("declared_sources") ==
                  "low_state,bms,mainboard,secondary_imu,odometry" &&
              r1_telemetry->parameters.at("required_sources") ==
                  "low_state" &&
              r1_telemetry->parameters.count("odometry_reason") == 0,
          "task-23 R1 Manifest telemetry metadata must expose five declared sources and keep only LowState required for health");

  g1_web::RuntimeOptions real_options;
  auto r1 = g1_web::RobotRegistry::CreateRuntime("r1", real_options);
  const auto find_capability = [&r1](g1_web::CapabilityKey key) {
    return std::find_if(r1.profile.capabilities.begin(),
                        r1.profile.capabilities.end(),
                        [key](const auto& capability) {
                          return capability.key == key;
                        });
  };
  const auto locomotion =
      find_capability(g1_web::CapabilityKey::kLocomotion);
  const auto audio = find_capability(g1_web::CapabilityKey::kAudio);
  const auto joint_debug =
      find_capability(g1_web::CapabilityKey::kJointDebug);
  const auto head_control =
      find_capability(g1_web::CapabilityKey::kHeadControl);
  const auto joint_teach =
      find_capability(g1_web::CapabilityKey::kJointTeach);
  Require(r1.locomotion && r1.audio && r1.devices && r1.joint_debug &&
              locomotion != r1.profile.capabilities.end() &&
              locomotion->available &&
              locomotion->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              locomotion->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              audio != r1.profile.capabilities.end() && audio->available &&
              audio->verification_level ==
                  g1_web::VerificationLevel::kControlVerified &&
              joint_debug != r1.profile.capabilities.end() &&
              joint_debug->available &&
              joint_debug->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              head_control != r1.profile.capabilities.end() &&
              head_control->available &&
              head_control->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              head_control->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              joint_teach != r1.profile.capabilities.end() &&
              joint_teach->implemented && joint_teach->available &&
              joint_teach->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_teach->parameters.at("real_control_policy") ==
                  "operator_validation",
          "R1 real runtime must expose operator-validation locomotion, arm/head debug, and local Joint Teach without claiming control-verified evidence");

  g1_web::DeviceCapabilityRuntime rgb_runtime;
  rgb_runtime.key = g1_web::CapabilityKey::kCameraRgb;
  rgb_runtime.hardware_presence = g1_web::HardwarePresence::kPresent;
  rgb_runtime.verification_level =
      g1_web::VerificationLevel::kReadonlyVerified;
  g1_web::DeviceCapabilityRuntime depth_runtime;
  depth_runtime.key = g1_web::CapabilityKey::kCameraDepth;
  depth_runtime.hardware_presence = g1_web::HardwarePresence::kAbsent;
  depth_runtime.external_preparation_ready = false;
  depth_runtime.reason = "camera_depth_receiver_not_ready";
  g1_web::DeviceCapabilityRuntime lidar_runtime;
  lidar_runtime.key = g1_web::CapabilityKey::kLidar;
  lidar_runtime.hardware_presence = g1_web::HardwarePresence::kPresent;
  g1_web::ApplyEffectiveDeviceCapabilities(
      r1.profile, {rgb_runtime, depth_runtime, lidar_runtime});

  const auto camera_rgb =
      find_capability(g1_web::CapabilityKey::kCameraRgb);
  const auto camera_depth =
      find_capability(g1_web::CapabilityKey::kCameraDepth);
  const auto lidar = find_capability(g1_web::CapabilityKey::kLidar);
  const auto slam = find_capability(g1_web::CapabilityKey::kSlam);
  Require(camera_rgb != r1.profile.capabilities.end() &&
              camera_rgb->implemented && camera_rgb->available &&
              camera_rgb->verification_level ==
                  g1_web::VerificationLevel::kReadonlyVerified &&
              camera_depth != r1.profile.capabilities.end() &&
              camera_depth->implemented && !camera_depth->available &&
              camera_depth->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              camera_depth->reason == "camera_depth_receiver_not_ready",
          "task-24 R1 camera capabilities must promote independently only from per-stream runtime evidence");
  Require(lidar != r1.profile.capabilities.end() && !lidar->implemented &&
              !lidar->available &&
              lidar->verification_level ==
                  g1_web::VerificationLevel::kUnsupported &&
              slam != r1.profile.capabilities.end() && slam->implemented &&
              !slam->available &&
              slam->reason == "required_attachment_disabled" &&
              slam->parameters.at("required_devices") == "mid360" &&
              slam->parameters.at("navigation_max_distance_m") == "10",
          "task-25 must keep R1 raw lidar unsupported and fail SLAM closed without --device mid360");
  Require(locomotion->available &&
              locomotion->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              locomotion->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              joint_debug->available &&
              joint_debug->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_debug->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              head_control->available &&
              head_control->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              head_control->parameters.at("real_control_policy") ==
                  "operator_validation" &&
              joint_teach->implemented && joint_teach->available &&
              joint_teach->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              joint_teach->parameters.at("remote_binding") == "true",
          "device runtime evidence must preserve R1 operator-validation control gates and remote teaching bindings");

  g1_web::RuntimeOptions r1_mid360_options;
  r1_mid360_options.enabled_devices.insert("mid360");
  auto r1_mid360 =
      g1_web::RobotRegistry::CreateRuntime("r1", r1_mid360_options);
  const auto r1_mid360_slam = std::find_if(
      r1_mid360.profile.capabilities.begin(),
      r1_mid360.profile.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kSlam;
      });
  Require(r1_mid360_slam != r1_mid360.profile.capabilities.end() &&
              r1_mid360_slam->implemented && r1_mid360_slam->available &&
              r1_mid360_slam->verification_level ==
                  g1_web::VerificationLevel::kMockVerified &&
              r1_mid360.devices->Perception().attachment_declared,
          "task-25 --device mid360 may open only the static R1 attachment gate before runtime evidence tightens it");

  bool unknown_failed_closed = false;
  try {
    auto unknown =
        g1_web::RobotRegistry::CreateRuntime("unknown", mock_options);
    (void)unknown;
  } catch (const std::invalid_argument&) {
    unknown_failed_closed = true;
  }
  Require(unknown_failed_closed,
          "task-22 unregistered products must fail closed in CreateRuntime");
}

void TestG1DeviceCapabilityPolicy() {
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must exist for device policy");
  auto policy = g1_web::CreateG1DeviceCapabilityPolicy();
  const auto& perception = policy->Perception();
  const auto& camera = policy->Camera();
  Require(perception.lidar_model == "Livox Mid-360" &&
              perception.lidar_service == "lidar_driver" &&
              perception.slam_service == "unitree_slam" &&
              perception.raw_lidar_points_topic ==
                  "rt/utlidar/cloud_livox_mid360" &&
              perception.raw_lidar_imu_topic ==
                  "rt/utlidar/imu_livox_mid360" &&
              perception.dependencies.size() == 2 &&
              perception.dependencies[0].name == "lidar_driver" &&
              perception.dependencies[0].lifecycle ==
                  g1_web::DependencyLifecycle::kManagedByWeb &&
              perception.dependencies[1].name == "unitree_slam" &&
              perception.dependencies[1].lifecycle ==
                  g1_web::DependencyLifecycle::kManagedByWeb &&
              perception.raw_input_required &&
              perception.low_obstacle_source == "raw_mid360" &&
              perception.rotate_raw_lidar_x_180,
          "task-25 must preserve the existing G1 Mid-360 lifecycle, raw-input, low-obstacle and rotation policy");
  Require(camera.camera_model == "Intel RealSense D435i" &&
              camera.first_person_helper ==
                  "/usr/local/sbin/g1-web-first-person-service" &&
              camera.exclusive_service_dependencies ==
                  std::vector<std::string>({"teleimager.service",
                                            "master_service.service",
                                            "videohub_pc4"}),
          "task-12 G1 policy must own D435i helper and service dependencies");
  Require(policy->DetectHardware(g1_web::CapabilityKey::kTelemetry) ==
              g1_web::HardwarePresence::kUnknown,
          "device policy must not infer unrelated hardware from product identity");

  auto effective = *profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      effective,
      {{g1_web::CapabilityKey::kLidar,
        g1_web::HardwarePresence::kAbsent, true, true, true,
        "lidar_hardware_not_detected"},
       {g1_web::CapabilityKey::kSlam,
        g1_web::HardwarePresence::kAbsent, true, true, true,
        "lidar_hardware_not_detected"},
       {g1_web::CapabilityKey::kCameraRgb,
        g1_web::HardwarePresence::kPresent, true, true, true, ""},
       {g1_web::CapabilityKey::kCameraDepth,
        g1_web::HardwarePresence::kAbsent, true, true, true,
        "camera_device_not_detected"}});
  const auto find_capability = [&effective](g1_web::CapabilityKey key) {
    return std::find_if(effective.capabilities.begin(),
                        effective.capabilities.end(),
                        [&](const auto& capability) {
                          return capability.key == key;
                        });
  };
  const auto telemetry = find_capability(g1_web::CapabilityKey::kTelemetry);
  const auto lidar = find_capability(g1_web::CapabilityKey::kLidar);
  const auto slam = find_capability(g1_web::CapabilityKey::kSlam);
  const auto rgb = find_capability(g1_web::CapabilityKey::kCameraRgb);
  const auto depth = find_capability(g1_web::CapabilityKey::kCameraDepth);
  Require(telemetry != effective.capabilities.end() && telemetry->available,
          "missing perception hardware must not disable unrelated telemetry");
  Require(lidar != effective.capabilities.end() && lidar->implemented &&
              !lidar->available &&
              lidar->reason == "lidar_hardware_not_detected" &&
              lidar->parameters.at("product_supported") == "true" &&
              lidar->parameters.at("hardware_presence") == "absent",
          "effective lidar capability must keep static support distinct from runtime hardware presence");
  Require(slam != effective.capabilities.end() && slam->implemented &&
              !slam->available &&
              slam->reason == "lidar_hardware_not_detected",
          "SLAM must degrade when its required lidar is absent");
  Require(rgb != effective.capabilities.end() && rgb->available &&
              rgb->parameters.at("hardware_presence") == "present" &&
              depth != effective.capabilities.end() && !depth->available &&
              depth->reason == "camera_device_not_detected",
          "RGB and depth effective capabilities must follow their own runtime device evidence");

  auto unverified = *profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      unverified,
      {{g1_web::CapabilityKey::kLidar,
        g1_web::HardwarePresence::kUnknown, true, true, true, ""}});
  const auto unverified_lidar = std::find_if(
      unverified.capabilities.begin(), unverified.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kLidar;
      });
  Require(unverified_lidar != unverified.capabilities.end() &&
              !unverified_lidar->available &&
              unverified_lidar->reason == "required_hardware_unverified" &&
              unverified_lidar->parameters.at("hardware_presence") == "unknown",
          "unknown hardware must fail closed until现场 detection verifies presence");

  auto service_missing = *profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      service_missing,
      {{g1_web::CapabilityKey::kSlam,
        g1_web::HardwarePresence::kPresent, false, true, true, ""}});
  const auto missing_service = std::find_if(
      service_missing.capabilities.begin(), service_missing.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kSlam;
      });
  Require(missing_service != service_missing.capabilities.end() &&
              !missing_service->available &&
              missing_service->reason == "required_service_unavailable",
          "missing services must fail Effective Capability closed");

  auto incompatible = *profile;
  g1_web::DeviceCapabilityRuntime incompatible_slam;
  incompatible_slam.key = g1_web::CapabilityKey::kSlam;
  incompatible_slam.hardware_presence = g1_web::HardwarePresence::kPresent;
  incompatible_slam.service_available = true;
  incompatible_slam.service_compatible = false;
  incompatible_slam.service_version_status = "incompatible";
  g1_web::ApplyEffectiveDeviceCapabilities(incompatible, {incompatible_slam});
  const auto incompatible_capability = std::find_if(
      incompatible.capabilities.begin(), incompatible.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kSlam;
      });
  Require(incompatible_capability != incompatible.capabilities.end() &&
              !incompatible_capability->available &&
              incompatible_capability->reason ==
                  "required_service_incompatible" &&
              incompatible_capability->parameters.at(
                  "service_version_status") == "incompatible",
          "incompatible services must tighten Effective Capability");

  auto unsafe = *profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      unsafe,
      {{g1_web::CapabilityKey::kCameraRgb,
        g1_web::HardwarePresence::kPresent, true, true, false, ""}});
  const auto unsafe_camera = std::find_if(
      unsafe.capabilities.begin(), unsafe.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kCameraRgb;
      });
  Require(unsafe_camera != unsafe.capabilities.end() &&
              !unsafe_camera->available &&
              unsafe_camera->reason == "safety_policy_disabled",
          "safety policy must tighten Effective Capability");

  auto closed = *profile;
  g1_web::ApplyEffectiveDeviceCapabilities(
      closed,
      {{g1_web::CapabilityKey::kCameraRgb,
        g1_web::HardwarePresence::kPresent, true, false, true, ""}});
  g1_web::ApplyEffectiveDeviceCapabilities(
      closed,
      {{g1_web::CapabilityKey::kCameraRgb,
        g1_web::HardwarePresence::kPresent, true, true, true, ""}});
  const auto closed_camera = std::find_if(
      closed.capabilities.begin(), closed.capabilities.end(),
      [](const auto& capability) {
        return capability.key == g1_web::CapabilityKey::kCameraRgb;
      });
  Require(closed_camera != closed.capabilities.end() &&
              !closed_camera->available &&
              closed_camera->reason == "deployment_disabled",
          "later runtime evidence must never reopen an Effective Capability that already failed");
}

void TestR1PerceptionPolicy() {
  auto unconfigured = g1_web::CreateR1DeviceCapabilityPolicy();
  const auto& closed = unconfigured->Perception();
  Require(closed.lidar_model == "Livox Mid-360" &&
              closed.lidar_service == "mid360_driver" &&
              closed.slam_service == "unitree_slam" &&
              closed.dependencies.size() == 2 &&
              closed.dependencies[0].name == "mid360_driver" &&
              closed.dependencies[0].lifecycle ==
                  g1_web::DependencyLifecycle::kExternalRequired &&
              closed.dependencies[1].name == "unitree_slam" &&
              closed.dependencies[1].lifecycle ==
                  g1_web::DependencyLifecycle::kManagedByWeb &&
              closed.attachment_required &&
              !closed.attachment_declared &&
              !closed.raw_input_required &&
              closed.raw_lidar_points_topic.empty() &&
              closed.raw_lidar_imu_topic.empty() &&
              closed.raw_point_source.empty() &&
              closed.low_obstacle_source.empty() &&
              !closed.rotate_raw_lidar_x_180,
          "task-25 R1 perception policy must keep external Mid360 and official SLAM facts separate from G1 raw-lidar assumptions");
  Require(closed.mapping_points_topic ==
                  "rt/unitree/slam_mapping/points" &&
              closed.mapping_odom_topic == "rt/unitree/slam_mapping/odom" &&
              closed.relocation_points_topic ==
                  "rt/unitree/slam_relocation/points" &&
              closed.relocation_odom_topic ==
                  "rt/unitree/slam_relocation/odom" &&
              closed.global_map_topic ==
                  "rt/unitree/slam_relocation/global_map" &&
              closed.slam_info_topic == "rt/slam_info" &&
              closed.slam_key_info_topic == "rt/slam_key_info",
          "task-25 R1 policy must use only the documented slam_operate DDS topics");

  std::set<std::string> enabled_devices{"mid360"};
  auto attached = g1_web::CreateR1DeviceCapabilityPolicy(enabled_devices);
  Require(attached->Perception().attachment_declared &&
              attached->DetectHardware(g1_web::CapabilityKey::kSlam) ==
                  g1_web::HardwarePresence::kPresent &&
              attached->DetectHardware(g1_web::CapabilityKey::kLidar) ==
                  g1_web::HardwarePresence::kPresent,
          "task-25 --device mid360 must declare only the attachment gate; runtime service/RPC/topic evidence remains separate");
}

void TestG1JointDebugPolicy() {
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must exist for joint-debug policy");
  auto policy = g1_web::CreateG1JointDebugPolicy(*profile);
  Require(policy->GroupsForMode("upper_body") ==
              std::vector<std::string>({"waist", "left_arm", "right_arm"}) &&
              policy->ArmSdkWeightMotorSlot() == 29 &&
              policy->SupportsArmActions(),
          "G1 policy must own upper-body grouping and the arm_sdk weight slot");

  const auto joint_at_slot = [&](std::size_t slot) -> const g1_web::JointDescriptor& {
    const auto joint = std::find_if(
        profile->joint_schema.joints.begin(), profile->joint_schema.joints.end(),
        [&](const auto& descriptor) { return descriptor.motor_slot == slot; });
    Require(joint != profile->joint_schema.joints.end(),
            "G1 policy test motor slot must exist");
    return *joint;
  };
  const auto high = policy->NormalGain(joint_at_slot(0));
  const auto weak = policy->NormalGain(joint_at_slot(4));
  const auto wrist = policy->NormalGain(joint_at_slot(19));
  Require(high.kp == 300.0F && high.kd == 3.0F &&
              weak.kp == 80.0F && weak.kd == 3.0F &&
              wrist.kp == 40.0F && wrist.kd == 1.5F,
          "G1 policy must retain the existing high, weak and wrist gains");

  const auto teach_waist_yaw = policy->RecordingGain(joint_at_slot(12));
  const auto teach_waist_roll = policy->RecordingGain(joint_at_slot(13));
  const auto teach_arm = policy->RecordingGain(joint_at_slot(15));
  const auto teach_wrist = policy->RecordingGain(joint_at_slot(19));
  Require(teach_waist_yaw.kp == 0.0F && teach_waist_yaw.kd == 10.0F &&
              !teach_waist_yaw.hold_record_start_pose &&
              teach_waist_roll.kp == 300.0F &&
              teach_waist_roll.kd == 3.0F &&
              teach_waist_roll.hold_record_start_pose &&
              teach_arm.kp == 0.0F && teach_arm.kd == 1.5F &&
              teach_wrist.kp == 0.0F && teach_wrist.kd == 0.5F,
          "G1 policy must retain recording gains and waist hold behavior");

  const auto* model = policy->ResolveModelVariant(2);
  Require(model != nullptr && model->urdf_file == "g1_29dof.urdf" &&
              policy->ResolveModelVariant(17) == nullptr,
          "G1 policy must own mode_machine to Profile model resolution");
  const auto teach_groups = policy->TeachGroups();
  const auto teach_joint_count = static_cast<std::size_t>(std::count_if(
      profile->joint_schema.joints.begin(), profile->joint_schema.joints.end(),
      [&](const auto& joint) {
        return std::find(teach_groups.begin(), teach_groups.end(),
                         joint.display_group) != teach_groups.end();
      }));
  Require(policy->LegacyTeachActionCompatible(2, teach_joint_count) &&
              !policy->LegacyTeachActionCompatible(17, teach_joint_count) &&
              !policy->LegacyTeachActionCompatible(2, teach_joint_count - 1),
          "legacy compatibility must require a supported G1 model selector and exact current G1 teach-frame shape");
}

void TestR1JointDebugPolicy() {
  const auto* profile = g1_web::RobotRegistry::Find("r1");
  Require(profile != nullptr, "R1 profile must exist for joint-debug policy");
  auto policy = g1_web::CreateR1JointDebugPolicy(*profile);
  Require(policy->GroupsForMode("upper_body") ==
              std::vector<std::string>(
                  {"waist", "head", "left_arm", "right_arm"}) &&
              policy->GroupsForMode("head").empty() &&
              policy->TeachGroups() ==
                  std::vector<std::string>(
                      {"waist", "head", "left_arm", "right_arm"}) &&
              policy->TransportForMode("upper_body") ==
                  g1_web::JointDebugTransport::kArmSdk &&
              policy->TransportForMode("full_body") ==
                  g1_web::JointDebugTransport::kLowCmd &&
              !policy->ArmSdkWeightMotorSlot().has_value() &&
              policy->ArmSdkWeightUsesModePr() &&
              std::abs(policy->ArmSdkPeriodSeconds() - 0.01F) < 1e-6F &&
              std::abs(policy->ArmSdkWeightRatePerSecond() - 1.0F) < 1e-6F &&
              policy->SupportsTeachRemoteControl() &&
              policy->SupportsArmActions() &&
              policy->SupportsCustomArmActions() &&
              policy->SupportsArmActionInterrupts() &&
              policy->IsKnownArmAction(11) &&
              policy->IsKnownArmAction(27) &&
              policy->IsKnownArmAction(36) &&
              policy->IsKnownArmAction(99) &&
              !policy->IsKnownArmAction(0),
          "R1 policy must expose the documented arm action service, firmware custom-action playback, and R1 arm_sdk debug control");

  const auto joint_at_slot = [&](std::size_t slot) -> const g1_web::JointDescriptor& {
    const auto joint = std::find_if(
        profile->joint_schema.joints.begin(), profile->joint_schema.joints.end(),
        [&](const auto& descriptor) { return descriptor.motor_slot == slot; });
    Require(joint != profile->joint_schema.joints.end(),
            "R1 policy test motor slot must exist");
    return *joint;
  };
  const auto leg = policy->NormalGain(joint_at_slot(0));
  const auto waist = policy->NormalGain(joint_at_slot(12));
  const auto arm = policy->NormalGain(joint_at_slot(15));
  const auto wrist = policy->NormalGain(joint_at_slot(19));
  const auto head_pitch = policy->NormalGain(joint_at_slot(29));
  const auto head_yaw = policy->NormalGain(joint_at_slot(30));
  Require(leg.kp == 200.0F && leg.kd == 3.0F &&
              waist.kp == 300.0F && waist.kd == 5.0F &&
              arm.kp == 100.0F && arm.kd == 2.0F &&
              wrist.kp == 50.0F && wrist.kd == 2.0F &&
              head_pitch.kp == 50.0F && head_pitch.kd == 2.0F &&
              head_yaw.kp == 10.0F && head_yaw.kd == 0.1F,
          "R1 lowcmd policy must preserve the official sparse-slot gains");

  const auto arm_pitch_sdk =
      policy->NormalGainForMode(joint_at_slot(15), "upper_body");
  const auto arm_yaw_sdk =
      policy->NormalGainForMode(joint_at_slot(17), "upper_body");
  const auto wrist_sdk =
      policy->NormalGainForMode(joint_at_slot(19), "upper_body");
  const auto waist_sdk =
      policy->NormalGainForMode(joint_at_slot(13), "upper_body");
  const auto head_sdk =
      policy->NormalGainForMode(joint_at_slot(29), "upper_body");
  const auto teach_arm =
      policy->RecordingGainForMode(joint_at_slot(15), "upper_body");
  const auto teach_wrist =
      policy->RecordingGainForMode(joint_at_slot(19), "upper_body");
  const auto teach_waist_yaw =
      policy->RecordingGainForMode(joint_at_slot(13), "upper_body");
  const auto teach_head_pitch =
      policy->RecordingGainForMode(joint_at_slot(29), "upper_body");
  const auto teach_head_yaw =
      policy->RecordingGainForMode(joint_at_slot(30), "upper_body");
  Require(arm_pitch_sdk.kp == 50.0F && arm_pitch_sdk.kd == 2.0F &&
              arm_yaw_sdk.kp == 40.0F && arm_yaw_sdk.kd == 2.0F &&
              wrist_sdk.kp == 30.0F && wrist_sdk.kd == 2.0F &&
              waist_sdk.kp == 50.0F && waist_sdk.kd == 3.0F &&
              head_sdk.kp == 15.0F && head_sdk.kd == 1.0F &&
              teach_arm.kp == 0.0F && teach_arm.kd == 1.0F &&
              teach_wrist.kp == 0.0F && teach_wrist.kd == 0.5F &&
              teach_waist_yaw.kp == 0.0F && teach_waist_yaw.kd == 1.0F &&
              teach_head_pitch.kp == 0.0F && teach_head_pitch.kd == 1.0F &&
              teach_head_yaw.kp == 0.0F && teach_head_yaw.kd == 0.1F &&
              policy->ArmSdkJoint(joint_at_slot(13)) &&
              policy->ArmSdkJoint(joint_at_slot(15)) &&
              policy->ArmSdkJoint(joint_at_slot(29)) &&
              !policy->ArmSdkJoint(joint_at_slot(12)),
          "R1 arm_sdk policy must match the documented R1 arm/head gains and passive teaching damping");
  Require(policy->UpperBodyFsmAllowed(811) &&
              policy->UpperBodyActiveFsmAllowed(811) &&
              policy->UpperBodyActiveFsmAllowed(816) &&
              !policy->UpperBodyFsmAllowed(500) &&
              !policy->UpperBodyFsmAllowed(816) &&
              !policy->UpperBodyActiveFsmAllowed(500) &&
              policy->ResolveModelVariant(1) != nullptr &&
              policy->ResolveModelVariant(2) == nullptr &&
              !policy->LegacyTeachActionCompatible(1, 12),
          "R1 arm_sdk must be restricted to FSM 811 and the R1 model selector");
}

void TestStaticAssets() {
  const std::filesystem::path source_file(__FILE__);
  const auto web_root = source_file.parent_path().parent_path() / "web";
  std::filesystem::path resolved;
  Require(g1_web::ResolveStaticAsset(web_root, "/", resolved),
          "root should resolve to index.html");
  Require(resolved.filename() == "index.html", "root asset filename");
  Require(g1_web::ResolveStaticAsset(web_root, "/i18n.js", resolved),
          "i18n module should be allowlisted");
  Require(resolved.filename() == "i18n.js", "i18n module filename");
  Require(g1_web::ResolveStaticAsset(web_root, "/perception.js", resolved),
          "perception module should be allowlisted");
  Require(resolved.filename() == "perception.js",
          "perception module filename");
  Require(!g1_web::ResolveStaticAsset(web_root, "/nav-store.js", resolved),
          "removed ROS navigation state module must not be exposed");
  Require(!g1_web::ResolveStaticAsset(web_root, "/nav-renderer.js", resolved),
          "removed ROS navigation renderer module must not be exposed");
  Require(g1_web::ResolveStaticAsset(web_root, "/joint-debug.js", resolved),
          "joint debugger module should be allowlisted");
  Require(g1_web::ResolveStaticAsset(
              web_root, "/assets/unitree/r1_description/R1.urdf", resolved),
          "official R1 URDF must be served as a local static asset");
  std::ifstream r1_urdf_stream(resolved);
  const std::string r1_official_urdf(
      (std::istreambuf_iterator<char>(r1_urdf_stream)),
      std::istreambuf_iterator<char>());
  Require(r1_official_urdf.find("<mesh filename=\"meshes/") != std::string::npos &&
              r1_official_urdf.find("<box") == std::string::npos &&
              r1_official_urdf.find("<cylinder") == std::string::npos &&
              r1_official_urdf.find("<sphere") == std::string::npos &&
              g1_web::ResolveStaticAsset(
                  web_root,
                  "/assets/unitree/r1_description/meshes/pelvis_link.STL",
                  resolved),
          "R1 viewer must use the official STL model instead of primitive box placeholders");
  std::ifstream index_stream(web_root / "index.html");
  const std::string index_html((std::istreambuf_iterator<char>(index_stream)),
                               std::istreambuf_iterator<char>());
  Require(index_html.find("value=\"/dev/video") == std::string::npos &&
              index_html.find("留空自动检测") != std::string::npos &&
              index_html.find("自动检测 RGB+深度") != std::string::npos,
          "camera UI must auto-detect unstable V4L2 device nodes");
  Require(index_html.find("id=\"languageSelect\"") != std::string::npos &&
              index_html.find("value=\"zh-CN\"") != std::string::npos &&
              index_html.find("value=\"en\"") != std::string::npos &&
              index_html.find("<script src=\"/i18n.js\"></script>") !=
                  std::string::npos,
          "workspace must expose the Chinese/English language selector and load i18n");
  Require(index_html.find("id=\"diagnosticOverallBadge\"") !=
              std::string::npos &&
              index_html.find("id=\"diagnosticMotorFaultList\"") !=
                  std::string::npos &&
              index_html.find("只解码官方已公开的故障位") != std::string::npos &&
              index_html.find("不擅自判定故障") != std::string::npos,
          "diagnostics must expose useful decoded faults without guessing undocumented raw states");
  std::ifstream viewer_stream(web_root / "robot-viewer.js");
  const std::string viewer_js((std::istreambuf_iterator<char>(viewer_stream)),
                              std::istreambuf_iterator<char>());
  for (const auto& model : g1_web::G1ModelCatalog()) {
    Require(viewer_js.find(std::string(model.urdf_file)) == std::string::npos,
            "task-06 viewer must not retain a G1 mode_machine to URDF table");
  }
  Require(viewer_js.find("MODEL_BY_MACHINE") == std::string::npos &&
              viewer_js.find("MODEL_ROOT") == std::string::npos &&
              viewer_js.find("telemetryNames.length !== 29") ==
                  std::string::npos &&
              viewer_js.find("window.robotManifest?.model") !=
                  std::string::npos &&
              viewer_js.find("window.robotManifest?.joints") !=
                  std::string::npos &&
              viewer_js.find("urdf_joint_name") != std::string::npos &&
              viewer_js.find("asset_root") != std::string::npos &&
              viewer_js.find("!manifestMatchesTelemetry && robotInfo?.model_supported === false") !=
                  std::string::npos &&
              viewer_js.find("manifestMatchesTelemetry ? manifestModel.urdf_file : robotInfo?.urdf_file") !=
                  std::string::npos &&
              viewer_js.find("fetchOptions = { cache: \"reload\" }") !=
                  std::string::npos &&
              viewer_js.find("product_id === \"g1\"") ==
                  std::string::npos &&
              viewer_js.find("product_id === 'g1'") ==
                  std::string::npos &&
              viewer_js.find("product_id === \"r1\"") ==
                  std::string::npos &&
              viewer_js.find("product_id === 'r1'") ==
                  std::string::npos,
          "viewer must resolve model assets and expected joints from Manifest metadata without product-specific branches");

  std::ifstream joint_debug_stream(web_root / "joint-debug.js");
  const std::string joint_debug_js(
      (std::istreambuf_iterator<char>(joint_debug_stream)),
      std::istreambuf_iterator<char>());
  Require(joint_debug_js.find("metadata.length !== 29") == std::string::npos &&
              joint_debug_js.find("joint.index >= 12") == std::string::npos &&
              joint_debug_js.find("[\"左腿\", 0, 6]") == std::string::npos &&
              joint_debug_js.find("joint.display_group") != std::string::npos &&
              joint_debug_js.find("joint.upper_body") != std::string::npos &&
              joint_debug_js.find(
                  "panel.dataset.capabilityState !== \"available\"") !=
                  std::string::npos &&
              joint_debug_js.find("product_id === \"g1\"") ==
                  std::string::npos &&
              joint_debug_js.find("product_id === 'g1'") ==
                  std::string::npos &&
              joint_debug_js.find("(joint.controlModes || []).join(\",\")") !=
                  std::string::npos &&
              joint_debug_js.find("section.dataset.group = group") !=
                  std::string::npos &&
              index_html.find("name=\"jointDebugMode\" value=\"head\"") ==
                  std::string::npos,
          "task-10 joint debugger UI must consume backend groups and upper-body membership without fixed G1 ranges or product branches");
  Require(index_html.find("<small>rt/arm_sdk</small>") == std::string::npos &&
              joint_debug_js.find("function renderControlTopics(next)") !=
                  std::string::npos &&
              joint_debug_js.find("activeTopic === \"rt/lowcmd\"") !=
                  std::string::npos,
          "task-26 joint-debug UI must render transport from backend status instead of exposing a static G1 arm_sdk label to R1");

  const auto project_root = source_file.parent_path().parent_path();
  std::ifstream joint_core_stream(project_root / "src/joint_debug_control.cpp");
  const std::string joint_core_cpp(
      (std::istreambuf_iterator<char>(joint_core_stream)),
      std::istreambuf_iterator<char>());
  Require(joint_core_cpp.find("kNamedJointCount") == std::string::npos &&
              joint_core_cpp.find("FindG1Model") == std::string::npos &&
              joint_core_cpp.find("JointNames()") == std::string::npos &&
              joint_core_cpp.find("index >= 12") == std::string::npos &&
              joint_core_cpp.find("index < 12") == std::string::npos &&
              joint_core_cpp.find("motor_cmd().at(29)") == std::string::npos,
          "task-10 Joint Debug Core must not retain G1 joint-count, upper-body range, model catalog or weight-slot constants");

  std::ifstream voice_header_stream(project_root / "include/g1_web/voice_service.hpp");
  const std::string voice_header(
      (std::istreambuf_iterator<char>(voice_header_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream voice_core_stream(project_root / "src/voice_service.cpp");
  const std::string voice_core(
      (std::istreambuf_iterator<char>(voice_core_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream g1_audio_stream(project_root / "src/g1_audio_capability.cpp");
  const std::string g1_audio_adapter(
      (std::istreambuf_iterator<char>(g1_audio_stream)),
      std::istreambuf_iterator<char>());
  Require(voice_header.find("g1_audio_client") == std::string::npos &&
              voice_header.find("unitree::robot::g1::AudioClient") ==
                  std::string::npos &&
              voice_core.find("unitree::robot::g1::AudioClient") ==
                  std::string::npos &&
              voice_core.find("audio_client_") == std::string::npos &&
              g1_audio_adapter.find("unitree::robot::g1::AudioClient") !=
                  std::string::npos,
          "task-11 Voice Core must depend on IAudioCapability while the G1 SDK AudioClient stays inside the G1 adapter");

  std::ifstream perception_core_stream(project_root / "src/perception_service.cpp");
  const std::string perception_core(
      (std::istreambuf_iterator<char>(perception_core_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream camera_core_stream(project_root / "src/camera_service.cpp");
  const std::string camera_core(
      (std::istreambuf_iterator<char>(camera_core_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream g1_device_stream(project_root / "src/g1_device_capability.cpp");
  const std::string g1_device_policy(
      (std::istreambuf_iterator<char>(g1_device_stream)),
      std::istreambuf_iterator<char>());
  std::ifstream r1_device_stream(project_root / "src/r1_device_capability.cpp");
  const std::string r1_device_policy(
      (std::istreambuf_iterator<char>(r1_device_stream)),
      std::istreambuf_iterator<char>());
  Require(perception_core.find("\"lidar_driver\"") == std::string::npos &&
              perception_core.find("\"unitree_slam\"") == std::string::npos &&
              perception_core.find("rt/utlidar/cloud_livox_mid360") ==
                  std::string::npos &&
              perception_core.find("rt/utlidar/imu_livox_mid360") ==
                  std::string::npos &&
              camera_core.find("/usr/local/sbin/g1-web-first-person-service") ==
                  std::string::npos &&
              camera_core.find("D435i") == std::string::npos &&
              g1_device_policy.find("\"lidar_driver\"") != std::string::npos &&
              g1_device_policy.find("\"unitree_slam\"") != std::string::npos &&
              g1_device_policy.find("rt/utlidar/cloud_livox_mid360") !=
                  std::string::npos &&
              g1_device_policy.find("rotate_raw_lidar_x_180 = true") !=
                  std::string::npos &&
              g1_device_policy.find("g1-web-first-person-service") !=
                  std::string::npos &&
              g1_device_policy.find("Intel RealSense D435i") !=
                  std::string::npos &&
              r1_device_policy.find("\"mid360_driver\"") !=
                  std::string::npos &&
              r1_device_policy.find("rt/unitree/slam_mapping/points") !=
                  std::string::npos &&
              r1_device_policy.find("rt/utlidar/cloud_livox_mid360") ==
                  std::string::npos &&
              r1_device_policy.find("Intel RealSense D435i") ==
                  std::string::npos &&
              r1_device_policy.find("rotate_raw_lidar_x_180 = true") ==
                  std::string::npos &&
              perception_core.find("product_id") == std::string::npos,
          "task-25 shared Perception Core must stay product-neutral while R1 and G1 own separate deployment facts");
  Require(perception_core.find("slam_dependency_start_failed") !=
                  std::string::npos &&
              perception_core.find("slam_rpc_unavailable") !=
                  std::string::npos &&
              perception_core.find("slam_topic_stale") !=
                  std::string::npos &&
              perception_core.find("slam_topic_timeout") !=
                  std::string::npos &&
              perception_core.find("raw_input_required") !=
                  std::string::npos &&
              perception_core.find("web_started_dependencies_") !=
                  std::string::npos &&
              perception_core.find("DependencyActiveLocked") !=
                  std::string::npos &&
              perception_core.find("RollbackStartedDependenciesLocked") !=
                  std::string::npos &&
              perception_core.find("mapping_verified") != std::string::npos &&
              perception_core.find("relocation_verified") != std::string::npos &&
              perception_core.find("topic_fresh(mapping_odom_last_ms_)") !=
                  std::string::npos &&
              perception_core.find("topic_fresh(relocation_odom_last_ms_)") !=
                  std::string::npos &&
              perception_core.find("AtomicAgeMs(slam_info_last_ms_) >= 0") ==
                  std::string::npos,
          "task-27 Core must retain fail-closed dependency/RPC/stale-topic ownership gates and require fresh points+odom evidence before readonly SLAM verification");

  std::ifstream app_stream(web_root / "app.js");
  const std::string app_js((std::istreambuf_iterator<char>(app_stream)),
                           std::istreambuf_iterator<char>());
  Require(app_js.find("web_mode_commands") != std::string::npos &&
              app_js.find("mode_targets") != std::string::npos &&
              app_js.find("mode_sources") != std::string::npos &&
              app_js.find("fsm_mode_unknown") != std::string::npos &&
              app_js.find("capabilityParameters(\"locomotion\").allowed_fsm") !=
                  std::string::npos &&
              index_html.find("data-command=\"zero_torque\"") !=
                  std::string::npos &&
              index_html.find("data-command=\"damp\"") != std::string::npos &&
              index_html.find("data-command=\"stand_up\"") !=
                  std::string::npos &&
              index_html.find("data-command=\"start\"") != std::string::npos &&
              app_js.find("product_id === \"r1\"") == std::string::npos &&
              app_js.find("g1-web-control") == std::string::npos,
          "task-27 shared Web behavior and operator guidance must stay product-neutral and Manifest-driven");
  Require(index_html.find("data-command=\"lie_to_stand\"") !=
                  std::string::npos &&
              index_html.find("data-command=\"stand_to_lie\"") !=
                  std::string::npos &&
              index_html.find("data-argument=\"11\" data-label=\"双手飞吻\"") !=
                  std::string::npos &&
              index_html.find("data-argument=\"27\" data-label=\"握手\"") !=
                  std::string::npos &&
              index_html.find("data-argument=\"36\" data-label=\"向前推\"") !=
                  std::string::npos &&
              index_html.find("data-argument=\"99\" data-label=\"恢复初始手臂位姿\"") !=
                  std::string::npos &&
              app_js.find("renderArmPresetActions(control)") !=
                  std::string::npos &&
              app_js.find("firmwareIds.has(button.dataset.argument)") !=
                  std::string::npos &&
              app_js.find("motionKeepaliveTimer") != std::string::npos &&
              app_js.find("syncMotionKeepalive(vector)") !=
                  std::string::npos &&
              app_js.find("function speedModeRequiresWalkRun(speedMode)") !=
                  std::string::npos &&
              app_js.find("!capabilityParameters(\"locomotion\").speed_presets") !=
                  std::string::npos &&
              app_js.find("!speedModeRequiresWalkRun(speedMode) || isWalkRunFsm()") !=
                  std::string::npos &&
              app_js.find("arm_action_interrupts") != std::string::npos &&
              app_js.find("lastCommand.command === \"execute_custom\"") !=
                  std::string::npos &&
              app_js.find("interrupt\n        ? serviceReady") !=
                  std::string::npos &&
              app_js.find("function motionKeepaliveIntervalMs()") !=
                  std::string::npos &&
              app_js.find("leaseSeconds * 500") != std::string::npos &&
              app_js.find("}, motionKeepaliveIntervalMs());") !=
                  std::string::npos &&
              app_js.find("queueMotionRequest(current);") !=
                  std::string::npos &&
              app_js.find("queueMotionRequest(current);",
                          app_js.find("queueMotionRequest(current);") + 1) ==
                  std::string::npos,
          "shared control UI must expose current arm actions, allow declared recovery/stop interrupts, keep G1 mode 3 in walk/run while allowing profile-defined R1 mode 3, and refresh each velocity lease from exactly one Manifest-driven timer");
  std::ifstream r1_locomotion_stream(
      project_root / "src/r1_locomotion_adapter.cpp");
  const std::string r1_locomotion_cpp(
      (std::istreambuf_iterator<char>(r1_locomotion_stream)),
      std::istreambuf_iterator<char>());
  const std::string r1_loco_client_construction =
      "make_unique<unitree::robot::r1::LocoClient>()";
  const auto r1_loco_client_position =
      r1_locomotion_cpp.find(r1_loco_client_construction);
  Require(r1_locomotion_cpp.find("SetSpeedMode") == std::string::npos &&
              r1_locomotion_cpp.find("kVelocityLeaseSeconds = 1.0F") !=
                  std::string::npos &&
              r1_locomotion_cpp.find("SetVelocity") != std::string::npos &&
              r1_locomotion_cpp.find("client_->Init();") <
                  r1_locomotion_cpp.find("client_->SetTimeout(10.0F);") &&
              r1_loco_client_position != std::string::npos &&
              r1_locomotion_cpp.find(
                  r1_loco_client_construction,
                  r1_loco_client_position + r1_loco_client_construction.size()) ==
                  std::string::npos &&
              r1_locomotion_cpp.find("lock(client_mutex_)") !=
                  std::string::npos &&
              r1_locomotion_cpp.find(
                  "if (result.api_result == 127) result.api_result = 0;") !=
                  std::string::npos &&
              r1_locomotion_cpp.find("r1_velocity_api_rejected_127") ==
                  std::string::npos,
          "R1 locomotion must match the official --set_velocity path and accept the firmware's executed velocity return code 127 without disabling Web motion");
  Require(joint_debug_js.find(
              "Math.min(joint.upper, Math.max(joint.lower, target))") !=
              std::string::npos,
          "Joint Debug current-pose loading must clamp live feedback to the official URDF limits before submission");
  Require(index_html.find(
              "data-target-workspace=\"joint-debug\" data-capability=\"joint_debug\" data-capability-entry data-capability-view-when-unavailable") !=
                  std::string::npos &&
              index_html.find(
                  "data-workspace=\"joint-debug\" data-capability=\"joint_debug\" data-capability-view-when-unavailable") !=
                  std::string::npos &&
              app_js.find("data-capability-view-when-unavailable") !=
                  std::string::npos &&
              app_js.find("const blocked = capabilityBlocked && !viewWhenUnavailable") !=
                  std::string::npos,
          "unavailable but implemented Joint Debug must remain viewable without opening its control capability gate");
  Require(app_js.find("const motorStateFaults") != std::string::npos &&
              app_js.find("0x00040000") != std::string::npos &&
              app_js.find("0x40000000") != std::string::npos &&
              app_js.find("0x80000000") != std::string::npos &&
              app_js.find("renderDiagnostics(data)") != std::string::npos,
          "diagnostics must retain official G1 motorstate fault decoding");
  Require(index_html.find("id=\"robotWorkstationLabel\"") != std::string::npos &&
              index_html.find("id=\"robotOverviewSummary\"") != std::string::npos &&
              index_html.find("id=\"robotCapabilityStrip\"") != std::string::npos &&
              index_html.find("data-capability=\"locomotion\"") != std::string::npos &&
              index_html.find("data-capability=\"joint_debug\"") != std::string::npos &&
              index_html.find("data-capability=\"joint_teach\"") != std::string::npos &&
              index_html.find("data-capability=\"audio\"") != std::string::npos &&
              index_html.find("data-capability=\"slam\"") != std::string::npos &&
              index_html.find("data-capability=\"camera_rgb\"") != std::string::npos &&
              index_html.find("data-capability=\"camera_depth\"") != std::string::npos,
          "task-05 UI must expose manifest-driven product metadata and bind only declared capabilities");
  Require(index_html.find("G1 · ROBOT WORKSTATION") == std::string::npos &&
              index_html.find("G1 POSE") == std::string::npos &&
              index_html.find("G1 关节状态") == std::string::npos &&
              index_html.find("G1 URDF") == std::string::npos &&
              index_html.find("29关节 · 拖动旋转 · 滚轮缩放") == std::string::npos &&
              index_html.find("新版 29DoF") == std::string::npos,
          "task-05 priority product/DoF help text must no longer be hard-coded to G1");
  Require(app_js.find("fetchWithTimeout(\"/api/robot/manifest\"") != std::string::npos &&
              app_js.find("window.robotManifest = manifest") != std::string::npos &&
              app_js.find("lidar: [\"激光雷达\", \"LiDAR\"]") !=
                  std::string::npos &&
              app_js.find("applyManifestCapabilities(null)") != std::string::npos &&
              app_js.find("manifest_schema_invalid") != std::string::npos &&
              app_js.find("element.title = reason") != std::string::npos &&
              app_js.find("需安装 Mid-360 雷达后才可使用") !=
                  std::string::npos &&
              app_js.find("外部视频服务待准备") != std::string::npos &&
              app_js.find("if (descriptor.key === \"lidar\") continue") !=
                  std::string::npos &&
              app_js.find("receiver_start_allowed_when_unavailable") !=
                  std::string::npos &&
              app_js.find("可直接启动") != std::string::npos &&
              app_js.find("可使用") != std::string::npos &&
              app_js.find("待就绪") != std::string::npos &&
              app_js.find("Backend reason:") != std::string::npos &&
              app_js.find("not_supported_by_g1_baseline") == std::string::npos &&
              app_js.find("product_id === \"g1\"") == std::string::npos &&
              app_js.find("product_id === 'g1'") == std::string::npos &&
              app_js.find("product_id === \"r1\"") == std::string::npos &&
              app_js.find("product_id === 'r1'") == std::string::npos,
          "task-05/task-13 frontend must fetch/cache manifest, fail closed, surface unavailable reasons, and avoid product-specific branches");
  const auto language_switcher_position = index_html.find("id=\"languageMenuButton\"");
  const auto robot_switcher_position = index_html.find("id=\"robotMenuButton\"");
  Require(language_switcher_position != std::string::npos &&
              robot_switcher_position != std::string::npos &&
              robot_switcher_position > language_switcher_position &&
              index_html.find("id=\"robotMenu\"") != std::string::npos &&
              app_js.find("fetchWithTimeout(\"/api/robot/products\"") != std::string::npos &&
              app_js.find("fetchWithTimeout(\"/api/robot/switch\"") != std::string::npos &&
              app_js.find("new AbortController()") != std::string::npos &&
              app_js.find("applyActiveRobotProduct(payload.active_product_id)") != std::string::npos &&
              app_js.find("async function waitForRobotSwitch(productId)") != std::string::npos &&
              app_js.find("Switch to ${product.display_name}?") != std::string::npos &&
              app_js.find("function robotSwitchFailureMessage(error, targetProduct)") !=
                  std::string::npos &&
              app_js.find("当前机器人是 ${currentName}，不是 ${targetName}") !=
                  std::string::npos &&
              app_js.find("Robot switch failed: this robot is ${currentName}") !=
                  std::string::npos &&
              app_js.find("机器人切换失败：${error.message || error}") ==
                  std::string::npos,
          "task-13 top bar must place a bilingual Registry-driven robot switcher to the right of language selection and keep technical switch errors out of user alerts");
  std::ifstream workspace_stream(web_root / "workspace.js");
  const std::string workspace_js(
      (std::istreambuf_iterator<char>(workspace_stream)),
      std::istreambuf_iterator<char>());
  Require(workspace_js.find("panel.dataset.capabilityState === \"unsupported\"") !=
              std::string::npos,
          "workspace navigation must not reopen an unsupported capability panel");
  std::ifstream i18n_stream(web_root / "i18n.js");
  const std::string i18n_js((std::istreambuf_iterator<char>(i18n_stream)),
                            std::istreambuf_iterator<char>());
  Require(i18n_js.find("\"机器人能力\": \"Robot Capabilities\"") !=
              std::string::npos &&
              app_js.find("Robot Manifest failed to load") != std::string::npos,
          "task-05 manifest and capability UI must retain Chinese/English coverage");
  Require(app_js.find("audio_capability_unavailable") != std::string::npos &&
              app_js.find("audio_asr_unavailable") != std::string::npos &&
              i18n_js.find("Robot audio output capability is unavailable") !=
                  std::string::npos &&
              i18n_js.find("ASR audio capability is unavailable on this robot") !=
                  std::string::npos,
          "task-11 audio capability degradation must be user-visible in Chinese and English");
  Require(index_html.find("navLayerPanel") == std::string::npos &&
              index_html.find("navTopicList") == std::string::npos &&
              index_html.find("ROS 2 导航桥接") == std::string::npos,
          "customer SLAM UI must not expose unavailable ROS/Nav2 layers");
  Require(index_html.find("mapping-control-group") != std::string::npos &&
              index_html.find("map-localization-controls") != std::string::npos &&
              index_html.find("map-navigation-controls") != std::string::npos &&
              index_html.find("id=\"initialPoseMode\"") != std::string::npos &&
              index_html.find("id=\"goalPickMode\"") != std::string::npos &&
              index_html.find("id=\"initialX\"") != std::string::npos &&
              index_html.find("id=\"goalX\"") != std::string::npos,
          "SLAM UI must distribute mapping, relocalization and navigation around the map");
  std::ifstream perception_stream(web_root / "perception.js");
  const std::string perception_js(
      (std::istreambuf_iterator<char>(perception_stream)),
      std::istreambuf_iterator<char>());
  Require(perception_js.find("window.robotManifest?.identity?.display_name") !=
              std::string::npos &&
              perception_js.find("G1 URDF") == std::string::npos &&
              perception_js.find("unirobo:manifest") != std::string::npos,
          "map help must use manifest identity without changing the task-06 URDF mapping");
  Require(perception_js.find("点击 RGB 时 Web 会自动关闭 video_hub") ==
                  std::string::npos &&
              perception_js.find("Web 将自动关闭 video_hub") ==
                  std::string::npos &&
              perception_js.find("自动切换 R1 视频服务后启动固定 RGB 接收") ==
                  std::string::npos &&
              perception_js.find("自动准备 R1 视频服务和深度 helper") ==
                  std::string::npos &&
              perception_js.find("停止后只恢复本次 Web 改动的服务") ==
                  std::string::npos &&
              perception_js.find("服务切换使用 RobotState 固定匹配") ==
                  std::string::npos &&
              perception_js.find("Fixed receivers online") ==
                  std::string::npos,
          "R1 camera service details must not occupy the SLAM workspace or RGB confirmation copy");
  std::ifstream styles_stream(web_root / "styles.css");
  const std::string styles_css(
      (std::istreambuf_iterator<char>(styles_stream)),
      std::istreambuf_iterator<char>());
  Require(styles_css.find(
              ".dashboard.console-active .map-panel > :not(.map-canvas)") !=
                  std::string::npos &&
              styles_css.find("flex: 1 1 0 !important") !=
                  std::string::npos &&
              styles_css.find("height: 420px !important") ==
                  std::string::npos,
          "SLAM point-cloud viewport must consume remaining map-card space even when no frame is available");
  Require(styles_css.find(
              "grid-template-areas: \"waist head\" \"left-arm right-arm\"") !=
                  std::string::npos &&
              styles_css.find("data-group=\"waist\"") !=
                  std::string::npos &&
              styles_css.find("data-group=\"head\"") !=
                  std::string::npos,
          "upper-body Joint Debug must place waist and head above the left and right arms");
  Require(styles_css.find(
              "\"left-leg left-leg head head right-leg right-leg\"") !=
                  std::string::npos &&
              styles_css.find(
                  "\"left-leg left-leg waist waist right-leg right-leg\"") !=
                  std::string::npos &&
              styles_css.find(
                  "\"left-arm left-arm left-arm right-arm right-arm right-arm\"") !=
                  std::string::npos,
          "full-body Joint Debug must place head above waist between the legs and both arms below");
  Require(perception_js.find(
              "stopRgbCamera.addEventListener(\"click\", () => stageCameraCommand(\"stop_rgb\"))") !=
                  std::string::npos &&
              perception_js.find(
                  "stopDepthCamera.addEventListener(\"click\", () => stageCameraCommand(\"stop_depth\"))") !=
                  std::string::npos &&
              perception_js.find("Only the Web RGB receiver is stopped") !=
                  std::string::npos &&
              perception_js.find("Only the Web depth receiver is stopped") !=
                  std::string::npos,
          "R1 camera stop controls must target RGB and depth independently with bilingual confirmation copy");
  Require(perception_js.find("body.command === \"initialize_pose\"") !=
              std::string::npos &&
              perception_js.find(
                  "[\"navigating\", \"paused\"].includes(status.mode) && "
                  "status.target_set") != std::string::npos,
          "SLAM markers must clear after relocalization and only persist during active navigation");
  Require(perception_js.find(
              "mode: \"paused\", paused: true") != std::string::npos &&
              perception_js.find(
                  "mode: \"navigating\", paused: false") != std::string::npos,
          "pause/resume UI must switch buttons immediately after accepted RPCs");
  Require(perception_js.find("readCommandResponse") != std::string::npos &&
              perception_js.find("SDK api_result=") != std::string::npos &&
              perception_js.find("官方返回：") != std::string::npos &&
              perception_js.find("原始响应：") != std::string::npos,
          "perception command failures must expose SDK and official responses");
  Require(index_html.find("id=\"downloadMap\"") != std::string::npos &&
              index_html.find("id=\"exitMap\"") != std::string::npos &&
              index_html.find("id=\"cancelNavigation\"") != std::string::npos &&
              perception_js.find("downloadMapFile") != std::string::npos &&
              perception_js.find("waitForLoadedGlobalMap") != std::string::npos &&
              perception_js.find("MAP_NAME_WARNING") != std::string::npos &&
              perception_js.find("cancel_navigation") != std::string::npos,
          "SLAM UI must expose map download/exit, navigation cancel, and the fixed map-name warning");
  Require(g1_web::ResolvePerceptionMapPath("test1.pcd") ==
              "/home/unitree/test1.pcd" &&
              g1_web::ResolvePerceptionMapPath("test10.pcd") ==
                  "/home/unitree/test10.pcd" &&
              g1_web::ResolvePerceptionMapDownloadPath("test1.pcd") ==
                  "/home/unitree/.cache/g1-web-control/maps/test1.pcd" &&
              g1_web::ResolvePerceptionMapPath("test1").empty() &&
              g1_web::ResolvePerceptionMapPath("test11.pcd").empty() &&
              g1_web::ResolvePerceptionMapDownloadPath("office.pcd").empty(),
          "map save/load/download must only accept test1.pcd through test10.pcd");
  std::ifstream service_stream(web_root.parent_path() / "deploy" /
                               "g1-web-control.service");
  const std::string service_unit(
      (std::istreambuf_iterator<char>(service_stream)),
      std::istreambuf_iterator<char>());
  Require(service_unit.find("--enable-navigation") != std::string::npos,
          "production Web service must enable SLAM navigation by default");
  Require(g1_web::ResolveStaticAsset(
              web_root, "/assets/unitree/g1_description/g1_29dof_mode_15.urdf",
              resolved),
          "allowlisted URDF asset should resolve");
  Require(g1_web::StaticContentType(resolved) ==
              "application/xml; charset=utf-8",
          "URDF should use XML content type");
  Require(g1_web::ResolveStaticAsset(
              web_root, "/assets/unitree/r1_description/R1.urdf", resolved),
          "task-13 R1 model asset should resolve through the existing local asset policy");
  Require(g1_web::StaticContentType("mesh.STL") == "model/stl",
          "STL content type should be case insensitive");
  Require(!g1_web::ResolveStaticAsset(web_root, "/README.md", resolved),
          "non-allowlisted root files must be hidden");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/../index.html", resolved),
          "plain directory traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%2e%2e/index.html", resolved),
          "encoded directory traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%ZZ/index.html", resolved),
          "invalid percent encoding must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "//etc/passwd", resolved),
          "absolute paths must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets\\..\\index.html", resolved),
          "backslash traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%5c../index.html", resolved),
          "encoded backslash traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%00index.html", resolved),
          "encoded NUL must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/app.js?cache-bust=1", resolved),
          "query strings must not bypass the static allowlist");

  static constexpr std::array<const char*, 11> kModels{{
      "g1_29dof.urdf",
      "g1_29dof_lock_waist.urdf",
      "g1_29dof_rev_1_0.urdf",
      "g1_29dof_lock_waist_rev_1_0.urdf",
      "g1_29dof_mode_11.urdf",
      "g1_29dof_mode_12.urdf",
      "g1_29dof_mode_13.urdf",
      "g1_29dof_mode_14.urdf",
      "g1_29dof_mode_15.urdf",
      "g1_29dof_mode_16.urdf",
      "g1_29dof_mode_18.urdf",
  }};
  for (const char* model : kModels) {
    const auto model_path =
        web_root / "assets/unitree/g1_description" / model;
    std::ifstream input(model_path);
    const std::string urdf((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    Require(!urdf.empty(), std::string("URDF should be readable: ") + model);
    for (const auto& joint : g1_web::JointNames()) {
      const std::string joint_name(joint.name);
      Require(urdf.find("name=\"" + joint_name + "_joint\"") !=
                  std::string::npos,
              std::string("URDF joint mapping: ") + model + " / " +
                  joint_name);
    }
  }

  const auto* r1_profile = g1_web::RobotRegistry::Find("r1");
  Require(r1_profile != nullptr, "R1 profile must exist for asset validation");
  const auto r1_model_path =
      web_root / "assets/unitree/r1_description/R1.urdf";
  std::ifstream r1_input(r1_model_path);
  const std::string r1_urdf((std::istreambuf_iterator<char>(r1_input)),
                            std::istreambuf_iterator<char>());
  Require(!r1_urdf.empty() &&
              r1_profile->static_constraints.at("model_source") ==
                  "unitree_ros@7d6075f7f58588b189b940130e3edab3c839b2df" &&
              r1_urdf.find("<mesh filename=\"meshes/") != std::string::npos,
          "R1 local model must pin and use the verified Unitree URDF mesh source");
  for (const auto& joint : r1_profile->joint_schema.joints) {
    Require(r1_urdf.find("name=\"" + joint.urdf_joint_name + "\"") !=
                std::string::npos,
            "R1 local model must contain every Profile JointSchema joint");
  }
}

void TestMockNavigationBridge() {
  g1_web::RosNavigationBridge bridge(true);
  std::string error;
  Require(bridge.Start(error), "mock navigation bridge should start");
  const auto scene = Parse(bridge.SerializeScene());
  Require(scene["connected"].asBool(), "mock navigation scene connected");
  Require(scene["map"]["width"].asUInt() > 0,
          "mock navigation scene contains OccupancyGrid");
  Require(scene["global_path"]["poses"].size() > 2,
          "mock navigation scene contains global path");
  Require(scene["particles_count"].asUInt64() > 10 &&
              scene["particles_encoding"].asString() ==
                  "base64_f32le_xyzw" &&
              !scene["particles_data"].asString().empty(),
          "mock navigation scene contains compact AMCL particles");
  const auto topics = Parse(bridge.SerializeTopics());
  Require(topics["bindings"]["map"].asString() == "/map",
          "default map topic binding");
  Require(bridge.ConfigureTopic("map", "/slam_toolbox/map", error),
          "runtime topic override should succeed");
  Require(!bridge.ConfigureTopic("map", "unsafe-topic", error) &&
              error == "invalid_ros_topic",
          "invalid topic should be rejected");
  bridge.Stop();
}

void TestFreshness() {
  using g1_web::ClassifyFreshness;
  using g1_web::Freshness;
  Require(ClassifyFreshness(false, 0) == Freshness::kOffline,
          "never received is offline");
  Require(ClassifyFreshness(true, 1000) == Freshness::kOnline,
          "1000 ms remains online");
  Require(ClassifyFreshness(true, 1001) == Freshness::kDelayed,
          "over 1000 ms is delayed");
  Require(ClassifyFreshness(true, 3001) == Freshness::kOffline,
          "over 3000 ms is offline");
}

void TestTask23SourceMappings() {
  g1_web::SnapshotStore store;

  unitree_hg::msg::dds_::LowState_ low_state;
  low_state.tick() = 321;
  low_state.mode_machine() = 1;
  low_state.imu_state().temperature(39);
  store.UpdateLowState(low_state);

  unitree_hg::msg::dds_::BmsState_ bms;
  bms.soc(73);
  bms.current(-512);
  bms.cycle(44);
  store.UpdateBms(bms);

  unitree_hg::msg::dds_::MainBoardState_ mainboard;
  mainboard.temperature()[0] = 51;
  mainboard.fan_state()[0] = 1234;
  mainboard.value()[0] = 12.5F;
  mainboard.state()[0] = 7;
  store.UpdateMainBoard(mainboard);

  unitree_hg::msg::dds_::IMUState_ imu;
  imu.rpy()[1] = 0.25F;
  imu.temperature(47);
  store.UpdateSecondaryImu(imu);

  unitree_go::msg::dds_::SportModeState_ odometry;
  odometry.error_code() = 9;
  odometry.position()[0] = 1.25F;
  odometry.velocity()[1] = -0.5F;
  odometry.body_height() = 0.77F;
  odometry.yaw_speed() = 0.2F;
  store.UpdateOdometry(odometry);

  const auto snapshot = store.GetSnapshot();
  Require(snapshot.tick == 321 && snapshot.mode_machine == 1 &&
              snapshot.hip_imu.temperature_raw == 39 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kLowState)]
                  .received,
          "task-23 HG LowState must update the low-state snapshot region");
  Require(snapshot.battery.soc == 73 &&
              snapshot.battery.current_raw == -512 &&
              snapshot.battery.cycle == 44 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kBms)]
                  .received,
          "task-23 HG BmsState must update the battery snapshot region");
  Require(snapshot.mainboard.temperature_raw[0] == 51 &&
              snapshot.mainboard.fan_state_raw[0] == 1234 &&
              std::abs(snapshot.mainboard.value_raw[0] - 12.5F) < 1e-6 &&
              snapshot.mainboard.state_raw[0] == 7 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kMainBoard)]
                  .received,
          "task-23 HG MainBoardState must update the mainboard snapshot region");
  Require(std::abs(snapshot.torso_imu.rpy[1] - 0.25F) < 1e-6 &&
              snapshot.torso_imu.temperature_raw == 47 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kSecondaryImu)]
                  .received,
          "task-23 HG IMUState must update the secondary-IMU snapshot region");
  Require(snapshot.odometry.error_code == 9 &&
              std::abs(snapshot.odometry.position[0] - 1.25F) < 1e-6 &&
              std::abs(snapshot.odometry.velocity[1] + 0.5F) < 1e-6 &&
              std::abs(snapshot.odometry.body_height - 0.77F) < 1e-6 &&
              std::abs(snapshot.odometry.yaw_speed - 0.2F) < 1e-6 &&
              snapshot.sources[static_cast<std::size_t>(
                  g1_web::SourceId::kOdometry)]
                  .received,
          "task-23 Go2 SportModeState must preserve the existing G1 odometry mapping");
}

void TestTask23RequiredTelemetryHealth() {
  g1_web::RuntimeOptions options;
  options.mock = true;

  const auto telemetry_only = [](g1_web::RobotProfile profile) {
    for (auto& capability : profile.capabilities) {
      if (capability.key != g1_web::CapabilityKey::kTelemetry) {
        capability.available = false;
      }
    }
    return profile;
  };

  g1_web::SnapshotStore store;
  store.SetDdsStatus(true);
  unitree_hg::msg::dds_::LowState_ low_state;
  store.UpdateLowState(low_state);

  auto r1 = g1_web::RobotRegistry::CreateRuntime("r1", options);
  const auto r1_health =
      Parse(g1_web::SerializeHealth(telemetry_only(r1.profile),
                                    store.GetSnapshot()));
  Require(r1_health["status"].asString() == "ok" &&
              r1_health["sources"]["low_state"]["status"].asString() ==
                  "online" &&
              r1_health["sources"]["bms"]["status"].asString() == "offline" &&
              r1_health["sources"]["mainboard"]["status"].asString() ==
                  "offline" &&
              r1_health["sources"]["secondary_imu"]["status"].asString() ==
                  "offline",
          "task-23 optional R1 telemetry sources must not degrade health while LowState is fresh");

  auto g1 = g1_web::RobotRegistry::CreateRuntime("g1", options);
  const auto g1_health =
      Parse(g1_web::SerializeHealth(telemetry_only(g1.profile),
                                    store.GetSnapshot()));
  Require(g1_health["status"].asString() == "degraded",
          "task-23 G1 must preserve its existing required telemetry health behavior");
}

void TestSerialization() {
  g1_web::SnapshotStore store;
  store.PopulateMock(1.0);
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must be registered for snapshot");
  const Json::Value root =
      Parse(g1_web::SerializeSnapshot(*profile, store.GetSnapshot()));

  Require(root["schema_version"].asInt() == 1, "schema version");
  Require(root["application"].asString() == "UniRoboGui",
          "application name");
  Require(root["application_version"].asString() == "1.0.0",
          "application version");
  Require(root["dds_initialized"].asBool(), "mock DDS status");
  Require(root["sources"]["low_state"]["status"].asString() == "online",
          "mock low state is online");
  Require(root["joints"].size() == 29, "named joint count");
  const auto& joint_names = g1_web::JointNames();
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    Require(root["joints"][static_cast<Json::ArrayIndex>(index)]["index"].asUInt() ==
                    index &&
                root["joints"][static_cast<Json::ArrayIndex>(index)]["name"].asString() ==
                    std::string(joint_names[index].name),
            "snapshot must preserve G1 semantic joint to motor-slot mapping");
  }
  Require(root["reserved_motor_slots"].size() == 6,
          "reserved motor slot count");

  auto sparse_profile = *profile;
  sparse_profile.joint_schema.joints = {
      {"semantic_slot_two", "槽二", "semantic_slot_two_joint", 2, "test"},
  };
  const auto sparse_snapshot = store.GetSnapshot();
  const Json::Value sparse_root =
      Parse(g1_web::SerializeSnapshot(sparse_profile, sparse_snapshot));
  Require(sparse_root["joints"].size() == 1 &&
              sparse_root["joints"][0]["index"].asUInt() == 2 &&
              sparse_root["joints"][0]["name"].asString() ==
                  "semantic_slot_two" &&
              std::abs(sparse_root["joints"][0]["q_rad"].asDouble() -
                       sparse_snapshot.motors[2].q) < 1e-6,
          "semantic joint order must be independent from raw motor slot");
  for (std::size_t index = 29; index < 35; ++index) {
    const auto& slot = root["reserved_motor_slots"]
                           [static_cast<Json::ArrayIndex>(index - 29)];
    Require(slot["index"].asUInt() == index &&
                slot["name"].asString() ==
                    "reserved_slot_" + std::to_string(index),
            "motor slots 29 through 34 must remain reserved");
  }
  Require(root["battery"]["soc_pct"].asInt() == 82, "mock battery SOC");
  Require(root["imu"].isMember("hip") && root["imu"].isMember("torso"),
          "both IMUs are serialized");
  Require(root["robot"]["model_supported"].asBool(),
          "mock mode_machine should resolve a local model");
  Require(root["robot"]["urdf_file"].asString() == "g1_29dof.urdf",
          "mock mode_machine 2 should select the old 29DOF model");
  Require(root["mainboard"].isMember("state_raw"),
          "mainboard raw state is present");
  Require(root["voice"]["initialized"].asBool(),
          "mock voice service is initialized");
  Require(!root["voice"]["chat_go_closed"].asBool() &&
              root["voice"]["llm"]["mode"].asString() == "builtin",
          "mock voice defaults to builtin chat_go interaction");
  Require(root["control"]["motion"]["state"].asString() == "stopped",
          "motion state is serialized");
  Require(root["control"]["motion"]["speed_mode"].asInt() == 0,
          "motion speed mode is serialized");

  unitree_hg::msg::dds_::LowState_ unknown_model;
  unknown_model.mode_machine() = 17;
  store.UpdateLowState(unknown_model);
  const Json::Value unknown_root =
      Parse(g1_web::SerializeSnapshot(*profile, store.GetSnapshot()));
  Require(!unknown_root["robot"]["model_supported"].asBool() &&
              unknown_root["robot"]["mode_machine_raw"].asUInt() == 17 &&
              unknown_root["robot"]["model_name"].isNull() &&
              unknown_root["robot"]["urdf_file"].isNull(),
          "unknown G1 mode_machine must remain visible without guessing a URDF");
}

void TestRobotManifestSerialization() {
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must be registered for manifest");

  g1_web::SnapshotStore store;
  store.PopulateMock(1.0);
  const std::string serialized =
      g1_web::SerializeRobotManifest(*profile, store.GetSnapshot());
  const Json::Value root = Parse(serialized);

  Require(root["schema_version"].asInt() == 1,
          "robot manifest schema version");
  Require(root["identity"]["vendor"].asString() == "unitree" &&
              root["identity"]["family"].asString() == "g1" &&
              root["identity"]["product_id"].asString() == "g1" &&
              root["identity"]["variant"].asString() == "29dof" &&
              root["identity"]["display_name"].asString() == "Unitree G1" &&
              root["identity"]["morphology"].asString() == "humanoid",
          "manifest identity must come from the selected RobotProfile");
  Require(root["identity"]["product_id"].isString() &&
              root["diagnostics"]["mode_machine_raw"].isUInt() &&
              root["identity"]["product_id"].asString() !=
                  std::to_string(
                      root["diagnostics"]["mode_machine_raw"].asUInt()),
          "product_id and mode_machine must remain distinct concepts");

  Require(root["model"]["supported"].asBool() &&
              root["model"]["model_name"].isString() &&
              root["model"]["asset_root"].asString() ==
                  "/assets/unitree/g1_description" &&
              root["model"]["package"].asString() == "g1_description" &&
              root["model"]["urdf_file"].asString() == "g1_29dof.urdf" &&
              root["model"]["dof"].asUInt() == 29,
          "manifest model must reuse the current G1 model allowlist");

  Require(root["joints"].size() == profile->joint_schema.joints.size(),
          "manifest must expose the G1 29-joint schema");
  for (std::size_t index = 0; index < profile->joint_schema.joints.size(); ++index) {
    const auto& descriptor = profile->joint_schema.joints[index];
    const auto& joint = root["joints"][static_cast<Json::ArrayIndex>(index)];
    Require(joint["name"].asString() == descriptor.name &&
                joint["name_zh"].asString() == descriptor.name_zh &&
                joint["urdf_joint_name"].asString() ==
                    descriptor.urdf_joint_name &&
                joint["motor_slot"].asUInt() == descriptor.motor_slot &&
                joint["display_group"].asString() == descriptor.display_group,
            "manifest joints must come directly from the selected profile schema");
  }
  Require(root["joints"][static_cast<Json::ArrayIndex>(0)]["display_group"].asString() == "left_leg" &&
              root["joints"][static_cast<Json::ArrayIndex>(6)]["display_group"].asString() == "right_leg" &&
              root["joints"][static_cast<Json::ArrayIndex>(12)]["display_group"].asString() == "waist" &&
              root["joints"][static_cast<Json::ArrayIndex>(15)]["display_group"].asString() == "left_arm" &&
              root["joints"][static_cast<Json::ArrayIndex>(22)]["display_group"].asString() == "right_arm",
          "manifest must expose stable G1 display groups");

  Require(root["capabilities"].size() == profile->capabilities.size(),
          "manifest capabilities must serialize the selected RobotProfile");
  const auto find_capability = [&root](const std::string& key) {
    for (const auto& capability : root["capabilities"]) {
      if (capability["key"].asString() == key) return capability;
    }
    return Json::Value(Json::nullValue);
  };
  const auto telemetry = find_capability("telemetry");
  const auto locomotion = find_capability("locomotion");
  const auto audio = find_capability("audio");
  const auto lidar = find_capability("lidar");
  const auto head = find_capability("head_control");
  Require(telemetry["available"].asBool() &&
              telemetry["verification_level"].asString() ==
                  "readonly_verified" &&
              telemetry["parameters"]["joint_count"].asString() == "29",
          "manifest telemetry capability must preserve task-02 metadata");
  Require(locomotion["implemented"].asBool() &&
              locomotion["available"].asBool() &&
              locomotion["verification_level"].asString() ==
                  "control_verified" &&
              locomotion["parameters"]["speed_modes"].asString() ==
                  "0,1,3",
          "manifest locomotion capability must expose the real-control verification baseline");
  Require(audio["available"].asBool() &&
              audio["parameters"]["play_stream"].asString() == "true" &&
              audio["parameters"]["tts"].asString() == "true" &&
              audio["parameters"]["volume"].asString() == "true" &&
              audio["parameters"]["asr"].asString() == "true",
          "task-11 manifest must expose G1 audio adapter features");
  Require(lidar["implemented"].asBool() && lidar["available"].asBool() &&
              lidar["verification_level"].asString() == "mock_verified",
          "task-12 manifest serialization must expose the static lidar capability");
  Require(!head["implemented"].asBool() && !head["available"].asBool() &&
              head["verification_level"].asString() == "unsupported" &&
              head["reason"].asString() == "not_supported_by_g1_baseline",
          "manifest must expose unsupported capabilities explicitly");

  Require(!root.isMember("voice") && !root.isMember("control") &&
              !root.isMember("dds_error") &&
              serialized.find("customer_api") == std::string::npos &&
              serialized.find("api_key") == std::string::npos,
          "manifest must not duplicate telemetry or expose customer credentials");

  unitree_hg::msg::dds_::LowState_ unknown_model;
  unknown_model.mode_machine() = 17;
  store.UpdateLowState(unknown_model);
  const Json::Value unknown = Parse(
      g1_web::SerializeRobotManifest(*profile, store.GetSnapshot()));
  Require(unknown["diagnostics"]["mode_machine_raw"].asUInt() == 17 &&
              !unknown["model"]["supported"].asBool() &&
              unknown["model"]["model_name"].isNull() &&
              unknown["model"]["urdf_file"].isNull(),
          "manifest must keep unknown mode_machine diagnostic without guessing assets");
}

void TestR1SerializationAndManifest() {
  const auto* profile = g1_web::RobotRegistry::Find("r1");
  Require(profile != nullptr,
          "R1 profile must be registered for task-23 telemetry");

  std::vector<std::size_t> semantic_slots;
  for (const auto& joint : profile->joint_schema.joints) {
    semantic_slots.push_back(joint.motor_slot);
  }
  g1_web::SnapshotStore store;
  store.PopulateMock(1.0, 1, semantic_slots);
  const auto snapshot = store.GetSnapshot();
  const auto root = Parse(g1_web::SerializeSnapshot(*profile, snapshot));

  Require(root["robot"]["mode_machine_raw"].asUInt() == 1 &&
              root["robot"]["model_supported"].asBool() &&
              root["robot"]["model_dof"].asUInt() == 26 &&
              root["robot"]["urdf_file"].asString() == "R1.urdf" &&
              root["joints"].size() == 26,
          "R1 mock telemetry must select the R1 model and expose 26 semantic joints");
  for (std::size_t index = 0; index < profile->joint_schema.joints.size(); ++index) {
    const auto& descriptor = profile->joint_schema.joints[index];
    const auto& joint = root["joints"][static_cast<Json::ArrayIndex>(index)];
    Require(joint["index"].asUInt() == descriptor.motor_slot &&
                joint["name"].asString() == descriptor.name &&
                std::abs(joint["q_rad"].asDouble() -
                         snapshot.motors[descriptor.motor_slot].q) < 1e-6,
            "R1 telemetry must map semantic joint order through sparse raw motor slots");
  }
  Require(root["joints"][24]["name"].asString() == "head_pitch" &&
              root["joints"][24]["index"].asUInt() == 29 &&
              root["joints"][24]["mode_raw"].asUInt() == 1 &&
              root["joints"][25]["name"].asString() == "head_yaw" &&
              root["joints"][25]["index"].asUInt() == 30 &&
              root["joints"][25]["mode_raw"].asUInt() == 1,
          "R1 mock telemetry must include active head pitch/yaw slots 29 and 30");
  Require(root["reserved_motor_slots"].size() == 5,
          "R1 raw telemetry must only expose the five reserved slots inside its 31-slot schema");
  Require(root["sources"]["low_state"]["status"].asString() == "online" &&
              root["sources"]["bms"]["status"].asString() == "online" &&
              root["sources"]["mainboard"]["status"].asString() == "online" &&
              root["sources"]["secondary_imu"]["status"].asString() ==
                  "online" &&
              root["sources"]["odometry"]["status"].asString() == "online" &&
              std::abs(snapshot.odometry.position[0]) > 1e-6,
          "task-23 R1 Mock must cover all five declared telemetry sources after odometry verification");
  static constexpr std::array<std::size_t, 5> kReserved{{14, 20, 21, 27, 28}};
  for (std::size_t index = 0; index < kReserved.size(); ++index) {
    Require(root["reserved_motor_slots"][static_cast<Json::ArrayIndex>(index)]
                    ["index"].asUInt() == kReserved[index],
            "R1 reserved slot order must match the official sparse IDL mapping");
  }

  const auto manifest =
      Parse(g1_web::SerializeRobotManifest(*profile, snapshot));
  Require(manifest["identity"]["product_id"].asString() == "r1" &&
              manifest["identity"]["display_name"].asString() == "Unitree R1" &&
              manifest["model"]["supported"].asBool() &&
              manifest["model"]["asset_root"].asString() ==
                  "/assets/unitree/r1_description" &&
              manifest["model"]["package"].asString() == "r1_description" &&
              manifest["model"]["dof"].asUInt() == 26 &&
              manifest["joints"].size() == 26 &&
              manifest["joints"][24]["display_group"].asString() == "head" &&
              manifest["joints"][25]["display_group"].asString() == "head",
          "R1 manifest must drive product identity, local model metadata, DOF and head group without frontend branches");

  for (const auto& capability : manifest["capabilities"]) {
    const auto key = capability["key"].asString();
    if (key == "telemetry") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "readonly_verified" &&
                  capability["parameters"]["declared_sources"].asString() ==
                      "low_state,bms,mainboard,secondary_imu,odometry" &&
                  capability["parameters"]["required_sources"].asString() ==
                      "low_state" &&
                  !capability["parameters"].isMember("odometry_reason"),
              "R1 telemetry Manifest must expose five declared sources while keeping only LowState required for health");
      continue;
    }
    if (key == "locomotion") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "mock_verified",
              "R1 locomotion must remain available only at Mock verification level");
      continue;
    }
    if (key == "audio") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "control_verified",
              "R1 audio must report completed real-device API verification");
      continue;
    }
    if (key == "joint_debug" || key == "head_control") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "mock_verified" &&
                  capability["parameters"]["real_control_policy"].asString() ==
                      "operator_validation",
              "R1 joint/head control must remain Mock-verified while exposing the operator-validation runtime path");
      continue;
    }
    if (key == "joint_teach") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "mock_verified" &&
                  capability["parameters"]["control_topic"].asString() ==
                      "rt/arm_sdk" &&
                  capability["parameters"]["remote_binding"].asString() ==
                      "true" &&
                  capability["parameters"]["real_control_policy"].asString() ==
                      "operator_validation",
              "R1 local Joint Teach must use arm_sdk with shared remote bindings");
      continue;
    }
    if (key == "camera_rgb" || key == "camera_depth") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "mock_verified" &&
                  capability["parameters"]["receiver_start_allowed_when_unavailable"].asString() ==
                      "true",
              "task-24 R1 camera manifest baseline must be implemented at Mock verification without claiming real frames");
      continue;
    }
    if (key == "slam") {
      Require(capability["available"].asBool() &&
                  capability["verification_level"].asString() ==
                      "mock_verified" &&
                  capability["parameters"]["required_devices"].asString() ==
                      "mid360" &&
                  capability["parameters"]["raw_input_required"].asString() ==
                      "false",
              "task-25 R1 product manifest must declare Mid360-gated SLAM support without claiming real verification");
      continue;
    }
    Require(!capability["available"].asBool(),
            "unsupported or disabled R1 capabilities must remain fail-closed");
  }
}

void TestR1MockLocomotionAndAudio() {
  const auto* profile = g1_web::RobotRegistry::Find("r1");
  Require(profile != nullptr, "R1 profile must exist for Mock control tests");
  {
    g1_web::SnapshotStore source_store;
    g1_web::MockDataSource mock_source(source_store, *profile);
    mock_source.Start();
    const auto initial = source_store.GetSnapshot().control;
    mock_source.Stop();
    Require(initial.sport_state_received && initial.fsm_id == 811 &&
                initial.fsm_mode == 0,
            "R1 Mock source must start in the profile-defined operational FSM 811");
  }

  auto locomotion = g1_web::CreateR1LocomotionAdapter(true);
  Require(locomotion->IsOperationalFsm(811) &&
              !locomotion->IsOperationalFsm(500) &&
              locomotion->IsKnownModeCommand("damp", 0) &&
              locomotion->IsKnownModeCommand("zero_torque", 0) &&
              locomotion->IsKnownModeCommand("stand_up", 0) &&
              locomotion->IsKnownModeCommand("lie_to_stand", 0) &&
              locomotion->IsKnownModeCommand("stand_to_lie", 0) &&
              locomotion->IsKnownModeCommand("start", 0) &&
              locomotion->IsKnownModeCommand("stop_move", 0) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 0) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 1) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 4) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 811) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 500) &&
              !locomotion->IsKnownModeCommand("sit", 0) &&
              !locomotion->IsKnownModeCommand("squat", 0) &&
              !locomotion->IsKnownModeCommand("balance_stand", 0),
          "R1 adapter must expose only the documented semantic commands without direct SetFsmId or G1 fallbacks");

  g1_web::SnapshotStore store;
  std::vector<std::size_t> slots;
  for (const auto& joint : profile->joint_schema.joints) {
    slots.push_back(joint.motor_slot);
  }
  store.PopulateMock(0.0, 1, slots);
  unitree_hg::msg::dds_::SportModeState_ state;
  state.fsm_id(811);
  state.fsm_mode(0);
  store.UpdateSportMode(state);
  g1_web::ControlService control(store, std::move(locomotion), *profile,
                                 nullptr, true);
  std::string error;
  Require(control.Start(error),
          "R1 Mock locomotion must start without constructing a G1 arm client");
  const auto expect_mode = [&](const char* request_key, const char* command,
                               std::uint32_t expected_fsm) {
    const auto submitted =
        control.Submit(request_key, "mode", command, 0, true);
    Require(submitted.accepted,
            std::string("R1 Mock mode command should be accepted: ") + command);
    for (int attempt = 0; attempt < 50; ++attempt) {
      const auto snapshot = store.GetSnapshot().control;
      if (snapshot.fsm_id == expected_fsm &&
          snapshot.last_command.state == "succeeded") {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto snapshot = store.GetSnapshot().control;
    Require(snapshot.fsm_id == expected_fsm &&
                snapshot.last_command.state == "succeeded",
            std::string("R1 Mock mode command must reach FSM ") +
                std::to_string(expected_fsm) + ": " + command);
  };
  expect_mode("r1-mode-damp-0001", "damp", 1);
  expect_mode("r1-mode-zero-0002", "zero_torque", 0);
  expect_mode("r1-mode-damp-0003", "damp", 1);
  expect_mode("r1-mode-stand-0004", "stand_up", 4);
  expect_mode("r1-mode-start-0005", "start", 811);
  expect_mode("r1-mode-lie-down-0006", "stand_to_lie", 702);
  expect_mode("r1-mode-lie-up-0007", "lie_to_stand", 701);
  expect_mode("r1-mode-damp-0008", "damp", 1);
  expect_mode("r1-mode-stand-0009", "stand_up", 4);
  expect_mode("r1-mode-start-0010", "start", 811);
  Require(control.Submit("r1-sit-unsupported", "mode", "sit", 0, true).error ==
              "unknown_or_disallowed_command" &&
              control.Submit("r1-fsm-direct-blocked", "mode", "set_fsm_id",
                             811, true)
                      .error == "unknown_or_disallowed_command",
          "R1 unsupported posture and direct SetFsmId must not fall through to a control API");
  Require(control.SubmitVelocity(0.4F, 0.4F, 0.9F, 0, true).accepted &&
              control.SubmitVelocity(0.5F, 0.5F, 1.0F, 1, true).accepted &&
              control.SubmitVelocity(1.0F, 0.6F, 1.2F, 3, true).accepted,
          "R1 Mock locomotion must accept all three speed presets in FSM 811");
  Require(control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false).accepted,
          "R1 Mock stop must use the shared control schema");
  Require(control.SubmitVelocity(0.1F, 0.0F, 0.0F, 2, true).error ==
              "invalid_speed_mode" &&
              control.SubmitVelocity(0.41F, 0.0F, 0.0F, 0, true).error ==
                  "velocity_out_of_range" &&
              control.SubmitVelocity(0.0F, 0.51F, 0.0F, 1, true).error ==
                  "velocity_out_of_range" &&
              control.SubmitVelocity(0.0F, 0.0F, 1.21F, 3, true).error ==
                  "velocity_out_of_range",
          "R1 unknown speed modes and per-preset excessive velocities must fail closed");
  control.Stop();

  auto audio = g1_web::CreateR1AudioCapability(true);
  Require(audio->Features().play_stream && audio->Features().tts &&
              audio->Features().volume && audio->Features().asr &&
              audio->Start(error),
          "R1 Mock audio adapter must expose the official AudioClient features");
  std::uint8_t volume = 0;
  Require(audio->GetVolume(volume) == 0 && volume == 82 &&
              audio->SetVolume(50) == 0 &&
              audio->TtsMaker("R1 mock", 1) == 0 &&
              audio->PlayStream("test", "stream", {0, 0}) == 0 &&
              audio->PlayStop("test") == 0,
          "R1 Audio adapter must satisfy the shared Voice capability contract");
  audio->Stop();
}

void WriteFloat(std::vector<std::uint8_t>& data, std::size_t offset,
                float value) {
  Require(offset + sizeof(value) <= data.size(), "point write bounds");
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void TestPointCloudDecode() {
  sensor_msgs::msg::dds_::PointCloud2_ cloud;
  cloud.height(1);
  cloud.width(3);
  cloud.point_step(16);
  cloud.row_step(48);
  cloud.is_bigendian(false);
  cloud.is_dense(true);
  cloud.header().frame_id("map");
  cloud.fields({
      sensor_msgs::msg::dds_::PointField_("x", 0, 7, 1),
      sensor_msgs::msg::dds_::PointField_("y", 4, 7, 1),
      sensor_msgs::msg::dds_::PointField_("z", 8, 7, 1),
      sensor_msgs::msg::dds_::PointField_("intensity", 12, 7, 1),
  });
  std::vector<std::uint8_t> bytes(48);
  for (std::size_t index = 0; index < 3; ++index) {
    WriteFloat(bytes, index * 16, static_cast<float>(index + 1));
    WriteFloat(bytes, index * 16 + 4, static_cast<float>(index + 2));
    WriteFloat(bytes, index * 16 + 8, static_cast<float>(index + 3));
    WriteFloat(bytes, index * 16 + 12, static_cast<float>(index * 10));
  }
  cloud.data(bytes);

  const auto decoded = g1_web::DecodePointCloud(cloud, 2);
  Require(decoded.valid, "valid PointCloud2 should decode");
  Require(decoded.frame_id == "map", "point cloud frame should be retained");
  Require(decoded.source_points == 3, "source point count");
  Require(decoded.points.size() == 2, "point cloud should be decimated");
  Require(std::abs(decoded.points.front().x - 1.0F) < 1e-6F,
          "decoded x value");
  Require(std::abs(decoded.points.back().intensity - 20.0F) < 1e-6F,
          "decoded intensity value");

  cloud.fields({sensor_msgs::msg::dds_::PointField_("x", 0, 7, 1)});
  const auto malformed = g1_web::DecodePointCloud(cloud, 100);
  Require(!malformed.valid && malformed.error == "xyz_fields_missing",
          "malformed PointCloud2 must be rejected");
}

void TestPointCloudWebFilter() {
  const std::vector<g1_web::PointSample> points{
      {1.01F, 1.01F, 0.0F, 10.0F},
      {1.02F, 1.02F, 0.0F, 20.0F},
      {1.11F, 1.01F, 0.0F, 30.0F},
      {4.0F, 4.0F, 0.0F, 40.0F},
      {20.0F, 0.0F, 0.0F, 50.0F},
      {1.0F, 1.0F, 5.0F, 60.0F},
  };
  g1_web::PointCloudFilterOptions options;
  options.voxel_size_m = 0.1F;
  options.maximum_range_m = 10.0F;
  options.minimum_z_m = -1.0F;
  options.maximum_z_m = 2.0F;
  options.maximum_points = 10;
  options.remove_isolated_voxels = true;
  const auto filtered = g1_web::FilterPointCloudForWeb(points, options);
  Require(filtered.size() == 2,
          "voxel filter should merge duplicates and reject isolated noise");
  bool found_centroid = false;
  for (const auto& point : filtered) {
    if (std::abs(point.x - 1.015F) < 1e-4F &&
        std::abs(point.intensity - 15.0F) < 1e-4F) {
      found_centroid = true;
    }
  }
  Require(found_centroid, "voxel filter should retain the cell centroid");

  options.maximum_points = 1;
  Require(g1_web::FilterPointCloudForWeb(points, options).size() == 1,
          "filtered cloud should honor the final Web point cap");
}

void TestLowObstacleSafetyDetector() {
  std::vector<g1_web::PointSample> ground;
  for (int xi = 0; xi < 20; ++xi) {
    const float x = 0.4F + xi * 0.05F;
    for (int yi = -8; yi <= 8; ++yi) {
      const float y = yi * 0.05F;
      ground.push_back({x, y, -1.05F + 0.01F * x + 0.005F * y, 10.0F});
    }
  }
  Require(!g1_web::DetectLowObstacleForSafety(ground).detected,
          "sloped ground alone must not trigger the low-obstacle safety layer");

  auto with_obstacle = ground;
  for (int i = 0; i < 12; ++i) {
    with_obstacle.push_back(
        {0.78F + (i % 3) * 0.02F, -0.04F + (i / 3) * 0.025F,
         -0.84F + (i % 2) * 0.01F, 80.0F});
  }
  const auto detected = g1_web::DetectLowObstacleForSafety(with_obstacle);
  Require(detected.detected && detected.nearest_range_m < 1.0F &&
              detected.support_points >= 2,
          "a 20 cm-class obstacle in the forward raw cloud must be retained by the safety detector");
}

void TestMockPerceptionSafety() {
  auto device_policy = g1_web::CreateG1DeviceCapabilityPolicy();
  g1_web::PerceptionService perception(true, false, *device_policy);
  std::string error;
  Require(perception.Start(error), "mock perception should start");
  Require(error.empty(), "mock perception start error");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const Json::Value status = Parse(perception.SerializeStatus());
  Require(status["initialized"].asBool(), "mock SLAM initialized");
  Require(status["navigation_enabled"].asBool(),
          "mock navigation is enabled for UI validation");
  Require(status["global_points"].asUInt64() > 1000,
          "mock global point cloud is populated");
  const Json::Value frame = Parse(perception.SerializeFrame());
  Require(frame["schema_version"].asInt() == 3 &&
              frame["live_points_encoding"].asString() ==
                  "base64_u16le_xyz" &&
              frame["live_points_count"].asUInt64() > 100 &&
              frame["live_points_origin"].size() == 3 &&
              frame["live_points_scale"].size() == 3 &&
              frame["live_points_data"].asString().size() <=
                  frame["live_points_count"].asUInt64() * 8 + 4,
          "live cloud uses quantized Wi-Fi encoding");
  const Json::Value global_map = Parse(perception.SerializeGlobalMap());
  Require(global_map["schema_version"].asInt() == 3 &&
              global_map["points_encoding"].asString() ==
                  "base64_u16le_xyz" &&
              global_map["points_origin"].size() == 3 &&
              global_map["points_scale"].size() == 3 &&
              !global_map["points_data"].asString().empty(),
          "global cloud uses quantized Wi-Fi encoding");
  Require(status["point_filter"]["live_max_points"].asUInt64() == 3600 &&
              status["point_filter"]["live_isolated_voxel_filter"].asBool() &&
              !status["point_filter"]["global_isolated_voxel_filter"].asBool(),
          "point filter profile should be visible in status");
  Require(status["services"]["lidar_driver"]["enabled"].asBool() &&
              status["services"]["unitree_slam"]["enabled"].asBool() &&
              status["device"]["lidar_model"].asString() == "Livox Mid-360" &&
              status["device"]["raw_lidar_rotate_x_180"].asBool(),
          "mock sensing services and G1 deployment status must preserve existing semantics");
  const auto perception_capabilities = perception.DeviceCapabilities();
  Require(perception_capabilities.size() == 2 &&
              perception_capabilities[0].key == g1_web::CapabilityKey::kLidar &&
              perception_capabilities[0].hardware_presence ==
                  g1_web::HardwarePresence::kPresent &&
              perception_capabilities[0].service_available &&
              perception_capabilities[1].key == g1_web::CapabilityKey::kSlam &&
              perception_capabilities[1].service_available,
          "mock perception must expose effective lidar and SLAM runtime evidence");

  g1_web::PerceptionRequest request;
  request.request_key = "slam-request-0001";
  request.command = "navigate";
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "confirmation_required",
          "SLAM command requires explicit confirmation");

  request.confirmed = true;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "navigation must stay locked until a map is loaded and localized");

  request.request_key = "slam-relocate-before-load";
  request.command = "initialize_pose";
  request.map_name = "test1.pcd";
  request.pose = {};
  request.pose.x = status["pose"]["x"].asDouble();
  request.pose.y = status["pose"]["y"].asDouble();
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "relocation must stay locked until the selected map is loaded");

  request.request_key = "slam-load-map-test1";
  request.command = "load_map";
  const auto loaded = perception.Submit(request);
  Require(loaded.accepted,
          "map loading should automatically start SLAM dependencies");
  Json::Value localized = Parse(perception.SerializeStatus());
  Require(localized["mode"].asString() == "localizing" &&
              localized["map_loaded"].asBool() &&
              localized["map_name"].asString() == "test1.pcd" &&
              localized["services"]["lidar_driver"]["enabled"].asBool() &&
              localized["services"]["unitree_slam"]["enabled"].asBool(),
          "successful map loading records the loaded map and starts localization");
  std::string topic_map_pcd;
  Require(perception.GetTopicGlobalMapPcd("test1.pcd", topic_map_pcd) &&
              topic_map_pcd.rfind("# .PCD v0.7", 0) == 0 &&
              topic_map_pcd.find("POINTS ") != std::string::npos,
          "localized global-map topic should be available as the preferred PCD download source");

  request.request_key = "slam-relocate-after-load";
  request.command = "initialize_pose";
  request.pose.x += 0.1;
  Require(perception.Submit(request).accepted,
          "relocation is accepted after the same map has been loaded");

  request.request_key = "slam-request-0003";
  request.command = "navigate";
  request.pose.x = localized["pose"]["x"].asDouble() + 11.0;
  request.pose.y = localized["pose"]["y"].asDouble();
  Require(perception.Submit(request).error == "goal_over_10m",
          "official ten metre goal limit must be enforced");

  request.request_key = "slam-request-0004";
  request.pose.x = localized["pose"]["x"].asDouble() + 1.0;
  const auto accepted = perception.Submit(request);
  Require(accepted.accepted && accepted.request_id > 0,
          "nearby mock navigation goal should be accepted after localization");
  const auto duplicate = perception.Submit(request);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == accepted.request_id,
          "duplicate SLAM request key must not execute twice");

  request.request_key = "slam-request-pause";
  request.command = "pause_navigation";
  Require(perception.Submit(request).accepted,
          "pause is accepted only while navigation is active");
  Json::Value paused_status = Parse(perception.SerializeStatus());
  Require(paused_status["mode"].asString() == "paused" &&
              paused_status["paused"].asBool() &&
              paused_status["target_set"].asBool() &&
              paused_status["map_loaded"].asBool(),
          "accepted pause must immediately expose paused state without dropping the active goal");

  request.request_key = "slam-request-pause-again";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "a second pause is rejected after the first pause already changed state");

  request.request_key = "slam-request-resume";
  request.command = "resume_navigation";
  Require(perception.Submit(request).accepted,
          "resume is accepted only from the paused state");
  Json::Value resumed_status = Parse(perception.SerializeStatus());
  Require(resumed_status["mode"].asString() == "navigating" &&
              !resumed_status["paused"].asBool() &&
              resumed_status["target_set"].asBool() &&
              resumed_status["map_loaded"].asBool(),
          "accepted resume must immediately return to the same active navigation goal");

  request.request_key = "slam-request-0005";
  request.command = "start_mapping";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "mapping must not start while navigation is active");

  request.request_key = "slam-request-cancel";
  request.command = "cancel_navigation";
  Require(perception.Submit(request).accepted,
          "cancel should stop the active navigation task");
  Json::Value cancelled_status = Parse(perception.SerializeStatus());
  Require(cancelled_status["mode"].asString() == "localizing" &&
              !cancelled_status["paused"].asBool() &&
              !cancelled_status["target_set"].asBool() &&
              cancelled_status["map_loaded"].asBool(),
          "navigation cancel must clear the active goal while preserving the loaded map");

  request.request_key = "slam-request-after-cancel";
  request.command = "navigate";
  Require(perception.Submit(request).accepted,
          "a new navigation goal should be accepted after cancellation");

  request.request_key = "slam-request-0006";
  request.command = "stop_slam";
  Require(perception.Submit(request).accepted, "mock SLAM should stop");

  request.request_key = "slam-request-0007";
  request.command = "start_mapping";
  Require(perception.Submit(request).accepted, "mock mapping should start");
  Json::Value running = Parse(perception.SerializeStatus());
  Require(running["services"]["lidar_driver"]["enabled"].asBool() &&
              running["services"]["unitree_slam"]["enabled"].asBool(),
          "mapping starts lidar and SLAM dependencies");
  Require(running["lidar_inputs"]["ready"].asBool() &&
              running["topics"]["raw_lidar_imu"].asString() ==
                  "rt/utlidar/imu_livox_mid360",
          "SLAM status must expose the official lidar IMU input");

  request.request_key = "slam-request-0008";
  request.command = "initialize_pose";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "map loading and relocation stay locked during mapping");

  request.request_key = "slam-request-invalid-map-name";
  request.command = "finish_mapping";
  request.map_name = "office.pcd";
  Require(perception.Submit(request).error == "invalid_map_name",
          "map saving must reject names outside test1.pcd through test10.pcd");

  request.request_key = "slam-request-0009";
  request.map_name = "test1.pcd";
  Require(perception.Submit(request).accepted,
          "map saving is accepted only while mapping with an allowed PCD name");

  request.request_key = "slam-request-0010";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "map saving locks again after mapping finishes");

  request.request_key = "slam-request-0011";
  request.command = "stop_slam";
  Require(perception.Submit(request).accepted, "mock SLAM should stop");
  Json::Value stopped_status = Parse(perception.SerializeStatus());
  Require(stopped_status["services"]["lidar_driver"]["enabled"].asBool() &&
              !stopped_status["services"]["unitree_slam"]["enabled"].asBool(),
          "SLAM stop keeps raw lidar and closes SLAM");
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  stopped_status = Parse(perception.SerializeStatus());
  const Json::Value stopped_frame = Parse(perception.SerializeFrame());
  Require(stopped_status["mode"].asString() == "stopped" &&
              !stopped_status["map_loaded"].asBool() &&
              stopped_status["trajectory_points"].asUInt64() == 0 &&
              stopped_status["global_points"].asUInt64() == 0 &&
              stopped_status["point_source"].asString() == "utlidar_mid360" &&
              stopped_frame["live_points_count"].asUInt64() > 100,
          "SLAM stop clears map and restores raw lidar without task trail");

  request.request_key = "slam-relocate-after-stop";
  request.command = "initialize_pose";
  request.map_name = "test1.pcd";
  request.pose = {};
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "stopping SLAM clears the loaded-map interlock");

  request.request_key = "slam-reload-after-stop";
  request.command = "load_map";
  Require(perception.Submit(request).accepted,
          "map loading can restart localization after SLAM was stopped");

  request.request_key = "slam-relocate-after-reload";
  request.command = "initialize_pose";
  Require(perception.Submit(request).accepted,
          "relocation unlocks again after the map reload succeeds");

  request.request_key = "slam-nav-trail-after-reload";
  request.command = "navigate";
  request.pose.x = 0.35;
  request.pose.y = 0.0;
  Require(perception.Submit(request).accepted,
          "navigation starts from the loaded map without another reload");
  std::this_thread::sleep_for(std::chrono::milliseconds(260));
  const Json::Value navigating_status = Parse(perception.SerializeStatus());
  const Json::Value navigating_frame = Parse(perception.SerializeFrame());
  Require(navigating_status["mode"].asString() == "navigating" &&
              navigating_status["map_loaded"].asBool() &&
              navigating_frame["trajectory_count"].asUInt64() >= 2,
          "navigation keeps the loaded map and publishes a visible travel trail");

  Json::Value arrived_status;
  for (int attempt = 0; attempt < 20; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arrived_status = Parse(perception.SerializeStatus());
    if (arrived_status["mode"].asString() == "arrived") break;
  }
  Require(arrived_status["mode"].asString() == "arrived" &&
              arrived_status["map_loaded"].asBool() &&
              arrived_status["pose_age_ms"].asInt64() <= 250 &&
              arrived_status["live_points"].asUInt64() > 100 &&
              !arrived_status["target_set"].asBool(),
          "arrival must keep localization, map and fresh pose alive");

  request.request_key = "slam-nav-again-without-reload";
  request.pose.x = arrived_status["pose"]["x"].asDouble() + 0.15;
  request.pose.y = arrived_status["pose"]["y"].asDouble();
  Require(perception.Submit(request).accepted,
          "a new navigation goal is accepted after arrival without reloading the map");
  perception.Stop();
}

void TestR1PerceptionMockPolicy() {
  std::set<std::string> enabled_devices{"mid360"};
  auto policy = g1_web::CreateR1DeviceCapabilityPolicy(enabled_devices);
  g1_web::PerceptionService perception(true, false, *policy);
  std::string error;
  Require(perception.Start(error), "task-25 R1 mock perception should start");

  Json::Value status = Parse(perception.SerializeStatus());
  Require(status["initialized"].asBool() &&
              status["slam_rpc_ready"].asBool() &&
              status["device"]["raw_input_required"].asBool() == false &&
              !status["device"]["raw_lidar_rotate_x_180"].asBool() &&
              !status["topics"].isMember("raw_lidar_points") &&
              !status["topics"].isMember("raw_lidar_imu") &&
              status["topics"]["mapping_points"].asString() ==
                  "rt/unitree/slam_mapping/points" &&
              status["topics"]["relocation_odom"].asString() ==
                  "rt/unitree/slam_relocation/odom" &&
              !status["low_obstacle_safety"]["available"].asBool() &&
              status["low_obstacle_safety"]["reason"].asString() ==
                  "raw_low_obstacle_input_unverified",
          "task-25 R1 status must omit unverified raw lidar inputs and expose low-obstacle safety as unavailable");

  Require(status["services"]["mid360_driver"]["enabled"].asBool() &&
              status["services"]["mid360_driver"]["lifecycle"].asString() ==
                  "external_required" &&
              status["services"]["unitree_slam"]["enabled"].asBool() &&
              status["services"]["unitree_slam"]["lifecycle"].asString() ==
                  "managed_by_web",
          "task-25 R1 mock status must preserve dependency ownership policy without importing G1 service names");

  g1_web::PerceptionRequest request;
  request.request_key = "r1-task25-stop-slam";
  request.command = "stop_slam";
  request.confirmed = true;
  Require(perception.Submit(request).accepted,
          "task-25 R1 mock stop_slam should preserve the common confirmed state machine");
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  status = Parse(perception.SerializeStatus());
  const Json::Value stopped_frame = Parse(perception.SerializeFrame());
  Require(status["mode"].asString() == "stopped" &&
              stopped_frame["live_points_count"].asUInt64() == 0 &&
              status["point_source"].asString() != "utlidar_mid360",
          "task-25 R1 stop must not fall back to the G1 raw Mid360 standby preview");

  request.request_key = "r1-task25-map";
  request.command = "start_mapping";
  Require(perception.Submit(request).accepted,
          "task-25 R1 mock should reuse the common mapping state machine");
  request.request_key = "r1-task25-bad-map";
  request.command = "finish_mapping";
  request.map_name = "office.pcd";
  Require(perception.Submit(request).error == "invalid_map_name",
          "task-25 R1 must preserve the test1.pcd through test10.pcd whitelist");
  request.request_key = "r1-task25-good-map";
  request.map_name = "test10.pcd";
  Require(perception.Submit(request).accepted,
          "task-25 R1 must preserve the existing allowed map whitelist");
  perception.Stop();
}

void TestR1CameraPolicyAndMock() {
  auto policy = g1_web::CreateR1DeviceCapabilityPolicy();
  const auto& camera_policy = policy->Camera();
  Require(camera_policy.provider == "unitree_r1_edu_stereo" &&
              !camera_policy.manage_first_person_service &&
              camera_policy.manage_external_services &&
              camera_policy.manage_privileged_receiver &&
              !camera_policy.allow_client_source_override &&
              camera_policy.conflicting_service_match == "videohub" &&
              camera_policy.required_service_match == "stereopatchpc1" &&
              camera_policy.privileged_receiver_helper ==
                  "/usr/local/sbin/r1-web-camera-service" &&
              camera_policy.service_version_status ==
                  "external_check_required" &&
              camera_policy.external_service_requirements.size() == 3,
          "task-24 R1 camera policy must own fixed RobotState service switching and the privileged depth helper");

  const auto* rgb =
      g1_web::FindCameraStream(camera_policy, g1_web::CameraStreamRole::kRgb);
  const auto* left = g1_web::FindCameraStream(
      camera_policy, g1_web::CameraStreamRole::kRgbLeft);
  const auto* right = g1_web::FindCameraStream(
      camera_policy, g1_web::CameraStreamRole::kRgbRight);
  const auto* depth = g1_web::FindCameraStream(
      camera_policy, g1_web::CameraStreamRole::kDepth);
  Require(rgb && left && right && depth &&
              rgb->transport == g1_web::CameraTransport::kRtpH264Udp &&
              rgb->port == 5003 && rgb->width == 544 && rgb->height == 448 &&
              rgb->fps == 10 && !rgb->allow_runtime_dimensions &&
              left->port == 5002 && left->width == 544 && left->height == 448 &&
              right->port == 5003 && right->width == 544 &&
              right->height == 448 &&
              depth->transport == g1_web::CameraTransport::kV4l2Raw16 &&
              depth->fixed_source == "/dev/video-dep" && depth->width == 544 &&
              depth->height == 448 && depth->fps == 10,
          "task-24 R1 policy must fix RGB/eye ports and the raw16 depth device geometry");

  const std::string pipeline = g1_web::BuildFixedRtpH264UdpPipeline(*rgb);
  Require(pipeline.find("udpsrc port=5003") != std::string::npos &&
              pipeline.find("application/x-rtp") != std::string::npos &&
              pipeline.find("watchdog timeout=2000") != std::string::npos &&
              pipeline.find("rtph264depay") != std::string::npos &&
              pipeline.find("avdec_h264") != std::string::npos &&
              pipeline.find("appsink") != std::string::npos &&
              pipeline.find("wait-on-eos=false") != std::string::npos &&
              pipeline.find("/dev/") == std::string::npos,
          "R1 RGB pipeline must come from the fixed RTP/H264 template");
  auto invalid_pipeline_spec = *rgb;
  invalid_pipeline_spec.port = 0;
  Require(g1_web::BuildFixedRtpH264UdpPipeline(invalid_pipeline_spec).empty() &&
              !g1_web::CameraTransportBackendAvailable(
                  g1_web::CameraTransport::kRtpH264Udp, true, false) &&
              g1_web::CameraTransportBackendAvailable(
                  g1_web::CameraTransport::kV4l2Raw16, true, false),
          "RTP capture must fail closed without a valid fixed port or GStreamer");

  std::string validation_error;
  Require(g1_web::ValidateDecodedCameraFrame(*rgb, 544, 448,
                                              validation_error) &&
              !g1_web::ValidateDecodedCameraFrame(*rgb, 448, 544,
                                                   validation_error) &&
              validation_error == "camera_frame_size_mismatch" &&
              !g1_web::ValidateDecodedCameraFrame(*rgb, 1280, 720,
                                                   validation_error) &&
              validation_error == "camera_frame_size_mismatch" &&
              !g1_web::ValidateDecodedCameraFrame(*rgb, 0, 0,
                                                   validation_error) &&
              validation_error == "camera_decode_failed",
          "R1 Web RGB must use the fixed 544x448 landscape stream and reject rotated or unrelated dimensions");
  const std::size_t depth_bytes =
      static_cast<std::size_t>(544) * 448 * 2U;
  Require(g1_web::ValidateRaw16DepthFrame(*depth, 544, 448, depth_bytes,
                                           validation_error) &&
              !g1_web::ValidateRaw16DepthFrame(*depth, 544, 448,
                                                depth_bytes - 2,
                                                validation_error) &&
              validation_error == "depth_short_frame" &&
              !g1_web::ValidateRaw16DepthFrame(*depth, 640, 480,
                                                depth_bytes,
                                                validation_error) &&
              validation_error == "depth_frame_size_mismatch",
          "raw16 R1 depth validation must reject short frames and wrong geometry");
  Require(g1_web::CameraFrameFresh(true, 2000) &&
              !g1_web::CameraFrameFresh(true, 2001) &&
              !g1_web::CameraFrameFresh(false, 0),
          "camera freshness must remain exactly two seconds and fail closed after stale");

  const auto source_file = std::filesystem::path(__FILE__);
  const auto project_root = source_file.parent_path().parent_path();
  std::ifstream camera_stream(project_root / "src/camera_service.cpp");
  const std::string camera_cpp((std::istreambuf_iterator<char>(camera_stream)),
                               std::istreambuf_iterator<char>());
  Require(camera_cpp.find("percentile(0.01)") != std::string::npos &&
              camera_cpp.find("percentile(0.99)") != std::string::npos &&
              camera_cpp.find("cv::COLORMAP_JET") != std::string::npos &&
              camera_cpp.find("ColorizeR1DocumentDepth(depth_view)") !=
                  std::string::npos &&
              camera_cpp.find("std::numeric_limits<std::uint16_t>::max()") !=
                  std::string::npos &&
              camera_cpp.find("std::array<std::uint32_t, 65536> histogram") !=
                  std::string::npos &&
              camera_cpp.find("std::sort(valid.begin(), valid.end())") ==
                  std::string::npos,
          "R1 raw16 visualization must follow the official percentile/JET tutorial, mask 0/65535, and avoid a full per-frame depth sort");
  Require(camera_cpp.find("V4L2_MEMORY_MMAP") != std::string::npos &&
              camera_cpp.find("VIDIOC_DQBUF") != std::string::npos &&
              camera_cpp.find("VIDIOC_QBUF") != std::string::npos &&
              camera_cpp.find("v4l2_fourcc('Y', '1', '6', ' ')") !=
                  std::string::npos &&
              camera_cpp.find("::read(fd, raw.data(), raw.size())") ==
                  std::string::npos,
          "R1 depth capture must use the stable G1-style V4L2 MMAP buffer path instead of partial nonblocking read() frames");
  Require(camera_cpp.find(
              "attempt < 10; ++attempt") != std::string::npos &&
              camera_cpp.find(
                  "if (depth_helper_stop_result_ == 0)") !=
                  std::string::npos &&
              camera_cpp.find(
                  "if (SwitchExternalCameraServiceLocked(stereo_patch_service_name_, false,") !=
                  std::string::npos,
          "R1 camera restore must wait for RobotState convergence and retain ownership until restore succeeds");
  std::ifstream helper_stream(project_root / "scripts/r1-web-camera-service");
  const std::string helper((std::istreambuf_iterator<char>(helper_stream)),
                           std::istreambuf_iterator<char>());
  Require(helper.find("/usr/local/bin/unitree_depth_start") !=
              std::string::npos &&
              helper.find("/usr/local/bin/unitree_depth_stop") !=
                  std::string::npos &&
              helper.find("depth-is-active") != std::string::npos &&
              helper.find("eval") == std::string::npos,
          "R1 privileged camera helper must expose only fixed official depth lifecycle commands");

  g1_web::CameraOptions options;
  options.mock = true;
  g1_web::CameraService camera(options, *policy);
  std::string error;
  Require(camera.Start(error), "R1 mock camera should start without external services");
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  Require(camera.GetFrame("rgb").available &&
              camera.GetFrame("depth").available,
          "R1 mock must publish independent RGB and depth frames");
  const Json::Value status = Parse(camera.SerializeStatus());
  Require(status["fixed_policy"].asBool() &&
              status["provider"].asString() == "unitree_r1_edu_stereo" &&
              status["device"]["streams"].size() == 4 &&
              status["device"]["manage_external_services"].asBool() &&
              status["device"]["manage_privileged_receiver"].asBool() &&
              status["service_version_status"].asString() ==
                  "external_check_required" &&
              !status["first_person_service"]["managed_by_web"].asBool() &&
              !status["first_person_service"]["paused_by_web"].asBool() &&
              !status["external_services"]["depth_receiver"]
                     ["started_by_web"].asBool(),
          "R1 mock status must expose fixed automatic service policy without claiming real service changes");
  const auto capabilities = camera.DeviceCapabilities();
  Require(capabilities.size() == 2 &&
              capabilities[0].hardware_presence ==
                  g1_web::HardwarePresence::kPresent &&
              capabilities[0].external_preparation_ready &&
              !capabilities[0].verification_level.has_value() &&
              capabilities[1].hardware_presence ==
                  g1_web::HardwarePresence::kPresent &&
              capabilities[1].external_preparation_ready &&
              !capabilities[1].verification_level.has_value(),
          "R1 mock camera evidence must not promote either stream to real read-only verification");

  g1_web::CameraRequest request;
  request.request_key = "r1-camera-override";
  request.command = "start_v4l2";
  request.rgb_source = "/dev/video0";
  request.confirmed = true;
  Require(camera.Submit(request).error == "camera_source_override_forbidden",
          "R1 must reject client attempts to override fixed camera sources");
  request.request_key = "r1-camera-fixed-start";
  request.rgb_source.clear();
  request.depth_source.clear();
  Require(camera.Submit(request).accepted,
          "R1 existing camera command schema must start the fixed receiver path");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  request.request_key = "r1-camera-stop-rgb";
  request.command = "stop_rgb";
  Require(camera.Submit(request).accepted,
          "R1 RGB stop should be accepted independently");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const Json::Value depth_only = Parse(camera.SerializeStatus());
  Require(depth_only["running"].asBool() &&
              !depth_only["rgb"]["configured"].asBool() &&
              depth_only["depth"]["configured"].asBool() &&
              !camera.GetFrame("rgb").available &&
              camera.GetFrame("depth").available,
          "stopping R1 RGB must keep the depth stream running");

  request.request_key = "r1-camera-restart-rgb";
  request.command = "start_v4l2";
  Require(camera.Submit(request).accepted,
          "R1 RGB should restart without disabling depth");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  request.request_key = "r1-camera-stop-depth";
  request.command = "stop_depth";
  Require(camera.Submit(request).accepted,
          "R1 depth stop should be accepted independently");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const Json::Value rgb_only = Parse(camera.SerializeStatus());
  Require(rgb_only["running"].asBool() &&
              rgb_only["rgb"]["configured"].asBool() &&
              !rgb_only["depth"]["configured"].asBool() &&
              camera.GetFrame("rgb").available &&
              !camera.GetFrame("depth").available,
          "stopping R1 depth must keep the RGB stream running");

  request.request_key = "r1-camera-stop-all";
  request.command = "stop";
  Require(camera.Submit(request).accepted,
          "R1 global camera stop should remain available");
  Require(!camera.GetFrame("rgb").available &&
              !camera.GetFrame("depth").available,
          "stopped R1 camera must never return cached RGB/depth frames as online");
}

void TestMockCameraCommands() {
  g1_web::CameraOptions options;
  options.mock = true;
  auto device_policy = g1_web::CreateG1DeviceCapabilityPolicy();
  g1_web::CameraService camera(options, *device_policy);
  std::string error;
  Require(camera.Start(error), "mock camera should start");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  Require(camera.GetFrame("rgb").available, "mock RGB frame available");
  const Json::Value low_bandwidth = Parse(camera.SerializeStatus());
  Require(low_bandwidth["running"].asBool() &&
              low_bandwidth["bandwidth_profile"].asString() ==
                  "wifi_low_bandwidth" &&
              low_bandwidth["output_width"].asUInt() == 480 &&
              low_bandwidth["output_height"].asUInt() == 360 &&
              low_bandwidth["rgb_jpeg_quality"].asInt() == 55 &&
              low_bandwidth["depth_jpeg_quality"].asInt() == 45,
          "camera should default to the Wi-Fi low-bandwidth JPEG profile");
  Require(low_bandwidth["rgb"]["jpeg_bytes"].asUInt64() > 0 &&
              low_bandwidth["device"]["model"].asString() ==
                  "Intel RealSense D435i" &&
              low_bandwidth["device"]["helper"].asString() ==
                  "/usr/local/sbin/g1-web-first-person-service" &&
              low_bandwidth["device"]["exclusive_service_dependencies"].size() ==
                  3,
          "camera status should preserve compressed frames and expose G1 deployment metadata");
  const auto camera_capabilities = camera.DeviceCapabilities();
  Require(camera_capabilities.size() == 2 &&
              camera_capabilities[0].key ==
                  g1_web::CapabilityKey::kCameraRgb &&
              camera_capabilities[0].hardware_presence ==
                  g1_web::HardwarePresence::kPresent &&
              camera_capabilities[0].service_available &&
              camera_capabilities[1].key ==
                  g1_web::CapabilityKey::kCameraDepth &&
              camera_capabilities[1].hardware_presence ==
                  g1_web::HardwarePresence::kPresent,
          "mock camera must expose independent RGB/depth runtime device evidence");

  g1_web::CameraRequest request;
  request.request_key = "camera-request-0001";
  request.command = "stop";
  Require(camera.Submit(request).error == "confirmation_required",
          "camera command requires confirmation");
  request.confirmed = true;
  const auto stopped = camera.Submit(request);
  Require(stopped.accepted, "camera stop should be accepted");
  Require(!Parse(camera.SerializeStatus())["rgb"]["configured"].asBool(),
          "stopped camera should become unconfigured");
  Require(!Parse(camera.SerializeStatus())["running"].asBool(),
          "stopped camera should report capture is not running");

  request.request_key = "camera-request-0002";
  request.command = "start_v4l2";
  request.rgb_source = "/tmp/video0";
  Require(camera.Submit(request).error == "invalid_camera_source",
          "camera source outside /dev/videoN must be rejected");
  request.request_key = "camera-request-0003";
  request.rgb_source.clear();
  request.depth_source.clear();
  const auto started = camera.Submit(request);
  Require(started.accepted, "mock runtime camera auto-detect start should succeed");
  const Json::Value auto_status = Parse(camera.SerializeStatus());
  Require(auto_status["auto_detect"].asBool(),
          "empty camera sources should enable automatic device detection");
  Require(auto_status["first_person_service"]["paused_by_web"].asBool(),
          "camera start pauses first-person service in mock mode");
  const auto duplicate = camera.Submit(request);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == started.request_id,
          "duplicate camera request must not restart capture");
  camera.Stop();
  Require(!Parse(camera.SerializeStatus())["first_person_service"]
               ["paused_by_web"].asBool(),
          "camera stop restores first-person service in mock mode");
}

void TestConcurrentSnapshotAccess() {
  g1_web::SnapshotStore store;
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must be registered for concurrent snapshot reads");
  std::atomic<bool> failed{false};
  std::thread writer([&store] {
    for (int i = 0; i < 500; ++i) {
      store.PopulateMock(static_cast<double>(i) / 20.0);
    }
  });
  std::thread reader([&store, &failed, profile] {
    for (int i = 0; i < 500; ++i) {
      const auto json = g1_web::SerializeSnapshot(*profile, store.GetSnapshot());
      if (json.empty() || json.front() != '{') {
        failed.store(true);
      }
    }
  });
  writer.join();
  reader.join();
  Require(!failed.load(), "concurrent reads should remain valid");
}

void TestMockVoiceService() {
  const std::filesystem::path customer_config_path =
      std::filesystem::current_path() / "config" / "customer_voice.json";
  std::error_code cleanup_error;
  std::filesystem::remove(customer_config_path, cleanup_error);

  g1_web::SnapshotStore store;
  g1_web::VoiceService voice(
      store, g1_web::CreateG1AudioCapability(true), true);
  std::string error;
  Require(voice.Start(error), "mock voice service should start");
  Require(error.empty(), "mock voice start should not report an error");

  Require(!voice.EnqueueTts("   ", 0).accepted,
          "empty TTS should be rejected");
  Require(!voice.EnqueueTts("hello", 7).accepted,
          "invalid speaker should be rejected");
  const auto invalid_backend = voice.EnqueueTts("hello", -1, "unknown");
  Require(!invalid_backend.accepted &&
              invalid_backend.error == "invalid_tts_backend",
          "invalid explicit TTS backend should be rejected");
  const auto mixed =
      voice.EnqueueTts("你好 Hello Unitree，欢迎使用 G1", -1, "kokoro");
  Require(mixed.accepted && mixed.speaker_id == -1 &&
              mixed.backend == "kokoro",
          "mixed Chinese and English TTS should accept explicit Kokoro playback");
  Require(!voice.EnqueueTts("你好 Hello", 0).accepted,
          "mixed TTS with a forced single-language speaker should be rejected");
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  Require(store.GetSnapshot().voice.tts.request_id == mixed.request_id &&
              store.GetSnapshot().voice.tts.state == "succeeded" &&
              store.GetSnapshot().voice.tts.speaker_id == -1,
          "mixed TTS should complete as one logical request in mock mode");

  const auto asr_off = voice.SetAsrEnabled(false);
  Require(asr_off.accepted &&
              !store.GetSnapshot().voice.asr_subscribed,
          "mock ASR receiver should be switchable off");
  const auto asr_on = voice.SetAsrEnabled(true);
  Require(asr_on.accepted && store.GetSnapshot().voice.asr_subscribed,
          "mock ASR receiver should be switchable on");
  Require(!voice.SetVolume(101).accepted,
          "volume above 100 must be rejected");
  const auto volume = voice.SetVolume(37);
  Require(volume.accepted && store.GetSnapshot().voice.volume_pct == 37,
          "mock volume change should be reflected in telemetry");

  const auto initial_voice = store.GetSnapshot().voice;
  Require(initial_voice.llm.mode == "builtin" &&
              !initial_voice.chat_go_closed &&
              initial_voice.llm.builtin_api_available &&
              initial_voice.llm.builtin_response_subscribed,
          "builtin LLM mode should expose chat_go DDS interaction by default");
  const auto invalid_mode = voice.ChatWithLlm("auto", "不能自动选择模式");
  Require(!invalid_mode.accepted &&
              invalid_mode.error == "invalid_llm_mode",
          "LLM chat requests must name an explicit supported mode");
  const auto builtin_reply =
      voice.ChatWithLlm("builtin", "你好，笨笨同学");
  Require(builtin_reply.accepted && !builtin_reply.pending &&
              builtin_reply.mode == "builtin" &&
              !builtin_reply.response.empty() &&
              store.GetSnapshot().voice.llm.request_state == "succeeded" &&
              store.GetSnapshot().voice.llm.last_user_message ==
                  "你好，笨笨同学",
          "mock builtin LLM should accept console text and expose a response");
  const auto customer_mode = voice.SetLlmMode(
      "customer", "https://customer.example/v1",
      "test-key", "customer-model");
  if (initial_voice.llm.customer_api_available) {
    Require(customer_mode.accepted &&
                store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "customer" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_api_key_configured,
            "customer LLM mode should normalize a base URL and retain API config");
    const auto serialized_voice =
        g1_web::SerializeVoiceStatus(store.GetSnapshot());
    Require(serialized_voice.find("customer_api_key_configured") !=
                std::string::npos &&
                serialized_voice.find("test-key") == std::string::npos,
            "telemetry may expose key presence but must never expose the API key");
    const auto preserved_key_mode = voice.SetLlmMode(
        "customer", "https://customer.example/v1", "",
        "customer-model-refresh", true);
    Require(preserved_key_mode.accepted &&
                store.GetSnapshot().voice.llm.customer_api_key_configured &&
                store.GetSnapshot().voice.llm.customer_model ==
                    "customer-model-refresh",
            "page refresh should be able to reuse an in-memory key without receiving it back");
    const auto wrong_endpoint_preserve = voice.SetLlmMode(
        "customer", "https://other.example/v1", "", "other-model", true);
    Require(!wrong_endpoint_preserve.accepted &&
                wrong_endpoint_preserve.error ==
                    "customer_api_key_endpoint_changed" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_api_key_configured,
            "an in-memory key must not be silently reused for a different API endpoint");
    store.PopulateMock(2.0);
    Require(store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "customer",
            "mock telemetry refresh must not overwrite the selected LLM mode");
    const auto stale_builtin_reply =
        voice.ChatWithLlm("builtin", "这条消息不能发布到内置 DDS");
    Require(!stale_builtin_reply.accepted &&
                stale_builtin_reply.error == "llm_mode_mismatch" &&
                store.GetSnapshot().voice.llm.last_user_message ==
                    "你好，笨笨同学",
            "customer mode must reject stale builtin requests before DDS publish");
    auto customer_config = voice.GetCustomerVoiceConfig().config;
    customer_config.proxy_url = "http://192.0.2.1:7890";
    customer_config.api_key.clear();
    customer_config.preserve_api_key = true;
    customer_config.update_api_config = true;
    customer_config.role_prompt = "你是展厅里的 G1 接待机器人，回答简洁。";
    customer_config.wake_word = "小兵小兵";
    customer_config.wake_enabled = true;
    customer_config.tts_backend = "kokoro";
    customer_config.qa_entries.push_back(
        {"你叫什么名字", "我是展厅里的 G1 接待机器人。"});
    const auto configured = voice.SetCustomerVoiceConfig(customer_config);
    Require(configured.accepted &&
                configured.config.proxy_url == customer_config.proxy_url &&
                store.GetSnapshot().voice.llm.customer_role_prompt ==
                    customer_config.role_prompt &&
                store.GetSnapshot().voice.llm.customer_wake_enabled &&
                store.GetSnapshot().voice.llm.customer_wake_word ==
                    customer_config.wake_word &&
                store.GetSnapshot().voice.llm.customer_qa_count == 1 &&
                store.GetSnapshot().voice.llm.customer_tts_backend ==
                    "kokoro",
            "customer role, wake word, QA library, and TTS backend should be configurable");
    auto invalid_proxy_config = customer_config;
    invalid_proxy_config.proxy_url = "socks5://192.0.2.1:1080";
    const auto invalid_proxy =
        voice.SetCustomerVoiceConfig(invalid_proxy_config);
    Require(!invalid_proxy.accepted &&
                invalid_proxy.error == "invalid_customer_proxy_url",
            "customer LLM proxy must be restricted to valid HTTP(S) URLs");
    const auto builtin_before_saved_switch =
        voice.SetLlmMode("builtin", "", "", "");
    Require(builtin_before_saved_switch.accepted,
            "switching away from customer mode should succeed before saved-config reuse");
    const auto saved_customer_switch =
        voice.SetLlmMode("customer", "", "", "");
    Require(saved_customer_switch.accepted &&
                store.GetSnapshot().voice.llm.mode == "customer" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_model ==
                    "customer-model-refresh",
            "customer mode switching without API fields should reuse the saved customer API config");
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9001,"text":"小兵，小兵","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Require(store.GetSnapshot().voice.tts.text == "我在" &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "speaking only the configured wake phrase should answer 我在");
    const auto wake_ack_request_id = store.GetSnapshot().voice.tts.request_id;
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9001,"text":"小兵，小兵","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.tts.request_id == wake_ack_request_id,
            "the same ASR index must not trigger the wake acknowledgement twice");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9002,"text":"你叫什么名字？","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa" &&
                store.GetSnapshot().voice.llm.last_response ==
                    "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.tts.text ==
                    "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "speech after a wake acknowledgement should route through QA/LLM without repeating the wake phrase");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9003,"text":"我是展厅里的 G1 接待机器人。","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？",
            "recent robot TTS must not feed back through ASR as a follow-up question");

    g1_web::VoiceServiceTestAccess::ExpireWakeSession(voice);
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9004,"text":"这句不应该自动发送","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？",
            "speech after the wake follow-up window expires must not be auto-submitted");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9005,"text":"小兵，小兵，你叫什么名字？","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa" &&
                store.GetSnapshot().voice.llm.last_response ==
                    "我是展厅里的 G1 接待机器人。",
            "wake phrase punctuation should be ignored and same-utterance trailing speech should route through QA/LLM");

    const auto qa_reply =
        voice.ChatWithLlm("customer", "你叫什么名字？");
    Require(qa_reply.accepted && qa_reply.response_source == "qa" &&
                qa_reply.response == "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa",
            "customer QA library should answer matching questions before the model path");

    g1_web::CustomerVoiceConfig duplicate_qa_config = customer_config;
    duplicate_qa_config.qa_entries = {
        {"您好", "旧回答"},
        {"您好", "新回答"},
        {"天气怎么样", "今天天气很好"},
        {"您好", "最终回答"},
    };
    const auto deduplicated = voice.SetCustomerVoiceConfig(duplicate_qa_config);
    Require(deduplicated.accepted &&
                deduplicated.config.qa_entries.size() == 2 &&
                store.GetSnapshot().voice.llm.customer_qa_count == 2,
            "saving QA should replace duplicate questions instead of appending them");
    const auto latest_duplicate_reply = voice.ChatWithLlm("customer", "您好");
    Require(latest_duplicate_reply.accepted &&
                latest_duplicate_reply.response_source == "qa" &&
                latest_duplicate_reply.response == "最终回答",
            "the last answer for a repeated QA question should replace older answers");
    const auto restored_customer_config =
        voice.SetCustomerVoiceConfig(customer_config);
    Require(restored_customer_config.accepted &&
                restored_customer_config.config.qa_entries.size() == 1,
            "saving the original QA library should fully replace the temporary library");
    const auto customer_reply =
        voice.ChatWithLlm("customer", "你好，客户模型");
    Require(customer_reply.accepted && !customer_reply.pending &&
                customer_reply.request_id == 0 &&
                customer_reply.mode == "customer" &&
                customer_reply.response_source == "llm" &&
                customer_reply.response.find("模拟客户大模型回复") == 0 &&
                customer_reply.tts_request_id == 0 &&
                store.GetSnapshot().voice.llm.request_state == "succeeded",
            "mock customer mode must use the customer model path when QA does not match");
    const auto spoken_customer_reply =
        voice.ChatWithLlm("customer", "请自动播报", true);
    Require(spoken_customer_reply.accepted &&
                spoken_customer_reply.tts_request_id > 0 &&
                spoken_customer_reply.tts_error.empty(),
            "customer replies should enqueue automatic TTS when requested");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Require(store.GetSnapshot().voice.tts.request_id ==
                spoken_customer_reply.tts_request_id &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "automatic customer reply TTS should complete in mock mode");
    const auto builtin_mode = voice.SetLlmMode("builtin", "", "", "");
    Require(builtin_mode.accepted &&
                !store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "builtin",
            "switching back to builtin mode should re-enable chat_go");
    store.PopulateMock(3.0);
    Require(!store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "builtin",
            "mock telemetry refresh must preserve builtin LLM mode");
    const auto stale_customer_reply =
        voice.ChatWithLlm("customer", "这条消息不能调用客户 API");
    Require(!stale_customer_reply.accepted &&
                stale_customer_reply.error == "llm_mode_mismatch" &&
                store.GetSnapshot().voice.llm.last_user_message ==
                    "请自动播报",
            "builtin mode must reject stale customer requests before HTTP call");
  } else {
    Require(!customer_mode.accepted &&
                customer_mode.error == "customer_api_unavailable",
            "customer mode should report unavailable without libcurl");
  }

  const auto accepted = voice.EnqueueTts("语音测试", 0);
  Require(accepted.accepted && accepted.request_id > 0,
          "valid TTS should enter the queue");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto snapshot = store.GetSnapshot();
  Require(snapshot.voice.tts.request_id == accepted.request_id,
          "TTS request id should be retained");
  Require(snapshot.voice.tts.state == "succeeded",
          "mock TTS should finish successfully");
  const auto english = voice.EnqueueTts("Hello from Unitree", -1);
  Require(english.accepted && english.speaker_id == 1,
          "automatic language selection should choose English speaker 1");
  Require(!voice.EnqueueTts("Hello from Unitree", -1).accepted,
          "a rapid duplicate TTS request must be rejected");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  Require(store.GetSnapshot().voice.tts.speaker_id == 1,
          "resolved English speaker should reach the TTS worker");

  const auto saved_config = voice.GetCustomerVoiceConfig();
  Require(saved_config.accepted && saved_config.config.api_key_configured &&
              saved_config.config.proxy_url == "http://192.0.2.1:7890" &&
              !saved_config.config.api_key.empty() &&
              saved_config.config.api_key != "test-key",
          "customer config API must expose only a masked saved-key indicator");
  Require(std::filesystem::exists(customer_config_path),
          "customer API and enhancement config should persist to disk");
  const auto config_permissions =
      std::filesystem::status(customer_config_path).permissions();
  const auto forbidden_permissions =
      std::filesystem::perms::group_read |
      std::filesystem::perms::group_write |
      std::filesystem::perms::group_exec |
      std::filesystem::perms::others_read |
      std::filesystem::perms::others_write |
      std::filesystem::perms::others_exec;
  Require((config_permissions & forbidden_permissions) ==
              std::filesystem::perms::none &&
              (config_permissions & std::filesystem::perms::owner_read) !=
                  std::filesystem::perms::none &&
              (config_permissions & std::filesystem::perms::owner_write) !=
                  std::filesystem::perms::none,
          "customer config containing API key must be owner read/write only");
  std::ifstream persisted_stream(customer_config_path);
  std::stringstream persisted_buffer;
  persisted_buffer << persisted_stream.rdbuf();
  Require(persisted_buffer.str().find("test-key") != std::string::npos,
          "persisted customer config should retain the API key across restart");

  // Simulate a user or an older build directly editing a polluted JSON file.
  // GET must reload the disk source, deduplicate by question (last answer wins),
  // update runtime state, and repair the file instead of preserving duplicates.
  Json::CharReaderBuilder disk_reader_builder;
  Json::Value polluted_root;
  std::string disk_parse_errors;
  {
    std::ifstream disk_input(customer_config_path);
    Require(Json::parseFromStream(disk_reader_builder, disk_input,
                                  &polluted_root, &disk_parse_errors),
            "persisted customer config should be valid JSON before disk-sync test");
  }
  Json::Value polluted_entries(Json::arrayValue);
  for (int i = 0; i < 9; ++i) {
    Json::Value item(Json::objectValue);
    item["question"] = "您好";
    item["answer"] = i == 8 ? "磁盘最新回答" : "磁盘旧回答";
    polluted_entries.append(item);
  }
  Json::Value second_item(Json::objectValue);
  second_item["question"] = "从文件同步";
  second_item["answer"] = "已经同步";
  polluted_entries.append(second_item);
  polluted_root["qa_entries"] = polluted_entries;
  {
    Json::StreamWriterBuilder disk_writer_builder;
    disk_writer_builder["indentation"] = "  ";
    std::ofstream disk_output(customer_config_path,
                              std::ios::binary | std::ios::trunc);
    disk_output << Json::writeString(disk_writer_builder, polluted_root);
  }
  const auto disk_synced_config = voice.GetCustomerVoiceConfig();
  Require(disk_synced_config.accepted &&
              disk_synced_config.config.qa_entries.size() == 2 &&
              disk_synced_config.config.qa_entries[0].question == "您好" &&
              disk_synced_config.config.qa_entries[0].answer == "磁盘最新回答",
          "customer config GET should reload disk QA and collapse repeated questions");
  const auto disk_synced_customer_mode = voice.SetLlmMode(
      "customer", "https://customer.example/v1", "",
      "customer-model-refresh", true);
  Require(disk_synced_customer_mode.accepted,
          "disk-sync QA runtime test should reactivate the persisted customer mode");
  const auto disk_synced_reply = voice.ChatWithLlm("customer", "您好");
  Require(disk_synced_reply.accepted &&
              disk_synced_reply.response_source == "qa" &&
              disk_synced_reply.response == "磁盘最新回答",
          "disk-reloaded fixed answer should immediately become active at runtime");
  {
    Json::Value repaired_root;
    std::string repaired_parse_errors;
    std::ifstream repaired_input(customer_config_path);
    Require(Json::parseFromStream(disk_reader_builder, repaired_input,
                                  &repaired_root, &repaired_parse_errors) &&
                repaired_root["qa_entries"].isArray() &&
                repaired_root["qa_entries"].size() == 2,
            "loading a polluted QA file should repair duplicates on disk");
  }

  voice.Stop();

  g1_web::SnapshotStore restarted_store;
  g1_web::VoiceService restarted_voice(
      restarted_store, g1_web::CreateG1AudioCapability(true), true);
  std::string restart_error;
  Require(restarted_voice.Start(restart_error),
          "restarted voice service should load persisted customer config");
  const auto restarted_snapshot = restarted_store.GetSnapshot().voice;
  Require(restarted_snapshot.llm.customer_api_url ==
              "https://customer.example/v1/chat/completions" &&
              restarted_snapshot.llm.customer_model ==
                  "customer-model-refresh" &&
              restarted_snapshot.llm.customer_api_key_configured,
          "restart should restore API URL, model, and saved API key");
  const auto loaded_customer_mode = restarted_voice.SetLlmMode(
      "customer", "https://customer.example/v1", "",
      "customer-model-refresh", true);
  Require(loaded_customer_mode.accepted,
          "persisted API key should be reusable after service restart");
  const auto replacement_mode = restarted_voice.SetLlmMode(
      "customer", "https://customer.example/v1", "replacement-key",
      "customer-model-v2", false);
  Require(replacement_mode.accepted,
          "entering a new API key should replace the persisted old key");
  std::ifstream replaced_stream(customer_config_path);
  std::stringstream replaced_buffer;
  replaced_buffer << replaced_stream.rdbuf();
  Require(replaced_buffer.str().find("replacement-key") != std::string::npos &&
              replaced_buffer.str().find("test-key") == std::string::npos,
          "persisted config should atomically overwrite the previous API key");
  restarted_voice.Stop();
  std::filesystem::remove(customer_config_path, cleanup_error);
}

void TestUnavailableAudioCapabilityDegradesOnlyAudio() {
  g1_web::SnapshotStore store;
  g1_web::VoiceService voice(
      store, std::make_unique<UnavailableAudioCapability>(), true);
  std::string error;
  Require(voice.Start(error) && error.empty(),
          "an unavailable audio adapter must not fail the shared Voice core");
  const auto snapshot = store.GetSnapshot().voice;
  Require(snapshot.initialized && snapshot.llm.mode == "builtin" &&
              snapshot.llm.builtin_api_available &&
              snapshot.initialization_error == "mock_audio_unavailable",
          "Voice/LLM must stay initialized while audio degradation remains diagnosable");

  const auto tts = voice.EnqueueTts("audio unavailable", -1, "unitree");
  Require(!tts.accepted && tts.error == "audio_capability_unavailable",
          "TTS must fail explicitly when the audio adapter is unavailable");
  const auto volume = voice.SetVolume(50);
  Require(!volume.accepted &&
              volume.error == "audio_capability_unavailable",
          "volume must fail explicitly when the audio adapter is unavailable");
  const auto builtin = voice.ChatWithLlm("builtin", "voice core remains active");
  Require(builtin.accepted,
          "builtin LLM must remain usable when only audio output is unavailable");
  voice.Stop();
}

class PollingStateLocomotion final : public g1_web::ILocomotion {
 public:
  PollingStateLocomotion(std::uint32_t fsm_id = 811,
                         std::uint32_t fsm_mode = 1)
      : fsm_id_(fsm_id), fsm_mode_(fsm_mode) {}

  bool Initialize(g1_web::LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    error.clear();
    return true;
  }
  bool QueryState(g1_web::LocomotionInitialization& state,
                  std::string& error) override {
    state.state_valid = true;
    state.fsm_id = fsm_id_;
    state.fsm_mode = fsm_mode_;
    error.clear();
    return true;
  }
  void Shutdown() override {}
  bool IsKnownModeCommand(const std::string&, std::int32_t) const override {
    return true;
  }
  bool IsOperationalFsm(std::uint32_t fsm_id) const override {
    return fsm_id == 811;
  }
  std::string ValidateVelocity(const g1_web::ControlData&, float, float,
                               float, std::int32_t) const override {
    return {};
  }
  std::string ValidateVelocityExecution(const g1_web::ControlData&,
                                        std::int32_t) const override {
    return {};
  }
  g1_web::LocomotionCommandResult ExecuteMode(
      const std::string&, std::int32_t) override {
    return {0, -1, {}};
  }
  g1_web::LocomotionCommandResult ApplyVelocity(
      float, float, float, std::int32_t, bool) override {
    return {0, -1, {}};
  }

 private:
  std::uint32_t fsm_id_;
  std::uint32_t fsm_mode_;
};

class FailingMockLocomotion final : public g1_web::ILocomotion {
 public:
  bool Initialize(g1_web::LocomotionInitialization& state,
                  std::string& error) override {
    state = {};
    error.clear();
    return true;
  }
  void Shutdown() override {}
  bool IsKnownModeCommand(const std::string&, std::int32_t) const override {
    return true;
  }
  bool IsOperationalFsm(std::uint32_t) const override { return true; }
  std::string ValidateVelocity(const g1_web::ControlData&, float, float,
                               float, std::int32_t) const override {
    return {};
  }
  std::string ValidateVelocityExecution(const g1_web::ControlData&,
                                        std::int32_t) const override {
    return {};
  }
  g1_web::LocomotionCommandResult ExecuteMode(
      const std::string&, std::int32_t) override {
    return {0, -1, {}};
  }
  g1_web::LocomotionCommandResult ApplyVelocity(
      float, float, float, std::int32_t, bool active) override {
    return active ? g1_web::LocomotionCommandResult{-1, -1, "sdk_api_error"}
                  : g1_web::LocomotionCommandResult{0, -1, {}};
  }
};

void TestSafetyManager() {
  g1_web::SafetyManager safety;
  const auto confirmation = safety.CheckConfirmation(false);
  Require(!confirmation.allowed &&
              confirmation.reason == "confirmation_required" &&
              confirmation.error == "confirmation_required",
          "SafetyManager must preserve confirmation_required mapping");
  const auto ready = safety.CheckServiceReady(false);
  Require(!ready.allowed && ready.reason == "service_not_ready" &&
              ready.error == "control_not_ready",
          "SafetyManager must preserve control_not_ready mapping");
  const auto freshness =
      safety.CheckFreshness(false, "sport_state_stale");
  Require(!freshness.allowed && freshness.reason == "state_stale" &&
              freshness.error == "sport_state_stale",
          "SafetyManager must preserve stale-state compatibility errors");
  const auto lease = safety.CheckLease(false);
  Require(!lease.allowed && lease.reason == "control_lease_expired" &&
              lease.error == "control_lease_expired",
          "SafetyManager must reject expired control leases");

  auto profile = *g1_web::RobotRegistry::Find("g1");
  auto* locomotion = static_cast<g1_web::CapabilityDescriptor*>(nullptr);
  for (auto& capability : profile.capabilities) {
    if (capability.key == g1_web::CapabilityKey::kLocomotion) {
      locomotion = &capability;
      break;
    }
  }
  Require(locomotion != nullptr, "G1 must declare locomotion capability");
  locomotion->verification_level = g1_web::VerificationLevel::kMockVerified;
  Require(safety.CheckCapability(profile, g1_web::CapabilityKey::kLocomotion,
                                 true)
              .allowed,
          "Mock control should accept mock_verified capability");
  const auto real_unverified = safety.CheckCapability(
      profile, g1_web::CapabilityKey::kLocomotion, false);
  Require(!real_unverified.allowed &&
              real_unverified.reason == "capability_verification" &&
              real_unverified.error == "control_not_ready",
          "real control must require control_verified capability unless the profile explicitly enables operator validation");
  const auto r1_profile = *g1_web::RobotRegistry::Find("r1");
  Require(safety.CheckCapability(r1_profile,
                                 g1_web::CapabilityKey::kLocomotion, false)
              .allowed,
          "R1 operator-validation policy must expose real control without claiming control_verified evidence");
  locomotion->verification_level =
      g1_web::VerificationLevel::kControlVerified;
  Require(safety.CheckCapability(profile, g1_web::CapabilityKey::kLocomotion,
                                 false)
              .allowed,
          "real control should accept control_verified capability");
  locomotion->available = false;
  const auto unavailable = safety.CheckCapability(
      profile, g1_web::CapabilityKey::kLocomotion, true);
  Require(!unavailable.allowed &&
              unavailable.reason == "capability_unavailable" &&
              unavailable.error == "control_not_ready",
          "unavailable capability must fail closed in Mock too");

  g1_web::SnapshotStore polling_store;
  g1_web::ControlService polling_control(
      polling_store, std::make_unique<PollingStateLocomotion>(), r1_profile,
      nullptr, false);
  std::string polling_error;
  Require(polling_control.Start(polling_error),
          "R1 lowstate-only control must start with a read-only locomotion state poller");
  for (int attempt = 0; attempt < 50; ++attempt) {
    const auto state = polling_store.GetSnapshot().control;
    if (state.sport_state_received && state.fsm_id == 811 &&
        state.fsm_mode == 1) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto polled_state = polling_store.GetSnapshot().control;
  Require(polled_state.sport_state_received && polled_state.fsm_id == 811 &&
              polled_state.fsm_mode == 1,
          "R1 runtime must refresh the real FSM mode through ILocomotion::QueryState");
  const auto blocked_mode = polling_control.Submit(
      "r1-dynamic-start-0001", "mode", "start", 0, true);
  Require(!blocked_mode.accepted &&
              blocked_mode.error == "robot_not_static",
          "R1 dynamic FSM mode must block ordinary mode switching");
  const auto damping_fallback = polling_control.Submit(
      "r1-dynamic-damp-0002", "mode", "damp", 0, true);
  Require(damping_fallback.accepted,
          "R1 damping fallback must remain available in dynamic FSM mode");
  polling_control.Stop();

  const auto check_r1_transition =
      [&r1_profile](std::uint32_t fsm_id, const std::string& command,
                    bool expected, const std::string& request_key) {
        g1_web::SnapshotStore transition_store;
        g1_web::ControlService transition_control(
            transition_store,
            std::make_unique<PollingStateLocomotion>(
                fsm_id, std::numeric_limits<std::uint32_t>::max()),
            r1_profile, nullptr, false);
        std::string transition_error;
        Require(transition_control.Start(transition_error),
                "R1 transition test control must start");
        for (int attempt = 0; attempt < 50; ++attempt) {
          const auto state = transition_store.GetSnapshot().control;
          if (state.sport_state_received && state.fsm_id == fsm_id) break;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const auto result = transition_control.Submit(
            request_key, "mode", command, 0, true);
        Require(result.accepted == expected &&
                    (expected || result.error == "mode_transition_not_allowed"),
                "R1 unknown-mode firmware must follow the explicit FSM transition table");
        transition_control.Stop();
      };
  check_r1_transition(1, "zero_torque", true, "r1-damp-zero-0003");
  check_r1_transition(1, "stand_up", true, "r1-damp-stand-0004");
  check_r1_transition(1, "start", false, "r1-damp-start-0005");
  check_r1_transition(4, "start", true, "r1-stand-start-0006");
  check_r1_transition(811, "stand_up", true, "r1-walk-stand-0007");
  check_r1_transition(4, "lie_to_stand", true, "r1-stand-lie-up-0008");
  check_r1_transition(811, "stand_to_lie", true, "r1-stand-lie-0009");
  check_r1_transition(702, "lie_to_stand", true, "r1-lie-stand-0010");
  check_r1_transition(811, "zero_torque", false, "r1-walk-zero-0011");

  g1_web::ResourceManager resources;
  Require(resources.Acquire(g1_web::ControlResource::Locomotion, "first")
              .acquired,
          "resource setup should acquire locomotion");
  const auto resource = safety.CheckResource(
      resources.Acquire(g1_web::ControlResource::Arm, "second"));
  Require(!resource.allowed && resource.reason == "resource_busy" &&
              resource.error == "control_busy",
          "SafetyManager must map ResourceManager conflicts consistently");
  safety.CleanupControlFailure(resources, "first");
  Require(resources.Query(g1_web::ControlResource::Locomotion).empty(),
          "failure cleanup must release resources owned by failed control");

  g1_web::SnapshotStore store;
  store.PopulateMock(0.0);
  const auto* g1_profile = g1_web::RobotRegistry::Find("g1");
  g1_web::ControlService control(
      store, std::make_unique<FailingMockLocomotion>(), *g1_profile,
      g1_web::CreateG1JointDebugPolicy(*g1_profile), true);
  std::string error;
  Require(control.Start(error), "failing Mock control should start");
  const auto submitted =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(submitted.accepted,
          "failing Mock velocity should pass logical safety checks first");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (store.GetSnapshot().control.motion.state == "failed") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto failed = store.GetSnapshot().control.motion;
  Require(failed.state == "failed" && !failed.active &&
              failed.vx_m_s == 0.0F && failed.vy_m_s == 0.0F &&
              failed.vyaw_rad_s == 0.0F && failed.speed_mode == 0,
          "control failure must clear active motion state and commanded speeds");
  Require(control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true).accepted,
          "control failure cleanup must release the locomotion resource");
  control.Stop();
}

void TestResourceManager() {
  g1_web::ResourceManager resources;
  const auto first = resources.Acquire(
      g1_web::ControlResource::Locomotion, "locomotion.motion");
  Require(first.acquired && !first.already_owned &&
              first.error.empty() &&
              resources.Query(g1_web::ControlResource::Locomotion) ==
                  "locomotion.motion",
          "resource manager should record the first owner");

  const auto duplicate = resources.Acquire(
      g1_web::ControlResource::Locomotion, "locomotion.motion");
  Require(duplicate.acquired && duplicate.already_owned,
          "repeated acquire by the same owner should be idempotent");

  const auto conflict = resources.Acquire(
      g1_web::ControlResource::Arm, "joint_debug.arm");
  Require(!conflict.acquired && conflict.error == "resource_busy",
          "a different control owner should see a resource conflict");
  Require(!resources.Release(g1_web::ControlResource::Locomotion,
                             "wrong_owner") &&
              resources.Query(g1_web::ControlResource::Locomotion) ==
                  "locomotion.motion",
          "a non-owner must not release a resource");
  Require(resources.Release(g1_web::ControlResource::Locomotion,
                            "locomotion.motion") &&
              resources.Query(g1_web::ControlResource::Locomotion).empty(),
          "the owner should release its resource");

  Require(resources.Acquire(g1_web::ControlResource::WholeBody,
                            "joint_debug.whole_body")
              .acquired &&
              resources.Acquire(g1_web::ControlResource::LowCmd,
                                "joint_debug.whole_body")
                  .acquired,
          "one owner should be able to hold its paired resources");
  resources.ReleaseOwner("joint_debug.whole_body");
  Require(resources.Query(g1_web::ControlResource::WholeBody).empty() &&
              resources.Query(g1_web::ControlResource::LowCmd).empty(),
          "release-owner should free every resource held by that owner");

  g1_web::ResourceManager concurrent;
  std::atomic<int> ready{0};
  std::atomic<int> winners{0};
  std::atomic<bool> start{false};
  const auto compete = [&](const char* owner) {
    ready.fetch_add(1);
    while (!start.load()) std::this_thread::yield();
    if (concurrent.Acquire(g1_web::ControlResource::Arm, owner).acquired)
      winners.fetch_add(1);
  };
  std::thread first_thread(compete, "arm.first");
  std::thread second_thread(compete, "arm.second");
  while (ready.load() != 2) std::this_thread::yield();
  start.store(true);
  first_thread.join();
  second_thread.join();
  Require(winners.load() == 1 &&
              !concurrent.Query(g1_web::ControlResource::Arm).empty(),
          "concurrent acquire should produce exactly one owner");
  concurrent.ReleaseOwner("arm.first");
  concurrent.ReleaseOwner("arm.second");
  Require(concurrent.Query(g1_web::ControlResource::Arm).empty(),
          "concurrent winner should be releasable");
}

void TestMockControlSafety() {
  g1_web::SnapshotStore store;
  store.PopulateMock(0.0);
  auto locomotion = g1_web::CreateG1LocomotionAdapter(true);
  Require(locomotion->IsOperationalFsm(500) &&
              locomotion->IsOperationalFsm(501) &&
              locomotion->IsOperationalFsm(801) &&
              locomotion->IsOperationalFsm(802) &&
              !locomotion->IsOperationalFsm(1),
          "G1 operational FSM policy must stay in the locomotion adapter");
  Require(locomotion->IsKnownModeCommand("set_fsm_id", 702) &&
              !locomotion->IsKnownModeCommand("set_fsm_id", 999),
          "G1 mode command allowlist must stay in the locomotion adapter");
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  g1_web::ControlService control(
      store, std::move(locomotion), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true);
  std::string error;
  Require(control.Start(error), "mock control service should start");
  Require(error.empty(), "mock control start should not report an error");

  auto stale_control = store.GetSnapshot().control;
  stale_control.sport_state_last_update =
      g1_web::SteadyClock::now() - std::chrono::seconds(2);
  store.SetControlState(stale_control);
  const auto stale_motion =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(!stale_motion.accepted &&
              stale_motion.error == "sport_state_stale",
          "SafetyManager must reject stale sport state before G1 policy");
  unitree_hg::msg::dds_::SportModeState_ restored_sport_state;
  restored_sport_state.fsm_id(500);
  restored_sport_state.fsm_mode(0);
  restored_sport_state.task_id(0);
  restored_sport_state.task_time(0.0F);
  store.UpdateSportMode(restored_sport_state);

  const auto excessive_motion =
      control.SubmitVelocity(0.51F, 0.0F, 0.0F, 0, true);
  Require(!excessive_motion.accepted &&
              excessive_motion.error == "velocity_out_of_range",
          "velocity above the low preset limit must be rejected");

  const auto invalid_speed_mode =
      control.SubmitVelocity(0.5F, 0.0F, 0.0F, 2, true);
  Require(!invalid_speed_mode.accepted &&
              invalid_speed_mode.error == "invalid_speed_mode",
          "unexposed speed modes must be rejected");

  const auto conventional_medium =
      control.SubmitVelocity(1.0F, 0.35F, 0.8F, 1, true);
  Require(conventional_medium.accepted,
          "regular medium preset should keep its existing limits");
  const auto excessive_regular_lateral =
      control.SubmitVelocity(0.0F, 0.36F, 0.0F, 1, true);
  Require(!excessive_regular_lateral.accepted &&
              excessive_regular_lateral.error == "velocity_out_of_range",
          "regular medium lateral limit must not increase");

  const auto walkrun_required =
      control.SubmitVelocity(3.0F, 0.0F, 0.0F, 3, true);
  Require(!walkrun_required.accepted &&
              walkrun_required.error == "speed_mode_requires_walkrun",
          "high preset should require walk-run mode");

  const auto forward_motion =
      control.SubmitVelocity(0.5F, 0.0F, 0.0F, 0, true);
  Require(forward_motion.accepted,
          "safe low forward velocity should be accepted in locomotion FSM");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (store.GetSnapshot().control.motion.state == "active") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Require(store.GetSnapshot().control.motion.state == "active",
          "mock velocity should become active");

  const auto stop_motion =
      control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);
  Require(stop_motion.accepted, "mock stop should always be accepted");
  for (int attempt = 0; attempt < 300; ++attempt) {
    if (store.GetSnapshot().control.motion.state == "stopped") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Require(store.GetSnapshot().control.motion.state == "stopped",
          "mock stop should immediately clear motion");

  unitree_hg::msg::dds_::SportModeState_ walkrun;
  walkrun.fsm_id(802);
  walkrun.fsm_mode(0);
  walkrun.task_id(0);
  walkrun.task_time(0.0F);
  store.UpdateSportMode(walkrun);
  const auto walkrun_low =
      control.SubmitVelocity(0.5F, 0.4F, 1.1F, 0, true);
  Require(walkrun_low.accepted,
          "walk-run low preset should use the boosted movement limits");
  const auto walkrun_medium =
      control.SubmitVelocity(1.0F, 0.6F, 1.3F, 1, true);
  Require(walkrun_medium.accepted,
          "walk-run medium preset should use the boosted movement limits");
  const auto excessive_walkrun_medium =
      control.SubmitVelocity(0.0F, 0.61F, 0.0F, 1, true);
  Require(!excessive_walkrun_medium.accepted &&
              excessive_walkrun_medium.error == "velocity_out_of_range",
          "walk-run medium lateral limit must still be enforced");
  const auto maximum_forward =
      control.SubmitVelocity(3.0F, 0.0F, 0.0F, 3, true);
  Require(maximum_forward.accepted,
          "official 3.0 m/s high preset should be accepted in walk-run FSM");
  const auto turn_left =
      control.SubmitVelocity(0.0F, 0.0F, 1.3F, 1, true);
  Require(turn_left.accepted,
          "boosted walk-run medium yaw should be accepted for A/D turning");
  const auto maximum_turn =
      control.SubmitVelocity(0.0F, 0.0F, 1.5F, 3, true);
  Require(maximum_turn.accepted,
          "high preset should accept the 1.5 rad/s yaw ceiling");
  const auto excessive_turn =
      control.SubmitVelocity(0.0F, 0.0F, 1.51F, 3, true);
  Require(!excessive_turn.accepted &&
              excessive_turn.error == "velocity_out_of_range",
          "yaw velocity above the server ceiling must be rejected");
  control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);

  const auto wrong_confirmation = control.Submit(
      "request-key-0001", "mode", "damp", 0, false);
  Require(!wrong_confirmation.accepted &&
              wrong_confirmation.error == "confirmation_required",
          "mode command requires explicit confirmation");

  const auto damp = control.Submit(
      "request-key-0002", "mode", "damp", 0, true);
  Require(damp.accepted, "mock damping command should be accepted");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  Require(store.GetSnapshot().control.fsm_id == 1,
          "mock damping command should reach fsm 1");

  const auto duplicate = control.Submit(
      "request-key-0002", "mode", "damp", 0, true);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == damp.request_id,
          "duplicate request key must not execute twice");

  const auto zero_torque = control.Submit(
      "request-key-0003", "mode", "zero_torque", 0, true);
  Require(zero_torque.accepted,
          "mock zero-torque command should be accepted");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto snapshot = store.GetSnapshot();
  Require(snapshot.control.fsm_id == 0,
          "mock zero-torque command should reach fsm 0");
  Require(snapshot.control.last_command.state == "succeeded",
          "mock mode command should finish successfully");

  Require(control.IsKnownCommand("arm_action", "execute", 27),
          "documented handshake action should be allowlisted");
  Require(!control.IsKnownCommand("arm_action", "execute", 999),
          "unknown arm action must be rejected");
  Require(control.IsKnownCommand("arm_action", "execute_custom", 0),
          "recorded teach actions should use the custom action command");
  Require(!control.Submit("request-key-0004", "arm_action",
                          "execute_custom", 0, true, "")
               .accepted,
          "teach action execution requires a non-empty action name");
  unitree_hg::msg::dds_::SportModeState_ arm_ready;
  arm_ready.fsm_id(500);
  arm_ready.fsm_mode(0);
  arm_ready.task_id(0);
  arm_ready.task_time(0.0F);
  store.UpdateSportMode(arm_ready);
  const auto teach_action = control.Submit(
      "request-key-0005", "arm_action", "execute_custom", 0, true,
      "Waist_Drum_Dance");
  Require(teach_action.accepted,
          "mock recorded teach action should be accepted in a safe arm FSM");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto teach_snapshot = store.GetSnapshot();
  Require(teach_snapshot.control.task_id == 100 &&
              teach_snapshot.control.last_command.action_name ==
                  "Waist_Drum_Dance" &&
              teach_snapshot.control.last_command.state == "succeeded",
          "teach action name and custom task id should reach telemetry");
  for (const int fsm_id :
       {0, 1, 2, 3, 4, 500, 501, 702, 706, 801, 802}) {
    Require(control.IsKnownCommand("mode", "set_fsm_id", fsm_id),
            "documented FSM id should be allowlisted");
  }
  Require(!control.IsKnownCommand("mode", "set_fsm_id", 999),
          "unknown FSM id must be rejected");
  control.Stop();
}

void TestR1MockJointDebugger() {
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  const auto teach_store = std::filesystem::temp_directory_path() /
      ("r1_web_joint_teach_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) +
       ".json");
  const auto* profile = g1_web::RobotRegistry::Find("r1");
  Require(profile != nullptr, "R1 profile must exist for Mock Joint Debug");
  std::vector<std::size_t> semantic_slots;
  for (const auto& joint : profile->joint_schema.joints)
    semantic_slots.push_back(joint.motor_slot);

  g1_web::SnapshotStore store;
  store.PopulateMock(0.0, 1, semantic_slots);
  g1_web::ControlService control(
      store, g1_web::CreateR1LocomotionAdapter(true), *profile,
      g1_web::CreateR1JointDebugPolicy(*profile), true, web_root.string(),
      teach_store.string());
  std::string error;
  Require(control.Start(error), "R1 Mock Joint Debug should start");

  const auto initial = Parse(control.SerializeJointDebugStatus());
  Require(initial["joints"].size() == 26 &&
              initial["fsm_id"].asUInt() == 811 &&
              initial["upper_body_allowed"].asBool() &&
              initial["control_topics"]["upper_body"].asString() ==
                  "rt/arm_sdk" &&
              initial["control_topics"]["full_body"].asString() ==
                  "rt/lowcmd" &&
              initial["joints"][24]["motor_slot"].asUInt() == 29 &&
              initial["joints"][25]["motor_slot"].asUInt() == 30 &&
              initial["joints"][12]["control_modes"].size() == 1 &&
              initial["joints"][12]["control_modes"][0].asString() ==
                  "full_body" &&
              initial["joints"][13]["control_modes"].size() == 2 &&
              initial["joints"][13]["control_modes"][0].asString() ==
                  "upper_body" &&
              initial["joints"][24]["control_modes"].size() == 2 &&
              initial["joints"][24]["control_modes"][0].asString() ==
                  "upper_body" &&
              initial["teach_actions"].empty() &&
              initial["remote_binding_supported"].asBool() &&
              initial["remote_binding_options"].size() == 16 &&
              initial["remote_control_ready"].asBool() &&
              initial["remote_control_error"].asString().empty(),
          "R1 Joint Debug status must expose head teaching and shared G1-style remote bindings");

  const auto waist_roll =
      control.ApplyJointDebug("upper_body", {{12, 0.0F}}, true);
  Require(!waist_roll.accepted &&
              waist_roll.error == "upper_body_joint_not_allowed",
          "R1 upper-body mode must reject waist_roll because arm_sdk only supports waist_yaw");

  Require(control.IsKnownCommand("arm_action", "execute", 11) &&
              control.IsKnownCommand("arm_action", "execute", 27) &&
              control.IsKnownCommand("arm_action", "execute", 36) &&
              control.IsKnownCommand("arm_action", "execute", 99) &&
              !control.IsKnownCommand("arm_action", "execute", 0) &&
              control.IsKnownCommand("arm_action", "execute_custom", 0) &&
              control.IsKnownCommand("arm_action", "stop_custom", 0),
          "R1 must expose the official arm service preset and custom-action commands");
  const auto wave =
      control.Submit("r1-arm-wave-0001", "arm_action", "execute", 27, true);
  Require(wave.accepted, "R1 Mock arm preset should use the shared safe command path");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (store.GetSnapshot().control.last_command.state == "running") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto action_running = store.GetSnapshot().control;
  action_running.fsm_id = 816;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);
  const auto release =
      control.Submit("r1-arm-release-0001", "arm_action", "execute", 99, true);
  Require(release.accepted,
          "R1 recovery action must interrupt a running preset action");
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto snapshot = store.GetSnapshot();
    if (snapshot.control.last_command.request_id == release.request_id &&
        snapshot.control.last_command.state == "succeeded") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(store.GetSnapshot().control.last_command.request_id ==
                  release.request_id &&
              store.GetSnapshot().control.last_command.state == "succeeded",
          "R1 Mock arm preset must complete through the R1 Arm Action path");

  action_running = store.GetSnapshot().control;
  action_running.fsm_id = 811;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);

  const auto custom = control.Submit(
      "r1-arm-custom-0001", "arm_action", "execute_custom", 0, true,
      "r1_demo");
  Require(custom.accepted,
          "R1 Mock firmware taught action should use the shared custom-action path");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (store.GetSnapshot().control.last_command.state == "running") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  action_running = store.GetSnapshot().control;
  action_running.fsm_id = 816;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);
  const auto stop_custom = control.Submit(
      "r1-arm-stop-custom-0001", "arm_action", "stop_custom", 0, true);
  Require(stop_custom.accepted,
          "R1 stop-teach action must interrupt running firmware playback");
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto snapshot = store.GetSnapshot();
    if (snapshot.control.last_command.request_id == stop_custom.request_id &&
        snapshot.control.last_command.state == "succeeded") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(store.GetSnapshot().control.last_command.request_id ==
                  stop_custom.request_id &&
              store.GetSnapshot().control.last_command.state == "succeeded",
          "an interrupted R1 action must not overwrite the newer stop result");

  action_running = store.GetSnapshot().control;
  action_running.fsm_id = 811;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);

  const auto head = control.ApplyJointDebug(
      "upper_body", {{13, 0.0F}, {24, 0.0F}, {25, 0.0F}}, true);
  Require(head.accepted,
          "R1 waist_yaw and head joints should use the shared upper-body rt/arm_sdk mode");
  action_running = store.GetSnapshot().control;
  action_running.fsm_id = 816;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto arm_fsm_status = Parse(control.SerializeJointDebugStatus());
  Require(arm_fsm_status["dds_state"].asString() == "active" &&
              arm_fsm_status["upper_body_fsm_allowed"].asBool() &&
              arm_fsm_status["upper_body_allowed"].asBool() &&
              control.ApplyJointDebug(
                         "upper_body",
                         {{13, 0.0F}, {24, 0.0F}, {25, 0.0F}}, true)
                  .accepted,
          "R1 arm_sdk control must remain active after its FSM changes from 811 to 816");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (control.GetJointDebugTestStats().publish_count > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto arm_stats = control.GetJointDebugTestStats();
  Require(!arm_stats.lowcmd_published && arm_stats.last_kp.size() == 31 &&
              arm_stats.last_kp[13] == 50.0F &&
              arm_stats.last_kd[13] == 3.0F &&
              arm_stats.last_kp[15] == 50.0F &&
              arm_stats.last_kd[15] == 2.0F &&
              arm_stats.last_kp[19] == 30.0F &&
              arm_stats.last_kd[19] == 2.0F &&
              arm_stats.last_kp[29] == 15.0F &&
              arm_stats.last_kd[29] == 1.0F &&
              arm_stats.last_kp[30] == 15.0F &&
              arm_stats.last_kd[30] == 1.0F &&
              arm_stats.last_kp[12] == 0.0F &&
              arm_stats.last_kp[14] == 0.0F &&
              arm_stats.last_kp[20] == 0.0F &&
              arm_stats.last_kp[21] == 0.0F &&
              arm_stats.last_kp[27] == 0.0F &&
              arm_stats.last_kp[28] == 0.0F,
          "R1 upper-body control must hold every official arm_sdk joint while moving selected head targets");
  Require(control.StopJointDebug(true).accepted,
          "R1 arm_sdk head control should release cleanly");
  action_running = store.GetSnapshot().control;
  action_running.fsm_id = 811;
  action_running.sport_state_last_update = g1_web::SteadyClock::now();
  store.SetControlState(action_running);

  std::vector<std::pair<std::size_t, float>> full_body_targets;
  for (Json::ArrayIndex index = 0; index < initial["joints"].size(); ++index) {
    full_body_targets.emplace_back(
        static_cast<std::size_t>(index),
        initial["joints"][index]["current"].asFloat());
  }
  const auto full_body =
      control.ApplyJointDebug("full_body", full_body_targets, true);
  Require(full_body.accepted,
          "R1 Mock full-body debugger should retain the G1-style guarded lowcmd path");
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (control.GetJointDebugTestStats().lowcmd_published) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(control.GetJointDebugTestStats().lowcmd_published,
          "R1 full-body debug must publish the guarded rt/lowcmd path");
  const auto motion_conflict =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(!motion_conflict.accepted && motion_conflict.error == "control_busy",
          "active R1 full-body debug must exclude locomotion through ResourceManager");
  Require(control.StopJointDebug(true).accepted,
          "R1 lowcmd full-body control should release cleanly");

  const auto record = control.StartJointTeachRecording("r1_teach", true);
  Require(record.accepted,
          "R1 local Joint Teach should enter passive arm_sdk recording");
  for (int attempt = 0; attempt < 20; ++attempt) {
    store.PopulateMock(0.01 * attempt, 1, semantic_slots);
    const auto teaching = Parse(control.SerializeJointDebugStatus());
    if (teaching["teach_recorded_frames"].asUInt() >= 3) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  const auto teaching_stats = control.GetJointDebugTestStats();
  Require(teaching_stats.last_kp[12] == 0.0F &&
              teaching_stats.last_kp[13] == 0.0F &&
              teaching_stats.last_kd[13] == 1.0F &&
              teaching_stats.last_kp[19] == 0.0F &&
              teaching_stats.last_kd[19] == 0.5F &&
              teaching_stats.last_kp[26] == 0.0F &&
              teaching_stats.last_kd[26] == 0.5F &&
              teaching_stats.last_kp[29] == 0.0F &&
              teaching_stats.last_kd[29] == 1.0F &&
              teaching_stats.last_kp[30] == 0.0F &&
              teaching_stats.last_kd[30] == 0.1F,
          "R1 hand-guided teaching must release waist yaw, both wrist rolls and both head joints with product-specific damping");
  const auto saved = control.FinishJointTeachRecording(true, true);
  Require(saved.accepted,
          "R1 local Joint Teach should save and release through the shared G1-style workflow");
  Json::Value saved_store;
  Json::CharReaderBuilder saved_store_builder;
  std::string saved_store_errors;
  std::ifstream saved_store_input(teach_store);
  Require(Json::parseFromStream(saved_store_builder, saved_store_input,
                                &saved_store, &saved_store_errors) &&
              saved_store["actions"][0]["frames"][0].size() == 13,
          "new R1 taught actions must persist waist yaw, both arms and both head joints");
  const auto taught = Parse(control.SerializeJointDebugStatus());
  Require(taught["teach_actions"].size() == 1 &&
              taught["teach_actions"][0]["name"].asString() == "r1_teach" &&
              control.SetJointTeachRemoteBinding("r1_teach", "F3+A").accepted,
          "R1 local teach should persist and accept a G1-style remote binding");
  const auto bound = Parse(control.SerializeJointDebugStatus());
  Require(bound["teach_actions"][0]["remote_binding"].asString() == "F3+A",
          "R1 remote teaching binding must be persisted in status");
  Require(control.DeleteJointTeachAction("r1_teach", true).accepted,
          "R1 local teach action should be deletable");
  store.PopulateMock(0.0, 1, semantic_slots);

  const auto upper = control.ApplyJointDebug(
      "upper_body",
      {{14, initial["joints"][14]["current"].asFloat()},
       {19, initial["joints"][19]["current"].asFloat()}},
      true);
  Require(upper.accepted,
          std::string("R1 upper-body arm_sdk should accept the declared left/right arm joints: ") +
              upper.error);
  Json::Value leased_out;
  for (int attempt = 0; attempt < 120; ++attempt) {
    store.PopulateMock(0.0, 1, semantic_slots);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    leased_out = Parse(control.SerializeJointDebugStatus());
    if (leased_out["dds_state"].asString() == "idle" &&
        leased_out["last_error"].asString() == "control_lease_expired") {
      break;
    }
  }
  Require(leased_out["dds_state"].asString() == "idle" &&
              leased_out["last_error"].asString() ==
                  "control_lease_expired",
          "R1 arm_sdk debug must release after the browser control lease expires");
  control.Stop();
  std::error_code remove_error;
  std::filesystem::remove(teach_store, remove_error);

}

void TestMockJointDebugger() {
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  const auto teach_store = std::filesystem::temp_directory_path() /
      ("g1_web_joint_teach_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) +
       ".json");
  g1_web::SnapshotStore store;
  store.PopulateMock(0.0);
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  g1_web::ControlService control(
      store, g1_web::CreateG1LocomotionAdapter(true), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true, web_root.string(),
      teach_store.string());
  std::string error;
  Require(control.Start(error), "mock joint debugger should start");

  const auto initial_status = Parse(control.SerializeJointDebugStatus());
  Require(initial_status["joints"].size() == 29,
          "joint debugger should expose all 29 mapped joints");
  Require(initial_status["joints"][0]["lower"].asFloat() < 0.0F &&
              initial_status["joints"][0]["upper"].asFloat() > 0.0F,
          "joint debugger should load limits from the selected URDF");

  unitree_hg::msg::dds_::LowState_ ab_state;
  ab_state.mode_pr() = 1;
  ab_state.mode_machine() = 2;
  store.UpdateLowState(ab_state);
  const auto ab_status = Parse(control.SerializeJointDebugStatus());
  Require(ab_status["upper_body_allowed"].asBool(),
          "arm_sdk mode must not require PR debug mode");

  const auto leg_in_arm_mode = control.ApplyJointDebug(
      "upper_body", {{0, 0.0F}}, true);
  Require(!leg_in_arm_mode.accepted &&
              leg_in_arm_mode.error == "upper_body_joint_not_allowed",
          "upper-body mode must reject leg joints");

  std::vector<std::pair<std::size_t, float>> upper_targets;
  for (std::size_t index = 12; index < 29; ++index) {
    upper_targets.emplace_back(index, 0.0F);
  }
  unitree_hg::msg::dds_::SportModeState_ upper_state;
  upper_state.fsm_mode(0);
  upper_state.task_id(0);
  upper_state.task_time(0.0F);
  upper_state.fsm_id(0);
  store.UpdateSportMode(upper_state);
  const auto zero_torque_upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(!zero_torque_upper.accepted &&
              zero_torque_upper.error == "upper_body_fsm_not_allowed" &&
              control.GetJointDebugTestStats().publish_count == 0,
          "zero-torque FSM must not publish rt/arm_sdk");
  upper_state.fsm_id(1);
  store.UpdateSportMode(upper_state);
  const auto damping_upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(!damping_upper.accepted &&
              damping_upper.error == "upper_body_fsm_not_allowed" &&
              control.GetJointDebugTestStats().publish_count == 0,
          "damping FSM must not publish rt/arm_sdk");

  upper_state.fsm_id(500);
  store.UpdateSportMode(upper_state);
  control.SetMockJointDebugAiSport(true);
  const auto locomotion_lock =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(locomotion_lock.accepted,
          "mock locomotion should acquire the locomotion resource");
  const auto blocked_upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(!blocked_upper.accepted && blocked_upper.error == "control_busy",
          "active locomotion should keep the existing Joint Debug conflict");
  Require(control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false).accepted,
          "mock locomotion stop should be accepted");
  for (int attempt = 0; attempt < 300; ++attempt) {
    if (store.GetSnapshot().control.motion.state == "stopped") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Require(store.GetSnapshot().control.motion.state == "stopped",
          "locomotion resource should release after mock stop");
  const auto upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(upper.accepted,
          "upper-body arm_sdk must not depend on service state");
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  const auto active_status = Parse(control.SerializeJointDebugStatus());
  Require(active_status["dds_state"].asString() == "active",
          "upper-body Apply should directly start the arm_sdk loop");
  const auto first_stats = control.GetJointDebugTestStats();
  Require(first_stats.publish_count > 0 &&
              first_stats.maximum_step_rad <= 0.0101F,
          "Apply should publish bounded interpolation steps, not jump");
  Require(first_stats.last_kp[12] == 300.0F &&
              first_stats.last_kd[12] == 3.0F &&
              first_stats.last_kp[15] == 80.0F &&
              first_stats.last_kd[15] == 3.0F &&
              first_stats.last_kp[19] == 40.0F &&
              first_stats.last_kd[19] == 1.5F,
          "upper-body mode should use high, low and wrist gains by motor id");
  const auto blocked_motion =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(!blocked_motion.accepted && blocked_motion.error == "control_busy",
          "active Joint Debug should keep the existing locomotion conflict");

  upper_targets[0].second = 0.1F;
  Require(control.ApplyJointDebug("upper_body", upper_targets, true).accepted,
          "repeat Apply should update the existing loop");
  Require(control.GetJointDebugTestStats().loop_starts ==
              first_stats.loop_starts,
          "repeat Apply must not create another control loop");
  Require(control.StopJointDebug(true).accepted,
          "upper-body debug control should stop");

  upper_state.fsm_id(802);
  store.UpdateSportMode(upper_state);
  Require(control.ApplyJointDebug("upper_body", upper_targets, true).accepted,
          "walk-run FSM should allow upper-body arm_sdk control");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  upper_state.fsm_id(1);
  store.UpdateSportMode(upper_state);
  Json::Value stopped_status;
  for (int attempt = 0; attempt < 100; ++attempt) {
    stopped_status = Parse(control.SerializeJointDebugStatus());
    if (stopped_status["dds_state"].asString() == "idle") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(stopped_status["dds_state"].asString() == "idle" &&
              stopped_status["arm_sdk_weight"].asFloat() == 0.0F,
          "leaving an allowed FSM must stop the arm_sdk loop");
  const auto stopped_publish_count =
      control.GetJointDebugTestStats().publish_count;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  Require(control.GetJointDebugTestStats().publish_count ==
              stopped_publish_count,
          "arm_sdk must not publish after entering damping FSM");
  store.PopulateMock(0.0);

  std::vector<std::pair<std::size_t, float>> full_targets;
  for (std::size_t index = 0; index < 29; ++index) {
    full_targets.emplace_back(index, 0.0F);
  }
  control.SetMockJointDebugAiSport(true);
  const auto blocked_full =
      control.ApplyJointDebug("full_body", full_targets, true);
  Require(!blocked_full.accepted &&
              blocked_full.error == "debug_mode_required" &&
              !control.GetJointDebugTestStats().lowcmd_published,
          "ai_sport must absolutely block rt/lowcmd publication");

  auto invalid_targets = full_targets;
  invalid_targets[0].second = 99.0F;
  control.SetMockJointDebugAiSport(false);
  const auto out_of_range =
      control.ApplyJointDebug("full_body", invalid_targets, true);
  Require(!out_of_range.accepted &&
              out_of_range.error == "joint_out_of_range",
          "backend must reject targets outside URDF limits");

  Require(control.ApplyJointDebug("full_body", full_targets, true).accepted,
          "full-body mock control should start only after ai_sport stops");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto full_stats = control.GetJointDebugTestStats();
  Require(full_stats.lowcmd_published,
          "allowed full-body mode should exercise the mock lowcmd writer");
  Require(full_stats.last_kp[0] == 300.0F &&
              full_stats.last_kd[0] == 3.0F &&
              full_stats.last_kp[4] == 80.0F &&
              full_stats.last_kd[4] == 3.0F &&
              full_stats.last_kp[5] == 300.0F &&
              full_stats.last_kp[12] == 300.0F &&
              full_stats.last_kp[15] == 80.0F &&
              full_stats.last_kp[19] == 40.0F &&
              full_stats.last_kd[19] == 1.5F,
          "full-body mode should use official gains for legs, waist and arms");
  Require(control.StopJointDebug(true).accepted,
          "full-body mock control should stop deterministically");

  store.PopulateMock(0.0);
  upper_state.fsm_id(500);
  store.UpdateSportMode(upper_state);
  const auto before_record = control.GetJointDebugTestStats().publish_count;
  Require(control.StartJointTeachRecording("mock_wave", true).accepted,
          "mock upper-body teach recording should start");
  const auto teach_blocks_motion =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(!teach_blocks_motion.accepted &&
              teach_blocks_motion.error == "control_busy",
          "Joint Teach should block locomotion through ResourceManager");
  Json::Value recording_status = Parse(control.SerializeJointDebugStatus());
  Require(recording_status["teach_state"].asString() == "recording" &&
              recording_status["dds_state"].asString() == "released" &&
              recording_status["arm_sdk_weight"].asFloat() == 1.0F,
          "teach recording must take arm_sdk ownership immediately");
  for (int attempt = 0; attempt < 10; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    if (control.GetJointDebugTestStats().publish_count > before_record) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto after_release = control.GetJointDebugTestStats();
  Require(after_release.publish_count > before_record &&
              after_release.last_kp[12] == 0.0F &&
              after_release.last_kd[12] == 10.0F &&
              after_release.last_kp[13] == 300.0F &&
              after_release.last_kd[13] == 3.0F &&
              after_release.last_kp[14] == 300.0F &&
              after_release.last_kd[14] == 3.0F &&
              after_release.last_kp[15] == 0.0F &&
              after_release.last_kd[15] == 1.5F &&
              after_release.last_kp[19] == 0.0F &&
              after_release.last_kd[19] == 0.5F &&
              after_release.last_kp[28] == 0.0F &&
              after_release.last_kd[28] == 0.5F,
          "teach recording must free waist yaw, lock waist roll/pitch and damp arms/wrists");
  const auto frames_before_over_limit =
      recording_status["teach_recorded_frames"].asUInt();
  unitree_hg::msg::dds_::LowState_ over_limit_state;
  over_limit_state.mode_machine() = 2;
  for (auto& motor : over_limit_state.motor_state()) motor.q(0.0F);
  over_limit_state.motor_state().at(14).q(0.7F);
  over_limit_state.motor_state().at(20).q(2.0F);
  store.UpdateLowState(over_limit_state);
  store.UpdateSportMode(upper_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  recording_status = Parse(control.SerializeJointDebugStatus());
  const auto held_stats = control.GetJointDebugTestStats();
  Require(recording_status["teach_recorded_frames"].asUInt() >
              frames_before_over_limit,
          "teach recording should sample a measured URDF limit overshoot");
  Require(std::abs(held_stats.last_q[14] - 0.028F) < 1e-4F &&
              std::abs(held_stats.last_q[15]) < 1e-4F,
          "teach waist must hold its start pose while arm targets follow LowState");
  const auto frames_after_release =
      recording_status["teach_recorded_frames"].asUInt();
  for (int attempt = 0; attempt < 110; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  recording_status = Parse(control.SerializeJointDebugStatus());
  Require(recording_status["teach_state"].asString() == "recording" &&
              recording_status["last_error"].asString() !=
                  "control_lease_expired" &&
              recording_status["teach_recorded_frames"].asUInt() >
                  frames_after_release &&
              control.GetJointDebugTestStats().publish_count >
                  after_release.publish_count,
          "passive teach must keep DDS ownership without a browser heartbeat");
  Require(control.FinishJointTeachRecording(true, true).accepted,
          "teach recording should clamp measured limit overshoot, save and release");
  const auto saved_status = Parse(control.SerializeJointDebugStatus());
  Require(saved_status["teach_actions"].size() == 1 &&
              saved_status["teach_actions"][0]["name"].asString() ==
                  "mock_wave" &&
              saved_status["teach_actions"][0]["frames"].asUInt() >= 2 &&
              saved_status["dds_state"].asString() == "idle" &&
              saved_status["arm_sdk_weight"].asFloat() == 0.0F,
          "save must persist the recording and return arm_sdk ownership");
  store.PopulateMock(0.0);
  store.UpdateSportMode(upper_state);
  const auto motion_after_teach =
      control.SubmitVelocity(0.1F, 0.0F, 0.0F, 0, true);
  Require(motion_after_teach.accepted,
          "releasing Joint Teach should free resources for locomotion");
  Require(control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false).accepted,
          "mock motion after Joint Teach should stop");
  for (int attempt = 0; attempt < 300; ++attempt) {
    if (store.GetSnapshot().control.motion.state == "stopped") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Require(store.GetSnapshot().control.motion.state == "stopped",
          "post-teach locomotion resource should release after stop");
  const auto permissions = std::filesystem::status(teach_store).permissions();
  Require((permissions & (std::filesystem::perms::group_all |
                          std::filesystem::perms::others_all)) ==
              std::filesystem::perms::none,
          "teach action store must not grant group or other permissions");
  const auto reserved_binding =
      control.SetJointTeachRemoteBinding("mock_wave", "L2+B");
  Require(!reserved_binding.accepted &&
              reserved_binding.error ==
                  "joint_teach_remote_binding_not_allowed",
          "official G1 remote combinations must be rejected for custom actions");
  const auto mislabeled_f2 =
      control.SetJointTeachRemoteBinding("mock_wave", "F2+A");
  Require(!mislabeled_f2.accepted &&
              mislabeled_f2.error ==
                  "joint_teach_remote_binding_not_allowed",
          "the physical G1 remote exposes F1/F3, not F2");
  Require(control.SetJointTeachRemoteBinding("mock_wave", "F3+A").accepted,
          "an unused F1/F3 remote combination should bind to a teach action");
  const auto bound_status = Parse(control.SerializeJointDebugStatus());
  Require(bound_status["remote_control_ready"].asBool() &&
              bound_status["remote_binding_options"].size() == 16 &&
              bound_status["remote_binding_options"][4]["label"].asString() ==
                  "F1+UP(↑)" &&
              bound_status["remote_binding_options"][5]["label"].asString() ==
                  "F1+RIGHT(→)" &&
              bound_status["remote_binding_options"][6]["label"].asString() ==
                  "F1+DOWN(↓)" &&
              bound_status["remote_binding_options"][7]["label"].asString() ==
                  "F1+LEFT(←)" &&
              bound_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "status should expose safe remote bindings with arrow labels and the saved assignment");

  store.PopulateMock(0.0);
  store.UpdateSportMode(upper_state);
  Require(control.StartJointTeachRecording("mock_wave", true).accepted,
          "second teach recording should start for hold-after-save mode");
  for (int attempt = 0; attempt < 150; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    if (Parse(control.SerializeJointDebugStatus())["teach_state"].asString() ==
        "recording")
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  for (int attempt = 0; attempt < 6; ++attempt) {
    store.PopulateMock(static_cast<double>(attempt) * 0.02);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(control.FinishJointTeachRecording(true, false).accepted,
          "teach recording should save without returning arm_sdk ownership");
  for (int attempt = 0; attempt < 110; ++attempt) {
    store.PopulateMock(0.12);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto held_status = Parse(control.SerializeJointDebugStatus());
  const auto holding_stats = control.GetJointDebugTestStats();
  Require(held_status["teach_state"].asString() == "holding" &&
              held_status["dds_state"].asString() == "active" &&
              held_status["arm_sdk_weight"].asFloat() == 1.0F &&
              held_status["teach_record_name"].asString().empty() &&
              held_status["teach_recorded_frames"].asUInt() == 0 &&
              held_status["last_error"].asString() !=
                  "control_lease_expired" &&
              held_status["teach_actions"].size() == 1 &&
              held_status["teach_actions"][0]["hold_after_playback"].asBool() &&
              held_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "hold-after-save must survive browser heartbeat loss while keeping arm_sdk ownership and preserve its remote binding");
  Require(holding_stats.last_kp[12] == 300.0F &&
              holding_stats.last_kd[12] == 3.0F &&
              holding_stats.last_kp[13] == 300.0F &&
              holding_stats.last_kd[13] == 3.0F &&
              holding_stats.last_kp[15] == 80.0F &&
              holding_stats.last_kd[15] == 3.0F &&
              holding_stats.last_kp[19] == 40.0F &&
              holding_stats.last_kd[19] == 1.5F,
          "held teach pose must use the normal upper-body gains");
  control.SetMockJointDebugRemoteKeys(0);
  control.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto released_hold_status = Parse(control.SerializeJointDebugStatus());
  Require(released_hold_status["dds_state"].asString() == "idle" &&
              released_hold_status["arm_sdk_weight"].asFloat() == 0.0F &&
              released_hold_status["remote_last_binding"].asString() ==
                  "F3+A",
          "pressing the held action binding again must release arm_sdk control");
  control.Stop();

  g1_web::ControlService restored(
      store, g1_web::CreateG1LocomotionAdapter(true), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true, web_root.string(),
      teach_store.string());
  Require(restored.Start(error), "saved teach action should reload");
  const auto restored_status = Parse(restored.SerializeJointDebugStatus());
  Require(restored_status["teach_actions"].size() == 1 &&
              restored_status["teach_actions"][0]["hold_after_playback"].asBool() &&
              restored_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "hold-after-playback and remote-binding metadata should persist across service restart");
  store.PopulateMock(0.0);
  store.UpdateSportMode(upper_state);
  restored.SetMockJointDebugRemoteKeys(0);
  restored.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto playback_start_status =
      Parse(restored.SerializeJointDebugStatus());
  Require(playback_start_status["teach_state"].asString() == "playing" &&
              playback_start_status["arm_sdk_weight"].asFloat() == 1.0F,
          "teach playback must take arm_sdk ownership immediately");
  Json::Value playback_hold_status;
  for (int attempt = 0; attempt < 250; ++attempt) {
    store.PopulateMock(0.12);
    store.UpdateSportMode(upper_state);
    const auto current = Parse(restored.SerializeJointDebugStatus());
    if (current["dds_state"].asString() == "active")
      Require(restored.HeartbeatJointDebug().accepted,
              "playback and hold should accept the browser heartbeat");
    playback_hold_status = current;
    if (current["teach_state"].asString() == "holding") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(playback_hold_status["teach_state"].asString() == "holding" &&
              playback_hold_status["dds_state"].asString() == "active" &&
              playback_hold_status["arm_sdk_weight"].asFloat() == 1.0F,
          "a hold-recorded action must keep its last pose after playback");
  restored.SetMockJointDebugRemoteKeys(0);
  restored.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto restored_released = Parse(restored.SerializeJointDebugStatus());
  Require(restored_released["dds_state"].asString() == "idle" &&
              restored_released["arm_sdk_weight"].asFloat() == 0.0F &&
              restored_released["remote_last_binding"].asString() == "F3+A",
          "pressing the held action binding again must restore control");
  Require(restored.DeleteJointTeachAction("mock_wave", true).accepted,
          "saved teach action should be deletable");
  Require(Parse(restored.SerializeJointDebugStatus())["teach_actions"].empty(),
          "deleted teach action should disappear from status");
  restored.Stop();
  std::error_code ignored;
  std::filesystem::remove(teach_store, ignored);
}

void TestMockJointTeachIdentityCompatibility() {
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  const auto suffix = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const auto identity_store = std::filesystem::temp_directory_path() /
      ("g1_web_joint_identity_" + suffix + ".json");
  const auto legacy_store = std::filesystem::temp_directory_path() /
      ("g1_web_joint_legacy_" + suffix + ".json");
  const auto invalid_legacy_store = std::filesystem::temp_directory_path() /
      ("g1_web_joint_legacy_invalid_" + suffix + ".json");
  const auto* profile = g1_web::RobotRegistry::Find("g1");
  Require(profile != nullptr, "G1 profile must exist for teach identity tests");
  auto count_policy = g1_web::CreateG1JointDebugPolicy(*profile);
  const auto teach_groups = count_policy->TeachGroups();
  const auto teach_joint_count = static_cast<std::size_t>(std::count_if(
      profile->joint_schema.joints.begin(), profile->joint_schema.joints.end(),
      [&](const auto& joint) {
        return std::find(teach_groups.begin(), teach_groups.end(),
                         joint.display_group) != teach_groups.end();
      }));
  const auto make_frame = [&] {
    Json::Value frame(Json::arrayValue);
    for (std::size_t index = 0; index < teach_joint_count; ++index)
      frame.append(0.0F);
    return frame;
  };
  const auto make_action = [&](const std::string& name,
                               const std::string& product_id,
                               const std::string& variant,
                               std::uint32_t mode_machine,
                               const std::string& binding = "") {
    Json::Value action(Json::objectValue);
    action["name"] = name;
    if (!product_id.empty()) action["product_id"] = product_id;
    if (!variant.empty()) action["variant"] = variant;
    action["mode_machine"] = mode_machine;
    action["hold_after_playback"] = false;
    action["remote_binding"] = binding;
    action["frames"] = Json::Value(Json::arrayValue);
    action["frames"].append(make_frame());
    action["frames"].append(make_frame());
    return action;
  };
  const auto write_store = [](const std::filesystem::path& path,
                              const Json::Value& root) {
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    std::ofstream output(path, std::ios::trunc);
    output << Json::writeString(writer, root) << '\n';
    Require(static_cast<bool>(output), "teach identity test store must be writable");
  };

  Json::Value identity_root(Json::objectValue);
  identity_root["schema_version"] = 2;
  identity_root["sample_period_ms"] = 50;
  identity_root["actions"] = Json::Value(Json::arrayValue);
  identity_root["actions"].append(
      make_action("cross_product", "other_product", profile->identity.variant, 2));
  identity_root["actions"].append(
      make_action("cross_variant", profile->identity.product_id,
                  "other_variant", 2));
  write_store(identity_store, identity_root);

  g1_web::SnapshotStore identity_snapshot;
  identity_snapshot.PopulateMock(0.0);
  g1_web::ControlService identity_control(
      identity_snapshot, g1_web::CreateG1LocomotionAdapter(true), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true, web_root.string(),
      identity_store.string());
  std::string error;
  Require(identity_control.Start(error),
          "identity-mismatch Mock control should start");
  const auto identity_status = Parse(identity_control.SerializeJointDebugStatus());
  Require(identity_status["teach_actions"].size() == 2,
          "schema-v2 actions with foreign identity should load for diagnosis");
  const auto product_reject =
      identity_control.PlayJointTeachAction("cross_product", true);
  Require(!product_reject.accepted &&
              product_reject.error == "joint_teach_product_mismatch",
          "teach playback must reject a cross-product action before takeover");
  const auto variant_reject =
      identity_control.PlayJointTeachAction("cross_variant", true);
  Require(!variant_reject.accepted &&
              variant_reject.error == "joint_teach_variant_mismatch",
          "teach playback must reject a cross-variant action before takeover");
  identity_control.Stop();

  Json::Value legacy_root(Json::objectValue);
  legacy_root["schema_version"] = 1;
  legacy_root["sample_period_ms"] = 50;
  legacy_root["actions"] = Json::Value(Json::arrayValue);
  legacy_root["actions"].append(
      make_action("legacy_g1", "", "", 2, "F2+B"));
  write_store(legacy_store, legacy_root);

  g1_web::SnapshotStore legacy_snapshot;
  legacy_snapshot.PopulateMock(0.0);
  unitree_hg::msg::dds_::SportModeState_ upper_state;
  upper_state.fsm_id(500);
  upper_state.fsm_mode(0);
  upper_state.task_id(0);
  upper_state.task_time(0.0F);
  legacy_snapshot.UpdateSportMode(upper_state);
  g1_web::ControlService legacy_control(
      legacy_snapshot, g1_web::CreateG1LocomotionAdapter(true), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true, web_root.string(),
      legacy_store.string());
  Require(legacy_control.Start(error), "legacy G1 Mock control should start");
  const auto legacy_status = Parse(legacy_control.SerializeJointDebugStatus());
  Require(legacy_status["teach_actions"].size() == 1 &&
              legacy_status["teach_actions"][0]["legacy_compatible"].asBool() &&
              legacy_status["teach_actions"][0]["product_id"].asString() ==
                  profile->identity.product_id &&
              legacy_status["teach_actions"][0]["variant"].asString() ==
                  profile->identity.variant &&
              legacy_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+B",
          "confirmed schema-v1 G1 data must load with stable identity and legacy F2-to-F3 binding normalization");
  Require(legacy_control.PlayJointTeachAction("legacy_g1", true).accepted,
          "confirmed existing G1 schema-v1 action must remain Mock-playable");
  Require(legacy_control.StopJointDebug(true).accepted,
          "legacy compatibility Mock playback must release deterministically");
  Require(legacy_control.SetJointTeachRemoteBinding("legacy_g1", "F3+B").accepted,
          "saving legacy metadata should upgrade the compatible action store");
  legacy_control.Stop();

  Json::CharReaderBuilder reader;
  Json::Value upgraded_root;
  std::string parse_errors;
  std::ifstream upgraded_input(legacy_store);
  Require(Json::parseFromStream(reader, upgraded_input, &upgraded_root,
                                &parse_errors) &&
              upgraded_root["schema_version"].asInt() == 2 &&
              upgraded_root["actions"][0]["product_id"].asString() ==
                  profile->identity.product_id &&
              upgraded_root["actions"][0]["variant"].asString() ==
                  profile->identity.variant,
          "compatible legacy data must upgrade to schema-v2 product/variant identity on save");

  Json::Value invalid_legacy_root(Json::objectValue);
  invalid_legacy_root["schema_version"] = 1;
  invalid_legacy_root["sample_period_ms"] = 50;
  invalid_legacy_root["actions"] = Json::Value(Json::arrayValue);
  invalid_legacy_root["actions"].append(
      make_action("unknown_legacy", "", "", 17));
  write_store(invalid_legacy_store, invalid_legacy_root);
  g1_web::SnapshotStore invalid_snapshot;
  invalid_snapshot.PopulateMock(0.0);
  g1_web::ControlService invalid_legacy_control(
      invalid_snapshot, g1_web::CreateG1LocomotionAdapter(true), *profile,
      g1_web::CreateG1JointDebugPolicy(*profile), true, web_root.string(),
      invalid_legacy_store.string());
  Require(invalid_legacy_control.Start(error),
          "invalid legacy store must not prevent service startup");
  const auto invalid_status =
      Parse(invalid_legacy_control.SerializeJointDebugStatus());
  Require(invalid_status["teach_actions"].empty() &&
              invalid_status["teach_store_error"].asString() ==
                  "invalid_joint_teach_store",
          "legacy data with an unsupported G1 mode selector must fail closed instead of being guessed compatible");
  invalid_legacy_control.Stop();

  std::error_code ignored;
  std::filesystem::remove(identity_store, ignored);
  std::filesystem::remove(legacy_store, ignored);
  std::filesystem::remove(invalid_legacy_store, ignored);
}

}  // namespace

int main() {
  TestJointNames();
  TestG1ModelCatalog();
  TestRobotProfileDomainModel();
  TestR1RobotProfile();
  TestRobotRegistry();
  TestTask22RuntimeAssemblyBoundary();
  TestRobotRuntimeBundle();
  TestG1DeviceCapabilityPolicy();
  TestR1PerceptionPolicy();
  TestG1JointDebugPolicy();
  TestR1JointDebugPolicy();
  TestStaticAssets();
  TestFreshness();
  TestTask23SourceMappings();
  TestTask23RequiredTelemetryHealth();
  TestSerialization();
  TestRobotManifestSerialization();
  TestR1SerializationAndManifest();
  TestR1MockLocomotionAndAudio();
  TestPointCloudDecode();
  TestPointCloudWebFilter();
  TestLowObstacleSafetyDetector();
  TestMockNavigationBridge();
  TestMockPerceptionSafety();
  TestR1PerceptionMockPolicy();
  TestR1CameraPolicyAndMock();
  TestMockCameraCommands();
  TestConcurrentSnapshotAccess();
  TestMockVoiceService();
  TestUnavailableAudioCapabilityDegradesOnlyAudio();
  TestSafetyManager();
  TestResourceManager();
  TestMockControlSafety();
  TestR1MockJointDebugger();
  TestMockJointDebugger();
  TestMockJointTeachIdentityCompatibility();
  std::cout << "All UniRoboGui core tests passed.\n";
  return EXIT_SUCCESS;
}
