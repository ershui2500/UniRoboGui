#include "g1_web/robot_registry.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

#include "g1_web/g1_audio_capability.hpp"
#include "g1_web/g1_device_capability.hpp"
#include "g1_web/g1_joint_debug_policy.hpp"
#include "g1_web/g1_locomotion_adapter.hpp"
#include "g1_web/r1_audio_capability.hpp"
#include "g1_web/r1_device_capability.hpp"
#include "g1_web/r1_joint_debug_policy.hpp"
#include "g1_web/r1_locomotion_adapter.hpp"

namespace g1_web {

const char* TelemetrySourceKeyName(TelemetrySourceKey key) {
  switch (key) {
    case TelemetrySourceKey::kLowState:
      return "low_state";
    case TelemetrySourceKey::kBms:
      return "bms";
    case TelemetrySourceKey::kSecondaryImu:
      return "secondary_imu";
    case TelemetrySourceKey::kMainBoard:
      return "mainboard";
    case TelemetrySourceKey::kOdometry:
      return "odometry";
    case TelemetrySourceKey::kSportMode:
      return "sport_mode";
  }
  return "unknown";
}

namespace {

TelemetrySubscriptionPlan G1TelemetryPlan() {
  return {{
      {TelemetrySourceKey::kLowState, TelemetrySourceKind::kHgLowState,
       "rt/lf/lowstate", true},
      {TelemetrySourceKey::kBms, TelemetrySourceKind::kHgBmsState,
       "rt/lf/bmsstate", true},
      {TelemetrySourceKey::kSecondaryImu, TelemetrySourceKind::kHgImuState,
       "rt/lf/secondary_imu", true},
      {TelemetrySourceKey::kMainBoard,
       TelemetrySourceKind::kHgMainBoardState, "rt/lf/mainboardstate", true},
      {TelemetrySourceKey::kOdometry,
       TelemetrySourceKind::kGo2SportModeState, "rt/odommodestate", true},
      {TelemetrySourceKey::kSportMode,
       TelemetrySourceKind::kHgSportModeState, "rt/sportmodestate", false},
  }};
}

TelemetrySubscriptionPlan R1TelemetryPlan() {
  return {{
      {TelemetrySourceKey::kLowState, TelemetrySourceKind::kHgLowState,
       "rt/lf/lowstate", true},
      {TelemetrySourceKey::kBms, TelemetrySourceKind::kHgBmsState,
       "rt/lf/bmsstate", false},
      {TelemetrySourceKey::kMainBoard,
       TelemetrySourceKind::kHgMainBoardState, "rt/lf/mainboardstate", false},
      {TelemetrySourceKey::kSecondaryImu, TelemetrySourceKind::kHgImuState,
       "rt/lf/secondary_imu", false},
      {TelemetrySourceKey::kOdometry,
       TelemetrySourceKind::kGo2SportModeState, "rt/odommodestate", false},
  }};
}

const RobotProfile& G1Profile() {
  static const RobotProfile profile = [] {
    RobotProfile result;
    result.identity = {"unitree", "g1", "g1", "29dof", "Unitree G1",
                       "humanoid"};
    result.idl_family = "unitree_sdk2_g1";
    result.control_api_family = "unitree_sdk2_g1";
    result.model_asset_root = "/assets/unitree/g1_description";
    result.model_package = "g1_description";
    result.joint_schema.motor_slot_count = 35;
    result.joint_schema.joints = {
        {"left_hip_pitch", "左髋俯仰", "left_hip_pitch_joint", 0, "left_leg"},
        {"left_hip_roll", "左髋横滚", "left_hip_roll_joint", 1, "left_leg"},
        {"left_hip_yaw", "左髋偏航", "left_hip_yaw_joint", 2, "left_leg"},
        {"left_knee", "左膝", "left_knee_joint", 3, "left_leg"},
        {"left_ankle_pitch", "左踝俯仰", "left_ankle_pitch_joint", 4, "left_leg"},
        {"left_ankle_roll", "左踝横滚", "left_ankle_roll_joint", 5, "left_leg"},
        {"right_hip_pitch", "右髋俯仰", "right_hip_pitch_joint", 6, "right_leg"},
        {"right_hip_roll", "右髋横滚", "right_hip_roll_joint", 7, "right_leg"},
        {"right_hip_yaw", "右髋偏航", "right_hip_yaw_joint", 8, "right_leg"},
        {"right_knee", "右膝", "right_knee_joint", 9, "right_leg"},
        {"right_ankle_pitch", "右踝俯仰", "right_ankle_pitch_joint", 10, "right_leg"},
        {"right_ankle_roll", "右踝横滚", "right_ankle_roll_joint", 11, "right_leg"},
        {"waist_yaw", "腰部偏航", "waist_yaw_joint", 12, "waist"},
        {"waist_roll", "腰部横滚", "waist_roll_joint", 13, "waist"},
        {"waist_pitch", "腰部俯仰", "waist_pitch_joint", 14, "waist"},
        {"left_shoulder_pitch", "左肩俯仰", "left_shoulder_pitch_joint", 15, "left_arm"},
        {"left_shoulder_roll", "左肩横滚", "left_shoulder_roll_joint", 16, "left_arm"},
        {"left_shoulder_yaw", "左肩偏航", "left_shoulder_yaw_joint", 17, "left_arm"},
        {"left_elbow", "左肘", "left_elbow_joint", 18, "left_arm"},
        {"left_wrist_roll", "左腕横滚", "left_wrist_roll_joint", 19, "left_arm"},
        {"left_wrist_pitch", "左腕俯仰", "left_wrist_pitch_joint", 20, "left_arm"},
        {"left_wrist_yaw", "左腕偏航", "left_wrist_yaw_joint", 21, "left_arm"},
        {"right_shoulder_pitch", "右肩俯仰", "right_shoulder_pitch_joint", 22, "right_arm"},
        {"right_shoulder_roll", "右肩横滚", "right_shoulder_roll_joint", 23, "right_arm"},
        {"right_shoulder_yaw", "右肩偏航", "right_shoulder_yaw_joint", 24, "right_arm"},
        {"right_elbow", "右肘", "right_elbow_joint", 25, "right_arm"},
        {"right_wrist_roll", "右腕横滚", "right_wrist_roll_joint", 26, "right_arm"},
        {"right_wrist_pitch", "右腕俯仰", "right_wrist_pitch_joint", 27, "right_arm"},
        {"right_wrist_yaw", "右腕偏航", "right_wrist_yaw_joint", 28, "right_arm"},
    };
    result.model_variants = {
        {2, "G1 29DOF", "g1_29dof.urdf"},
        {3, "G1 29DOF (locked waist)", "g1_29dof_lock_waist.urdf"},
        {5, "G1 29DOF rev 1.0", "g1_29dof_rev_1_0.urdf"},
        {6, "G1 29DOF rev 1.0 (locked waist)", "g1_29dof_lock_waist_rev_1_0.urdf"},
        {11, "G1 29DOF mode 11", "g1_29dof_mode_11.urdf"},
        {12, "G1 29DOF mode 12", "g1_29dof_mode_12.urdf"},
        {13, "G1 29DOF mode 13", "g1_29dof_mode_13.urdf"},
        {14, "G1 29DOF mode 14", "g1_29dof_mode_14.urdf"},
        {15, "G1 29DOF mode 15", "g1_29dof_mode_15.urdf"},
        {16, "G1 29DOF mode 16", "g1_29dof_mode_16.urdf"},
        {18, "G1 29DOF mode 18", "g1_29dof_mode_18.urdf"},
    };
    result.static_constraints = {
        {"motor_slot_count", "35"},
        {"reserved_motor_slots", "29-34"},
        {"mode_machine_role", "urdf_selector_not_product_id"},
    };
    result.capabilities = {
        {CapabilityKey::kTelemetry, true, true,
         VerificationLevel::kReadonlyVerified,
         {{"joint_count", "29"}, {"websocket", "/ws/telemetry"}}, ""},
        {CapabilityKey::kLocomotion, true, true,
         VerificationLevel::kControlVerified,
         {{"speed_modes", "0,1,3"},
          {"allowed_fsm", "500,501,801,802"},
          {"web_mode_commands", "zero_torque,damp,stand_up,start,set_fsm_id:501,set_fsm_id:702,set_fsm_id:706,set_fsm_id:802"},
          {"mode_targets", "zero_torque:0,damp:1,stand_up:4,start:500"},
          {"forward_hard_limit_mps", "3.0"},
          {"lateral_hard_limit_mps", "1.0"},
          {"yaw_hard_limit_radps", "1.5"}},
         ""},
        {CapabilityKey::kJointDebug, true, true,
         VerificationLevel::kControlVerified,
         {{"joint_count", "29"},
          {"upper_body_slots", "12-28"},
          {"arm_action_ids", "11,12,15,17,18,19,20,21,22,23,24,25,26,27,99"},
          {"firmware_teach_actions", "true"}},
         ""},
        {CapabilityKey::kJointTeach, true, true,
         VerificationLevel::kControlVerified,
         {{"sample_hz", "20"}, {"max_record_seconds", "120"}}, ""},
        {CapabilityKey::kAudio, true, true,
         VerificationLevel::kMockVerified,
         {{"tts_backends", "kokoro,unitree"},
          {"play_stream", "true"},
          {"tts", "true"},
          {"volume", "true"},
          {"asr", "true"}},
         ""},
        {CapabilityKey::kCameraRgb, true, true,
         VerificationLevel::kMockVerified,
         {{"output", "480x360"}, {"jpeg_quality", "55"}}, ""},
        {CapabilityKey::kCameraDepth, true, true,
         VerificationLevel::kMockVerified,
         {{"output", "480x360"}, {"jpeg_quality", "45"}}, ""},
        {CapabilityKey::kLidar, true, true,
         VerificationLevel::kMockVerified, {}, ""},
        {CapabilityKey::kSlam, true, true,
         VerificationLevel::kMockVerified,
         {{"map_names", "test1.pcd..test10.pcd"}}, ""},
        {CapabilityKey::kHeadControl, false, false,
         VerificationLevel::kUnsupported, {},
         "not_supported_by_g1_baseline"},
    };
    return result;
  }();
  return profile;
}

const RobotProfile& R1Profile() {
  static const RobotProfile profile = [] {
    RobotProfile result;
    result.identity = {"unitree", "r1", "r1", "26dof", "Unitree R1",
                       "humanoid"};
    result.idl_family = "unitree_sdk2_hg";
    result.control_api_family = "unitree_sdk2_r1";
    result.model_asset_root = "/assets/unitree/r1_description";
    result.model_package = "r1_description";
    result.joint_schema.motor_slot_count = 31;
    result.joint_schema.joints = {
        {"left_hip_pitch", "左髋俯仰", "left_hip_pitch_joint", 0, "left_leg"},
        {"left_hip_roll", "左髋横滚", "left_hip_roll_joint", 1, "left_leg"},
        {"left_hip_yaw", "左髋偏航", "left_hip_yaw_joint", 2, "left_leg"},
        {"left_knee", "左膝", "left_knee_joint", 3, "left_leg"},
        {"left_ankle_pitch", "左踝俯仰", "left_ankle_pitch_joint", 4, "left_leg"},
        {"left_ankle_roll", "左踝横滚", "left_ankle_roll_joint", 5, "left_leg"},
        {"right_hip_pitch", "右髋俯仰", "right_hip_pitch_joint", 6, "right_leg"},
        {"right_hip_roll", "右髋横滚", "right_hip_roll_joint", 7, "right_leg"},
        {"right_hip_yaw", "右髋偏航", "right_hip_yaw_joint", 8, "right_leg"},
        {"right_knee", "右膝", "right_knee_joint", 9, "right_leg"},
        {"right_ankle_pitch", "右踝俯仰", "right_ankle_pitch_joint", 10, "right_leg"},
        {"right_ankle_roll", "右踝横滚", "right_ankle_roll_joint", 11, "right_leg"},
        {"waist_roll", "腰部横滚", "waist_roll_joint", 12, "waist"},
        {"waist_yaw", "腰部偏航", "waist_yaw_joint", 13, "waist"},
        {"left_shoulder_pitch", "左肩俯仰", "left_shoulder_pitch_joint", 15, "left_arm"},
        {"left_shoulder_roll", "左肩横滚", "left_shoulder_roll_joint", 16, "left_arm"},
        {"left_shoulder_yaw", "左肩偏航", "left_shoulder_yaw_joint", 17, "left_arm"},
        {"left_elbow", "左肘", "left_elbow_joint", 18, "left_arm"},
        {"left_wrist_roll", "左腕横滚", "left_wrist_roll_joint", 19, "left_arm"},
        {"right_shoulder_pitch", "右肩俯仰", "right_shoulder_pitch_joint", 22, "right_arm"},
        {"right_shoulder_roll", "右肩横滚", "right_shoulder_roll_joint", 23, "right_arm"},
        {"right_shoulder_yaw", "右肩偏航", "right_shoulder_yaw_joint", 24, "right_arm"},
        {"right_elbow", "右肘", "right_elbow_joint", 25, "right_arm"},
        {"right_wrist_roll", "右腕横滚", "right_wrist_roll_joint", 26, "right_arm"},
        {"head_pitch", "头部俯仰", "head_pitch_joint", 29, "head"},
        {"head_yaw", "头部偏航", "head_yaw_joint", 30, "head"},
    };
    result.model_variants = {
        {1, "R1 26DOF", "R1.urdf"},
    };
    result.static_constraints = {
        {"motor_slot_count", "31"},
        {"reserved_motor_slots", "14,20,21,27,28"},
        {"mode_machine_role", "urdf_selector_not_product_id"},
        {"lowstate_topic", "rt/lf/lowstate"},
        {"lowstate_type", "unitree_hg::msg::dds_::LowState_"},
        {"model_source", "unitree_ros@7d6075f7f58588b189b940130e3edab3c839b2df"},
        {"model_asset_kind", "official_urdf_and_meshes"},
    };
    result.capabilities = {
        {CapabilityKey::kTelemetry, true, true,
         VerificationLevel::kReadonlyVerified,
         {{"joint_count", "26"},
          {"lowstate_topic", "rt/lf/lowstate"},
          {"lowstate_type", "unitree_hg::msg::dds_::LowState_"},
          {"declared_sources", "low_state,bms,mainboard,secondary_imu,odometry"},
          {"required_sources", "low_state"},
          {"websocket", "/ws/telemetry"}},
         ""},
        {CapabilityKey::kLocomotion, true, true,
         VerificationLevel::kMockVerified,
         {{"speed_modes", "0,1,3"},
          {"speed_presets", "0:0.40:0.40:0.90,1:0.50:0.50:1.00,3:1.00:0.60:1.20"},
          {"allowed_fsm", "811"},
          {"state_refresh_policy", "client_poll"},
          {"real_control_policy", "operator_validation"},
          {"mode_commands", "damp,start,stand_up,lie_to_stand,stand_to_lie,zero_torque,stop_move"},
          {"web_mode_commands", "zero_torque,damp,stand_up,lie_to_stand,stand_to_lie,start"},
          {"mode_targets", "zero_torque:0,damp:1,stand_up:4,lie_to_stand:701,stand_to_lie:702,start:811"},
          {"mode_sources", "zero_torque:1;stand_up:1|811;start:4;lie_to_stand:4|702;stand_to_lie:811"},
          {"fsm_mode_unknown", "4294967295"},
          {"forward_hard_limit_mps", "1.0"},
          {"lateral_hard_limit_mps", "0.6"},
          {"yaw_hard_limit_radps", "1.2"},
          {"velocity_lease_seconds", "1.0"}},
         "r1_real_control_not_verified"},
        {CapabilityKey::kJointDebug, true, true,
         VerificationLevel::kMockVerified,
         {{"control_modes", "upper_body,full_body"},
          {"upper_body_groups", "waist,head,left_arm,right_arm"},
          {"upper_body_excluded_joints", "waist_roll"},
          {"upper_body_topic", "rt/arm_sdk"},
          {"full_body_topic", "rt/lowcmd"},
          {"arm_action_ids", "11,12,13,15,17,18,19,22,23,24,25,26,27,28,29,30,31,33,34,35,36,99"},
          {"firmware_teach_actions", "true"},
          {"arm_action_interrupts", "true"},
          {"arm_action_service", "arm"},
          {"arm_action_api_ids", "7106,7107,7108,7113"},
          {"arm_sdk_weight_field", "mode_pr_percent"},
          {"real_control_policy", "operator_validation"}},
         "r1_real_joint_control_requires_operator_validation"},
        {CapabilityKey::kJointTeach, true, true,
         VerificationLevel::kMockVerified,
         {{"sample_hz", "20"},
          {"max_record_seconds", "120"},
          {"control_topic", "rt/arm_sdk"},
          {"remote_binding", "true"},
          {"real_control_policy", "operator_validation"}},
         "r1_joint_teach_requires_operator_validation"},
        {CapabilityKey::kAudio, true, true,
         VerificationLevel::kControlVerified,
         {{"tts_backends", "kokoro,unitree"},
          {"play_stream", "true"},
          {"tts", "true"},
          {"volume", "true"},
          {"asr", "true"}},
         ""},
        {CapabilityKey::kCameraRgb, true, true,
         VerificationLevel::kMockVerified,
         {{"required_variant", "r1_edu_26dof"},
          {"stream_role", "rgb"},
          {"transport", "rtp_h264_udp"},
          {"fixed_source", "udp://0.0.0.0:5003"},
          {"port", "5003"},
          {"img_port", "5001"},
          {"left_eye_port", "5002"},
          {"right_eye_port", "5003"},
          {"output", "480x360"},
          {"receiver_start_allowed_when_unavailable", "true"}},
         "r1_camera_readonly_not_verified"},
        {CapabilityKey::kCameraDepth, true, true,
         VerificationLevel::kMockVerified,
         {{"required_variant", "r1_edu_26dof"},
          {"stream_role", "depth"},
          {"transport", "v4l2_raw16"},
          {"fixed_source", "/dev/video-dep"},
          {"width", "544"},
          {"height", "448"},
          {"fps", "10"},
          {"receiver_start_allowed_when_unavailable", "true"}},
         "r1_camera_readonly_not_verified"},
        {CapabilityKey::kLidar, false, false,
         VerificationLevel::kUnsupported, {}, "r1_raw_lidar_topic_unverified"},
        {CapabilityKey::kSlam, true, true,
         VerificationLevel::kMockVerified,
         {{"required_variant", "r1_edu_26dof"},
          {"required_devices", "mid360"},
          {"raw_input_required", "false"},
          {"navigation_max_distance_m", "10"},
          {"map_whitelist", "test1.pcd..test10.pcd"}},
         "r1_slam_runtime_conditions_unverified"},
        {CapabilityKey::kHeadControl, true, true,
         VerificationLevel::kMockVerified,
         {{"control_modes", "upper_body"},
          {"joint_group", "head"},
          {"motor_slots", "29,30"},
          {"control_topic", "rt/arm_sdk"},
          {"real_control_policy", "operator_validation"}},
         "r1_real_head_control_requires_operator_validation"},
    };
    return result;
  }();
  return profile;
}

bool CapabilityAvailable(const RobotProfile& profile,
                         CapabilityKey key) {
  const auto capability = std::find_if(
      profile.capabilities.begin(), profile.capabilities.end(),
      [key](const CapabilityDescriptor& descriptor) {
        return descriptor.key == key;
      });
  return capability != profile.capabilities.end() && capability->implemented &&
         capability->available;
}

bool RequiredDevicesEnabled(const CapabilityDescriptor& capability,
                            const RuntimeOptions& options) {
  const auto required = capability.parameters.find("required_devices");
  if (required == capability.parameters.end() || required->second.empty()) {
    return true;
  }
  std::size_t start = 0;
  while (start < required->second.size()) {
    const auto end = required->second.find(',', start);
    const auto device = required->second.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    if (!device.empty() && options.enabled_devices.count(device) == 0) {
      return false;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return true;
}

void ApplyRuntimeCapabilityPolicy(RobotProfile& profile,
                                  const RuntimeOptions& options) {
  for (auto& capability : profile.capabilities) {
    capability.parameters["product_supported"] =
        capability.implemented ? "true" : "false";
    capability.parameters["variant_supported"] = "true";

    capability.available = capability.implemented && capability.available;
    if (!capability.available) continue;

    if (!RequiredDevicesEnabled(capability, options)) {
      capability.available = false;
      capability.reason = "required_attachment_disabled";
      continue;
    }

    const auto policy = capability.parameters.find("real_control_policy");
    if (!options.mock && policy != capability.parameters.end() &&
        policy->second == "mock_only") {
      capability.available = false;
      if (capability.reason.empty()) {
        capability.reason = "mock_only_non_mock_runtime";
      }
      continue;
    }

    if (options.mock &&
        capability.verification_level == VerificationLevel::kControlVerified) {
      capability.verification_level = VerificationLevel::kMockVerified;
    }
  }
}

void ValidateProfile(const RobotProfile& profile) {
  if (profile.identity.product_id.empty() || profile.identity.variant.empty()) {
    throw std::runtime_error("runtime_profile_identity_missing");
  }
  static constexpr std::array<CapabilityKey, 10> kRequiredCapabilities{{
      CapabilityKey::kTelemetry, CapabilityKey::kLocomotion,
      CapabilityKey::kJointDebug, CapabilityKey::kJointTeach,
      CapabilityKey::kAudio, CapabilityKey::kCameraRgb,
      CapabilityKey::kCameraDepth, CapabilityKey::kLidar,
      CapabilityKey::kSlam, CapabilityKey::kHeadControl,
  }};
  for (const auto key : kRequiredCapabilities) {
    if (std::count_if(profile.capabilities.begin(), profile.capabilities.end(),
                      [key](const CapabilityDescriptor& capability) {
                        return capability.key == key;
                      }) != 1) {
      throw std::runtime_error("runtime_profile_capability_missing");
    }
  }
}

TelemetrySourceKind ExpectedSourceKind(TelemetrySourceKey key) {
  switch (key) {
    case TelemetrySourceKey::kLowState:
      return TelemetrySourceKind::kHgLowState;
    case TelemetrySourceKey::kBms:
      return TelemetrySourceKind::kHgBmsState;
    case TelemetrySourceKey::kSecondaryImu:
      return TelemetrySourceKind::kHgImuState;
    case TelemetrySourceKey::kMainBoard:
      return TelemetrySourceKind::kHgMainBoardState;
    case TelemetrySourceKey::kOdometry:
      return TelemetrySourceKind::kGo2SportModeState;
    case TelemetrySourceKey::kSportMode:
      return TelemetrySourceKind::kHgSportModeState;
  }
  throw std::runtime_error("runtime_telemetry_source_unknown");
}

void ApplyTelemetryCapabilityParameters(
    RobotProfile& profile, const TelemetrySubscriptionPlan& plan) {
  auto capability = std::find_if(
      profile.capabilities.begin(), profile.capabilities.end(),
      [](const CapabilityDescriptor& descriptor) {
        return descriptor.key == CapabilityKey::kTelemetry;
      });
  if (capability == profile.capabilities.end()) {
    throw std::runtime_error("runtime_telemetry_capability_missing");
  }

  std::string declared;
  std::string required;
  for (const auto& source : plan.sources) {
    if (source.key != TelemetrySourceKey::kSportMode) {
      if (!declared.empty()) declared += ",";
      declared += TelemetrySourceKeyName(source.key);
    }
    if (source.required_for_health) {
      if (!required.empty()) required += ",";
      required += TelemetrySourceKeyName(source.key);
    }
  }
  if (capability->parameters.count("declared_sources") == 0) {
    capability->parameters["declared_sources"] = declared;
  }
  capability->parameters["required_sources"] = required;
}

void ValidateTelemetryPlan(const TelemetrySubscriptionPlan& plan) {
  if (plan.sources.empty()) {
    throw std::runtime_error("runtime_telemetry_plan_missing");
  }
  std::set<TelemetrySourceKey> keys;
  for (const auto& source : plan.sources) {
    if (source.topic.empty() || !keys.insert(source.key).second ||
        source.kind != ExpectedSourceKind(source.key)) {
      throw std::runtime_error("runtime_telemetry_plan_invalid");
    }
  }
}

void ValidateBundle(const RobotRuntimeBundle& bundle) {
  if (!bundle.devices) throw std::runtime_error("runtime_device_policy_missing");
  ValidateTelemetryPlan(bundle.telemetry);
  const auto low_state = std::find_if(
      bundle.telemetry.sources.begin(), bundle.telemetry.sources.end(),
      [](const TelemetrySourceSpec& source) {
        return source.key == TelemetrySourceKey::kLowState;
      });
  if (CapabilityAvailable(bundle.profile, CapabilityKey::kTelemetry) &&
      low_state == bundle.telemetry.sources.end()) {
    throw std::runtime_error("runtime_telemetry_plan_missing");
  }
  if (CapabilityAvailable(bundle.profile, CapabilityKey::kLocomotion) &&
      !bundle.locomotion) {
    throw std::runtime_error("runtime_locomotion_missing");
  }
  if (CapabilityAvailable(bundle.profile, CapabilityKey::kAudio) &&
      !bundle.audio) {
    throw std::runtime_error("runtime_audio_missing");
  }
  if ((CapabilityAvailable(bundle.profile, CapabilityKey::kJointDebug) ||
       CapabilityAvailable(bundle.profile, CapabilityKey::kHeadControl)) &&
      !bundle.joint_debug) {
    throw std::runtime_error("runtime_joint_debug_policy_missing");
  }
}

}  // namespace

const RobotProfile* RobotRegistry::Find(const std::string& product_id) {
  if (product_id == "g1") return &G1Profile();
  if (product_id == "r1") return &R1Profile();
  return nullptr;
}

std::vector<const RobotProfile*> RobotRegistry::All() {
  return {&G1Profile(), &R1Profile()};
}

RobotRuntimeBundle RobotRegistry::CreateRuntime(
    const std::string& product_id, const RuntimeOptions& options) {
  RobotRuntimeBundle bundle;
  if (product_id == "g1") {
    bundle.profile = G1Profile();
    ValidateProfile(bundle.profile);
    bundle.telemetry = G1TelemetryPlan();
    ApplyTelemetryCapabilityParameters(bundle.profile, bundle.telemetry);
    ApplyRuntimeCapabilityPolicy(bundle.profile, options);
    bundle.locomotion = CreateG1LocomotionAdapter(options.mock);
    bundle.audio = CreateG1AudioCapability(options.mock);
    if (CapabilityAvailable(bundle.profile, CapabilityKey::kJointDebug)) {
      bundle.joint_debug = CreateG1JointDebugPolicy(G1Profile());
    }
    bundle.devices = CreateG1DeviceCapabilityPolicy();
  } else if (product_id == "r1") {
    bundle.profile = R1Profile();
    ValidateProfile(bundle.profile);
    bundle.telemetry = R1TelemetryPlan();
    ApplyTelemetryCapabilityParameters(bundle.profile, bundle.telemetry);
    ApplyRuntimeCapabilityPolicy(bundle.profile, options);
    bundle.locomotion = CreateR1LocomotionAdapter(options.mock);
    bundle.audio = CreateR1AudioCapability(options.mock);
    if (CapabilityAvailable(bundle.profile, CapabilityKey::kJointDebug) ||
        CapabilityAvailable(bundle.profile, CapabilityKey::kHeadControl)) {
      bundle.joint_debug = CreateR1JointDebugPolicy(R1Profile());
    }
    bundle.devices = CreateR1DeviceCapabilityPolicy(options.enabled_devices);
  } else {
    throw std::invalid_argument("unregistered_robot_product");
  }

  ValidateBundle(bundle);
  return bundle;
}

}  // namespace g1_web
