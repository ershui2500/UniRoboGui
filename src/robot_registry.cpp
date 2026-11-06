#include "g1_web/robot_registry.hpp"

#include "g1_web/a2_profile.hpp"
#include "g1_web/a2_audio_capability.hpp"
#include "g1_web/a2_joint_debug_policy.hpp"
#include "g1_web/a2_locomotion_adapter.hpp"
#include "g1_web/a2_device_capability.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

#include "g1_web/g1_audio_capability.hpp"
#include "g1_web/g1_device_capability.hpp"
#include "g1_web/g1_joint_debug_policy.hpp"
#include "g1_web/g1_locomotion_adapter.hpp"
#include "g1_web/go2_audio_capability.hpp"
#include "g1_web/go2_device_capability.hpp"
#include "g1_web/go2_joint_debug_policy.hpp"
#include "g1_web/go2_locomotion_adapter.hpp"
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
    case TelemetrySourceKey::kSecondaryBms:
      return "secondary_bms";
    case TelemetrySourceKey::kSecondaryImu:
      return "secondary_imu";
    case TelemetrySourceKey::kMainBoard:
      return "mainboard";
    case TelemetrySourceKey::kOdometry:
      return "odometry";
    case TelemetrySourceKey::kSportMode:
      return "sport_mode";
    case TelemetrySourceKey::kFaults:
      return "faults";
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

TelemetrySubscriptionPlan Go2TelemetryPlan() {
  return {{
      {TelemetrySourceKey::kLowState, TelemetrySourceKind::kGo2LowState,
       "rt/lowstate", true},
      {TelemetrySourceKey::kOdometry,
       TelemetrySourceKind::kGo2SportModeState, "rt/sportmodestate", true,
       true},
      {TelemetrySourceKey::kFaults, TelemetrySourceKind::kGo2ErrorList,
       "rt/errorlist", false},
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
         {{"joint_count", "29"},
          {"motor_fault_encoding", "hg_motorstate"},
          {"motor_temperature_channels", "2"},
          {"websocket", "/ws/telemetry"}},
         ""},
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
          {"motor_fault_encoding", "hg_motorstate"},
          {"motor_temperature_channels", "2"},
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

const RobotProfile& Go2Profile() {
  static const RobotProfile profile = [] {
    RobotProfile result;
    result.identity = {"unitree", "go2", "go2", "go2_edu", "Unitree GO2",
                       "quadruped"};
    result.idl_family = "unitree_sdk2_go2";
    result.control_api_family = "unitree_sdk2_go2_v2_sport";
    result.model_asset_root = "/assets/unitree/go2_description";
    result.model_package = "go2_description";
    result.joint_schema.motor_slot_count = 20;
    result.joint_schema.joints = {
        {"front_right_hip", "右前髋", "FR_hip_joint", 0, "front_right_leg"},
        {"front_right_thigh", "右前大腿", "FR_thigh_joint", 1,
         "front_right_leg"},
        {"front_right_calf", "右前小腿", "FR_calf_joint", 2,
         "front_right_leg"},
        {"front_left_hip", "左前髋", "FL_hip_joint", 3, "front_left_leg"},
        {"front_left_thigh", "左前大腿", "FL_thigh_joint", 4,
         "front_left_leg"},
        {"front_left_calf", "左前小腿", "FL_calf_joint", 5,
         "front_left_leg"},
        {"rear_right_hip", "右后髋", "RR_hip_joint", 6, "rear_right_leg"},
        {"rear_right_thigh", "右后大腿", "RR_thigh_joint", 7,
         "rear_right_leg"},
        {"rear_right_calf", "右后小腿", "RR_calf_joint", 8,
         "rear_right_leg"},
        {"rear_left_hip", "左后髋", "RL_hip_joint", 9, "rear_left_leg"},
        {"rear_left_thigh", "左后大腿", "RL_thigh_joint", 10,
         "rear_left_leg"},
        {"rear_left_calf", "左后小腿", "RL_calf_joint", 11,
         "rear_left_leg"},
    };
    result.model_variants = {
        {0, "GO2", "go2_description.urdf"},
    };
    result.static_constraints = {
        {"motor_slot_count", "20"},
        {"reserved_motor_slots", "12-19"},
        {"mode_machine_role", "fixed_profile_selector"},
        {"lowstate_topic", "rt/lowstate"},
        {"lowstate_type", "unitree_go::msg::dds_::LowState_"},
        {"sport_state_topic", "rt/sportmodestate"},
        {"sport_state_type", "unitree_go::msg::dds_::SportModeState_"},
        {"bms_source", "embedded_in_lowstate"},
        {"minimum_software_version", "1.1.6"},
        {"model_source",
         "unitree_ros@ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee"},
        {"model_asset_kind", "official_urdf_and_meshes"},
    };
    result.capabilities = {
        {CapabilityKey::kTelemetry, true, true,
         VerificationLevel::kMockVerified,
         {{"joint_count", "12"},
          {"lowstate_topic", "rt/lowstate"},
          {"lowstate_type", "unitree_go::msg::dds_::LowState_"},
          {"sport_state_topic", "rt/sportmodestate"},
          {"sport_state_type", "unitree_go::msg::dds_::SportModeState_"},
          {"declared_sources", "low_state,bms,odometry,faults"},
          {"motor_fault_encoding", "go2_reserve0"},
          {"fault_service_topic", "rt/errorlist"},
          {"fault_service_encoding", "unitree_errorlist_v1"},
          {"odometry_error_code_role", "motion_state_id"},
          {"motor_temperature_channels", "1"},
          {"battery_soh_supported", "false"},
          {"websocket", "/ws/telemetry"}},
         "go2_readonly_runtime_not_verified"},
        {CapabilityKey::kLocomotion, true, true,
         VerificationLevel::kMockVerified,
         {{"minimum_software_version", "1.1.6"},
          {"control_client", "go2_v2_sport_client"},
          {"speed_modes", "0,1,3"},
          {"speed_presets",
           "0:0.50:0.40:0.90,1:1.00:0.70:1.60,3:1.50:1.00:2.30"},
          {"allowed_fsm",
           "100,1013,1015,1016,1017,2007,2008,2009,2010,2011,2016,2017"},
          {"state_labels",
           "100:灵动|Free walk;1001:阻尼|Damping;"
           "1002:站立锁定|Locked stand;1004:蹲下|Crouch;"
           "1006:互动动作|Interaction action;1007:坐下|Sit;"
           "1008:前跳|Front jump;1009:前扑|Front pounce;"
           "1013:平衡站立|Balance stand;1015:常规行走|Static walk;"
           "1016:常规跑步|Trot run;1017:常规续航|Economic gait;"
           "1091:摆姿势|Pose;2006:蹲下|Crouch;"
           "2007:闪避|Free avoid;2008:并腿跑|Free bound;"
           "2009:跳跃跑|Free jump;2010:经典步态|Classic walk;"
           "2011:倒立|Hand stand;2012:前空翻|Front flip;"
           "2013:后空翻|Back flip;2014:左空翻|Left flip;"
           "2016:交叉步|Cross step;2017:后腿直立|Upright;"
           "2019:牵引|Traction"},
          {"real_control_policy", "operator_validation"},
          {"mode_commands",
           "damp,balance_stand,stop_move,stand_up,stand_down,recovery_stand,"
           "sit,rise_sit,free_walk,"
           "classic_walk_on,classic_walk_off,free_avoid_on,free_avoid_off,"
           "free_bound_on,free_bound_off,free_jump_on,free_jump_off,"
           "hand_stand_on,hand_stand_off,walk_upright_on,walk_upright_off,"
           "cross_step_on,cross_step_off,pose_on,pose_off,hello,stretch,"
           "content,heart,scrape,dance1,dance2,front_jump,front_pounce,"
           "front_flip,left_flip,back_flip"},
          {"web_mode_commands",
           "damp,stand_up,stand_down,recovery_stand,stop_move,"
           "sit,rise_sit,free_walk,"
           "classic_walk_on,classic_walk_off,free_avoid_on,free_avoid_off,"
           "free_bound_on,free_bound_off,free_jump_on,free_jump_off,"
           "hand_stand_on,hand_stand_off,walk_upright_on,walk_upright_off,"
           "cross_step_on,cross_step_off,pose_on,pose_off"},
          {"web_mode_labels",
           "damp:阻尼模式|Damping|Damp;stand_up:起立|Stand up|StandUp;"
           "balance_stand:平衡站立|Balance stand|BalanceStand;"
           "stop_move:停止移动|Stop move|StopMove;"
           "stand_down:趴下|Lie down|StandDown;"
           "recovery_stand:恢复站立|Recovery stand|RecoveryStand;"
           "sit:坐下|Sit|Sit;rise_sit:坐下后起身|Rise from sit|RiseSit;"
           "free_walk:灵动模式|Free walk|FreeWalk · FSM 100;"
           "classic_walk_on:经典步态|Classic walk|ClassicWalk · FSM 2010;"
           "classic_walk_off:退出经典步态|Exit classic walk|ClassicWalk(false) · → FSM 100;"
           "free_avoid_on:闪避模式|Free avoid|FreeAvoid · FSM 2007;"
           "free_avoid_off:退出闪避模式|Exit free avoid|FreeAvoid(false) · → FSM 100;"
           "free_bound_on:并腿跑|Free bound|FreeBound · FSM 2008;"
           "free_bound_off:退出并腿跑|Exit free bound|FreeBound(false) · → FSM 100;"
           "free_jump_on:跳跃跑|Free jump|FreeJump · FSM 2009;"
           "free_jump_off:退出跳跃跑|Exit free jump|FreeJump(false) · → FSM 100;"
           "hand_stand_on:倒立行走|Hand stand|HandStand · FSM 2011;"
           "hand_stand_off:退出倒立|Exit hand stand|HandStand(false) · → FSM 100;"
           "walk_upright_on:后腿直立|Upright walk|WalkUpright · FSM 2017;"
           "walk_upright_off:退出后腿直立|Exit upright walk|WalkUpright(false) · → FSM 100;"
           "cross_step_on:交叉步|Cross step|CrossStep · FSM 2016;"
           "cross_step_off:退出交叉步|Exit cross step|CrossStep(false) · → FSM 100;"
           "pose_on:摆姿势|Pose|Pose(true) · FSM 1091;"
           "pose_off:退出摆姿势|Exit pose|Pose(false)"},
          {"mode_targets",
           "damp:1001,stand_up:1002,balance_stand:1013,"
           "recovery_stand:1013,sit:1007,rise_sit:1013,free_walk:100,"
           "classic_walk_on:2010,free_avoid_on:2007,free_bound_on:2008,"
           "free_jump_on:2009,hand_stand_on:2011,walk_upright_on:2017,"
           "cross_step_on:2016,pose_on:1091"},
          {"interaction_action_commands",
           "hello,stretch,content,heart,scrape,dance1,dance2,front_jump,"
           "front_pounce,front_flip,left_flip,back_flip"},
          {"interaction_action_labels",
           "hello:打招呼|Hello|Hello;stretch:伸懒腰|Stretch|Stretch;"
           "content:开心|Happy|Content;heart:比心|Heart|Heart;"
           "scrape:拜年作揖|Greeting bow|Scrape;"
           "dance1:舞蹈 1|Dance 1|Dance1;dance2:舞蹈 2|Dance 2|Dance2;"
           "front_jump:前跳|Front jump|FrontJump;"
           "front_pounce:前扑|Front pounce|FrontPounce;"
           "front_flip:前空翻|Front flip|FrontFlip;"
           "left_flip:左空翻|Left flip|LeftFlip;"
           "back_flip:后空翻|Back flip|BackFlip"},
          {"dangerous_commands",
           "free_bound_on,free_jump_on,hand_stand_on,"
           "walk_upright_on,cross_step_on,front_jump,front_pounce,"
           "front_flip,left_flip,back_flip"},
          {"mode_sources",
           "stand_up:100|1001|1002|1004|1007|1013|2006;"
           "stand_down:100|1001|1002|1004|1013|1015|1016|1017|2006|2010;"
           "recovery_stand:100|1001|1002|1004|1007|1013|2006;"
           "balance_stand:100|1002|1004|1007|1013|1015|1016|1017|2006|2007|2008|2009|2010|2011|2016|2017;"
           "sit:100|1002|1013;rise_sit:1007;"
           "free_walk:100|1002|1013|1015|1016|1017|1091|2007|2008|2009|2010|2011|2016|2017;"
           "classic_walk_on:100|1013|1015|1016|1017|2010;"
           "classic_walk_off:2010;"
           "free_avoid_on:100|1013|2010;free_avoid_off:2007;"
           "free_bound_on:100|1013|2010;free_bound_off:2008;"
           "free_jump_on:100|1013|2010;free_jump_off:2009;"
           "hand_stand_on:100|1013;hand_stand_off:2011;"
           "walk_upright_on:100|1013;walk_upright_off:2017;"
           "cross_step_on:100|1013;cross_step_off:2016;"
           "pose_on:100|1013;pose_off:1091;"
           "hello:100|1002|1013;stretch:100|1002|1013;"
           "content:100|1002|1013;heart:100|1002|1013;"
           "scrape:100|1002|1013;dance1:100|1002|1013;"
           "dance2:100|1002|1013;front_jump:100|1002|1013;"
           "front_pounce:100|1002|1013;front_flip:100|1002|1013;"
           "left_flip:100|1002|1013;back_flip:100|1002|1013"},
          {"forward_hard_limit_mps", "1.5"},
          {"lateral_hard_limit_mps", "1.0"},
          {"yaw_hard_limit_radps", "2.3"},
          {"velocity_lease_seconds", "1.0"}},
         "go2_real_control_requires_operator_validation"},
        {CapabilityKey::kJointDebug, true, true,
         VerificationLevel::kMockVerified,
         {{"control_modes", "full_body"},
          {"control_topic", "rt/lowcmd"},
          {"real_control_policy", "operator_validation"}},
         "go2_real_joint_debug_requires_operator_validation"},
        {CapabilityKey::kJointTeach, true, true,
         VerificationLevel::kMockVerified,
         {{"control_mode", "full_body"},
          {"control_topic", "rt/lowcmd"},
          {"remote_binding", "true"},
          {"real_control_policy", "operator_validation"}},
         "go2_joint_teach_requires_operator_validation"},
        {CapabilityKey::kAudio, true, true,
         VerificationLevel::kMockVerified,
         {{"tts_backends", "kokoro"},
          {"play_stream", "true"},
          {"tts", "false"},
          {"volume", "true"},
          {"asr", "false"},
          {"builtin_llm", "false"},
          {"real_control_policy", "operator_validation"},
          {"play_stream_transport", "audiohub_file_player"},
          {"audio_format", "wav_pcm_s16le_16000_mono"}},
         "go2_audio_hardware_unverified"},
        {CapabilityKey::kCameraRgb, true, true,
         VerificationLevel::kMockVerified,
         {{"stream_role", "rgb"},
          {"transport", "rtp_h264_udp"},
          {"fixed_source", "udp://230.1.1.1:1720"},
          {"port", "1720"},
          {"width", "1280"},
          {"height", "720"},
          {"fps", "15"},
          {"allow_client_source_override", "false"}},
         "go2_rgb_readonly_not_verified"},
        {CapabilityKey::kCameraDepth, false, false,
         VerificationLevel::kUnsupported, {}, "go2_depth_unsupported"},
        {CapabilityKey::kLidar, true, true,
         VerificationLevel::kMockVerified,
         {{"raw_points_topic", "rt/utlidar/cloud"},
          {"raw_points_frame", "utlidar_lidar"},
          {"deskewed_points_topic", "rt/utlidar/cloud_deskewed"},
          {"deskewed_points_frame", "odom"},
          {"height_map_topic", "rt/utlidar/height_map_array"},
          {"range_info_topic", "rt/utlidar/range_info"}},
         "go2_lidar_readonly_not_verified"},
        {CapabilityKey::kSlam, true, true,
         VerificationLevel::kMockVerified,
         {{"required_devices", "dock"},
          {"required_device_any", "mid360,xt16"},
          {"required_firmware_version", "1.1.7"},
          {"service_name", "slam_operate"},
          {"service_version", "1.0.0.1"},
          {"map_whitelist", "test1.pcd..test10.pcd"}},
         "go2_slam_runtime_conditions_unverified"},
        {CapabilityKey::kHeadControl, false, false,
         VerificationLevel::kUnsupported, {}, "go2_head_control_unsupported"},
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
  if (required != capability.parameters.end() && !required->second.empty()) {
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
  }
  const auto required_any = capability.parameters.find("required_device_any");
  if (required_any == capability.parameters.end() ||
      required_any->second.empty()) {
    return true;
  }
  std::size_t start = 0;
  while (start < required_any->second.size()) {
    const auto end = required_any->second.find(',', start);
    const auto device = required_any->second.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    if (!device.empty() && options.enabled_devices.count(device) != 0) {
      return true;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return false;
}

void ApplyRuntimeCapabilityPolicy(RobotProfile& profile,
                                  const RuntimeOptions& options) {
  for (auto& capability : profile.capabilities) {
    capability.parameters["product_supported"] =
        capability.implemented ? "true" : "false";
    capability.parameters["variant_supported"] = "true";

    capability.available = capability.implemented && capability.available;
    if (!capability.available) continue;

    if (options.read_only &&
        (capability.key == CapabilityKey::kLocomotion ||
         capability.key == CapabilityKey::kSlam ||
         capability.key == CapabilityKey::kJointDebug ||
         capability.key == CapabilityKey::kJointTeach ||
         capability.key == CapabilityKey::kHeadControl)) {
      capability.available = false;
      capability.reason = "readonly_runtime";
      continue;
    }

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

bool SourceKindMatches(TelemetrySourceKey key, TelemetrySourceKind kind) {
  switch (key) {
    case TelemetrySourceKey::kLowState:
      return kind == TelemetrySourceKind::kHgLowState ||
             kind == TelemetrySourceKind::kGo2LowState;
    case TelemetrySourceKey::kSecondaryBms:
    case TelemetrySourceKey::kBms:
      return kind == TelemetrySourceKind::kHgBmsState;
    case TelemetrySourceKey::kSecondaryImu:
      return kind == TelemetrySourceKind::kHgImuState;
    case TelemetrySourceKey::kMainBoard:
      return kind == TelemetrySourceKind::kHgMainBoardState;
    case TelemetrySourceKey::kOdometry:
      return kind == TelemetrySourceKind::kGo2SportModeState;
    case TelemetrySourceKey::kSportMode:
      return kind == TelemetrySourceKind::kHgSportModeState;
    case TelemetrySourceKey::kFaults:
      return kind == TelemetrySourceKind::kGo2ErrorList;
  }
  return false;
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
        !SourceKindMatches(source.key, source.kind)) {
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
  if (product_id == "a2") return &A2Profile();
  if (product_id == "g1") return &G1Profile();
  if (product_id == "r1") return &R1Profile();
  if (product_id == "go2") return &Go2Profile();
  return nullptr;
}

std::vector<const RobotProfile*> RobotRegistry::All() {
  return {&G1Profile(), &R1Profile(), &Go2Profile(), &A2Profile()};
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
  } else if (product_id == "go2") {
    bundle.profile = Go2Profile();
    ValidateProfile(bundle.profile);
    bundle.telemetry = Go2TelemetryPlan();
    ApplyTelemetryCapabilityParameters(bundle.profile, bundle.telemetry);
    ApplyRuntimeCapabilityPolicy(bundle.profile, options);
    bundle.locomotion = CreateGo2LocomotionAdapter(options.mock);
    bundle.audio = CreateGo2AudioCapability(options.mock);
    if (CapabilityAvailable(bundle.profile, CapabilityKey::kJointDebug)) {
      bundle.joint_debug = CreateGo2JointDebugPolicy(Go2Profile());
    }
    bundle.devices = CreateGo2DeviceCapabilityPolicy(options.enabled_devices);
  } else if (product_id == "a2") {
    bundle.profile = A2Profile();
    ValidateProfile(bundle.profile);
    bundle.telemetry = A2TelemetryPlan();
    ApplyTelemetryCapabilityParameters(bundle.profile, bundle.telemetry);
    ApplyRuntimeCapabilityPolicy(bundle.profile, options);
    A2JointGains gains;
    const bool gains_ready = !options.a2_joint_gains_file.empty() &&
                             LoadA2JointGains(options.a2_joint_gains_file, gains);
    if (!options.mock && !gains_ready) {
      for (auto& capability : bundle.profile.capabilities) {
        if (capability.key == CapabilityKey::kJointDebug ||
            capability.key == CapabilityKey::kJointTeach) {
          capability.available = false;
          if (!options.read_only) {
            capability.reason = options.a2_joint_gains_file.empty()
                ? "a2_joint_gains_required" : "a2_joint_gains_invalid";
            capability.parameters["reason_zh"] = "缺少有效的 A2 关节增益配置；操作前还须手动停用运控";
            capability.parameters["reason_en"] = "Valid A2 joint gains are required; the operator must also stop motion control manually";
          }
        }
      }
    }
    bundle.locomotion = CreateA2LocomotionAdapter(options.mock);
    bundle.audio = CreateA2AudioCapability(options.mock);
    bundle.joint_debug = CreateA2JointDebugPolicy(A2Profile(), gains);
    bundle.devices = CreateA2DeviceCapabilityPolicy(options.enabled_devices);
  } else {
    throw std::invalid_argument("unregistered_robot_product");
  }

  ValidateBundle(bundle);
  return bundle;
}

}  // namespace g1_web
