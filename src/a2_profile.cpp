#include "g1_web/a2_profile.hpp"

namespace g1_web {
TelemetrySubscriptionPlan A2TelemetryPlan() {
  return {{{TelemetrySourceKey::kLowState, TelemetrySourceKind::kHgLowState, "rt/lowstate", true},
           {TelemetrySourceKey::kBms, TelemetrySourceKind::kHgBmsState, "rt/lf/bmsstate", false},
           {TelemetrySourceKey::kSecondaryBms, TelemetrySourceKind::kHgBmsState, "rt/lf/secondary_bmsstate", false},
           {TelemetrySourceKey::kMainBoard, TelemetrySourceKind::kHgMainBoardState, "rt/lf/mainboardstate", false},
           {TelemetrySourceKey::kOdometry, TelemetrySourceKind::kGo2SportModeState,
            "rt/lf/sportmodestate", false, true, SportControlStateField::kMode}}};
}

const RobotProfile& A2Profile() {
  static const RobotProfile profile = [] {
    RobotProfile result;
    result.identity = {"unitree", "a2", "a2", "a2", "Unitree A2", "quadruped"};
    result.idl_family = "unitree_sdk2_hg";
    result.control_api_family = "unitree_sdk2_a2_sport";
    result.model_asset_root = "/assets/unitree/a2_description/urdf";
    result.model_package = "a2_description";
    result.joint_schema.motor_slot_count = 35;
    result.joint_schema.joints = {
        {"front_right_hip", "右前髋", "FR_hip_joint", 0, "front_right_leg"},
        {"front_right_thigh", "右前大腿", "FR_thigh_joint", 1, "front_right_leg"},
        {"front_right_calf", "右前小腿", "FR_calf_joint", 2, "front_right_leg"},
        {"front_left_hip", "左前髋", "FL_hip_joint", 3, "front_left_leg"},
        {"front_left_thigh", "左前大腿", "FL_thigh_joint", 4, "front_left_leg"},
        {"front_left_calf", "左前小腿", "FL_calf_joint", 5, "front_left_leg"},
        {"rear_right_hip", "右后髋", "RR_hip_joint", 6, "rear_right_leg"},
        {"rear_right_thigh", "右后大腿", "RR_thigh_joint", 7, "rear_right_leg"},
        {"rear_right_calf", "右后小腿", "RR_calf_joint", 8, "rear_right_leg"},
        {"rear_left_hip", "左后髋", "RL_hip_joint", 9, "rear_left_leg"},
        {"rear_left_thigh", "左后大腿", "RL_thigh_joint", 10, "rear_left_leg"},
        {"rear_left_calf", "左后小腿", "RL_calf_joint", 11, "rear_left_leg"},
    };
    result.model_variants = {{1, "A2", "a2.urdf"}};
    result.static_constraints = {{"reserved_motor_slots", "12-34"},
        {"mode_machine_role", "official_model_selector_not_product_id"},
        {"sport_state_topic_status", "unverified_documentation_topic_conflict"},
        {"model_asset_kind", "official_urdf_and_meshes"}};
    result.capabilities = {
      {CapabilityKey::kTelemetry, true, true, VerificationLevel::kMockVerified,
       {{"joint_count", "12"},
        {"reason_zh", "真实遥测尚未验收"},
        {"reason_en", "Real telemetry has not been validated"}, {"batteries", "2:bms:rt/lf/bmsstate,1:secondary_bms:rt/lf/secondary_bmsstate"},
        {"odometry_error_code_role", "error_code"}}, "a2_readonly_runtime_unverified"},
      {CapabilityKey::kLocomotion, true, true, VerificationLevel::kMockVerified,
       {{"real_control_policy", "operator_validation"},
        {"reason_zh", "已开放现场操作员逐项验证；真实控制尚未验收"},
        {"reason_en", "Open for operator validation; real control is not yet verified"}, {"control_client", "a2_sport_client"},
        {"allowed_fsm", "3,4,5"}, {"speed_modes", "0,1"},
        {"speed_labels", "0:低速|Low;1:高速|High"},
        {"speed_presets", "0:0.6:0.4:1.5,1:1.5:0.5:2.5"},
        {"fsm_speed_presets", "3:0:0.6:0.4:1.5,3:1:1.5:0.5:2.5,4:0:2.0:0.25:1.2,4:1:4.0:0.3:1.0,5:0:0.5:0.25:0.8,5:1:0.6:0.3:1.0"},
        {"forward_hard_limit_mps", "4.0"}, {"lateral_hard_limit_mps", "0.5"},
        {"yaw_hard_limit_radps", "2.5"}, {"velocity_lease_seconds", "0.25"}, {"velocity_timeout_policy", "backend_stop"},
        {"mode_commands", "damp,stand_up,stand_down,balance_stand,stop_move,recovery_stand,walk,run,climb,left_side_gait_on,left_side_gait_off,right_side_gait_on,right_side_gait_off,hand_stand_on,hand_stand_off,biped_stand_on,biped_stand_off,front_flip,back_flip"},
        {"web_mode_commands", "damp,stand_up,stand_down,balance_stand,stop_move,recovery_stand,walk,run,climb,left_side_gait_on,left_side_gait_off,right_side_gait_on,right_side_gait_off,hand_stand_on,hand_stand_off,biped_stand_on,biped_stand_off"},
        {"web_mode_labels",
         "damp:阻尼模式|Damping|Damp;"
         "stand_up:站高（关节锁定）|Stand high (locked)|StandUp;"
         "stand_down:站低（关节锁定）|Stand low (locked)|StandDown;"
         "balance_stand:平衡站立|Balance stand|BalanceStand;"
         "stop_move:停止移动|Stop move|StopMove;"
         "recovery_stand:恢复站立|Recovery stand|RecoveryStand;"
         "walk:行走步态|Walking gait|SwitchGait(0);"
         "run:跑步步态|Running gait|SwitchGait(1);"
         "climb:攀爬步态|Climbing gait|SwitchGait(2);"
         "left_side_gait_on:左侧步|Left side gait|LeftSideGait(1);"
         "left_side_gait_off:退出左侧步|Exit left side gait|LeftSideGait(0);"
         "right_side_gait_on:右侧步|Right side gait|RightSideGait(1);"
         "right_side_gait_off:退出右侧步|Exit right side gait|RightSideGait(0);"
         "hand_stand_on:倒立|Handstand|HandStand(1);"
         "hand_stand_off:退出倒立|Exit handstand|HandStand(0);"
         "biped_stand_on:直立|Biped stand|BipedStand(1);"
         "biped_stand_off:退出直立|Exit biped stand|BipedStand(0)"},
        {"interaction_action_commands", "front_flip,back_flip"},
        {"interaction_action_labels", "front_flip:前空翻|Front flip|FrontFlip;back_flip:后空翻|Back flip|BackFlip"},
        {"mode_targets", "damp:0,stand_down:1,stand_up:2,balance_stand:3,walk:3,run:4,climb:5,left_side_gait_on:6,right_side_gait_on:7,hand_stand_on:8,biped_stand_on:9"},
        {"mode_sources",
         "damp:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15;"
         "stop_move:0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15;"
         "stand_up:0|1|2|3|4|5;"
         "stand_down:0|1|2|3|4|5;"
         "balance_stand:1|2|3|4|5;"
         "recovery_stand:0|1;"
         "walk:3|4|5;"
         "run:3|4|5;"
         "climb:3|4|5;"
         "left_side_gait_on:3;left_side_gait_off:6;"
         "right_side_gait_on:3;right_side_gait_off:7;"
         "hand_stand_on:3;hand_stand_off:8;"
         "biped_stand_on:3;biped_stand_off:9;"
         "front_flip:3;back_flip:3"},
        {"dangerous_commands", "damp,recovery_stand,run,left_side_gait_on,right_side_gait_on,hand_stand_on,biped_stand_on,front_flip,back_flip"},
        {"state_labels",
         "0:阻尼|Passive;"
         "1:站低|Stand down;"
         "2:站高|Stand up;"
         "3:行走|Walking;"
         "4:跑步|Running;"
         "5:攀爬|Climbing;"
         "6:左侧步|Left side gait;"
         "7:右侧步|Right side gait;"
         "8:倒立|Handstand;"
         "9:直立|Biped stand;"
         "10:前空翻|Front flip;"
         "11:后空翻|Back flip;"
         "12:恢复|Recovery;"
         "13:高度控制|Height control;"
         "14:姿态|Pose;"
         "15:欧拉姿态|Euler"}},
       "a2_real_control_requires_operator_validation"},
      {CapabilityKey::kJointDebug, true, true,
       VerificationLevel::kMockVerified,
       {{"control_modes", "full_body"},
        {"control_topic", "rt/lowcmd"},
        {"real_control_policy", "operator_validation"},
        {"reason_zh", "A2 关节调试需要操作员配置增益并验证"},
        {"reason_en", "A2 joint debug requires operator-configured gains and validation"}},
       "a2_joint_debug_requires_operator_validation"},
      {CapabilityKey::kJointTeach, true, true, VerificationLevel::kMockVerified,
       {{"control_mode", "full_body"}, {"control_topic", "rt/lowcmd"},
        {"remote_binding", "true"}, {"real_control_policy", "operator_validation"},
        {"reason_zh", "A2 示教与遥控绑定需要操作员配置增益并验证"},
        {"reason_en", "A2 teaching and remote bindings require operator-configured gains and validation"}},
       "a2_joint_teach_requires_operator_validation"},
      {CapabilityKey::kCameraRgb, true, true, VerificationLevel::kMockVerified,
       {{"transport", "rtp_h264_udp"}, {"fixed_source", "udp://230.1.1.1:1720"},
        {"port", "1720"}, {"resolution", "1280x720"}, {"fps", "15"},
        {"receiver_start_allowed_when_unavailable", "true"},
        {"reason_zh", "等待 A2 原生 RGB 图传的持续画面"},
        {"reason_en", "Waiting for sustained A2 native RGB video frames"}},
       "a2_camera_stream_unverified"},
      {CapabilityKey::kAudio, true, true, VerificationLevel::kMockVerified,
       {{"tts_backends", "kokoro,unitree"},
        {"play_stream_transport", "a2_audio_client"},
        {"audio_format", "pcm_s16le_16000_mono"},
        {"default_tts_backend", "unitree"},
        {"play_stream", "true"}, {"tts", "true"},
        {"volume", "true"}, {"asr", "true"},
        {"builtin_llm", "false"},
        {"reason_zh", "A2 音频尚待真机验收"},
        {"reason_en", "A2 audio awaits on-robot validation"}},
       "a2_audio_hardware_unverified"},
      {CapabilityKey::kLidar, true, true, VerificationLevel::kMockVerified,
       {{"required_devices", "jt128"}, {"lidar_model", "JT128"},
        {"raw_preview_only", "true"}}, ""},
      {CapabilityKey::kSlam, true, true, VerificationLevel::kMockVerified,
       {{"required_devices", "jt128"}, {"lidar_model", "JT128"},
        {"raw_input_required", "false"}, {"service_name", "slam_operate"},
        {"service_version", "1.0.0.1"},
        {"empty_rpc_warning_whitelist", "3203"},
        {"navigation_max_distance_m", "30"},
        {"map_whitelist", "test1.pcd..test10.pcd"}},
       ""},
    };
    for (auto key : {CapabilityKey::kCameraDepth,
         CapabilityKey::kHeadControl}) {
      result.capabilities.push_back({key, false, false, VerificationLevel::kUnsupported, {}, ""});
    }
    return result;
  }();
  return profile;
}
}  // namespace g1_web
