#include "g1_web/joint_names.hpp"

namespace g1_web {

const std::array<JointName, kNamedJointCount>& JointNames() {
  static constexpr std::array<JointName, kNamedJointCount> kNames{{
      {"left_hip_pitch", "左髋俯仰"},
      {"left_hip_roll", "左髋横滚"},
      {"left_hip_yaw", "左髋偏航"},
      {"left_knee", "左膝"},
      {"left_ankle_pitch", "左踝俯仰"},
      {"left_ankle_roll", "左踝横滚"},
      {"right_hip_pitch", "右髋俯仰"},
      {"right_hip_roll", "右髋横滚"},
      {"right_hip_yaw", "右髋偏航"},
      {"right_knee", "右膝"},
      {"right_ankle_pitch", "右踝俯仰"},
      {"right_ankle_roll", "右踝横滚"},
      {"waist_yaw", "腰部偏航"},
      {"waist_roll", "腰部横滚"},
      {"waist_pitch", "腰部俯仰"},
      {"left_shoulder_pitch", "左肩俯仰"},
      {"left_shoulder_roll", "左肩横滚"},
      {"left_shoulder_yaw", "左肩偏航"},
      {"left_elbow", "左肘"},
      {"left_wrist_roll", "左腕横滚"},
      {"left_wrist_pitch", "左腕俯仰"},
      {"left_wrist_yaw", "左腕偏航"},
      {"right_shoulder_pitch", "右肩俯仰"},
      {"right_shoulder_roll", "右肩横滚"},
      {"right_shoulder_yaw", "右肩偏航"},
      {"right_elbow", "右肘"},
      {"right_wrist_roll", "右腕横滚"},
      {"right_wrist_pitch", "右腕俯仰"},
      {"right_wrist_yaw", "右腕偏航"},
  }};
  return kNames;
}

}  // namespace g1_web
