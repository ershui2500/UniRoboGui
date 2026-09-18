#include "g1_web/joint_names.hpp"

#include "g1_web/robot_registry.hpp"

namespace g1_web {

const std::array<JointName, kNamedJointCount>& JointNames() {
  static const std::array<JointName, kNamedJointCount> kNames = [] {
    std::array<JointName, kNamedJointCount> result{};
    const auto* profile = RobotRegistry::Find("g1");
    for (std::size_t index = 0; index < result.size(); ++index) {
      const auto& joint = profile->joint_schema.joints[index];
      result[index] = {joint.name, joint.name_zh};
    }
    return result;
  }();
  return kNames;
}

}  // namespace g1_web
