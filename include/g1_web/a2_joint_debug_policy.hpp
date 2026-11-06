#pragma once

#include <memory>
#include <array>

#include "g1_web/joint_debug_policy.hpp"

namespace g1_web {

struct A2JointGains {
  bool remote_protocol_confirmed{false};
  std::array<JointDebugGain, 12> normal{};
  std::array<JointDebugGain, 12> recording{};
};
bool LoadA2JointGains(const std::string& path, A2JointGains& gains);

std::unique_ptr<IJointDebugPolicy> CreateA2JointDebugPolicy(
    const RobotProfile& profile, const A2JointGains& gains = {});

}  // namespace g1_web
