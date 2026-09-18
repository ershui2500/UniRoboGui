#pragma once

#include <memory>

#include "g1_web/joint_debug_policy.hpp"

namespace g1_web {

std::unique_ptr<IJointDebugPolicy> CreateR1JointDebugPolicy(
    const RobotProfile& profile);

}  // namespace g1_web
