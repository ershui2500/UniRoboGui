#pragma once

#include <memory>

#include "g1_web/joint_debug_policy.hpp"

namespace g1_web {

std::unique_ptr<IJointDebugPolicy> CreateG1JointDebugPolicy(
    const RobotProfile& profile);

}  // namespace g1_web
