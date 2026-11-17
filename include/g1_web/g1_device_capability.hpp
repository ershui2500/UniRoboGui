#pragma once

#include <memory>

#include "g1_web/device_capability.hpp"

namespace g1_web {

std::unique_ptr<IDeviceCapabilityPolicy> CreateG1DeviceCapabilityPolicy(
    bool robot_local_resources = true);

}  // namespace g1_web
