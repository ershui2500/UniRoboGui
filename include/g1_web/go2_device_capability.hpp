#pragma once

#include <memory>
#include <set>
#include <string>

#include "g1_web/device_capability.hpp"

namespace g1_web {

std::unique_ptr<IDeviceCapabilityPolicy> CreateGo2DeviceCapabilityPolicy(
    const std::set<std::string>& enabled_devices = {});

}  // namespace g1_web
