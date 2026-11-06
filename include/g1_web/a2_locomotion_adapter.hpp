#pragma once

#include <functional>
#include <memory>
#include "g1_web/locomotion.hpp"

namespace g1_web {
using A2SportCall = std::function<std::int32_t(
    const std::string&, std::int32_t, float, float, float)>;
std::unique_ptr<ILocomotion> CreateA2LocomotionAdapter(bool mock);
std::unique_ptr<ILocomotion> CreateA2LocomotionAdapterForTesting(A2SportCall call);
}  // namespace g1_web
