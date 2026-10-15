#pragma once

#include <memory>
#include <functional>

namespace Json { class Value; }

#include "g1_web/audio_capability.hpp"

namespace g1_web {

std::unique_ptr<IAudioCapability> CreateGo2AudioCapability(bool mock);

using Go2AudioRequest =
    std::function<std::int32_t(std::int32_t, const Json::Value&)>;
// Exercises packet framing and cleanup without initializing DDS or waiting.
std::unique_ptr<IAudioCapability> CreateGo2AudioCapabilityForTesting(
    Go2AudioRequest request);

}  // namespace g1_web
