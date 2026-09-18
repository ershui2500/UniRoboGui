#pragma once

#include <memory>

#include "g1_web/audio_capability.hpp"

namespace g1_web {

std::unique_ptr<IAudioCapability> CreateG1AudioCapability(bool mock);

}  // namespace g1_web
