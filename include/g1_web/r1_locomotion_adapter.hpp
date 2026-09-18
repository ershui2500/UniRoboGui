#pragma once

#include <memory>

#include "g1_web/locomotion.hpp"

namespace g1_web {

std::unique_ptr<ILocomotion> CreateR1LocomotionAdapter(bool mock);

}  // namespace g1_web
