#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace g1_web {

struct JointName {
  std::string_view name;
  std::string_view name_zh;
};

constexpr std::size_t kNamedJointCount = 29;
constexpr std::size_t kMotorSlotCount = 35;

const std::array<JointName, kNamedJointCount>& JointNames();

}  // namespace g1_web
