#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace g1_web {

struct G1ModelDescription {
  std::uint8_t mode_machine;
  std::string_view name;
  std::string_view urdf_file;
};

const std::array<G1ModelDescription, 11>& G1ModelCatalog();
const G1ModelDescription* FindG1Model(std::uint8_t mode_machine);

}  // namespace g1_web
