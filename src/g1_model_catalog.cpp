#include "g1_web/g1_model_catalog.hpp"

#include "g1_web/robot_registry.hpp"

namespace g1_web {

const std::array<G1ModelDescription, 11>& G1ModelCatalog() {
  static const std::array<G1ModelDescription, 11> kModels = [] {
    std::array<G1ModelDescription, 11> result{};
    const auto* profile = RobotRegistry::Find("g1");
    for (std::size_t index = 0; index < result.size(); ++index) {
      const auto& model = profile->model_variants[index];
      result[index] = {static_cast<std::uint8_t>(model.selector_value),
                       model.name, model.urdf_file};
    }
    return result;
  }();
  return kModels;
}

const G1ModelDescription* FindG1Model(std::uint8_t mode_machine) {
  for (const auto& model : G1ModelCatalog()) {
    if (model.mode_machine == mode_machine) {
      return &model;
    }
  }
  return nullptr;
}

}  // namespace g1_web
