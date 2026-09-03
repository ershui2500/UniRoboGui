#include "g1_web/g1_model_catalog.hpp"

namespace g1_web {

const std::array<G1ModelDescription, 11>& G1ModelCatalog() {
  static constexpr std::array<G1ModelDescription, 11> kModels{{
      {2, "G1 29DOF", "g1_29dof.urdf"},
      {3, "G1 29DOF (locked waist)", "g1_29dof_lock_waist.urdf"},
      {5, "G1 29DOF rev 1.0", "g1_29dof_rev_1_0.urdf"},
      {6, "G1 29DOF rev 1.0 (locked waist)",
       "g1_29dof_lock_waist_rev_1_0.urdf"},
      {11, "G1 29DOF mode 11", "g1_29dof_mode_11.urdf"},
      {12, "G1 29DOF mode 12", "g1_29dof_mode_12.urdf"},
      {13, "G1 29DOF mode 13", "g1_29dof_mode_13.urdf"},
      {14, "G1 29DOF mode 14", "g1_29dof_mode_14.urdf"},
      {15, "G1 29DOF mode 15", "g1_29dof_mode_15.urdf"},
      {16, "G1 29DOF mode 16", "g1_29dof_mode_16.urdf"},
      {18, "G1 29DOF mode 18", "g1_29dof_mode_18.urdf"},
  }};
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
