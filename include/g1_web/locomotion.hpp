#pragma once

#include <cstdint>
#include <string>

namespace g1_web {

struct ControlData;

struct LocomotionInitialization {
  bool state_valid{false};
  std::uint32_t fsm_id{0};
  std::uint32_t fsm_mode{0};
};

struct LocomotionCommandResult {
  std::int32_t api_result{-1};
  std::int32_t target_fsm{-1};
  std::string error;
};

class ILocomotion {
 public:
  virtual ~ILocomotion() = default;

  virtual bool Initialize(LocomotionInitialization& state,
                          std::string& error) = 0;
  virtual bool QueryState(LocomotionInitialization& state,
                          std::string& error) {
    state = {};
    error = "locomotion_state_query_unsupported";
    return false;
  }
  virtual void Shutdown() = 0;

  virtual bool IsKnownModeCommand(const std::string& command,
                                  std::int32_t argument) const = 0;
  virtual bool IsOperationalFsm(std::uint32_t fsm_id) const = 0;

  virtual std::string ValidateVelocity(const ControlData& control,
                                       float vx_m_s, float vy_m_s,
                                       float vyaw_rad_s,
                                       std::int32_t speed_mode) const = 0;
  virtual std::string ValidateVelocityExecution(
      const ControlData& control, std::int32_t speed_mode) const = 0;

  virtual LocomotionCommandResult ExecuteMode(
      const std::string& command, std::int32_t argument) = 0;
  virtual LocomotionCommandResult ApplyVelocity(
      float vx_m_s, float vy_m_s, float vyaw_rad_s,
      std::int32_t speed_mode, bool active) = 0;
};

}  // namespace g1_web
