#pragma once

#include <string>
#include <utility>

#include "g1_web/resource_manager.hpp"
#include "g1_web/robot_profile.hpp"

namespace g1_web {

struct SafetyDecision {
  bool allowed{true};
  std::string reason;
  std::string error;
};

class SafetyManager {
 public:
  SafetyDecision CheckConfirmation(bool confirmed) const {
    return confirmed ? Allow()
                     : Reject("confirmation_required",
                              "confirmation_required");
  }

  SafetyDecision CheckServiceReady(bool ready) const {
    return ready ? Allow()
                 : Reject("service_not_ready", "control_not_ready");
  }

  SafetyDecision CheckFreshness(
      bool fresh, const std::string& compatibility_error) const {
    return fresh ? Allow()
                 : Reject("state_stale", compatibility_error);
  }

  SafetyDecision CheckLease(bool valid) const {
    return valid ? Allow()
                 : Reject("control_lease_expired",
                          "control_lease_expired");
  }

  SafetyDecision CheckCapability(const RobotProfile& profile,
                                 CapabilityKey key, bool mock) const {
    const CapabilityDescriptor* descriptor = nullptr;
    for (const auto& capability : profile.capabilities) {
      if (capability.key == key) {
        descriptor = &capability;
        break;
      }
    }
    if (descriptor == nullptr || !descriptor->implemented ||
        !descriptor->available ||
        descriptor->verification_level == VerificationLevel::kUnsupported ||
        descriptor->verification_level == VerificationLevel::kDisabled) {
      return Reject("capability_unavailable", "control_not_ready");
    }

    const auto policy = descriptor->parameters.find("real_control_policy");
    const bool operator_validation =
        descriptor->verification_level == VerificationLevel::kMockVerified &&
        policy != descriptor->parameters.end() &&
        policy->second == "operator_validation";
    const bool verified =
        mock ? descriptor->verification_level == VerificationLevel::kMockVerified ||
                   descriptor->verification_level == VerificationLevel::kControlVerified
             : descriptor->verification_level == VerificationLevel::kControlVerified ||
                   operator_validation;
    return verified ? Allow()
                    : Reject("capability_verification",
                             "control_not_ready");
  }

  SafetyDecision CheckResource(
      const ResourceAcquireResult& result,
      const std::string& compatibility_error = "control_busy") const {
    if (result.acquired) return Allow();
    if (result.error == "resource_busy") {
      return Reject("resource_busy", compatibility_error);
    }
    return Reject("resource_rejected",
                  result.error.empty() ? compatibility_error : result.error);
  }

  void CleanupControlFailure(ResourceManager& resources,
                             ControlResource resource,
                             const std::string& owner) const {
    resources.Release(resource, owner);
  }

  void CleanupControlFailure(ResourceManager& resources,
                             const std::string& owner) const {
    resources.ReleaseOwner(owner);
  }

 private:
  static SafetyDecision Allow() { return {}; }

  static SafetyDecision Reject(const char* reason, std::string error) {
    return {false, reason, std::move(error)};
  }
};

}  // namespace g1_web
