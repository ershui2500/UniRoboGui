#include "g1_web/device_capability.hpp"

#include <algorithm>

namespace g1_web {
namespace {

class UnavailableDeviceCapabilityPolicy final : public IDeviceCapabilityPolicy {
 public:
  const PerceptionDeploymentPolicy& Perception() const override {
    return perception_;
  }

  const CameraDeploymentPolicy& Camera() const override { return camera_; }

  HardwarePresence DetectHardware(CapabilityKey) const override {
    return HardwarePresence::kUnknown;
  }

 private:
  PerceptionDeploymentPolicy perception_{};
  CameraDeploymentPolicy camera_{};
};

}  // namespace

std::unique_ptr<IDeviceCapabilityPolicy> CreateUnavailableDeviceCapabilityPolicy() {
  return std::make_unique<UnavailableDeviceCapabilityPolicy>();
}

const char* HardwarePresenceName(HardwarePresence presence) {
  switch (presence) {
    case HardwarePresence::kUnknown:
      return "unknown";
    case HardwarePresence::kPresent:
      return "present";
    case HardwarePresence::kAbsent:
      return "absent";
  }
  return "unknown";
}

const char* DependencyLifecycleName(DependencyLifecycle lifecycle) {
  switch (lifecycle) {
    case DependencyLifecycle::kManagedByWeb:
      return "managed_by_web";
    case DependencyLifecycle::kExternalRequired:
      return "external_required";
  }
  return "external_required";
}

const char* CameraStreamRoleName(CameraStreamRole role) {
  switch (role) {
    case CameraStreamRole::kRgb:
      return "rgb";
    case CameraStreamRole::kRgbLeft:
      return "rgb_left";
    case CameraStreamRole::kRgbRight:
      return "rgb_right";
    case CameraStreamRole::kDepth:
      return "depth";
  }
  return "unknown";
}

const char* CameraTransportName(CameraTransport transport) {
  switch (transport) {
    case CameraTransport::kOpenCvDevice:
      return "opencv_v4l2";
    case CameraTransport::kRealSense:
      return "realsense";
    case CameraTransport::kRtpH264Udp:
      return "rtp_h264_udp";
    case CameraTransport::kV4l2Raw16:
      return "v4l2_raw16";
  }
  return "unknown";
}

const CameraStreamSpec* FindCameraStream(const CameraDeploymentPolicy& policy,
                                         CameraStreamRole role) {
  const auto stream = std::find_if(
      policy.streams.begin(), policy.streams.end(),
      [role](const CameraStreamSpec& spec) { return spec.role == role; });
  return stream == policy.streams.end() ? nullptr : &*stream;
}

void ApplyEffectiveDeviceCapabilities(
    RobotProfile& profile,
    const std::vector<DeviceCapabilityRuntime>& runtime_capabilities) {
  for (const auto& runtime : runtime_capabilities) {
    auto descriptor = std::find_if(
        profile.capabilities.begin(), profile.capabilities.end(),
        [&](const CapabilityDescriptor& capability) {
          return capability.key == runtime.key;
        });
    if (descriptor == profile.capabilities.end()) continue;

    descriptor->parameters["product_supported"] =
        descriptor->implemented ? "true" : "false";
    descriptor->parameters["hardware_presence"] =
        HardwarePresenceName(runtime.hardware_presence);
    descriptor->parameters["service_available"] =
        runtime.service_available ? "true" : "false";
    descriptor->parameters["receiver_available"] =
        runtime.receiver_available ? "true" : "false";
    descriptor->parameters["external_preparation_ready"] =
        runtime.external_preparation_ready ? "true" : "false";
    descriptor->parameters["service_version_status"] =
        runtime.service_version_status;
    descriptor->parameters["deployment_enabled"] =
        runtime.deployment_enabled ? "true" : "false";
    descriptor->parameters["safety_allowed"] =
        runtime.safety_allowed ? "true" : "false";

    if (!descriptor->implemented) continue;
    if (runtime.verification_level) {
      descriptor->verification_level = *runtime.verification_level;
    }
    const bool hardware_available =
        runtime.hardware_presence == HardwarePresence::kPresent;
    descriptor->available =
        descriptor->available && hardware_available &&
        runtime.service_available && runtime.receiver_available &&
        runtime.external_preparation_ready && runtime.service_compatible &&
        runtime.deployment_enabled && runtime.safety_allowed;
    if (descriptor->available) {
      descriptor->reason.clear();
      continue;
    }
    if (!runtime.reason.empty()) {
      descriptor->reason = runtime.reason;
    } else if (runtime.hardware_presence == HardwarePresence::kUnknown) {
      descriptor->reason = "required_hardware_unverified";
    } else if (!hardware_available) {
      descriptor->reason = "required_hardware_not_present";
    } else if (!runtime.service_available) {
      descriptor->reason = "required_service_unavailable";
    } else if (!runtime.receiver_available) {
      descriptor->reason = "camera_receiver_unavailable";
    } else if (!runtime.external_preparation_ready) {
      descriptor->reason = "camera_external_preparation_required";
    } else if (!runtime.service_compatible) {
      descriptor->reason = "required_service_incompatible";
    } else if (!runtime.deployment_enabled) {
      descriptor->reason = "deployment_disabled";
    } else if (!runtime.safety_allowed) {
      descriptor->reason = "safety_policy_disabled";
    } else if (descriptor->reason.empty()) {
      descriptor->reason = "product_capability_unavailable";
    }
  }
}

}  // namespace g1_web
