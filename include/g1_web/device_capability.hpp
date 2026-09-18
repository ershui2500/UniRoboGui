#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "g1_web/robot_profile.hpp"

namespace g1_web {

enum class HardwarePresence {
  kUnknown,
  kPresent,
  kAbsent,
};

struct DeviceCapabilityRuntime {
  CapabilityKey key{CapabilityKey::kTelemetry};
  HardwarePresence hardware_presence{HardwarePresence::kUnknown};
  bool service_available{true};
  bool deployment_enabled{true};
  bool safety_allowed{true};
  std::string reason;
  bool service_compatible{true};
  std::string service_version_status{"compatible"};
  std::optional<VerificationLevel> verification_level;
  bool receiver_available{true};
  bool external_preparation_ready{true};
};

enum class DependencyLifecycle {
  kManagedByWeb,
  kExternalRequired,
};

struct PerceptionDependency {
  std::string name;
  DependencyLifecycle lifecycle{DependencyLifecycle::kExternalRequired};
};

struct PerceptionDeploymentPolicy {
  std::string lidar_model;
  std::string lidar_service;
  std::string slam_service;
  std::vector<PerceptionDependency> dependencies;
  std::string raw_lidar_points_topic;
  std::string raw_lidar_imu_topic;
  std::string mapping_points_topic;
  std::string mapping_odom_topic;
  std::string relocation_points_topic;
  std::string relocation_odom_topic;
  std::string global_map_topic;
  std::string slam_info_topic;
  std::string slam_key_info_topic;
  std::string raw_point_source;
  std::string low_obstacle_source;
  bool attachment_required{false};
  bool attachment_declared{true};
  bool raw_input_required{true};
  bool rotate_raw_lidar_x_180{false};
};

enum class CameraStreamRole {
  kRgb,
  kRgbLeft,
  kRgbRight,
  kDepth,
};

enum class CameraTransport {
  kOpenCvDevice,
  kRealSense,
  kRtpH264Udp,
  kV4l2Raw16,
};

struct CameraStreamSpec {
  CameraStreamRole role{CameraStreamRole::kRgb};
  CameraTransport transport{CameraTransport::kOpenCvDevice};
  std::string fixed_source;
  unsigned int port{0};
  unsigned int width{0};
  unsigned int height{0};
  unsigned int fps{0};
  bool allow_runtime_dimensions{false};
};

struct CameraDeploymentPolicy {
  std::string provider;
  std::string camera_model;
  std::string first_person_helper;
  std::vector<std::string> exclusive_service_dependencies;
  std::vector<CameraStreamSpec> streams;
  std::vector<std::string> external_service_requirements;
  std::string service_version_status{"compatible"};
  std::string conflicting_service_match;
  std::string required_service_match;
  std::string privileged_receiver_helper;
  bool manage_first_person_service{true};
  bool manage_external_services{false};
  bool manage_privileged_receiver{false};
  bool allow_client_source_override{true};
};

class IDeviceCapabilityPolicy {
 public:
  virtual ~IDeviceCapabilityPolicy() = default;
  virtual const PerceptionDeploymentPolicy& Perception() const = 0;
  virtual const CameraDeploymentPolicy& Camera() const = 0;
  virtual HardwarePresence DetectHardware(CapabilityKey key) const = 0;
};

std::unique_ptr<IDeviceCapabilityPolicy> CreateUnavailableDeviceCapabilityPolicy();

const char* HardwarePresenceName(HardwarePresence presence);
const char* DependencyLifecycleName(DependencyLifecycle lifecycle);
const char* CameraStreamRoleName(CameraStreamRole role);
const char* CameraTransportName(CameraTransport transport);
const CameraStreamSpec* FindCameraStream(const CameraDeploymentPolicy& policy,
                                         CameraStreamRole role);
void ApplyEffectiveDeviceCapabilities(
    RobotProfile& profile,
    const std::vector<DeviceCapabilityRuntime>& runtime_capabilities);

}  // namespace g1_web
