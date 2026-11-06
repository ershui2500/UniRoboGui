#include "g1_web/a2_device_capability.hpp"

namespace g1_web {
namespace {

class A2DeviceCapabilityPolicy final : public IDeviceCapabilityPolicy {
 public:
  explicit A2DeviceCapabilityPolicy(const std::set<std::string>& enabled_devices)
      : jt128_declared_(enabled_devices.count("jt128") != 0) {
    perception_.lidar_model = "JT128";
    perception_.slam_service = "unitree_slam";
    if (jt128_declared_) {
      perception_.raw_lidar_points_topic = "rt/unitree/slam_lidar/points";
      perception_.raw_lidar_imu_topic = "rt/unitree/slam_lidar/imu1";
      perception_.raw_point_source = "jt128";
      perception_.raw_lidar_frame = "base_link";
      // Official structural lidar-to-base transform, not a calibrated extrinsic.
      perception_.raw_lidar_to_base = {0, 0, 1, 0.33767F,
                                      1, 0, 0, 0,
                                      0, 1, 0, 0.08134F};
    }
    perception_.dependencies = {{"unitree_slam", DependencyLifecycle::kExternalRequired}};
    perception_.mapping_points_topic = "rt/unitree/slam_mapping/points";
    perception_.mapping_odom_topic = "rt/unitree/slam_mapping/odom";
    perception_.relocation_points_topic = "rt/unitree/slam_relocation/points";
    perception_.relocation_odom_topic = "rt/unitree/slam_relocation/odom";
    perception_.global_map_topic = "rt/unitree/slam_relocation/global_map";
    perception_.slam_info_topic = "rt/slam_info";
    perception_.slam_key_info_topic = "rt/slam_key_info";
    perception_.navigation_max_distance_m = 30.0;
    perception_.attachment_required = true;
    perception_.attachment_declared = jt128_declared_;
    perception_.raw_input_required = false;
    perception_.allow_unverified_control_rpc = true;
    camera_.provider = "unitree_a2";
    camera_.camera_model = "Unitree A2 front camera";
    camera_.manage_first_person_service = false;
    camera_.allow_client_source_override = false;
    camera_.streams = {{CameraStreamRole::kRgb,
                        CameraTransport::kRtpH264Udp,
                        "udp://230.1.1.1:1720", 1720, 1280, 720, 15,
                        false, "eth0"}};
  }

  const PerceptionDeploymentPolicy& Perception() const override {
    return perception_;
  }
  const CameraDeploymentPolicy& Camera() const override { return camera_; }
  HardwarePresence DetectHardware(CapabilityKey key) const override {
    if (key == CapabilityKey::kSlam && jt128_declared_) {
      return HardwarePresence::kPresent;
    }
    return HardwarePresence::kUnknown;
  }

 private:
  bool jt128_declared_{false};
  PerceptionDeploymentPolicy perception_{};
  CameraDeploymentPolicy camera_{};
};

}  // namespace

std::unique_ptr<IDeviceCapabilityPolicy> CreateA2DeviceCapabilityPolicy(
    const std::set<std::string>& enabled_devices) {
  return std::make_unique<A2DeviceCapabilityPolicy>(enabled_devices);
}

}  // namespace g1_web
