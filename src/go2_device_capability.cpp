#include "g1_web/go2_device_capability.hpp"

#include <set>
#include <string>

namespace g1_web {
namespace {

class Go2DeviceCapabilityPolicy final : public IDeviceCapabilityPolicy {
 public:
  explicit Go2DeviceCapabilityPolicy(
      const std::set<std::string>& enabled_devices) {
    const bool dock = enabled_devices.count("dock") != 0;
    const bool mid360 = enabled_devices.count("mid360") != 0;
    const bool xt16 = enabled_devices.count("xt16") != 0;
    const bool one_official_lidar = mid360 != xt16;

    perception_.lidar_model = "Unitree Go2 onboard LiDAR";
    perception_.raw_lidar_points_topic = "rt/utlidar/cloud";
    perception_.raw_point_source = "utlidar_lidar";
    perception_.raw_input_required = false;
    perception_.rotate_raw_lidar_x_180 = false;
    perception_.attachment_required = true;
    perception_.attachment_declared = dock && one_official_lidar;
    perception_.service_version_status = "unverified";

    if (perception_.attachment_declared) {
      perception_.slam_service = "unitree_slam";
      perception_.dependencies = {
          {mid360 ? "mid360_driver" : "xt16_driver",
           DependencyLifecycle::kExternalRequired},
          {"unitree_slam", DependencyLifecycle::kExternalRequired},
      };
      perception_.mapping_points_topic =
          "rt/unitree/slam_mapping/points";
      perception_.mapping_odom_topic = "rt/unitree/slam_mapping/odom";
      perception_.relocation_points_topic =
          "rt/unitree/slam_relocation/points";
      perception_.relocation_odom_topic =
          "rt/unitree/slam_relocation/odom";
      perception_.global_map_topic =
          "rt/unitree/slam_relocation/global_map";
      perception_.slam_info_topic = "rt/slam_info";
      perception_.slam_key_info_topic = "rt/slam_key_info";
    }

    camera_.provider = "unitree_go2";
    camera_.camera_model = "Unitree Go2 front camera";
    camera_.manage_first_person_service = false;
    camera_.manage_external_services = false;
    camera_.manage_privileged_receiver = false;
    camera_.allow_client_source_override = false;
    camera_.streams = {{
        CameraStreamRole::kRgb,
        CameraTransport::kRtpH264Udp,
        "udp://230.1.1.1:1720",
        1720,
        1280,
        720,
        15,
        false,
    }};
  }

  const PerceptionDeploymentPolicy& Perception() const override {
    return perception_;
  }

  const CameraDeploymentPolicy& Camera() const override { return camera_; }

  HardwarePresence DetectHardware(CapabilityKey key) const override {
    if (key == CapabilityKey::kSlam) {
      return perception_.attachment_declared ? HardwarePresence::kPresent
                                             : HardwarePresence::kAbsent;
    }
    return HardwarePresence::kUnknown;
  }

 private:
  PerceptionDeploymentPolicy perception_;
  CameraDeploymentPolicy camera_;
};

}  // namespace

std::unique_ptr<IDeviceCapabilityPolicy> CreateGo2DeviceCapabilityPolicy(
    const std::set<std::string>& enabled_devices) {
  return std::make_unique<Go2DeviceCapabilityPolicy>(enabled_devices);
}

}  // namespace g1_web
