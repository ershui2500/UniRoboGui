#include "g1_web/r1_device_capability.hpp"

#include <filesystem>
#include <set>
#include <string>

namespace g1_web {
namespace {

class R1DeviceCapabilityPolicy final : public IDeviceCapabilityPolicy {
 public:
  explicit R1DeviceCapabilityPolicy(
      const std::set<std::string>& enabled_devices)
      : mid360_declared_(enabled_devices.count("mid360") != 0) {
    perception_.lidar_model = "Livox Mid-360";
    perception_.lidar_service = "mid360_driver";
    perception_.slam_service = "unitree_slam";
    perception_.dependencies = {
        {"mid360_driver", DependencyLifecycle::kExternalRequired},
        {"unitree_slam", DependencyLifecycle::kManagedByWeb},
    };
    perception_.mapping_points_topic = "rt/unitree/slam_mapping/points";
    perception_.mapping_odom_topic = "rt/unitree/slam_mapping/odom";
    perception_.relocation_points_topic =
        "rt/unitree/slam_relocation/points";
    perception_.relocation_odom_topic =
        "rt/unitree/slam_relocation/odom";
    perception_.global_map_topic =
        "rt/unitree/slam_relocation/global_map";
    perception_.slam_info_topic = "rt/slam_info";
    perception_.slam_key_info_topic = "rt/slam_key_info";
    perception_.attachment_required = true;
    perception_.attachment_declared = mid360_declared_;
    perception_.raw_input_required = false;

    camera_.provider = "unitree_r1_edu_stereo";
    camera_.camera_model = "Unitree R1 EDU stereo camera";
    camera_.streams = {
        {CameraStreamRole::kRgb, CameraTransport::kRtpH264Udp,
         "udp://0.0.0.0:5003", 5003, 544, 448, 10},
        {CameraStreamRole::kRgbLeft, CameraTransport::kRtpH264Udp,
         "udp://0.0.0.0:5002", 5002, 544, 448, 0},
        {CameraStreamRole::kRgbRight, CameraTransport::kRtpH264Udp,
         "udp://0.0.0.0:5003", 5003, 544, 448, 0},
        {CameraStreamRole::kDepth, CameraTransport::kV4l2Raw16,
         "/dev/video-dep", 0, 544, 448, 10},
    };
    camera_.external_service_requirements = {
        "video_hub_managed_via_robot_state",
        "stereo_patch_pc1_managed_via_robot_state",
        "unitree_depth_receiver_managed_via_helper",
    };
    camera_.service_version_status = "external_check_required";
    camera_.conflicting_service_match = "videohub";
    camera_.required_service_match = "stereopatchpc1";
    camera_.privileged_receiver_helper =
        "/usr/local/sbin/r1-web-camera-service";
    camera_.manage_first_person_service = false;
    camera_.manage_external_services = true;
    camera_.manage_privileged_receiver = true;
    camera_.allow_client_source_override = false;
  }

  const PerceptionDeploymentPolicy& Perception() const override {
    return perception_;
  }

  const CameraDeploymentPolicy& Camera() const override { return camera_; }

  HardwarePresence DetectHardware(CapabilityKey key) const override {
    if (key == CapabilityKey::kCameraDepth) {
      std::error_code error;
      const bool exists = std::filesystem::exists("/dev/video-dep", error);
      if (error) return HardwarePresence::kUnknown;
      return exists ? HardwarePresence::kPresent : HardwarePresence::kAbsent;
    }
    if (key == CapabilityKey::kCameraRgb) return HardwarePresence::kUnknown;
    if (key == CapabilityKey::kSlam || key == CapabilityKey::kLidar) {
      return mid360_declared_ ? HardwarePresence::kPresent
                              : HardwarePresence::kUnknown;
    }
    return HardwarePresence::kUnknown;
  }

 private:
  bool mid360_declared_{false};
  PerceptionDeploymentPolicy perception_{};
  CameraDeploymentPolicy camera_{};
};

}  // namespace

std::unique_ptr<IDeviceCapabilityPolicy> CreateR1DeviceCapabilityPolicy(
    const std::set<std::string>& enabled_devices) {
  return std::make_unique<R1DeviceCapabilityPolicy>(enabled_devices);
}

}  // namespace g1_web
