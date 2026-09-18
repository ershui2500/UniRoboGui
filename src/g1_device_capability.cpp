#include "g1_web/g1_device_capability.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace g1_web {
namespace {

class G1DeviceCapabilityPolicy final : public IDeviceCapabilityPolicy {
 public:
  G1DeviceCapabilityPolicy() {
    perception_.lidar_model = "Livox Mid-360";
    perception_.lidar_service = "lidar_driver";
    perception_.slam_service = "unitree_slam";
    perception_.dependencies = {
        {"lidar_driver", DependencyLifecycle::kManagedByWeb},
        {"unitree_slam", DependencyLifecycle::kManagedByWeb},
    };
    perception_.raw_lidar_points_topic = "rt/utlidar/cloud_livox_mid360";
    perception_.raw_lidar_imu_topic = "rt/utlidar/imu_livox_mid360";
    perception_.mapping_points_topic = "rt/unitree/slam_mapping/points";
    perception_.mapping_odom_topic = "rt/unitree/slam_mapping/odom";
    perception_.relocation_points_topic = "rt/unitree/slam_relocation/points";
    perception_.relocation_odom_topic = "rt/unitree/slam_relocation/odom";
    perception_.global_map_topic = "rt/unitree/slam_relocation/global_map";
    perception_.slam_info_topic = "rt/slam_info";
    perception_.slam_key_info_topic = "rt/slam_key_info";
    perception_.raw_point_source = "utlidar_mid360";
    perception_.low_obstacle_source = "raw_mid360";
    perception_.rotate_raw_lidar_x_180 = true;

    camera_.camera_model = "Intel RealSense D435i";
    camera_.first_person_helper =
        "/usr/local/sbin/g1-web-first-person-service";
    camera_.exclusive_service_dependencies = {
        "teleimager.service", "master_service.service", "videohub_pc4"};
  }

  const PerceptionDeploymentPolicy& Perception() const override {
    return perception_;
  }

  const CameraDeploymentPolicy& Camera() const override { return camera_; }

  HardwarePresence DetectHardware(CapabilityKey key) const override {
    if (key != CapabilityKey::kCameraRgb &&
        key != CapabilityKey::kCameraDepth) {
      return HardwarePresence::kUnknown;
    }

    const std::filesystem::path video_root("/sys/class/video4linux");
    std::error_code error;
    if (!std::filesystem::exists(video_root, error)) {
      return error ? HardwarePresence::kUnknown : HardwarePresence::kAbsent;
    }
    std::filesystem::directory_iterator entries(video_root, error);
    if (error) return HardwarePresence::kUnknown;
    for (const auto& entry : entries) {
      std::ifstream name_stream(entry.path() / "name");
      std::string name;
      std::getline(name_stream, name);
      if (name.find("RealSense") != std::string::npos ||
          name.find("D435") != std::string::npos) {
        return HardwarePresence::kPresent;
      }
    }
    return HardwarePresence::kAbsent;
  }

 private:
  PerceptionDeploymentPolicy perception_;
  CameraDeploymentPolicy camera_;
};

}  // namespace

std::unique_ptr<IDeviceCapabilityPolicy> CreateG1DeviceCapabilityPolicy() {
  return std::make_unique<G1DeviceCapabilityPolicy>();
}

}  // namespace g1_web
