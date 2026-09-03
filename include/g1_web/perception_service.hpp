#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <unitree/idl/ros2/PointCloud2_.hpp>

namespace g1_web {

struct PointSample {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct PoseData {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct PerceptionRequest {
  std::string request_key;
  std::string command;
  std::string map_name;
  bool confirmed{false};
  PoseData pose{};
};

struct PerceptionResult {
  bool accepted{false};
  bool duplicate{false};
  bool lidar_started{false};
  bool slam_started{false};
  std::uint64_t request_id{0};
  std::int32_t api_result{0};
  std::string error;
  std::string response;
};

struct DecodedPointCloud {
  bool valid{false};
  std::string error;
  std::string frame_id;
  std::size_t source_points{0};
  std::vector<PointSample> points;
};

struct PointCloudFilterOptions {
  float voxel_size_m{0.0F};
  float minimum_range_m{0.0F};
  float maximum_range_m{0.0F};
  float minimum_z_m{-20.0F};
  float maximum_z_m{20.0F};
  std::size_t maximum_points{0};
  bool remove_isolated_voxels{false};
};

struct LowObstacleDetection {
  bool detected{false};
  float nearest_range_m{0.0F};
  std::size_t support_points{0};
};

// Resolve the exact PCD path used by the official SLAM save/load APIs. The
// strict map-name whitelist also protects the HTTP map-download endpoint.
std::string ResolvePerceptionMapPath(const std::string& name);
std::string ResolvePerceptionMapDownloadPath(const std::string& name);

// Decode and uniformly decimate ROS PointCloud2 data. Exposed for regression
// tests because malformed field layouts must never reach the browser renderer.
DecodedPointCloud DecodePointCloud(
    const sensor_msgs::msg::dds_::PointCloud2_& message,
    std::size_t maximum_points);

// Merge points into a deterministic voxel grid, crop impossible ranges and
// optionally reject isolated voxels before applying the final Web point cap.
std::vector<PointSample> FilterPointCloudForWeb(
    const std::vector<PointSample>& points,
    const PointCloudFilterOptions& options);

// Safety detector for low obstacles that are present in the raw Mid-360 cloud
// but may be discarded by the closed-source navigation obstacle filter.
// It cannot recover points inside the lidar's physical blind area.
LowObstacleDetection DetectLowObstacleForSafety(
    const std::vector<PointSample>& upright_raw_points);

class PerceptionService {
 public:
  PerceptionService(bool mock, bool navigation_enabled);
  ~PerceptionService();

  bool Start(std::string& error);
  void Stop();
  PerceptionResult Submit(const PerceptionRequest& request);

  std::string SerializeStatus() const;
  std::string SerializeFrame() const;
  std::string SerializeGlobalMap() const;
  bool GetTopicGlobalMapPcd(const std::string& map_name,
                            std::string& body) const;
  std::string SerializeNavigationScene() const;
  std::string SerializeNavigationTopics() const;
  bool ConfigureNavigationTopic(const std::string& kind,
                                const std::string& topic,
                                std::string& error);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_web
