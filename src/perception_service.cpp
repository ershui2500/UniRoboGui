#include "g1_web/perception_service.hpp"
#include "g1_web/ros_navigation_bridge.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

#include <json/json.h>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/robot/client/client.hpp>

namespace g1_web {
namespace {

using Clock = std::chrono::steady_clock;
using PointCloudMessage = sensor_msgs::msg::dds_::PointCloud2_;
using ImuMessage = sensor_msgs::msg::dds_::Imu_;
using OdometryMessage = nav_msgs::msg::dds_::Odometry_;
using StringMessage = std_msgs::msg::dds_::String_;

// Decode a larger candidate set, then voxel-filter and quantize the Web-only
// copy. The official SLAM inputs remain untouched; these limits only reduce
// HTTP/Wi-Fi traffic and browser rendering work.
constexpr std::size_t kMaximumLiveDecodePoints = 24000;
constexpr std::size_t kMaximumGlobalDecodePoints = 60000;
constexpr std::size_t kMaximumLivePoints = 3600;
constexpr std::size_t kMaximumGlobalPoints = 30000;
constexpr std::size_t kMaximumTrajectoryPoints = 1200;
constexpr PointCloudFilterOptions kRawLiveFilter{
    0.10F, 0.35F, 25.0F, -2.5F, 5.0F, kMaximumLivePoints, true};
constexpr PointCloudFilterOptions kSlamLiveFilter{
    0.08F, 0.0F, 0.0F, -2.5F, 5.0F, kMaximumLivePoints, true};
constexpr PointCloudFilterOptions kGlobalMapFilter{
    0.08F, 0.0F, 0.0F, -3.0F, 6.0F, kMaximumGlobalPoints, false};
constexpr double kMaximumMapCoordinateM = 45.0;
constexpr double kMaximumNavigationDistanceM = 10.0;
constexpr auto kPoseFreshness = std::chrono::milliseconds(1500);
constexpr std::int64_t kLidarInputFreshnessMs = 500;
constexpr float kLowObstacleCellM = 0.10F;
constexpr float kLowObstacleMinForwardM = 0.35F;
constexpr float kLowObstacleMaxForwardM = 1.60F;
constexpr float kLowObstacleHalfWidthM = 0.55F;
constexpr float kLowObstacleMinReliefM = 0.05F;
constexpr std::size_t kLowObstacleMinCellPoints = 2;

constexpr std::int32_t kApiNavigate = 1102;
constexpr std::int32_t kApiPause = 1201;
constexpr std::int32_t kApiResume = 1202;
constexpr std::int32_t kApiStartMapping = 1801;
constexpr std::int32_t kApiFinishMapping = 1802;
constexpr std::int32_t kApiInitializePose = 1804;
constexpr std::int32_t kApiStopSlam = 1901;

bool IsFinite(double value) { return std::isfinite(value); }

std::int64_t AgeMs(bool received, Clock::time_point timestamp) {
  if (!received) return -1;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now() - timestamp)
      .count();
}

std::int64_t MonotonicMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now().time_since_epoch())
      .count();
}

std::int64_t AtomicAgeMs(const std::atomic<std::int64_t>& timestamp_ms) {
  const auto timestamp = timestamp_ms.load(std::memory_order_relaxed);
  return timestamp > 0 ? MonotonicMs() - timestamp : -1;
}

std::string JsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

bool ParseJson(const std::string& text, Json::Value& root) {
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream stream(text);
  return Json::parseFromStream(builder, stream, &root, &errors) &&
         root.isObject();
}

bool IsSupportedMapFilename(const std::string& name) {
  constexpr char kPrefix[] = "test";
  constexpr char kSuffix[] = ".pcd";
  if (name.size() < 9 || name.size() > 10 ||
      name.compare(0, 4, kPrefix) != 0 ||
      name.compare(name.size() - 4, 4, kSuffix) != 0) {
    return false;
  }
  const std::string number_text = name.substr(4, name.size() - 8);
  if (number_text.empty() || number_text.size() > 2 ||
      (number_text.size() > 1 && number_text.front() == '0')) {
    return false;
  }
  int number = 0;
  for (const unsigned char value : number_text) {
    if (!std::isdigit(value)) return false;
    number = number * 10 + (value - '0');
  }
  return number >= 1 && number <= 10;
}

std::string CanonicalReportedMapFilename(const std::string& name) {
  if (IsSupportedMapFilename(name)) return name;
  const std::string with_suffix = name + ".pcd";
  return IsSupportedMapFilename(with_suffix) ? with_suffix : std::string{};
}

std::string SafeMapPath(const std::string& name) {
  if (!IsSupportedMapFilename(name)) return {};
  return "/home/unitree/" + name;
}

std::string SafeMapDownloadPath(const std::string& name) {
  if (SafeMapPath(name).empty()) return {};
  return "/home/unitree/.cache/g1-web-control/maps/" + name;
}

std::string BuildWebMapPcd(const std::vector<PointSample>& points) {
  if (points.empty()) return {};
  std::ostringstream output;
  output.precision(9);
  output << "# .PCD v0.7 - Unitree SLAM global map\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << points.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << "\n"
         << "DATA ascii\n";
  for (const auto& point : points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' '
           << point.intensity << '\n';
  }
  return output.str();
}

bool WriteWebMapPcd(const std::string& path,
                    const std::vector<PointSample>& points) {
  const std::string body = BuildWebMapPcd(points);
  if (path.empty() || body.empty()) return false;
  std::error_code error;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(),
                                      error);
  if (error) return false;
  const std::string temporary = path + ".web.tmp";
  std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
  if (!output) return false;
  output.write(body.data(), static_cast<std::streamsize>(body.size()));
  output.close();
  if (!output) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    return false;
  }
  std::filesystem::rename(temporary, path, error);
  if (!error) return true;
  std::filesystem::remove(path, error);
  error.clear();
  std::filesystem::rename(temporary, path, error);
  if (!error) return true;
  std::filesystem::remove(temporary, error);
  return false;
}

template <typename T>
T ByteSwap(T value) {
  std::array<unsigned char, sizeof(T)> source{};
  std::array<unsigned char, sizeof(T)> target{};
  std::memcpy(source.data(), &value, sizeof(T));
  std::reverse_copy(source.begin(), source.end(), target.begin());
  std::memcpy(&value, target.data(), sizeof(T));
  return value;
}

template <typename T>
bool ReadScalar(const std::vector<std::uint8_t>& data, std::size_t offset,
                bool big_endian, T& value) {
  if (offset > data.size() || sizeof(T) > data.size() - offset) return false;
  std::memcpy(&value, data.data() + offset, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (big_endian) value = ByteSwap(value);
#else
  if (!big_endian) value = ByteSwap(value);
#endif
  return true;
}

struct FieldLocation {
  std::uint32_t offset{0};
  std::uint8_t type{0};
  bool found{false};
};

bool ReadNumeric(const std::vector<std::uint8_t>& data, std::size_t base,
                 const FieldLocation& field, bool big_endian, float& value) {
  if (!field.found) return false;
  const std::size_t offset = base + field.offset;
  using namespace sensor_msgs::msg::dds_::PointField_Constants;
  switch (field.type) {
    case FLOAT32_: {
      return ReadScalar(data, offset, big_endian, value);
    }
    case FLOAT64_: {
      double temporary = 0.0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = static_cast<float>(temporary);
      return true;
    }
    case UINT8_: {
      std::uint8_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = temporary;
      return true;
    }
    case INT8_: {
      std::int8_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = temporary;
      return true;
    }
    case UINT16_: {
      std::uint16_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = temporary;
      return true;
    }
    case INT16_: {
      std::int16_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = temporary;
      return true;
    }
    case UINT32_: {
      std::uint32_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = static_cast<float>(temporary);
      return true;
    }
    case INT32_: {
      std::int32_t temporary = 0;
      if (!ReadScalar(data, offset, big_endian, temporary)) return false;
      value = static_cast<float>(temporary);
      return true;
    }
    default:
      return false;
  }
}

PoseData ToPose(const OdometryMessage& message) {
  PoseData result;
  const auto& pose = message.pose().pose();
  result.x = pose.position().x();
  result.y = pose.position().y();
  result.z = pose.position().z();
  result.qx = pose.orientation().x();
  result.qy = pose.orientation().y();
  result.qz = pose.orientation().z();
  result.qw = pose.orientation().w();
  return result;
}

Json::Value PoseJson(const PoseData& pose) {
  Json::Value value(Json::objectValue);
  value["x"] = pose.x;
  value["y"] = pose.y;
  value["z"] = pose.z;
  value["q_x"] = pose.qx;
  value["q_y"] = pose.qy;
  value["q_z"] = pose.qz;
  value["q_w"] = pose.qw;
  return value;
}

class SlamClient final : public unitree::robot::Client {
 public:
  SlamClient() : Client("slam_operate", false) {}

  void Init() override {
    SetApiVersion("1.0.0.1");
    SetTimeout(10.0F);
    for (const std::int32_t api : {kApiNavigate, kApiPause, kApiResume,
                                   kApiStartMapping, kApiFinishMapping,
                                   kApiInitializePose, kApiStopSlam}) {
      RegistApi(api, 0);
    }
  }

  std::int32_t Invoke(std::int32_t api, const std::string& parameter,
                      std::string& response) {
    return Call(api, parameter, response);
  }
};

struct VoxelKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

std::uint64_t StableVoxelHash(const VoxelKey& key) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const std::int32_t value : {key.x, key.y, key.z}) {
    hash ^= static_cast<std::uint32_t>(value);
    hash *= 1099511628211ULL;
  }
  return hash;
}

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    return static_cast<std::size_t>(StableVoxelHash(key));
  }
};

struct VoxelAccumulator {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
  std::uint32_t count{0};
};

struct FilteredVoxel {
  VoxelKey key{};
  PointSample point{};
  std::uint64_t hash{0};
};

struct ObstacleColumn {
  float minimum_z{std::numeric_limits<float>::infinity()};
  float maximum_z{-std::numeric_limits<float>::infinity()};
  std::size_t count{0};
};

struct PerceptionState {
  bool initialized{false};
  bool mock{false};
  bool navigation_enabled{false};
  std::string initialization_error;
  bool lidar_driver_enabled{false};
  bool unitree_slam_enabled{false};
  std::int32_t lidar_driver_api_result{0};
  std::int32_t unitree_slam_api_result{0};
  std::int32_t lidar_driver_status_raw{-1};
  std::int32_t unitree_slam_status_raw{-1};
  std::string service_error;
  std::string mode{"offline"};
  std::int32_t error_code{0};
  std::string info;
  std::string raw_info;
  std::string raw_key_info;
  bool pose_received{false};
  Clock::time_point pose_time{};
  PoseData pose{};
  bool target_set{false};
  PoseData target{};
  bool arrived{false};
  bool paused{false};
  bool obstacle{false};
  bool low_obstacle{false};
  float low_obstacle_range_m{0.0F};
  std::size_t low_obstacle_support_points{0};
  bool low_obstacle_pause_latched{false};
  std::int64_t low_obstacle_last_pause_attempt_ms{0};
  std::string low_obstacle_safety_error;
  // A stopped/completed command can still be followed by DDS samples that
  // were queued before the official API returned.  Gate task-scoped samples
  // so those late messages cannot resurrect an old mode or trajectory.
  bool task_stream_active{false};
  double progress{0.0};
  std::string controller;
  bool map_loaded{false};
  std::string map_name;
  std::string map_address;
  std::string point_source;
  std::string point_frame;
  std::size_t source_point_count{0};
  std::vector<PointSample> live_points;
  std::string global_frame;
  std::size_t global_source_point_count{0};
  std::vector<PointSample> global_points;
  bool global_map_topic_received{false};
  std::string global_map_topic_name;
  std::uint64_t global_sequence{0};
  std::vector<PointSample> trajectory;
  std::uint64_t sequence{0};
  std::uint64_t last_request_id{0};
  std::string last_request_key;
  PerceptionResult last_result{};
};

}  // namespace

std::string ResolvePerceptionMapPath(const std::string& name) {
  return SafeMapPath(name);
}

std::string ResolvePerceptionMapDownloadPath(const std::string& name) {
  return SafeMapDownloadPath(name);
}

DecodedPointCloud DecodePointCloud(const PointCloudMessage& message,
                                   std::size_t maximum_points) {
  DecodedPointCloud result;
  result.frame_id = message.header().frame_id();
  if (maximum_points == 0 || message.point_step() == 0 ||
      message.width() == 0 || message.height() == 0) {
    result.error = "empty_point_cloud";
    return result;
  }

  FieldLocation x;
  FieldLocation y;
  FieldLocation z;
  FieldLocation intensity;
  for (const auto& field : message.fields()) {
    FieldLocation* target = nullptr;
    if (field.name() == "x") target = &x;
    else if (field.name() == "y") target = &y;
    else if (field.name() == "z") target = &z;
    else if (field.name() == "intensity") target = &intensity;
    if (target) {
      target->offset = field.offset();
      target->type = field.datatype();
      target->found = true;
    }
  }
  if (!x.found || !y.found || !z.found) {
    result.error = "xyz_fields_missing";
    return result;
  }
  if (x.offset >= message.point_step() || y.offset >= message.point_step() ||
      z.offset >= message.point_step()) {
    result.error = "field_offset_out_of_range";
    return result;
  }

  const std::size_t total =
      static_cast<std::size_t>(message.width()) * message.height();
  result.source_points = total;
  const std::size_t stride = std::max<std::size_t>(
      1, (total + maximum_points - 1) / maximum_points);
  result.points.reserve(std::min(total, maximum_points));
  const std::size_t row_step = message.row_step() > 0
                                   ? message.row_step()
                                   : message.width() * message.point_step();
  const auto& bytes = message.data();
  for (std::size_t index = 0; index < total; index += stride) {
    const std::size_t row = index / message.width();
    const std::size_t column = index % message.width();
    const std::size_t base = row * row_step + column * message.point_step();
    if (base > bytes.size() || message.point_step() > bytes.size() - base) {
      continue;
    }
    PointSample point;
    if (!ReadNumeric(bytes, base, x, message.is_bigendian(), point.x) ||
        !ReadNumeric(bytes, base, y, message.is_bigendian(), point.y) ||
        !ReadNumeric(bytes, base, z, message.is_bigendian(), point.z)) {
      continue;
    }
    if (intensity.found) {
      ReadNumeric(bytes, base, intensity, message.is_bigendian(),
                  point.intensity);
    }
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z) && std::abs(point.x) <= 100.0F &&
        std::abs(point.y) <= 100.0F && std::abs(point.z) <= 20.0F) {
      result.points.push_back(point);
    }
  }
  if (result.points.empty()) {
    result.error = "no_finite_points";
    return result;
  }
  result.valid = true;
  return result;
}

std::vector<PointSample> FilterPointCloudForWeb(
    const std::vector<PointSample>& points,
    const PointCloudFilterOptions& options) {
  if (points.empty() || options.maximum_points == 0) return {};

  std::vector<PointSample> cropped;
  cropped.reserve(points.size());
  const float minimum_range_squared =
      options.minimum_range_m * options.minimum_range_m;
  const float maximum_range_squared =
      options.maximum_range_m * options.maximum_range_m;
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.z < options.minimum_z_m ||
        point.z > options.maximum_z_m) {
      continue;
    }
    const float range_squared = point.x * point.x + point.y * point.y +
                                point.z * point.z;
    if ((options.minimum_range_m > 0.0F &&
         range_squared < minimum_range_squared) ||
        (options.maximum_range_m > 0.0F &&
         range_squared > maximum_range_squared)) {
      continue;
    }
    cropped.push_back(point);
  }
  if (cropped.empty()) return {};

  if (!(options.voxel_size_m > 0.0F)) {
    if (cropped.size() <= options.maximum_points) return cropped;
    std::vector<PointSample> result;
    result.reserve(options.maximum_points);
    const std::size_t stride =
        (cropped.size() + options.maximum_points - 1) /
        options.maximum_points;
    for (std::size_t index = 0;
         index < cropped.size() && result.size() < options.maximum_points;
         index += stride) {
      result.push_back(cropped[index]);
    }
    return result;
  }

  const float inverse_voxel = 1.0F / options.voxel_size_m;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
  voxels.reserve(cropped.size());
  for (const auto& point : cropped) {
    const VoxelKey key{
        static_cast<std::int32_t>(std::floor(point.x * inverse_voxel)),
        static_cast<std::int32_t>(std::floor(point.y * inverse_voxel)),
        static_cast<std::int32_t>(std::floor(point.z * inverse_voxel))};
    auto& accumulator = voxels[key];
    accumulator.x += point.x;
    accumulator.y += point.y;
    accumulator.z += point.z;
    accumulator.intensity += point.intensity;
    ++accumulator.count;
  }

  auto build_voxels = [&](bool remove_isolated) {
    std::vector<FilteredVoxel> filtered;
    filtered.reserve(voxels.size());
    for (const auto& entry : voxels) {
      const auto& key = entry.first;
      const auto& accumulator = entry.second;
      bool connected = accumulator.count >= 2;
      if (remove_isolated && !connected) {
        for (int dx = -1; dx <= 1 && !connected; ++dx) {
          for (int dy = -1; dy <= 1 && !connected; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              if (dx == 0 && dy == 0 && dz == 0) continue;
              if (voxels.find({key.x + dx, key.y + dy, key.z + dz}) !=
                  voxels.end()) {
                connected = true;
                break;
              }
            }
          }
        }
      }
      if (remove_isolated && !connected) continue;
      const float divisor = static_cast<float>(accumulator.count);
      filtered.push_back({
          key,
          {static_cast<float>(accumulator.x / divisor),
           static_cast<float>(accumulator.y / divisor),
           static_cast<float>(accumulator.z / divisor),
           static_cast<float>(accumulator.intensity / divisor)},
          StableVoxelHash(key)});
    }
    return filtered;
  };

  auto filtered = build_voxels(options.remove_isolated_voxels);
  // Never turn a valid but unusually sparse scan into a blank viewport.
  if (filtered.empty() && !voxels.empty()) filtered = build_voxels(false);
  if (filtered.size() > options.maximum_points) {
    const auto compare_hash =
        [](const FilteredVoxel& left, const FilteredVoxel& right) {
          if (left.hash != right.hash) return left.hash < right.hash;
          if (left.key.x != right.key.x) return left.key.x < right.key.x;
          if (left.key.y != right.key.y) return left.key.y < right.key.y;
          return left.key.z < right.key.z;
        };
    std::nth_element(filtered.begin(),
                     filtered.begin() + options.maximum_points,
                     filtered.end(), compare_hash);
    filtered.resize(options.maximum_points);
  }

  std::vector<PointSample> result;
  result.reserve(filtered.size());
  for (const auto& voxel : filtered) result.push_back(voxel.point);
  return result;
}

LowObstacleDetection DetectLowObstacleForSafety(
    const std::vector<PointSample>& upright_raw_points) {
  std::unordered_map<VoxelKey, ObstacleColumn, VoxelKeyHash> columns;
  columns.reserve(upright_raw_points.size() / 4 + 1);
  const float inverse_cell = 1.0F / kLowObstacleCellM;
  for (const auto& point : upright_raw_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.x < kLowObstacleMinForwardM ||
        point.x > kLowObstacleMaxForwardM ||
        std::abs(point.y) > kLowObstacleHalfWidthM) {
      continue;
    }
    const VoxelKey key{
        static_cast<std::int32_t>(std::floor(point.x * inverse_cell)),
        static_cast<std::int32_t>(std::floor(point.y * inverse_cell)), 0};
    auto& column = columns[key];
    column.minimum_z = std::min(column.minimum_z, point.z);
    column.maximum_z = std::max(column.maximum_z, point.z);
    ++column.count;
  }

  LowObstacleDetection result;
  for (const auto& entry : columns) {
    const auto& key = entry.first;
    const auto& column = entry.second;
    if (column.count < kLowObstacleMinCellPoints) continue;

    float local_floor = column.minimum_z;
    std::size_t neighborhood_points = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const auto found = columns.find({key.x + dx, key.y + dy, 0});
        if (found == columns.end()) continue;
        local_floor = std::min(local_floor, found->second.minimum_z);
        neighborhood_points += found->second.count;
      }
    }
    if (neighborhood_points < 4 ||
        column.maximum_z - local_floor < kLowObstacleMinReliefM) {
      continue;
    }

    const float center_x = (static_cast<float>(key.x) + 0.5F) * kLowObstacleCellM;
    const float center_y = (static_cast<float>(key.y) + 0.5F) * kLowObstacleCellM;
    const float range = std::hypot(center_x, center_y);
    if (!result.detected || range < result.nearest_range_m) {
      result.detected = true;
      result.nearest_range_m = range;
      result.support_points = column.count;
    }
  }
  return result;
}

class PerceptionService::Impl {
 public:
  Impl(bool mock, bool navigation_enabled) {
    state_.mock = mock;
    state_.navigation_enabled = navigation_enabled || mock;
    navigation_bridge_ = std::make_unique<RosNavigationBridge>(mock);
  }

  ~Impl() { Stop(); }

  bool Start(std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.initialized) {
      error.clear();
      return true;
    }
    try {
      if (state_.mock) {
        state_.initialized = true;
        state_.mode = "ready";
        state_.lidar_driver_enabled = true;
        state_.unitree_slam_enabled = true;
        state_.lidar_driver_status_raw = 0;
        state_.unitree_slam_status_raw = 0;
        PopulateMockCloudsLocked();
        running_.store(true);
        mock_thread_ = std::thread([this] { MockLoop(); });
      } else {
        client_ = std::make_unique<SlamClient>();
        client_->Init();
        robot_state_client_ =
            std::make_unique<unitree::robot::b2::RobotStateClient>();
        robot_state_client_->SetTimeout(10.0F);
        robot_state_client_->Init();
        RefreshServiceStatesLocked();
        SubscribeLocked();
        // Keep the sensing pipeline warm so opening the web workspace does
        // not pay the service-launch cost before the first live point cloud.
        // This does not invoke mapping, localization or navigation APIs.
        bool started_lidar = false;
        bool started_slam = false;
        if (EnsureServiceActiveLocked("lidar_driver", started_lidar)) {
          lidar_started_by_web_ = started_lidar;
          if (EnsureServiceActiveLocked("unitree_slam", started_slam)) {
            slam_started_by_web_ = started_slam;
          }
        }
        // Accept live sensing topics immediately. Mapping/localization state is
        // still changed only by an explicit confirmed slam_operate command.
        state_.task_stream_active = true;
        state_.initialized = true;
        state_.mode = "ready";
      }
      state_.initialization_error.clear();
      std::string navigation_error;
      // The Unitree SDK perception path remains usable when ROS 2/Nav2 is not
      // installed. Surface that optional capability as status instead of
      // failing the complete web service.
      navigation_bridge_->Start(navigation_error);
      ++state_.sequence;
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "unknown_slam_initialization_error";
    }
    state_.initialization_error = error;
    state_.mode = "offline";
    return false;
  }

  void Stop() {
    navigation_bridge_->Stop();
    running_.store(false);
    if (mock_thread_.joinable()) mock_thread_.join();
    // DDS CloseChannel waits for an in-flight callback. Close outside mutex_
    // because point-cloud and odometry callbacks take the same lock.
    CloseSubscriber(global_map_);
    CloseSubscriber(relocation_odom_);
    CloseSubscriber(mapping_odom_);
    CloseSubscriber(relocation_points_);
    CloseSubscriber(mapping_points_);
    CloseSubscriber(raw_lidar_imu_);
    CloseSubscriber(raw_lidar_points_);
    CloseSubscriber(slam_key_info_);
    CloseSubscriber(slam_info_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_state_client_) {
      if (slam_started_by_web_) SwitchServiceLocked("unitree_slam", false);
      if (lidar_started_by_web_) SwitchServiceLocked("lidar_driver", false);
    }
    slam_started_by_web_ = false;
    lidar_started_by_web_ = false;
    client_.reset();
    robot_state_client_.reset();
    state_.initialized = false;
    if (state_.mode != "offline") state_.mode = "stopped";
    ++state_.sequence;
  }

  PerceptionResult Submit(const PerceptionRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    PerceptionResult result;
    if (!state_.initialized) {
      result.error = "perception_not_ready";
      return result;
    }
    if (request.request_key.size() < 8 || request.request_key.size() > 128) {
      result.error = "invalid_request_key";
      return result;
    }
    if (request.request_key == state_.last_request_key) {
      result = state_.last_result;
      result.duplicate = true;
      return result;
    }
    if (!request.confirmed) {
      result.error = "confirmation_required";
      return result;
    }

    const auto mode_allowed = [&](std::initializer_list<const char*> modes) {
      return std::any_of(modes.begin(), modes.end(), [&](const char* mode) {
        return state_.mode == mode;
      });
    };
    const bool state_allowed =
        (request.command == "start_mapping" &&
         mode_allowed({"ready", "stopped"})) ||
        (request.command == "finish_mapping" && state_.mode == "mapping") ||
        (request.command == "load_map" &&
         mode_allowed({"ready", "stopped", "localizing", "arrived"})) ||
        (request.command == "initialize_pose" && state_.map_loaded &&
         state_.map_name == request.map_name &&
         mode_allowed({"ready", "localizing", "arrived"})) ||
        (request.command == "navigate" && state_.map_loaded &&
         mode_allowed({"localizing", "arrived"})) ||
        (request.command == "pause_navigation" &&
         state_.mode == "navigating") ||
        (request.command == "resume_navigation" && state_.mode == "paused") ||
        (request.command == "cancel_navigation" && state_.map_loaded &&
         mode_allowed({"navigating", "paused"})) ||
        request.command == "stop_slam";
    if (!state_allowed &&
        (request.command == "start_mapping" ||
         request.command == "finish_mapping" ||
         request.command == "load_map" ||
         request.command == "initialize_pose" ||
         request.command == "navigate" ||
         request.command == "pause_navigation" ||
         request.command == "resume_navigation" ||
         request.command == "cancel_navigation")) {
      result.error = "invalid_slam_state";
      return result;
    }

    std::int32_t api = 0;
    Json::Value parameter(Json::objectValue);
    parameter["data"] = Json::Value(Json::objectValue);
    if (request.command == "start_mapping") {
      api = kApiStartMapping;
      parameter["data"]["slam_type"] = "indoor";
    } else if (request.command == "finish_mapping") {
      api = kApiFinishMapping;
      const std::string address = SafeMapPath(request.map_name);
      if (address.empty()) {
        result.error = "invalid_map_name";
        return result;
      }
      parameter["data"]["address"] = address;
    } else if (request.command == "load_map" ||
               request.command == "initialize_pose") {
      api = kApiInitializePose;
      const std::string address = SafeMapPath(request.map_name);
      if (address.empty() || !ValidatePose(request.pose)) {
        result.error = address.empty() ? "invalid_map_name" : "invalid_pose";
        return result;
      }
      AddPose(parameter["data"], request.pose);
      parameter["data"]["address"] = address;
    } else if (request.command == "navigate") {
      api = kApiNavigate;
      if (!state_.navigation_enabled) {
        result.error = "navigation_runtime_disabled";
        return result;
      }
      if (!ValidatePose(request.pose)) {
        result.error = "invalid_pose";
        return result;
      }
      if (!state_.pose_received ||
          Clock::now() - state_.pose_time > kPoseFreshness) {
        result.error = "slam_pose_stale";
        return result;
      }
      const double distance = std::hypot(request.pose.x - state_.pose.x,
                                         request.pose.y - state_.pose.y);
      if (distance > kMaximumNavigationDistanceM) {
        result.error = "goal_over_10m";
        return result;
      }
      parameter["data"]["targetPose"] = Json::Value(Json::objectValue);
      AddPose(parameter["data"]["targetPose"], request.pose);
      parameter["data"]["mode"] = 1;
    } else if (request.command == "pause_navigation" ||
               request.command == "cancel_navigation") {
      api = kApiPause;
    } else if (request.command == "resume_navigation") {
      api = kApiResume;
      if (!state_.navigation_enabled) {
        result.error = "navigation_runtime_disabled";
        return result;
      }
    } else if (request.command == "stop_slam") {
      api = kApiStopSlam;
    } else {
      result.error = "unknown_perception_command";
      return result;
    }

    const bool needs_slam_services =
        request.command == "start_mapping" ||
        request.command == "load_map" ||
        request.command == "initialize_pose" ||
        request.command == "navigate" ||
        request.command == "resume_navigation";
    bool started_lidar = false;
    bool started_slam = false;
    if (!state_.mock && needs_slam_services) {
      if (!EnsureServiceActiveLocked("lidar_driver", started_lidar) ||
          !EnsureServiceActiveLocked("unitree_slam", started_slam)) {
        if (started_slam) SwitchServiceLocked("unitree_slam", false);
        if (started_lidar) SwitchServiceLocked("lidar_driver", false);
        result.error = "slam_dependency_start_failed";
        return result;
      }
      WaitForLidarInputsLocked();
    }
    result.lidar_started = started_lidar;
    result.slam_started = started_slam;

    result.request_id = ++state_.last_request_id;
    result.api_result = 0;
    const bool cancel_already_paused =
        request.command == "cancel_navigation" && state_.mode == "paused";
    if (state_.mock) {
      ApplyMockCommandLocked(request);
      result.accepted = true;
      result.response = R"({"succeed":true,"errorCode":0,"info":"mock"})";
    } else if (cancel_already_paused) {
      // The official interface has pause/resume but no separate cancel RPC.
      // A paused task is already physically stopped, so cancellation only
      // discards the Web task state instead of issuing a redundant 1201.
      result.accepted = true;
      result.response =
          R"({"succeed":true,"errorCode":0,"info":"already paused; Web task cancelled"})";
    } else {
      std::string response;
      result.api_result = client_->Invoke(api, JsonText(parameter), response);
      // RobotState can report the process as active before slam_operate has
      // advertised its RPC endpoint. Retry at short intervals instead of
      // imposing the old unconditional 3s + 2s sleeps on every cold start.
      for (int attempt = 0;
           result.api_result == 3104 && started_slam && attempt < 20;
           ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        response.clear();
        result.api_result = client_->Invoke(api, JsonText(parameter), response);
      }
      result.response = response;
      bool response_success = result.api_result == 0;
      Json::Value response_json;
      if (ParseJson(response, response_json) &&
          response_json.isMember("succeed")) {
        response_success = response_success && response_json["succeed"].asBool();
      }
      result.accepted = response_success;
      if (!result.accepted) {
        result.error = "slam_api_failed";
        std::cerr << "[SLAM] command=" << request.command
                  << " api=" << api
                  << " api_result=" << result.api_result
                  << " error=" << result.error
                  << " response=" << result.response << '\n';
      }
      if (!result.accepted && needs_slam_services) {
        if (started_slam) SwitchServiceLocked("unitree_slam", false);
        if (started_lidar) SwitchServiceLocked("lidar_driver", false);
      }
      if (request.command == "stop_slam") {
        const bool slam_closed = SwitchServiceLocked("unitree_slam", false);
        slam_started_by_web_ = false;
        // Keep lidar_driver warm: its raw Mid-360 topic is the default live
        // preview after the SLAM task ends.
        if (!slam_closed) {
          result.accepted = false;
          result.error = "slam_service_stop_failed";
        } else if (result.api_result != 0) {
          // Closing the dependencies is the requested terminal state and is
          // idempotent even when API 1901 reports that SLAM was already down.
          result.accepted = true;
          result.error.clear();
        }
        if (result.accepted) {
          state_.mode = "stopped";
          state_.target_set = false;
          state_.paused = false;
        }
      }
    }
    if (result.accepted) {
      if (request.command == "start_mapping") {
        state_.mode = "mapping";
        state_.map_loaded = false;
        state_.task_stream_active = true;
        state_.trajectory.clear();
        state_.global_points.clear();
        state_.global_source_point_count = 0;
        state_.global_frame.clear();
        state_.global_map_topic_received = false;
        state_.global_map_topic_name.clear();
        ++state_.global_sequence;
      } else if (request.command == "load_map" ||
                 request.command == "initialize_pose") {
        state_.mode = "localizing";
        state_.map_loaded = true;
        state_.task_stream_active = true;
        state_.map_name = request.map_name;
        state_.map_address = SafeMapPath(request.map_name);
        state_.trajectory.clear();
        if (request.command == "load_map") {
          if (state_.mock) {
            // Mirror the real robot's one-shot global-map publication without
            // discarding the deterministic mock cloud.
            state_.global_map_topic_received = true;
            state_.global_map_topic_name = request.map_name;
            ++state_.global_sequence;
          } else {
            // Unitree publishes global_map only once when localization starts.
            // Drop the previous map before releasing this mutex so the queued
            // one-shot sample from API 1804 becomes the only visible map.
            state_.global_points.clear();
            state_.global_source_point_count = 0;
            state_.global_frame.clear();
            state_.global_map_topic_received = false;
            state_.global_map_topic_name = request.map_name;
            ++state_.global_sequence;
          }
        }
      } else if (request.command == "navigate") {
        state_.mode = "navigating";
        state_.task_stream_active = true;
        state_.trajectory.clear();
        state_.target = request.pose;
        state_.target_set = true;
        state_.arrived = false;
        state_.paused = false;
        state_.low_obstacle_pause_latched = false;
        state_.progress = 0.0;
      } else if (request.command == "pause_navigation") {
        state_.mode = "paused";
        state_.paused = true;
      } else if (request.command == "resume_navigation") {
        state_.mode = "navigating";
        state_.paused = false;
        state_.low_obstacle_pause_latched = false;
        state_.task_stream_active = true;
      } else if (request.command == "cancel_navigation") {
        // Unitree exposes no dedicated cancel RPC. API 1201 stops motion;
        // after it succeeds, discard the active Web goal/route while keeping
        // localization and the loaded map alive for a future new goal.
        state_.mode = "localizing";
        state_.target_set = false;
        state_.paused = false;
        state_.arrived = false;
        state_.progress = 0.0;
        state_.controller = "cancelled";
        state_.low_obstacle_pause_latched = false;
        state_.task_stream_active = true;
        state_.trajectory.clear();
      } else if (request.command == "finish_mapping") {
        state_.mode = "ready";
        state_.map_loaded = false;
        state_.task_stream_active = false;
        state_.map_name = request.map_name;
        state_.map_address = SafeMapPath(request.map_name);
        state_.global_map_topic_received = false;
        state_.global_map_topic_name.clear();
        const std::string download_path =
            SafeMapDownloadPath(request.map_name);
        if (!download_path.empty() &&
            !WriteWebMapPcd(download_path, state_.global_points)) {
          std::cerr << "[SLAM] official map saved but Web-visible PCD copy failed: "
                    << download_path << '\n';
        }
        state_.trajectory.clear();
        state_.live_points.clear();
        state_.source_point_count = 0;
        last_slam_cloud_ = Clock::time_point{};
      } else if (request.command == "stop_slam") {
        state_.mode = "stopped";
        state_.map_loaded = false;
        state_.task_stream_active = false;
        state_.trajectory.clear();
        state_.live_points.clear();
        state_.source_point_count = 0;
        state_.global_points.clear();
        state_.global_source_point_count = 0;
        state_.global_frame.clear();
        state_.global_map_topic_received = false;
        state_.global_map_topic_name.clear();
        ++state_.global_sequence;
        state_.target_set = false;
        last_slam_cloud_ = Clock::time_point{};
      }
    }
    state_.last_request_key = request.request_key;
    state_.last_result = result;
    ++state_.sequence;
    return result;
  }

  std::string SerializeStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["schema_version"] = 1;
    root["initialized"] = state_.initialized;
    root["mock"] = state_.mock;
    root["navigation_enabled"] = state_.navigation_enabled;
    root["initialization_error"] = state_.initialization_error;
    Json::Value services(Json::objectValue);
    services["lidar_driver"]["enabled"] = state_.lidar_driver_enabled;
    services["lidar_driver"]["api_result"] =
        state_.lidar_driver_api_result;
    services["lidar_driver"]["status_raw"] =
        state_.lidar_driver_status_raw;
    services["unitree_slam"]["enabled"] = state_.unitree_slam_enabled;
    services["unitree_slam"]["api_result"] =
        state_.unitree_slam_api_result;
    services["unitree_slam"]["status_raw"] =
        state_.unitree_slam_status_raw;
    services["error"] = state_.service_error;
    root["services"] = std::move(services);
    root["mode"] = state_.mode;
    root["error_code"] = state_.error_code;
    root["info"] = state_.info;
    root["pose_received"] = state_.pose_received;
    root["pose_age_ms"] = Json::Int64(AgeMs(state_.pose_received, state_.pose_time));
    root["pose"] = PoseJson(state_.pose);
    root["target_set"] = state_.target_set;
    root["target"] = PoseJson(state_.target);
    root["arrived"] = state_.arrived;
    root["paused"] = state_.paused;
    root["obstacle"] = state_.obstacle || state_.low_obstacle;
    root["slam_obstacle"] = state_.obstacle;
    root["low_obstacle"] = state_.low_obstacle;
    root["low_obstacle_range_m"] = state_.low_obstacle_range_m;
    root["low_obstacle_support_points"] =
        Json::UInt64(state_.low_obstacle_support_points);
    root["low_obstacle_safety_error"] = state_.low_obstacle_safety_error;
    root["progress"] = state_.progress;
    root["controller"] = state_.controller;
    root["map_loaded"] = state_.map_loaded;
    root["map_name"] = state_.map_name;
    root["map_address"] = state_.map_address;
    root["point_source"] = state_.point_source;
    root["point_frame"] = state_.point_frame;
    root["live_points"] = Json::UInt64(state_.live_points.size());
    root["live_source_points"] = Json::UInt64(state_.source_point_count);
    root["global_frame"] = state_.global_frame;
    root["global_points"] = Json::UInt64(state_.global_points.size());
    root["global_source_points"] =
        Json::UInt64(state_.global_source_point_count);
    root["global_sequence"] = Json::UInt64(state_.global_sequence);
    root["trajectory_points"] = Json::UInt64(state_.trajectory.size());
    root["sequence"] = Json::UInt64(state_.sequence);
    root["raw_info"] = state_.raw_info;
    root["raw_key_info"] = state_.raw_key_info;
    Json::Value lidar_inputs(Json::objectValue);
    lidar_inputs["point_cloud_age_ms"] =
        Json::Int64(AtomicAgeMs(raw_lidar_last_ms_));
    lidar_inputs["imu_age_ms"] =
        Json::Int64(AtomicAgeMs(raw_lidar_imu_last_ms_));
    lidar_inputs["ready"] =
        state_.mock ||
        (lidar_inputs["point_cloud_age_ms"].asInt64() >= 0 &&
         lidar_inputs["point_cloud_age_ms"].asInt64() <= kLidarInputFreshnessMs &&
         lidar_inputs["imu_age_ms"].asInt64() >= 0 &&
         lidar_inputs["imu_age_ms"].asInt64() <= kLidarInputFreshnessMs);
    root["lidar_inputs"] = std::move(lidar_inputs);
    Json::Value topics(Json::objectValue);
    topics["raw_lidar_points"] = "rt/utlidar/cloud_livox_mid360";
    topics["raw_lidar_imu"] = "rt/utlidar/imu_livox_mid360";
    topics["mapping_points"] = "rt/unitree/slam_mapping/points";
    topics["mapping_odom"] = "rt/unitree/slam_mapping/odom";
    topics["relocation_points"] = "rt/unitree/slam_relocation/points";
    topics["relocation_odom"] = "rt/unitree/slam_relocation/odom";
    topics["global_map"] = "rt/unitree/slam_relocation/global_map";
    topics["info"] = "rt/slam_info";
    topics["key_info"] = "rt/slam_key_info";
    root["topics"] = std::move(topics);
    Json::Value point_filter(Json::objectValue);
    point_filter["transport_encoding"] = "base64_u16le_xyz";
    point_filter["live_max_points"] = Json::UInt64(kMaximumLivePoints);
    point_filter["global_max_points"] = Json::UInt64(kMaximumGlobalPoints);
    point_filter["raw_voxel_m"] = kRawLiveFilter.voxel_size_m;
    point_filter["slam_voxel_m"] = kSlamLiveFilter.voxel_size_m;
    point_filter["global_voxel_m"] = kGlobalMapFilter.voxel_size_m;
    point_filter["live_isolated_voxel_filter"] = true;
    point_filter["global_isolated_voxel_filter"] = false;
    root["point_filter"] = std::move(point_filter);
    Json::Value low_obstacle_safety(Json::objectValue);
    low_obstacle_safety["source"] = "raw_mid360";
    low_obstacle_safety["forward_min_m"] = kLowObstacleMinForwardM;
    low_obstacle_safety["forward_max_m"] = kLowObstacleMaxForwardM;
    low_obstacle_safety["half_width_m"] = kLowObstacleHalfWidthM;
    low_obstacle_safety["minimum_relief_m"] = kLowObstacleMinReliefM;
    low_obstacle_safety["action"] = "pause_navigation";
    low_obstacle_safety["physical_blind_area_remains"] = true;
    root["low_obstacle_safety"] = std::move(low_obstacle_safety);
    return JsonText(root);
  }

  std::string SerializeFrame() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["schema_version"] = 3;
    root["sequence"] = Json::UInt64(state_.sequence);
    root["pose_received"] = state_.pose_received;
    root["pose"] = PoseJson(state_.pose);
    root["target_set"] = state_.target_set;
    root["target"] = PoseJson(state_.target);
    root["point_source"] = state_.point_source;
    root["point_frame"] = state_.point_frame;
    root["global_frame"] = state_.global_frame;
    AddPackedPoints(root, "live_points", state_.live_points);
    AddPackedPoints(root, "trajectory", state_.trajectory);
    return JsonText(root);
  }

  std::string SerializeGlobalMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["schema_version"] = 3;
    root["global_sequence"] = Json::UInt64(state_.global_sequence);
    root["global_frame"] = state_.global_frame;
    root["source_points"] = Json::UInt64(state_.global_source_point_count);
    AddPackedPoints(root, "points", state_.global_points);
    return JsonText(root);
  }

  bool GetTopicGlobalMapPcd(const std::string& map_name,
                            std::string& body) const {
    if (SafeMapPath(map_name).empty()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.global_map_topic_received ||
        state_.global_map_topic_name != map_name ||
        state_.global_points.empty()) {
      return false;
    }
    body = BuildWebMapPcd(state_.global_points);
    return !body.empty();
  }

  std::string SerializeNavigationScene() const {
    return navigation_bridge_->SerializeScene();
  }

  std::string SerializeNavigationTopics() const {
    return navigation_bridge_->SerializeTopics();
  }

  bool ConfigureNavigationTopic(const std::string& kind,
                                const std::string& topic,
                                std::string& error) {
    return navigation_bridge_->ConfigureTopic(kind, topic, error);
  }

 private:
  void RefreshServiceStatesLocked() {
    std::vector<unitree::robot::b2::ServiceState> services;
    const std::int32_t result = robot_state_client_->ServiceList(services);
    if (result != 0) {
      state_.service_error = "service_list_failed:" + std::to_string(result);
      return;
    }
    for (const auto& service : services) {
      if (service.name == "lidar_driver") {
        state_.lidar_driver_status_raw = service.status;
        state_.lidar_driver_enabled = service.status == 0;
      } else if (service.name == "unitree_slam") {
        state_.unitree_slam_status_raw = service.status;
        state_.unitree_slam_enabled = service.status == 0;
      }
    }
    state_.service_error.clear();
  }

  bool SwitchServiceLocked(const std::string& name, bool enable) {
    std::int32_t status = -1;
    const std::int32_t result =
        robot_state_client_->ServiceSwitch(name, enable ? 1 : 0, status);
    const bool expected = enable ? status == 0 : status == 1;
    if (name == "lidar_driver") {
      state_.lidar_driver_api_result = result;
      state_.lidar_driver_status_raw = status;
      state_.lidar_driver_enabled = result == 0 && status == 0;
    } else {
      state_.unitree_slam_api_result = result;
      state_.unitree_slam_status_raw = status;
      state_.unitree_slam_enabled = result == 0 && status == 0;
    }
    if (result != 0 || !expected) {
      state_.service_error = name + (enable ? "_start_failed:" : "_stop_failed:") +
                             std::to_string(result) + ":" +
                             std::to_string(status);
      return false;
    }
    state_.service_error.clear();
    return true;
  }

  bool WaitForServiceActiveLocked(const std::string& name) {
    // ServiceSwitch can acknowledge before the process has reached its
    // running state. Confirm RobotState readiness before calling slam_operate.
    for (int attempt = 0; attempt < 50; ++attempt) {
      RefreshServiceStatesLocked();
      const bool enabled = name == "lidar_driver"
                               ? state_.lidar_driver_enabled
                               : state_.unitree_slam_enabled;
      if (enabled) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    state_.service_error = name + "_start_timeout";
    return false;
  }

  bool EnsureServiceActiveLocked(const std::string& name, bool& started) {
    RefreshServiceStatesLocked();
    const bool enabled = name == "lidar_driver"
                             ? state_.lidar_driver_enabled
                             : state_.unitree_slam_enabled;
    if (enabled) return true;
    if (!SwitchServiceLocked(name, true)) return false;
    started = true;
    return WaitForServiceActiveLocked(name);
  }

  bool WaitForLidarInputsLocked() {
    for (int attempt = 0; attempt < 50; ++attempt) {
      const auto cloud_age = AtomicAgeMs(raw_lidar_last_ms_);
      const auto imu_age = AtomicAgeMs(raw_lidar_imu_last_ms_);
      if (cloud_age >= 0 && cloud_age <= kLidarInputFreshnessMs &&
          imu_age >= 0 && imu_age <= kLidarInputFreshnessMs) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << "[SLAM] lidar input wait timeout"
              << " point_cloud_age_ms=" << AtomicAgeMs(raw_lidar_last_ms_)
              << " imu_age_ms=" << AtomicAgeMs(raw_lidar_imu_last_ms_)
              << '\n';
    return false;
  }

  template <typename T>
  static void CloseSubscriber(std::shared_ptr<T>& subscriber) {
    if (subscriber) {
      subscriber->CloseChannel();
      subscriber.reset();
    }
  }

  static bool ValidatePose(const PoseData& pose) {
    if (!IsFinite(pose.x) || !IsFinite(pose.y) || !IsFinite(pose.z) ||
        !IsFinite(pose.qx) || !IsFinite(pose.qy) || !IsFinite(pose.qz) ||
        !IsFinite(pose.qw) || std::abs(pose.x) > kMaximumMapCoordinateM ||
        std::abs(pose.y) > kMaximumMapCoordinateM || std::abs(pose.z) > 5.0) {
      return false;
    }
    const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy +
                                  pose.qz * pose.qz + pose.qw * pose.qw);
    return norm >= 0.99 && norm <= 1.01;
  }

  static void AddPose(Json::Value& target, const PoseData& pose) {
    target["x"] = pose.x;
    target["y"] = pose.y;
    target["z"] = pose.z;
    target["q_x"] = pose.qx;
    target["q_y"] = pose.qy;
    target["q_z"] = pose.qz;
    target["q_w"] = pose.qw;
  }

  static void AddPoints(Json::Value& target,
                        const std::vector<PointSample>& points) {
    target = Json::Value(Json::arrayValue);
    target.resize(static_cast<Json::ArrayIndex>(points.size()));
    for (Json::ArrayIndex index = 0; index < target.size(); ++index) {
      const auto& point = points[index];
      Json::Value packed(Json::arrayValue);
      packed.append(point.x);
      packed.append(point.y);
      packed.append(point.z);
      packed.append(point.intensity);
      target[index] = std::move(packed);
    }
  }

  static std::string Base64(const std::vector<std::uint8_t>& input) {
    static constexpr char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string output;
    output.reserve((input.size() + 2) / 3 * 4);
    for (std::size_t index = 0; index < input.size(); index += 3) {
      const std::size_t remaining = input.size() - index;
      const std::uint32_t a = input[index];
      const std::uint32_t b = remaining > 1 ? input[index + 1] : 0;
      const std::uint32_t c = remaining > 2 ? input[index + 2] : 0;
      const std::uint32_t packed = (a << 16U) | (b << 8U) | c;
      output.push_back(table[(packed >> 18U) & 63U]);
      output.push_back(table[(packed >> 12U) & 63U]);
      output.push_back(remaining > 1 ? table[(packed >> 6U) & 63U] : '=');
      output.push_back(remaining > 2 ? table[packed & 63U] : '=');
    }
    return output;
  }

  static void AddPackedPoints(Json::Value& root, const char* name,
                              const std::vector<PointSample>& points) {
    std::array<float, 3> minimum{{0.0F, 0.0F, 0.0F}};
    std::array<float, 3> scale{{0.0F, 0.0F, 0.0F}};
    if (!points.empty()) {
      std::array<float, 3> maximum{{points.front().x, points.front().y,
                                    points.front().z}};
      minimum = maximum;
      for (const auto& point : points) {
        minimum[0] = std::min(minimum[0], point.x);
        minimum[1] = std::min(minimum[1], point.y);
        minimum[2] = std::min(minimum[2], point.z);
        maximum[0] = std::max(maximum[0], point.x);
        maximum[1] = std::max(maximum[1], point.y);
        maximum[2] = std::max(maximum[2], point.z);
      }
      for (std::size_t axis = 0; axis < 3; ++axis) {
        scale[axis] = (maximum[axis] - minimum[axis]) / 65535.0F;
      }
    }

    std::vector<std::uint8_t> bytes(points.size() * 3 * sizeof(std::uint16_t));
    for (std::size_t index = 0; index < points.size(); ++index) {
      const std::array<float, 3> xyz{{
          points[index].x, points[index].y, points[index].z}};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const float normalized = scale[axis] > 0.0F
                                     ? (xyz[axis] - minimum[axis]) / scale[axis]
                                     : 0.0F;
        const auto quantized = static_cast<std::uint16_t>(std::lround(
            std::clamp(normalized, 0.0F, 65535.0F)));
        const std::size_t offset = (index * 3 + axis) * 2;
        bytes[offset] = static_cast<std::uint8_t>(quantized & 0xFFU);
        bytes[offset + 1] = static_cast<std::uint8_t>(quantized >> 8U);
      }
    }

    Json::Value origin(Json::arrayValue);
    Json::Value quantization_scale(Json::arrayValue);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      origin.append(minimum[axis]);
      quantization_scale.append(scale[axis]);
    }
    const std::string prefix(name);
    root[prefix + "_encoding"] = "base64_u16le_xyz";
    root[prefix + "_count"] = Json::UInt64(points.size());
    root[prefix + "_origin"] = std::move(origin);
    root[prefix + "_scale"] = std::move(quantization_scale);
    root[prefix + "_data"] = Base64(bytes);
  }

  void SubscribeLocked() {
    slam_info_ = MakeSubscriber<StringMessage>(
        "rt/slam_info", [this](const StringMessage& message) {
          HandleInfo(message.data(), false);
        });
    slam_key_info_ = MakeSubscriber<StringMessage>(
        "rt/slam_key_info", [this](const StringMessage& message) {
          HandleInfo(message.data(), true);
        });
    mapping_points_ = MakeSubscriber<PointCloudMessage>(
        "rt/unitree/slam_mapping/points",
        [this](const PointCloudMessage& message) {
          HandleCloud(message, "mapping", false);
        });
    raw_lidar_points_ = MakeSubscriber<PointCloudMessage>(
        "rt/utlidar/cloud_livox_mid360",
        [this](const PointCloudMessage& message) {
          HandleRawLidarCloud(message);
        });
    raw_lidar_imu_ = MakeSubscriber<ImuMessage>(
        "rt/utlidar/imu_livox_mid360",
        [this](const ImuMessage&) {
          raw_lidar_imu_last_ms_.store(MonotonicMs(), std::memory_order_relaxed);
        });
    relocation_points_ = MakeSubscriber<PointCloudMessage>(
        "rt/unitree/slam_relocation/points",
        [this](const PointCloudMessage& message) {
          HandleCloud(message, "relocation", false);
        });
    global_map_ = MakeSubscriber<PointCloudMessage>(
        "rt/unitree/slam_relocation/global_map",
        [this](const PointCloudMessage& message) {
          HandleCloud(message, "global_map", true);
        });
    mapping_odom_ = MakeSubscriber<OdometryMessage>(
        "rt/unitree/slam_mapping/odom",
        [this](const OdometryMessage& message) {
          HandleOdometry(message, "mapping");
        });
    relocation_odom_ = MakeSubscriber<OdometryMessage>(
        "rt/unitree/slam_relocation/odom",
        [this](const OdometryMessage& message) {
          HandleOdometry(message, "relocation");
        });
  }

  template <typename Message, typename Callback>
  static std::shared_ptr<unitree::robot::ChannelSubscriber<Message>>
  MakeSubscriber(const std::string& topic, Callback callback) {
    auto subscriber = std::make_shared<
        unitree::robot::ChannelSubscriber<Message>>(topic);
    subscriber->InitChannel(
        [callback = std::move(callback)](const void* data) {
          callback(*static_cast<const Message*>(data));
        },
        1);
    return subscriber;
  }

  void HandleCloud(const PointCloudMessage& message,
                   const std::string& source, bool global) {
    auto decoded = DecodePointCloud(
        message,
        global ? kMaximumGlobalDecodePoints : kMaximumLiveDecodePoints);
    if (!decoded.valid) return;

    std::vector<PointSample> mapping_points;
    if (!global && source == "mapping") {
      mapping_points = FilterPointCloudForWeb(decoded.points, kGlobalMapFilter);
    }
    decoded.points = FilterPointCloudForWeb(
        decoded.points, global ? kGlobalMapFilter : kSlamLiveFilter);
    if (decoded.points.empty() && mapping_points.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (global) {
      // A retained/queued global-map sample must not restore walls after the
      // operator explicitly closed SLAM. A later mapping/localization command
      // changes the mode before its new global samples arrive.
      if (state_.mode == "stopped") return;
      state_.global_points = std::move(decoded.points);
      state_.global_frame = decoded.frame_id;
      state_.global_source_point_count = decoded.source_points;
      state_.global_map_topic_received = true;
      state_.global_map_topic_name = state_.map_name;
      ++state_.global_sequence;
      const std::string path = SafeMapDownloadPath(state_.map_name);
      if (!path.empty() && !WriteWebMapPcd(path, state_.global_points)) {
        std::cerr << "[SLAM] failed to create Web-visible map copy: " << path
                  << '\n';
      }
    } else {
      if (!state_.task_stream_active) return;
      if (source == "mapping" && !mapping_points.empty()) {
        std::vector<PointSample> accumulated;
        accumulated.reserve(state_.global_points.size() + mapping_points.size());
        accumulated.insert(accumulated.end(), state_.global_points.begin(),
                           state_.global_points.end());
        accumulated.insert(accumulated.end(), mapping_points.begin(),
                           mapping_points.end());
        state_.global_points =
            FilterPointCloudForWeb(accumulated, kGlobalMapFilter);
        state_.global_frame = decoded.frame_id;
        state_.global_source_point_count += decoded.source_points;
        ++state_.global_sequence;
      }
      if (!decoded.points.empty()) {
        state_.live_points = std::move(decoded.points);
        state_.point_frame = std::move(decoded.frame_id);
        state_.source_point_count = decoded.source_points;
        state_.point_source = source;
        last_slam_cloud_ = Clock::now();
      }
    }
    ++state_.sequence;
  }

  void HandleRawLidarCloud(const PointCloudMessage& message) {
    raw_lidar_last_ms_.store(MonotonicMs(), std::memory_order_relaxed);
    auto decoded = DecodePointCloud(message, kMaximumLiveDecodePoints);
    if (!decoded.valid) return;
    // The G1 Mid-360 is mounted upside-down. Convert livox_frame into the
    // upright robot display frame with a proper 180-degree rotation about X
    // (x, y, z -> x, -y, -z). Applying a screen-space mirror would reverse
    // handedness and make navigation geometry misleading.
    for (auto& point : decoded.points) {
      point.y = -point.y;
      point.z = -point.z;
    }
    const auto low_obstacle = DetectLowObstacleForSafety(decoded.points);
    decoded.points = FilterPointCloudForWeb(decoded.points, kRawLiveFilter);

    std::lock_guard<std::mutex> lock(mutex_);
    state_.low_obstacle = low_obstacle.detected;
    state_.low_obstacle_range_m =
        low_obstacle.detected ? low_obstacle.nearest_range_m : 0.0F;
    state_.low_obstacle_support_points = low_obstacle.support_points;
    if (!low_obstacle.detected) {
      state_.low_obstacle_pause_latched = false;
      state_.low_obstacle_safety_error.clear();
    } else if (!state_.mock && state_.mode == "navigating" &&
               !state_.paused && !state_.low_obstacle_pause_latched && client_ &&
               MonotonicMs() - state_.low_obstacle_last_pause_attempt_ms >= 500) {
      state_.low_obstacle_last_pause_attempt_ms = MonotonicMs();
      Json::Value parameter(Json::objectValue);
      parameter["data"] = Json::Value(Json::objectValue);
      std::string response;
      const auto api_result = client_->Invoke(kApiPause, JsonText(parameter), response);
      bool paused = api_result == 0;
      Json::Value response_json;
      if (ParseJson(response, response_json) && response_json.isMember("succeed")) {
        paused = paused && response_json["succeed"].asBool();
      }
      if (paused) {
        state_.paused = true;
        state_.mode = "paused";
        state_.low_obstacle_pause_latched = true;
        state_.low_obstacle_safety_error.clear();
      } else {
        state_.low_obstacle_safety_error = "low_obstacle_pause_failed";
      }
    }

    if (decoded.points.empty()) {
      ++state_.sequence;
      return;
    }
    // Prefer task-specific map-frame data while it is flowing. Otherwise the
    // always-on Mid-360 topic supplies the default live preview without
    // starting mapping or localization.
    if (last_slam_cloud_ != Clock::time_point{} &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - last_slam_cloud_).count() < 500) {
      ++state_.sequence;
      return;
    }
    state_.live_points = std::move(decoded.points);
    state_.point_frame = std::move(decoded.frame_id);
    state_.source_point_count = decoded.source_points;
    state_.point_source = "utlidar_mid360";
    ++state_.sequence;
  }

  void HandleOdometry(const OdometryMessage& message,
                      const std::string& source) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.task_stream_active) return;
    state_.pose = ToPose(message);
    state_.pose_received = true;
    state_.pose_time = Clock::now();
    state_.point_source = source;
    AppendTrajectoryLocked(state_.pose);
    ++state_.sequence;
  }

  void HandleInfo(const std::string& text, bool key_info) {
    Json::Value root;
    std::lock_guard<std::mutex> lock(mutex_);
    if (key_info) state_.raw_key_info = text;
    else state_.raw_info = text;
    if (!ParseJson(text, root)) {
      state_.info = "invalid_slam_json";
      ++state_.sequence;
      return;
    }
    if (!state_.task_stream_active) {
      ++state_.sequence;
      return;
    }
    state_.error_code = root.get("errorCode", 0).asInt();
    state_.info = root.get("info", "").asString();
    const std::string type = root.get("type", "").asString();
    const auto& data = root["data"];
    if ((type == "pos_info" || type == "mapping_info") && data.isObject()) {
      const auto& pose = data["currentPose"];
      if (pose.isObject()) {
        state_.pose.x = pose.get("x", 0.0).asDouble();
        state_.pose.y = pose.get("y", 0.0).asDouble();
        state_.pose.z = pose.get("z", 0.0).asDouble();
        state_.pose.qx = pose.get("q_x", 0.0).asDouble();
        state_.pose.qy = pose.get("q_y", 0.0).asDouble();
        state_.pose.qz = pose.get("q_z", 0.0).asDouble();
        state_.pose.qw = pose.get("q_w", 1.0).asDouble();
        state_.pose_received = true;
        state_.pose_time = Clock::now();
        AppendTrajectoryLocked(state_.pose);
      }
      if (type == "mapping_info") {
        state_.mode = "mapping";
      } else if (!state_.target_set && state_.mode != "arrived") {
        state_.mode = "localizing";
      }
      const std::string reported_map_name =
          CanonicalReportedMapFilename(data.get("pcdName", "").asString());
      if (!reported_map_name.empty()) state_.map_name = reported_map_name;
      state_.map_address = data.get("address", "").asString();
      state_.map_loaded = type == "pos_info" && !state_.map_name.empty();
    } else if (type == "ctrl_info" && data.isObject()) {
      state_.arrived = data.get("is_arrived", false).asBool();
      const auto& machine = data["stateMachine"];
      const bool reported_paused =
          machine.get("isPause", false).asBool();
      state_.controller = machine.get("ctrName", "").asString();
      // API 1201/1202 success is authoritative for Web-requested pause/resume.
      // A ctrl_info sample queued before that RPC returned must not undo the
      // new state. A reported pause may still pause an active Web goal, while
      // clearing pause is owned by an accepted resume command.
      if (state_.target_set) {
        if (reported_paused) state_.paused = true;
        state_.mode = state_.paused
                          ? "paused"
                          : (state_.arrived ? "arrived" : "navigating");
      } else {
        state_.paused = false;
      }
      state_.obstacle = data["obsInfo"].get("state", false).asBool();
      state_.progress =
          data["progress"].get("completion_percentage", 0.0).asDouble();
    } else if (type == "task_result" && data.isObject()) {
      state_.arrived = data.get("is_arrived", false).asBool();
      if (state_.arrived) {
        state_.mode = "arrived";
        state_.target_set = false;
        state_.paused = false;
        state_.progress = 1.0;
        // Arrival ends only the current navigation goal. Keep relocation
        // point-cloud/odometry streams alive so the loaded map, fresh pose and
        // next navigation goal remain available without reloading the map.
        state_.task_stream_active = true;
      }
    }
    ++state_.sequence;
  }

  void AppendTrajectoryLocked(const PoseData& pose) {
    if (!state_.trajectory.empty()) {
      const auto& last = state_.trajectory.back();
      if (std::hypot(pose.x - last.x, pose.y - last.y) < 0.025) return;
    }
    state_.trajectory.push_back(
        {static_cast<float>(pose.x), static_cast<float>(pose.y),
         static_cast<float>(pose.z), 0.0F});
    if (state_.trajectory.size() > kMaximumTrajectoryPoints) {
      state_.trajectory.erase(
          state_.trajectory.begin(),
          state_.trajectory.begin() + state_.trajectory.size() / 4);
    }
  }

  void PopulateMockCloudsLocked() {
    // Keep the initial demo scene animated until the first explicit terminal
    // command; stop/finish then disable the stream just like the real robot.
    state_.task_stream_active = true;
    state_.global_points.clear();
    for (int x = -80; x <= 80; ++x) {
      for (int y = -60; y <= 60; ++y) {
        if ((x + y) % 3 != 0) continue;
        const float xf = x * 0.06F;
        const float yf = y * 0.06F;
        state_.global_points.push_back({xf, yf, -0.02F, 15.0F});
        if (x == -80 || x == 80 || y == -60 || y == 60) {
          for (int z = 0; z < 18; ++z) {
            state_.global_points.push_back(
                {xf, yf, z * 0.06F, 80.0F + z});
          }
        }
      }
    }
    for (int step = 0; step < 1400; ++step) {
      const float angle = step * 0.018F;
      const float radius = 1.0F + 0.0012F * step;
      state_.global_points.push_back(
          {1.4F + std::cos(angle) * radius,
           -0.4F + std::sin(angle) * radius,
           0.25F + 0.32F * std::sin(angle * 0.45F), 160.0F});
    }
    state_.global_frame = "map";
    state_.global_source_point_count = state_.global_points.size();
    ++state_.global_sequence;
    state_.point_source = "mock_relocation";
    state_.point_frame = "map";
    state_.map_name = "test1.pcd";
    state_.map_address = "/home/unitree/test1.pcd";
    state_.pose_received = true;
    state_.pose_time = Clock::now();
  }

  void MockLoop() {
    const auto started = Clock::now();
    while (running_.load()) {
      const double elapsed =
          std::chrono::duration<double>(Clock::now() - started).count();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_.task_stream_active) {
          if (state_.mode == "navigating" && state_.target_set &&
            !state_.paused) {
          const double dx = state_.target.x - state_.pose.x;
          const double dy = state_.target.y - state_.pose.y;
          const double distance = std::hypot(dx, dy);
          if (distance < 0.04) {
            state_.pose = state_.target;
            state_.arrived = true;
            state_.progress = 1.0;
            state_.mode = "arrived";
            state_.target_set = false;
          } else {
            const double step = std::min(0.055, distance);
            state_.pose.x += dx / distance * step;
            state_.pose.y += dy / distance * step;
            state_.progress = std::min(0.98, state_.progress + 0.018);
          }
          } else if (state_.mode == "ready" || state_.mode == "mapping" ||
                     state_.mode == "localizing") {
          state_.pose.x = std::cos(elapsed * 0.17) * 0.7;
          state_.pose.y = std::sin(elapsed * 0.17) * 0.45;
          const double yaw = elapsed * 0.17 + 1.57079632679;
          state_.pose.qz = std::sin(yaw / 2.0);
          state_.pose.qw = std::cos(yaw / 2.0);
          }
          state_.pose_time = Clock::now();
          state_.live_points.clear();
          for (int i = 0; i < 1700; ++i) {
          const float angle = i * 0.043F + static_cast<float>(elapsed * 0.12);
          const float radius = 1.1F + static_cast<float>((i % 47) * 0.045);
          state_.live_points.push_back(
              {static_cast<float>(state_.pose.x) + std::cos(angle) * radius,
               static_cast<float>(state_.pose.y) + std::sin(angle) * radius,
               0.06F + static_cast<float>((i % 23) * 0.035),
               static_cast<float>(i % 255)});
          }
          state_.source_point_count = state_.live_points.size();
          AppendTrajectoryLocked(state_.pose);
          ++state_.sequence;
        } else if (state_.lidar_driver_enabled) {
          // Mock the always-on raw Mid-360 fallback used by the real service.
          state_.live_points.clear();
          for (int i = 0; i < 1700; ++i) {
            const float angle = i * 0.043F + static_cast<float>(elapsed * 0.12);
            const float radius = 1.1F + static_cast<float>((i % 47) * 0.045);
            state_.live_points.push_back(
                {std::cos(angle) * radius, std::sin(angle) * radius,
                 0.06F + static_cast<float>((i % 23) * 0.035),
                 static_cast<float>(i % 255)});
          }
          state_.source_point_count = state_.live_points.size();
          state_.point_source = "utlidar_mid360";
          state_.point_frame = "livox_frame";
          ++state_.sequence;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void ApplyMockCommandLocked(const PerceptionRequest& request) {
    if (request.command == "start_mapping") {
      state_.map_loaded = false;
      state_.lidar_driver_enabled = true;
      state_.unitree_slam_enabled = true;
      state_.lidar_driver_status_raw = 0;
      state_.unitree_slam_status_raw = 0;
      state_.mode = "mapping";
      state_.task_stream_active = true;
      state_.trajectory.clear();
    } else if (request.command == "finish_mapping") {
      state_.mode = "ready";
      state_.map_loaded = false;
      state_.task_stream_active = false;
      state_.map_name = request.map_name;
      state_.map_address = SafeMapPath(request.map_name);
      state_.trajectory.clear();
      state_.live_points.clear();
    } else if (request.command == "load_map" ||
               request.command == "initialize_pose") {
      state_.lidar_driver_enabled = true;
      state_.unitree_slam_enabled = true;
      state_.map_loaded = true;
      state_.mode = "localizing";
      state_.task_stream_active = true;
      state_.pose = request.pose;
      state_.map_name = request.map_name;
      state_.map_address = SafeMapPath(request.map_name);
      state_.trajectory.clear();
    } else if (request.command == "navigate") {
      state_.mode = "navigating";
      state_.task_stream_active = true;
      state_.trajectory.clear();
      state_.target = request.pose;
      state_.target_set = true;
      state_.arrived = false;
      state_.paused = false;
      state_.progress = 0.0;
      state_.controller = "mock";
    } else if (request.command == "pause_navigation") {
      state_.paused = true;
      state_.mode = "paused";
    } else if (request.command == "resume_navigation") {
      state_.paused = false;
      state_.task_stream_active = true;
      state_.mode = state_.target_set ? "navigating" : "localizing";
    } else if (request.command == "cancel_navigation") {
      state_.paused = false;
      state_.target_set = false;
      state_.arrived = false;
      state_.progress = 0.0;
      state_.controller = "cancelled";
      state_.task_stream_active = true;
      state_.mode = "localizing";
      state_.trajectory.clear();
    } else if (request.command == "stop_slam") {
      state_.map_loaded = false;
      state_.lidar_driver_enabled = true;
      state_.unitree_slam_enabled = false;
      state_.lidar_driver_status_raw = 0;
      state_.unitree_slam_status_raw = 1;
      state_.mode = "stopped";
      state_.task_stream_active = false;
      state_.target_set = false;
      state_.paused = false;
      state_.trajectory.clear();
      state_.live_points.clear();
    }
  }

  mutable std::mutex mutex_;
  PerceptionState state_;
  std::unique_ptr<SlamClient> client_;
  std::unique_ptr<RosNavigationBridge> navigation_bridge_;
  std::unique_ptr<unitree::robot::b2::RobotStateClient>
      robot_state_client_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<StringMessage>> slam_info_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<StringMessage>>
      slam_key_info_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<PointCloudMessage>>
      mapping_points_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<PointCloudMessage>>
      raw_lidar_points_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<ImuMessage>>
      raw_lidar_imu_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<PointCloudMessage>>
      relocation_points_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<PointCloudMessage>>
      global_map_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<OdometryMessage>>
      mapping_odom_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<OdometryMessage>>
      relocation_odom_;
  std::atomic<bool> running_{false};
  std::thread mock_thread_;
  bool lidar_started_by_web_{false};
  bool slam_started_by_web_{false};
  std::atomic<std::int64_t> raw_lidar_last_ms_{0};
  std::atomic<std::int64_t> raw_lidar_imu_last_ms_{0};
  Clock::time_point last_slam_cloud_{};
};

PerceptionService::PerceptionService(bool mock, bool navigation_enabled)
    : impl_(std::make_unique<Impl>(mock, navigation_enabled)) {}

PerceptionService::~PerceptionService() = default;

bool PerceptionService::Start(std::string& error) {
  return impl_->Start(error);
}

void PerceptionService::Stop() { impl_->Stop(); }

PerceptionResult PerceptionService::Submit(const PerceptionRequest& request) {
  return impl_->Submit(request);
}

std::string PerceptionService::SerializeStatus() const {
  return impl_->SerializeStatus();
}

std::string PerceptionService::SerializeFrame() const {
  return impl_->SerializeFrame();
}

std::string PerceptionService::SerializeGlobalMap() const {
  return impl_->SerializeGlobalMap();
}

bool PerceptionService::GetTopicGlobalMapPcd(const std::string& map_name,
                                             std::string& body) const {
  return impl_->GetTopicGlobalMapPcd(map_name, body);
}

std::string PerceptionService::SerializeNavigationScene() const {
  return impl_->SerializeNavigationScene();
}

std::string PerceptionService::SerializeNavigationTopics() const {
  return impl_->SerializeNavigationTopics();
}

bool PerceptionService::ConfigureNavigationTopic(const std::string& kind,
                                                 const std::string& topic,
                                                 std::string& error) {
  return impl_->ConfigureNavigationTopic(kind, topic, error);
}

}  // namespace g1_web
