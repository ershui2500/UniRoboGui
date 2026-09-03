#include "g1_web/ros_navigation_bridge.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json/json.h>

#ifdef G1_WEB_HAS_ROS2_NAV
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace g1_web {
namespace {

using Clock = std::chrono::steady_clock;

std::string JsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

std::string Base64(const std::vector<std::uint8_t>& input) {
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

std::string EnvironmentTopic(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  if (!value || value[0] != '/' || std::strlen(value) > 180) return fallback;
  return value;
}

std::string ReadSidecar(const char* path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

bool SafeTopic(const std::string& topic) {
  if (topic.empty() || topic.front() != '/' || topic.size() > 180) return false;
  return std::all_of(topic.begin(), topic.end(), [](unsigned char value) {
    return std::isalnum(value) || value == '/' || value == '_' || value == '-';
  });
}

struct PoseValue {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct GridValue {
  std::string frame;
  std::uint32_t width{0};
  std::uint32_t height{0};
  float resolution{0.0F};
  PoseValue origin;
  std::vector<std::uint8_t> data;
  std::uint64_t revision{0};
};

struct PathValue {
  std::string frame;
  std::vector<PoseValue> poses;
  std::uint64_t revision{0};
};

struct PointValue {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float weight{0.0F};
};

struct TransformValue {
  std::string parent;
  std::string child;
  PoseValue pose;
  bool is_static{false};
};

struct MarkerValue {
  std::string key;
  std::string frame;
  std::int32_t type{0};
  std::int32_t action{0};
  PoseValue pose;
  float sx{1.0F};
  float sy{1.0F};
  float sz{1.0F};
  float r{0.2F};
  float g{0.8F};
  float b{1.0F};
  float a{1.0F};
  std::string text;
  std::vector<PointValue> points;
};

struct TopicHealth {
  std::string topic;
  std::string type;
  std::uint64_t messages{0};
  Clock::time_point first{};
  Clock::time_point last{};
};

struct SceneState {
  bool connected{false};
  std::string error;
  std::string fixed_frame{"map"};
  GridValue map;
  GridValue global_costmap;
  GridValue local_costmap;
  PathValue global_path;
  PathValue local_path;
  std::vector<PointValue> particles;
  std::uint64_t particles_revision{0};
  PoseValue amcl_pose;
  std::vector<double> covariance;
  bool amcl_received{false};
  std::uint64_t amcl_revision{0};
  std::vector<PointValue> laser;
  std::uint64_t laser_revision{0};
  std::vector<PointValue> pointcloud;
  std::uint64_t pointcloud_revision{0};
  std::vector<PointValue> footprint;
  std::uint64_t footprint_revision{0};
  std::map<std::string, TransformValue> transforms;
  std::uint64_t tf_revision{0};
  std::map<std::string, double> joints;
  std::uint64_t joints_revision{0};
  std::map<std::string, MarkerValue> markers;
  std::uint64_t markers_revision{0};
  std::vector<std::pair<std::string, std::vector<std::string>>> discovered;
  Clock::time_point graph_updated{};
  std::map<std::string, TopicHealth> health;
  std::map<std::string, std::string> bindings;
  std::uint64_t sequence{0};
};

Json::Value PoseJson(const PoseValue& pose) {
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

Json::Value GridJson(const GridValue& grid) {
  Json::Value value(Json::objectValue);
  value["frame"] = grid.frame;
  value["width"] = grid.width;
  value["height"] = grid.height;
  value["resolution"] = grid.resolution;
  value["origin"] = PoseJson(grid.origin);
  value["encoding"] = "base64_u8";
  value["data"] = Base64(grid.data);
  value["revision"] = Json::UInt64(grid.revision);
  return value;
}

Json::Value PathJson(const PathValue& path) {
  Json::Value value(Json::objectValue);
  value["frame"] = path.frame;
  value["revision"] = Json::UInt64(path.revision);
  auto& poses = value["poses"];
  poses = Json::Value(Json::arrayValue);
  for (const auto& pose : path.poses) poses.append(PoseJson(pose));
  return value;
}

void AddPoints(Json::Value& output, const std::vector<PointValue>& points) {
  output = Json::Value(Json::arrayValue);
  output.resize(static_cast<Json::ArrayIndex>(points.size()));
  for (Json::ArrayIndex index = 0; index < output.size(); ++index) {
    Json::Value point(Json::arrayValue);
    point.append(points[index].x);
    point.append(points[index].y);
    point.append(points[index].z);
    point.append(points[index].weight);
    output[index] = std::move(point);
  }
}

void AddPackedPoints(Json::Value& root, const char* name,
                     const std::vector<PointValue>& points) {
  std::vector<std::uint8_t> bytes(points.size() * 4 * sizeof(float));
  for (std::size_t index = 0; index < points.size(); ++index) {
    const float values[4] = {points[index].x, points[index].y,
                             points[index].z, points[index].weight};
    std::memcpy(bytes.data() + index * 4 * sizeof(float), values,
                4 * sizeof(float));
  }
  root[std::string(name) + "_encoding"] = "base64_f32le_xyzw";
  root[std::string(name) + "_count"] = Json::UInt64(points.size());
  root[std::string(name) + "_data"] = Base64(bytes);
}

#ifdef G1_WEB_HAS_ROS2_NAV
PoseValue ConvertPose(const geometry_msgs::msg::Pose& pose) {
  return {pose.position.x, pose.position.y, pose.position.z,
          pose.orientation.x, pose.orientation.y, pose.orientation.z,
          pose.orientation.w};
}

PoseValue ConvertTransform(const geometry_msgs::msg::Transform& transform) {
  return {transform.translation.x, transform.translation.y,
          transform.translation.z, transform.rotation.x,
          transform.rotation.y, transform.rotation.z, transform.rotation.w};
}

template <typename Message>
std::vector<PoseValue> ConvertPath(const Message& message,
                                   std::size_t maximum = 6000) {
  std::vector<PoseValue> result;
  const std::size_t stride =
      std::max<std::size_t>(1, (message.poses.size() + maximum - 1) / maximum);
  result.reserve(std::min(maximum, message.poses.size()));
  for (std::size_t index = 0; index < message.poses.size(); index += stride) {
    result.push_back(ConvertPose(message.poses[index].pose));
  }
  return result;
}
#endif

}  // namespace

class RosNavigationBridge::Impl {
 public:
  explicit Impl(bool mock) : mock_(mock) {
    state_.bindings = {
        {"map", EnvironmentTopic("G1_WEB_TOPIC_MAP", "/map")},
        {"global_costmap", EnvironmentTopic("G1_WEB_TOPIC_GLOBAL_COSTMAP", "/global_costmap/costmap")},
        {"global_costmap_raw", EnvironmentTopic("G1_WEB_TOPIC_GLOBAL_COSTMAP_RAW", "/global_costmap/costmap_raw")},
        {"local_costmap", EnvironmentTopic("G1_WEB_TOPIC_LOCAL_COSTMAP", "/local_costmap/costmap")},
        {"local_costmap_raw", EnvironmentTopic("G1_WEB_TOPIC_LOCAL_COSTMAP_RAW", "/local_costmap/costmap_raw")},
        {"global_path", EnvironmentTopic("G1_WEB_TOPIC_GLOBAL_PATH", "/plan")},
        {"local_path", EnvironmentTopic("G1_WEB_TOPIC_LOCAL_PATH", "/local_plan")},
        {"particles", EnvironmentTopic("G1_WEB_TOPIC_PARTICLES", "/particle_cloud")},
        {"amcl_pose", EnvironmentTopic("G1_WEB_TOPIC_AMCL_POSE", "/amcl_pose")},
        {"scan", EnvironmentTopic("G1_WEB_TOPIC_SCAN", "/scan")},
        {"pointcloud", EnvironmentTopic("G1_WEB_TOPIC_POINTCLOUD", "/points")},
        {"footprint", EnvironmentTopic("G1_WEB_TOPIC_FOOTPRINT", "/local_costmap/published_footprint")},
        {"markers", EnvironmentTopic("G1_WEB_TOPIC_MARKERS", "/marker_array")},
        {"tf", EnvironmentTopic("G1_WEB_TOPIC_TF", "/tf")},
        {"tf_static", EnvironmentTopic("G1_WEB_TOPIC_TF_STATIC", "/tf_static")},
        {"joint_states", EnvironmentTopic("G1_WEB_TOPIC_JOINT_STATES", "/joint_states")},
    };
  }

  ~Impl() { Stop(); }

  bool Start(std::string& error) {
    if (mock_) {
      std::lock_guard<std::mutex> lock(mutex_);
      PopulateMockLocked();
      state_.connected = true;
      error.clear();
      return true;
    }
#ifdef G1_WEB_HAS_ROS2_NAV
    try {
      context_ = std::make_shared<rclcpp::Context>();
      context_->init(0, nullptr);
      rclcpp::NodeOptions node_options;
      node_options.context(context_);
      node_ = std::make_shared<rclcpp::Node>("g1_web_navigation_bridge",
                                             node_options);
      rclcpp::ExecutorOptions executor_options;
      executor_options.context = context_;
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(
          executor_options);
      executor_->add_node(node_);
      RebindAll();
      graph_timer_ = node_->create_wall_timer(
          std::chrono::seconds(2), [this] { RefreshGraph(); });
      running_.store(true);
      spin_thread_ = std::thread([this] {
        executor_->spin();
      });
      {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.connected = true;
        state_.error.clear();
      }
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    }
#else
    error = "ros2_navigation_bridge_not_built";
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
    state_.error = error;
    return false;
  }

  void Stop() {
#ifdef G1_WEB_HAS_ROS2_NAV
    running_.store(false);
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    if (executor_ && node_) executor_->remove_node(node_);
    subscriptions_.clear();
    graph_timer_.reset();
    executor_.reset();
    node_.reset();
    if (context_ && context_->is_valid()) context_->shutdown("g1 web stop");
    context_.reset();
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
  }

  bool Available() const {
#ifdef G1_WEB_HAS_ROS2_NAV
    return true;
#else
    return mock_;
#endif
  }

  bool ConfigureTopic(const std::string& kind, const std::string& topic,
                      std::string& error) {
    if (!SafeTopic(topic)) {
      error = "invalid_ros_topic";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_.bindings.find(kind) == state_.bindings.end()) {
        error = "unknown_topic_kind";
        return false;
      }
      state_.bindings[kind] = topic;
      ++state_.sequence;
    }
#ifdef G1_WEB_HAS_ROS2_NAV
    if (node_) Rebind(kind);
#else
    if (!mock_) {
      std::ofstream output("/tmp/g1_web_navigation_topic.cfg",
                           std::ios::binary | std::ios::trunc);
      if (!output) {
        error = "ros2_navigation_sidecar_config_unavailable";
        return false;
      }
      output << kind << '\t' << topic << '\n';
    }
#endif
    error.clear();
    return true;
  }

  std::string SerializeTopics() const {
#ifndef G1_WEB_HAS_ROS2_NAV
    if (!mock_) {
      const std::string sidecar =
          ReadSidecar("/tmp/g1_web_navigation_topics.json");
      if (!sidecar.empty()) return sidecar;
    }
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["available"] = Available();
    root["connected"] = state_.connected;
    root["error"] = state_.error;
    root["fixed_frame"] = state_.fixed_frame;
    for (const auto& binding : state_.bindings) {
      root["bindings"][binding.first] = binding.second;
    }
    auto& discovered = root["discovered"];
    discovered = Json::Value(Json::arrayValue);
    for (const auto& entry : state_.discovered) {
      Json::Value topic(Json::objectValue);
      topic["name"] = entry.first;
      for (const auto& type : entry.second) topic["types"].append(type);
      discovered.append(std::move(topic));
    }
    const auto now = Clock::now();
    for (const auto& item : state_.health) {
      const auto& health = item.second;
      Json::Value value(Json::objectValue);
      value["topic"] = health.topic;
      value["type"] = health.type;
      value["messages"] = Json::UInt64(health.messages);
      value["age_ms"] = Json::Int64(
          health.messages == 0 ? -1 : mock_ ? 0 :
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - health.last).count());
      const double seconds = health.messages < 2 ? 0.0 :
          std::chrono::duration<double>(health.last - health.first).count();
      value["hz"] = mock_ ? 10.0 : seconds > 0.0 ? (health.messages - 1) / seconds : 0.0;
      root["health"][item.first] = std::move(value);
    }
    return JsonText(root);
  }

  std::string SerializeScene() const {
#ifndef G1_WEB_HAS_ROS2_NAV
    if (!mock_) {
      const std::string sidecar =
          ReadSidecar("/tmp/g1_web_navigation_scene.json");
      if (!sidecar.empty()) return sidecar;
    }
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["schema_version"] = 2;
    root["sequence"] = Json::UInt64(state_.sequence);
    root["connected"] = state_.connected;
    root["error"] = state_.error;
    root["fixed_frame"] = state_.fixed_frame;
    root["map"] = GridJson(state_.map);
    root["global_costmap"] = GridJson(state_.global_costmap);
    root["local_costmap"] = GridJson(state_.local_costmap);
    root["global_path"] = PathJson(state_.global_path);
    root["local_path"] = PathJson(state_.local_path);
    root["particles_revision"] = Json::UInt64(state_.particles_revision);
    AddPackedPoints(root, "particles", state_.particles);
    root["amcl_received"] = state_.amcl_received;
    root["amcl_revision"] = Json::UInt64(state_.amcl_revision);
    root["amcl_pose"] = PoseJson(state_.amcl_pose);
    for (double value : state_.covariance) root["covariance"].append(value);
    root["laser_revision"] = Json::UInt64(state_.laser_revision);
    AddPackedPoints(root, "laser", state_.laser);
    root["pointcloud_revision"] = Json::UInt64(state_.pointcloud_revision);
    AddPackedPoints(root, "pointcloud", state_.pointcloud);
    root["footprint_revision"] = Json::UInt64(state_.footprint_revision);
    AddPackedPoints(root, "footprint", state_.footprint);
    root["tf_revision"] = Json::UInt64(state_.tf_revision);
    for (const auto& item : state_.transforms) {
      Json::Value transform = PoseJson(item.second.pose);
      transform["parent"] = item.second.parent;
      transform["child"] = item.second.child;
      transform["static"] = item.second.is_static;
      root["transforms"].append(std::move(transform));
    }
    root["joints_revision"] = Json::UInt64(state_.joints_revision);
    for (const auto& joint : state_.joints) root["joints"][joint.first] = joint.second;
    root["markers_revision"] = Json::UInt64(state_.markers_revision);
    for (const auto& item : state_.markers) {
      const auto& marker = item.second;
      Json::Value value(Json::objectValue);
      value["key"] = marker.key;
      value["frame"] = marker.frame;
      value["type"] = marker.type;
      value["action"] = marker.action;
      value["pose"] = PoseJson(marker.pose);
      value["scale"].append(marker.sx);
      value["scale"].append(marker.sy);
      value["scale"].append(marker.sz);
      value["color"].append(marker.r);
      value["color"].append(marker.g);
      value["color"].append(marker.b);
      value["color"].append(marker.a);
      value["text"] = marker.text;
      AddPoints(value["points"], marker.points);
      root["markers"].append(std::move(value));
    }
    return JsonText(root);
  }

 private:
  void PopulateMockLocked() {
    state_.map.frame = "map";
    state_.map.width = 180;
    state_.map.height = 140;
    state_.map.resolution = 0.05F;
    state_.map.origin.x = -4.5;
    state_.map.origin.y = -3.5;
    state_.map.data.assign(state_.map.width * state_.map.height, 0);
    for (std::uint32_t y = 0; y < state_.map.height; ++y) {
      for (std::uint32_t x = 0; x < state_.map.width; ++x) {
        auto& cell = state_.map.data[y * state_.map.width + x];
        if (x < 3 || y < 3 || x + 3 >= state_.map.width ||
            y + 3 >= state_.map.height || (x > 92 && x < 98 && y < 100)) {
          cell = 100;
        } else if ((x + y) % 29 == 0) {
          cell = 255;
        }
      }
    }
    state_.map.revision = 1;
    state_.global_costmap = state_.map;
    state_.global_costmap.data.assign(state_.map.data.size(), 0);
    for (std::size_t index = 0; index < state_.global_costmap.data.size(); ++index) {
      if (state_.map.data[index] == 100) state_.global_costmap.data[index] = 254;
      else if (index % 71 == 0) state_.global_costmap.data[index] = 90;
    }
    state_.global_costmap.revision = 1;
    state_.local_costmap.width = 80;
    state_.local_costmap.height = 80;
    state_.local_costmap.resolution = 0.05F;
    state_.local_costmap.origin.x = -2.0;
    state_.local_costmap.origin.y = -2.0;
    state_.local_costmap.data.assign(6400, 0);
    for (std::size_t i = 0; i < 6400; ++i) {
      const int x = static_cast<int>(i % 80) - 40;
      const int y = static_cast<int>(i / 80) - 40;
      const double radius = std::hypot(x - 18, y + 8);
      if (radius < 5) state_.local_costmap.data[i] = 254;
      else if (radius < 15) state_.local_costmap.data[i] =
          static_cast<std::uint8_t>(220 - radius * 9);
    }
    state_.local_costmap.revision = 1;
    for (int i = 0; i < 90; ++i) {
      PoseValue pose;
      pose.x = -2.0 + i * 0.055;
      pose.y = -1.0 + std::sin(i * 0.06) * 0.65;
      state_.global_path.poses.push_back(pose);
      if (i < 24) state_.local_path.poses.push_back(pose);
    }
    state_.global_path.frame = state_.local_path.frame = "map";
    state_.global_path.revision = state_.local_path.revision = 1;
    for (int i = 0; i < 700; ++i) {
      const double angle = i * 0.23;
      const double radius = 0.08 + (i % 29) * 0.012;
      state_.particles.push_back({static_cast<float>(std::cos(angle) * radius),
                                  static_cast<float>(std::sin(angle) * radius),
                                  0.02F, static_cast<float>(1.0 / (1 + i % 12))});
    }
    state_.particles_revision = 1;
    state_.amcl_received = true;
    state_.amcl_pose.qw = 1.0;
    state_.covariance.assign(36, 0.0);
    state_.covariance[0] = 0.08;
    state_.covariance[7] = 0.04;
    state_.covariance[35] = 0.03;
    state_.amcl_revision = 1;
    for (int i = 0; i < 720; ++i) {
      const float angle = static_cast<float>(i * 0.00872664626);
      const float radius = 2.3F + 0.5F * std::sin(angle * 7.0F);
      state_.laser.push_back({std::cos(angle) * radius,
                              std::sin(angle) * radius, 0.08F, radius});
    }
    state_.laser_revision = 1;
    state_.footprint = {{0.32F, 0.22F, 0.02F, 0}, {0.32F, -0.22F, 0.02F, 0},
                        {-0.25F, -0.22F, 0.02F, 0}, {-0.25F, 0.22F, 0.02F, 0}};
    state_.footprint_revision = 1;
    state_.transforms["base_link"] = {"map", "base_link", {}, false};
    state_.tf_revision = 1;
    state_.sequence = 1;
    for (auto& binding : state_.bindings) {
      state_.health[binding.first] = {binding.second, "mock", 50,
                                      Clock::now() - std::chrono::seconds(5),
                                      Clock::now()};
      state_.discovered.push_back({binding.second, {"mock"}});
    }
  }

#ifdef G1_WEB_HAS_ROS2_NAV
  void Touch(const std::string& kind, const std::string& type) {
    auto& health = state_.health[kind];
    health.topic = state_.bindings[kind];
    health.type = type;
    health.last = Clock::now();
    if (health.messages++ == 0) health.first = health.last;
    ++state_.sequence;
  }

  template <typename Message, typename Callback>
  void Bind(const std::string& kind, const rclcpp::QoS& qos,
            Callback callback, const std::string& subscription_key = {}) {
    std::string topic;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      topic = state_.bindings[kind];
    }
    subscriptions_[subscription_key.empty() ? kind : subscription_key] = node_->create_subscription<Message>(
        topic, qos, std::move(callback));
  }

  void RebindAll() {
    for (const auto& item : state_.bindings) Rebind(item.first);
  }

  void Rebind(const std::string& kind) {
    subscriptions_.erase(kind);
    subscriptions_.erase(kind + "_raw");
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1))
                             .reliable().transient_local();
    if (kind == "map" || kind == "global_costmap" || kind == "local_costmap") {
      Bind<nav_msgs::msg::OccupancyGrid>(kind, latched,
          [this, kind](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            GridValue* grid = kind == "map" ? &state_.map :
                              kind == "global_costmap" ? &state_.global_costmap :
                              &state_.local_costmap;
            grid->frame = message->header.frame_id;
            grid->width = message->info.width;
            grid->height = message->info.height;
            grid->resolution = message->info.resolution;
            grid->origin = ConvertPose(message->info.origin);
            grid->data.resize(message->data.size());
            std::transform(message->data.begin(), message->data.end(),
                           grid->data.begin(), [](std::int8_t value) {
                             return value < 0 ? std::uint8_t{255} :
                                                static_cast<std::uint8_t>(value);
                           });
            ++grid->revision;
            Touch(kind, "nav_msgs/msg/OccupancyGrid");
          });
    } else if (kind == "global_costmap_raw" || kind == "local_costmap_raw") {
      Bind<nav2_msgs::msg::Costmap>(kind, reliable,
          [this, kind](nav2_msgs::msg::Costmap::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            GridValue* grid = kind == "global_costmap_raw"
                                  ? &state_.global_costmap
                                  : &state_.local_costmap;
            grid->frame = message->header.frame_id;
            grid->width = message->metadata.size_x;
            grid->height = message->metadata.size_y;
            grid->resolution = message->metadata.resolution;
            grid->origin = ConvertPose(message->metadata.origin);
            grid->data.assign(message->data.begin(), message->data.end());
            ++grid->revision;
            Touch(kind, "nav2_msgs/msg/Costmap");
          });
    } else if (kind == "global_path" || kind == "local_path") {
      Bind<nav_msgs::msg::Path>(kind, reliable,
          [this, kind](nav_msgs::msg::Path::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto& path = kind == "global_path" ? state_.global_path : state_.local_path;
            path.frame = message->header.frame_id;
            path.poses = ConvertPath(*message);
            ++path.revision;
            Touch(kind, "nav_msgs/msg/Path");
          });
    } else if (kind == "amcl_pose") {
      Bind<geometry_msgs::msg::PoseWithCovarianceStamped>(kind, reliable,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.amcl_pose = ConvertPose(message->pose.pose);
            state_.covariance.assign(message->pose.covariance.begin(),
                                     message->pose.covariance.end());
            state_.amcl_received = true;
            ++state_.amcl_revision;
            Touch("amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped");
          });
    } else if (kind == "particles") {
      Bind<nav2_msgs::msg::ParticleCloud>(kind, sensor_qos,
          [this](nav2_msgs::msg::ParticleCloud::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.particles.clear();
            const std::size_t stride = std::max<std::size_t>(
                1, (message->particles.size() + 2499) / 2500);
            for (std::size_t i = 0; i < message->particles.size(); i += stride) {
              const auto& item = message->particles[i];
              state_.particles.push_back({static_cast<float>(item.pose.position.x),
                                          static_cast<float>(item.pose.position.y),
                                          static_cast<float>(item.pose.position.z),
                                          static_cast<float>(item.weight)});
            }
            ++state_.particles_revision;
            Touch("particles", "nav2_msgs/msg/ParticleCloud");
          });
    } else if (kind == "scan") {
      Bind<sensor_msgs::msg::LaserScan>(kind, sensor_qos,
          [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.laser.clear();
            const std::size_t stride = std::max<std::size_t>(
                1, (message->ranges.size() + 5999) / 6000);
            for (std::size_t i = 0; i < message->ranges.size(); i += stride) {
              const float range = message->ranges[i];
              if (!std::isfinite(range) || range < message->range_min ||
                  range > message->range_max) continue;
              const float angle = message->angle_min + i * message->angle_increment;
              state_.laser.push_back({std::cos(angle) * range,
                                      std::sin(angle) * range, 0.0F, range});
            }
            ++state_.laser_revision;
            Touch("scan", "sensor_msgs/msg/LaserScan");
          });
    } else if (kind == "footprint") {
      Bind<geometry_msgs::msg::PolygonStamped>(kind, reliable,
          [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.footprint.clear();
            for (const auto& point : message->polygon.points) {
              state_.footprint.push_back({point.x, point.y, point.z, 0.0F});
            }
            ++state_.footprint_revision;
            Touch("footprint", "geometry_msgs/msg/PolygonStamped");
          });
    } else if (kind == "tf" || kind == "tf_static") {
      Bind<tf2_msgs::msg::TFMessage>(kind, kind == "tf_static" ? latched : sensor_qos,
          [this, kind](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& transform : message->transforms) {
              state_.transforms[transform.child_frame_id] = {
                  transform.header.frame_id, transform.child_frame_id,
                  ConvertTransform(transform.transform), kind == "tf_static"};
            }
            while (state_.transforms.size() > 600) state_.transforms.erase(state_.transforms.begin());
            ++state_.tf_revision;
            Touch(kind, "tf2_msgs/msg/TFMessage");
          });
    } else if (kind == "joint_states") {
      Bind<sensor_msgs::msg::JointState>(kind, sensor_qos,
          [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            const std::size_t count = std::min(message->name.size(), message->position.size());
            for (std::size_t i = 0; i < count; ++i) state_.joints[message->name[i]] = message->position[i];
            ++state_.joints_revision;
            Touch("joint_states", "sensor_msgs/msg/JointState");
          });
    } else if (kind == "pointcloud") {
      Bind<sensor_msgs::msg::PointCloud2>(kind, sensor_qos,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            std::uint32_t ox = 0, oy = 0, oz = 0;
            bool hx = false, hy = false, hz = false;
            for (const auto& field : message->fields) {
              if (field.name == "x") { ox = field.offset; hx = true; }
              else if (field.name == "y") { oy = field.offset; hy = true; }
              else if (field.name == "z") { oz = field.offset; hz = true; }
            }
            if (!hx || !hy || !hz || message->point_step == 0) return;
            const std::size_t total = static_cast<std::size_t>(message->width) * message->height;
            const std::size_t stride = std::max<std::size_t>(1, (total + 11999) / 12000);
            std::vector<PointValue> points;
            points.reserve(std::min<std::size_t>(total, 12000));
            for (std::size_t i = 0; i < total; i += stride) {
              const std::size_t base = i * message->point_step;
              if (base + std::max({ox, oy, oz}) + sizeof(float) > message->data.size()) break;
              float x, y, z;
              std::memcpy(&x, message->data.data() + base + ox, sizeof(float));
              std::memcpy(&y, message->data.data() + base + oy, sizeof(float));
              std::memcpy(&z, message->data.data() + base + oz, sizeof(float));
              if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) points.push_back({x, y, z, 0});
            }
            std::lock_guard<std::mutex> lock(mutex_);
            state_.pointcloud = std::move(points);
            ++state_.pointcloud_revision;
            Touch("pointcloud", "sensor_msgs/msg/PointCloud2");
          });
    } else if (kind == "markers") {
      Bind<visualization_msgs::msg::MarkerArray>(kind, reliable,
          [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& source : message->markers) {
              const std::string key = source.ns + ":" + std::to_string(source.id);
              if (source.action == visualization_msgs::msg::Marker::DELETE) {
                state_.markers.erase(key);
                continue;
              }
              if (source.action == visualization_msgs::msg::Marker::DELETEALL) {
                state_.markers.clear();
                continue;
              }
              MarkerValue marker;
              marker.key = key;
              marker.frame = source.header.frame_id;
              marker.type = source.type;
              marker.action = source.action;
              marker.pose = ConvertPose(source.pose);
              marker.sx = source.scale.x;
              marker.sy = source.scale.y;
              marker.sz = source.scale.z;
              marker.r = source.color.r;
              marker.g = source.color.g;
              marker.b = source.color.b;
              marker.a = source.color.a;
              marker.text = source.text.substr(0, 256);
              const std::size_t stride = std::max<std::size_t>(1, (source.points.size() + 1999) / 2000);
              for (std::size_t i = 0; i < source.points.size(); i += stride) {
                const auto& p = source.points[i];
                marker.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), 0});
              }
              state_.markers[key] = std::move(marker);
            }
            while (state_.markers.size() > 300) state_.markers.erase(state_.markers.begin());
            ++state_.markers_revision;
            Touch("markers", "visualization_msgs/msg/MarkerArray");
          });
    }
  }

  void RefreshGraph() {
    const auto graph = node_->get_topic_names_and_types();
    std::vector<std::pair<std::string, std::vector<std::string>>> discovered;
    discovered.reserve(std::min<std::size_t>(graph.size(), 500));
    for (const auto& item : graph) {
      if (discovered.size() >= 500) break;
      discovered.push_back(item);
    }
    std::sort(discovered.begin(), discovered.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    std::lock_guard<std::mutex> lock(mutex_);
    state_.discovered = std::move(discovered);
    state_.graph_updated = Clock::now();
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
#endif

  bool mock_{false};
  mutable std::mutex mutex_;
  SceneState state_;
};

RosNavigationBridge::RosNavigationBridge(bool mock)
    : impl_(std::make_unique<Impl>(mock)) {}
RosNavigationBridge::~RosNavigationBridge() = default;
bool RosNavigationBridge::Start(std::string& error) { return impl_->Start(error); }
void RosNavigationBridge::Stop() { impl_->Stop(); }
bool RosNavigationBridge::Available() const { return impl_->Available(); }
std::string RosNavigationBridge::SerializeScene() const { return impl_->SerializeScene(); }
std::string RosNavigationBridge::SerializeTopics() const { return impl_->SerializeTopics(); }
bool RosNavigationBridge::ConfigureTopic(const std::string& kind,
                                         const std::string& topic,
                                         std::string& error) {
  return impl_->ConfigureTopic(kind, topic, error);
}

}  // namespace g1_web
