#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <json/json.h>

#include "g1_web/control_service.hpp"
#include "g1_web/camera_service.hpp"
#include "g1_web/g1_model_catalog.hpp"
#include "g1_web/joint_names.hpp"
#include "g1_web/json_serializer.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/ros_navigation_bridge.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/static_assets.hpp"
#include "g1_web/voice_service.hpp"

namespace g1_web {
struct VoiceServiceTestAccess {
  static void HandleAudioMessage(VoiceService& voice,
                                 const std::string& message) {
    voice.HandleAudioMessage(message);
  }

  static void ExpireWakeSession(VoiceService& voice) {
    std::lock_guard<std::mutex> lock(voice.llm_mutex_);
    voice.customer_wake_until_ = std::chrono::steady_clock::now() -
                                 std::chrono::seconds(1);
  }
};
}  // namespace g1_web

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "TEST FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

Json::Value Parse(const std::string& text) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(text);
  Require(Json::parseFromStream(builder, stream, &root, &errors),
          "JSON should parse: " + errors);
  return root;
}

void TestJointNames() {
  const auto& names = g1_web::JointNames();
  Require(names.size() == 29, "G1 should expose 29 named joint slots");
  Require(names[0].name == "left_hip_pitch", "joint 0 mapping");
  Require(names[12].name == "waist_yaw", "joint 12 mapping");
  Require(names[28].name == "right_wrist_yaw", "joint 28 mapping");
}

void TestG1ModelCatalog() {
  const auto& models = g1_web::G1ModelCatalog();
  Require(models.size() == 11, "all supported G1 mode_machine models");
  Require(g1_web::FindG1Model(5) != nullptr,
          "mode_machine 5 should be recognized");
  Require(g1_web::FindG1Model(5)->urdf_file ==
              "g1_29dof_rev_1_0.urdf",
          "mode_machine 5 should select rev 1.0");
  Require(g1_web::FindG1Model(17) == nullptr,
          "unknown mode_machine must not be guessed");
}

void TestStaticAssets() {
  const std::filesystem::path source_file(__FILE__);
  const auto web_root = source_file.parent_path().parent_path() / "web";
  std::filesystem::path resolved;
  Require(g1_web::ResolveStaticAsset(web_root, "/", resolved),
          "root should resolve to index.html");
  Require(resolved.filename() == "index.html", "root asset filename");
  Require(g1_web::ResolveStaticAsset(web_root, "/i18n.js", resolved),
          "i18n module should be allowlisted");
  Require(resolved.filename() == "i18n.js", "i18n module filename");
  Require(g1_web::ResolveStaticAsset(web_root, "/perception.js", resolved),
          "perception module should be allowlisted");
  Require(resolved.filename() == "perception.js",
          "perception module filename");
  Require(!g1_web::ResolveStaticAsset(web_root, "/nav-store.js", resolved),
          "removed ROS navigation state module must not be exposed");
  Require(!g1_web::ResolveStaticAsset(web_root, "/nav-renderer.js", resolved),
          "removed ROS navigation renderer module must not be exposed");
  Require(g1_web::ResolveStaticAsset(web_root, "/joint-debug.js", resolved),
          "joint debugger module should be allowlisted");
  std::ifstream index_stream(web_root / "index.html");
  const std::string index_html((std::istreambuf_iterator<char>(index_stream)),
                               std::istreambuf_iterator<char>());
  Require(index_html.find("value=\"/dev/video") == std::string::npos &&
              index_html.find("留空自动检测") != std::string::npos &&
              index_html.find("自动检测 RGB+深度") != std::string::npos,
          "camera UI must auto-detect unstable V4L2 device nodes");
  Require(index_html.find("id=\"languageSelect\"") != std::string::npos &&
              index_html.find("value=\"zh-CN\"") != std::string::npos &&
              index_html.find("value=\"en\"") != std::string::npos &&
              index_html.find("<script src=\"/i18n.js\"></script>") !=
                  std::string::npos,
          "workspace must expose the Chinese/English language selector and load i18n");
  Require(index_html.find("id=\"diagnosticOverallBadge\"") !=
              std::string::npos &&
              index_html.find("id=\"diagnosticMotorFaultList\"") !=
                  std::string::npos &&
              index_html.find("只解码官方已公开的故障位") != std::string::npos &&
              index_html.find("不擅自判定故障") != std::string::npos,
          "diagnostics must expose useful decoded faults without guessing undocumented raw states");
  std::ifstream app_stream(web_root / "app.js");
  const std::string app_js((std::istreambuf_iterator<char>(app_stream)),
                           std::istreambuf_iterator<char>());
  Require(app_js.find("const motorStateFaults") != std::string::npos &&
              app_js.find("0x00040000") != std::string::npos &&
              app_js.find("0x40000000") != std::string::npos &&
              app_js.find("0x80000000") != std::string::npos &&
              app_js.find("renderDiagnostics(data)") != std::string::npos,
          "diagnostics must retain official G1 motorstate fault decoding");
  Require(index_html.find("navLayerPanel") == std::string::npos &&
              index_html.find("navTopicList") == std::string::npos &&
              index_html.find("ROS 2 导航桥接") == std::string::npos,
          "customer SLAM UI must not expose unavailable ROS/Nav2 layers");
  Require(index_html.find("mapping-control-group") != std::string::npos &&
              index_html.find("map-localization-controls") != std::string::npos &&
              index_html.find("map-navigation-controls") != std::string::npos &&
              index_html.find("id=\"initialPoseMode\"") != std::string::npos &&
              index_html.find("id=\"goalPickMode\"") != std::string::npos &&
              index_html.find("id=\"initialX\"") != std::string::npos &&
              index_html.find("id=\"goalX\"") != std::string::npos,
          "SLAM UI must distribute mapping, relocalization and navigation around the map");
  std::ifstream perception_stream(web_root / "perception.js");
  const std::string perception_js(
      (std::istreambuf_iterator<char>(perception_stream)),
      std::istreambuf_iterator<char>());
  Require(perception_js.find("body.command === \"initialize_pose\"") !=
              std::string::npos &&
              perception_js.find(
                  "[\"navigating\", \"paused\"].includes(status.mode) && "
                  "status.target_set") != std::string::npos,
          "SLAM markers must clear after relocalization and only persist during active navigation");
  Require(perception_js.find(
              "mode: \"paused\", paused: true") != std::string::npos &&
              perception_js.find(
                  "mode: \"navigating\", paused: false") != std::string::npos,
          "pause/resume UI must switch buttons immediately after accepted RPCs");
  Require(perception_js.find("readCommandResponse") != std::string::npos &&
              perception_js.find("SDK api_result=") != std::string::npos &&
              perception_js.find("官方返回：") != std::string::npos &&
              perception_js.find("原始响应：") != std::string::npos,
          "perception command failures must expose SDK and official responses");
  Require(index_html.find("id=\"downloadMap\"") != std::string::npos &&
              index_html.find("id=\"exitMap\"") != std::string::npos &&
              index_html.find("id=\"cancelNavigation\"") != std::string::npos &&
              perception_js.find("downloadMapFile") != std::string::npos &&
              perception_js.find("waitForLoadedGlobalMap") != std::string::npos &&
              perception_js.find("MAP_NAME_WARNING") != std::string::npos &&
              perception_js.find("cancel_navigation") != std::string::npos,
          "SLAM UI must expose map download/exit, navigation cancel, and the fixed map-name warning");
  Require(g1_web::ResolvePerceptionMapPath("test1.pcd") ==
              "/home/unitree/test1.pcd" &&
              g1_web::ResolvePerceptionMapPath("test10.pcd") ==
                  "/home/unitree/test10.pcd" &&
              g1_web::ResolvePerceptionMapDownloadPath("test1.pcd") ==
                  "/home/unitree/.cache/g1-web-control/maps/test1.pcd" &&
              g1_web::ResolvePerceptionMapPath("test1").empty() &&
              g1_web::ResolvePerceptionMapPath("test11.pcd").empty() &&
              g1_web::ResolvePerceptionMapDownloadPath("office.pcd").empty(),
          "map save/load/download must only accept test1.pcd through test10.pcd");
  std::ifstream service_stream(web_root.parent_path() / "deploy" /
                               "g1-web-control.service");
  const std::string service_unit(
      (std::istreambuf_iterator<char>(service_stream)),
      std::istreambuf_iterator<char>());
  Require(service_unit.find("--enable-navigation") != std::string::npos,
          "production Web service must enable SLAM navigation by default");
  Require(g1_web::ResolveStaticAsset(
              web_root, "/assets/unitree/g1_description/g1_29dof_mode_15.urdf",
              resolved),
          "allowlisted URDF asset should resolve");
  Require(g1_web::StaticContentType(resolved) ==
              "application/xml; charset=utf-8",
          "URDF should use XML content type");
  Require(g1_web::StaticContentType("mesh.STL") == "model/stl",
          "STL content type should be case insensitive");
  Require(!g1_web::ResolveStaticAsset(web_root, "/README.md", resolved),
          "non-allowlisted root files must be hidden");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/../index.html", resolved),
          "plain directory traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%2e%2e/index.html", resolved),
          "encoded directory traversal must be rejected");
  Require(!g1_web::ResolveStaticAsset(
              web_root, "/assets/%ZZ/index.html", resolved),
          "invalid percent encoding must be rejected");

  static constexpr std::array<const char*, 11> kModels{{
      "g1_29dof.urdf",
      "g1_29dof_lock_waist.urdf",
      "g1_29dof_rev_1_0.urdf",
      "g1_29dof_lock_waist_rev_1_0.urdf",
      "g1_29dof_mode_11.urdf",
      "g1_29dof_mode_12.urdf",
      "g1_29dof_mode_13.urdf",
      "g1_29dof_mode_14.urdf",
      "g1_29dof_mode_15.urdf",
      "g1_29dof_mode_16.urdf",
      "g1_29dof_mode_18.urdf",
  }};
  for (const char* model : kModels) {
    const auto model_path =
        web_root / "assets/unitree/g1_description" / model;
    std::ifstream input(model_path);
    const std::string urdf((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    Require(!urdf.empty(), std::string("URDF should be readable: ") + model);
    for (const auto& joint : g1_web::JointNames()) {
      const std::string joint_name(joint.name);
      Require(urdf.find("name=\"" + joint_name + "_joint\"") !=
                  std::string::npos,
              std::string("URDF joint mapping: ") + model + " / " +
                  joint_name);
    }
  }
}

void TestMockNavigationBridge() {
  g1_web::RosNavigationBridge bridge(true);
  std::string error;
  Require(bridge.Start(error), "mock navigation bridge should start");
  const auto scene = Parse(bridge.SerializeScene());
  Require(scene["connected"].asBool(), "mock navigation scene connected");
  Require(scene["map"]["width"].asUInt() > 0,
          "mock navigation scene contains OccupancyGrid");
  Require(scene["global_path"]["poses"].size() > 2,
          "mock navigation scene contains global path");
  Require(scene["particles_count"].asUInt64() > 10 &&
              scene["particles_encoding"].asString() ==
                  "base64_f32le_xyzw" &&
              !scene["particles_data"].asString().empty(),
          "mock navigation scene contains compact AMCL particles");
  const auto topics = Parse(bridge.SerializeTopics());
  Require(topics["bindings"]["map"].asString() == "/map",
          "default map topic binding");
  Require(bridge.ConfigureTopic("map", "/slam_toolbox/map", error),
          "runtime topic override should succeed");
  Require(!bridge.ConfigureTopic("map", "unsafe-topic", error) &&
              error == "invalid_ros_topic",
          "invalid topic should be rejected");
  bridge.Stop();
}

void TestFreshness() {
  using g1_web::ClassifyFreshness;
  using g1_web::Freshness;
  Require(ClassifyFreshness(false, 0) == Freshness::kOffline,
          "never received is offline");
  Require(ClassifyFreshness(true, 1000) == Freshness::kOnline,
          "1000 ms remains online");
  Require(ClassifyFreshness(true, 1001) == Freshness::kDelayed,
          "over 1000 ms is delayed");
  Require(ClassifyFreshness(true, 3001) == Freshness::kOffline,
          "over 3000 ms is offline");
}

void TestSerialization() {
  g1_web::SnapshotStore store;
  store.PopulateMock(1.0);
  const Json::Value root =
      Parse(g1_web::SerializeSnapshot(store.GetSnapshot()));

  Require(root["schema_version"].asInt() == 1, "schema version");
  Require(root["application"].asString() == "UniRoboGui",
          "application name");
  Require(root["application_version"].asString() == "1.0.0",
          "application version");
  Require(root["dds_initialized"].asBool(), "mock DDS status");
  Require(root["sources"]["low_state"]["status"].asString() == "online",
          "mock low state is online");
  Require(root["joints"].size() == 29, "named joint count");
  Require(root["reserved_motor_slots"].size() == 6,
          "reserved motor slot count");
  Require(root["battery"]["soc_pct"].asInt() == 82, "mock battery SOC");
  Require(root["imu"].isMember("hip") && root["imu"].isMember("torso"),
          "both IMUs are serialized");
  Require(root["robot"]["model_supported"].asBool(),
          "mock mode_machine should resolve a local model");
  Require(root["robot"]["urdf_file"].asString() == "g1_29dof.urdf",
          "mock mode_machine 2 should select the old 29DOF model");
  Require(root["mainboard"].isMember("state_raw"),
          "mainboard raw state is present");
  Require(root["voice"]["initialized"].asBool(),
          "mock voice service is initialized");
  Require(!root["voice"]["chat_go_closed"].asBool() &&
              root["voice"]["llm"]["mode"].asString() == "builtin",
          "mock voice defaults to builtin chat_go interaction");
  Require(root["control"]["motion"]["state"].asString() == "stopped",
          "motion state is serialized");
  Require(root["control"]["motion"]["speed_mode"].asInt() == 0,
          "motion speed mode is serialized");
}

void WriteFloat(std::vector<std::uint8_t>& data, std::size_t offset,
                float value) {
  Require(offset + sizeof(value) <= data.size(), "point write bounds");
  std::memcpy(data.data() + offset, &value, sizeof(value));
}

void TestPointCloudDecode() {
  sensor_msgs::msg::dds_::PointCloud2_ cloud;
  cloud.height(1);
  cloud.width(3);
  cloud.point_step(16);
  cloud.row_step(48);
  cloud.is_bigendian(false);
  cloud.is_dense(true);
  cloud.header().frame_id("map");
  cloud.fields({
      sensor_msgs::msg::dds_::PointField_("x", 0, 7, 1),
      sensor_msgs::msg::dds_::PointField_("y", 4, 7, 1),
      sensor_msgs::msg::dds_::PointField_("z", 8, 7, 1),
      sensor_msgs::msg::dds_::PointField_("intensity", 12, 7, 1),
  });
  std::vector<std::uint8_t> bytes(48);
  for (std::size_t index = 0; index < 3; ++index) {
    WriteFloat(bytes, index * 16, static_cast<float>(index + 1));
    WriteFloat(bytes, index * 16 + 4, static_cast<float>(index + 2));
    WriteFloat(bytes, index * 16 + 8, static_cast<float>(index + 3));
    WriteFloat(bytes, index * 16 + 12, static_cast<float>(index * 10));
  }
  cloud.data(bytes);

  const auto decoded = g1_web::DecodePointCloud(cloud, 2);
  Require(decoded.valid, "valid PointCloud2 should decode");
  Require(decoded.frame_id == "map", "point cloud frame should be retained");
  Require(decoded.source_points == 3, "source point count");
  Require(decoded.points.size() == 2, "point cloud should be decimated");
  Require(std::abs(decoded.points.front().x - 1.0F) < 1e-6F,
          "decoded x value");
  Require(std::abs(decoded.points.back().intensity - 20.0F) < 1e-6F,
          "decoded intensity value");

  cloud.fields({sensor_msgs::msg::dds_::PointField_("x", 0, 7, 1)});
  const auto malformed = g1_web::DecodePointCloud(cloud, 100);
  Require(!malformed.valid && malformed.error == "xyz_fields_missing",
          "malformed PointCloud2 must be rejected");
}

void TestPointCloudWebFilter() {
  const std::vector<g1_web::PointSample> points{
      {1.01F, 1.01F, 0.0F, 10.0F},
      {1.02F, 1.02F, 0.0F, 20.0F},
      {1.11F, 1.01F, 0.0F, 30.0F},
      {4.0F, 4.0F, 0.0F, 40.0F},
      {20.0F, 0.0F, 0.0F, 50.0F},
      {1.0F, 1.0F, 5.0F, 60.0F},
  };
  g1_web::PointCloudFilterOptions options;
  options.voxel_size_m = 0.1F;
  options.maximum_range_m = 10.0F;
  options.minimum_z_m = -1.0F;
  options.maximum_z_m = 2.0F;
  options.maximum_points = 10;
  options.remove_isolated_voxels = true;
  const auto filtered = g1_web::FilterPointCloudForWeb(points, options);
  Require(filtered.size() == 2,
          "voxel filter should merge duplicates and reject isolated noise");
  bool found_centroid = false;
  for (const auto& point : filtered) {
    if (std::abs(point.x - 1.015F) < 1e-4F &&
        std::abs(point.intensity - 15.0F) < 1e-4F) {
      found_centroid = true;
    }
  }
  Require(found_centroid, "voxel filter should retain the cell centroid");

  options.maximum_points = 1;
  Require(g1_web::FilterPointCloudForWeb(points, options).size() == 1,
          "filtered cloud should honor the final Web point cap");
}

void TestLowObstacleSafetyDetector() {
  std::vector<g1_web::PointSample> ground;
  for (int xi = 0; xi < 20; ++xi) {
    const float x = 0.4F + xi * 0.05F;
    for (int yi = -8; yi <= 8; ++yi) {
      const float y = yi * 0.05F;
      ground.push_back({x, y, -1.05F + 0.01F * x + 0.005F * y, 10.0F});
    }
  }
  Require(!g1_web::DetectLowObstacleForSafety(ground).detected,
          "sloped ground alone must not trigger the low-obstacle safety layer");

  auto with_obstacle = ground;
  for (int i = 0; i < 12; ++i) {
    with_obstacle.push_back(
        {0.78F + (i % 3) * 0.02F, -0.04F + (i / 3) * 0.025F,
         -0.84F + (i % 2) * 0.01F, 80.0F});
  }
  const auto detected = g1_web::DetectLowObstacleForSafety(with_obstacle);
  Require(detected.detected && detected.nearest_range_m < 1.0F &&
              detected.support_points >= 2,
          "a 20 cm-class obstacle in the forward raw cloud must be retained by the safety detector");
}

void TestMockPerceptionSafety() {
  g1_web::PerceptionService perception(true, false);
  std::string error;
  Require(perception.Start(error), "mock perception should start");
  Require(error.empty(), "mock perception start error");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  const Json::Value status = Parse(perception.SerializeStatus());
  Require(status["initialized"].asBool(), "mock SLAM initialized");
  Require(status["navigation_enabled"].asBool(),
          "mock navigation is enabled for UI validation");
  Require(status["global_points"].asUInt64() > 1000,
          "mock global point cloud is populated");
  const Json::Value frame = Parse(perception.SerializeFrame());
  Require(frame["schema_version"].asInt() == 3 &&
              frame["live_points_encoding"].asString() ==
                  "base64_u16le_xyz" &&
              frame["live_points_count"].asUInt64() > 100 &&
              frame["live_points_origin"].size() == 3 &&
              frame["live_points_scale"].size() == 3 &&
              frame["live_points_data"].asString().size() <=
                  frame["live_points_count"].asUInt64() * 8 + 4,
          "live cloud uses quantized Wi-Fi encoding");
  const Json::Value global_map = Parse(perception.SerializeGlobalMap());
  Require(global_map["schema_version"].asInt() == 3 &&
              global_map["points_encoding"].asString() ==
                  "base64_u16le_xyz" &&
              global_map["points_origin"].size() == 3 &&
              global_map["points_scale"].size() == 3 &&
              !global_map["points_data"].asString().empty(),
          "global cloud uses quantized Wi-Fi encoding");
  Require(status["point_filter"]["live_max_points"].asUInt64() == 3600 &&
              status["point_filter"]["live_isolated_voxel_filter"].asBool() &&
              !status["point_filter"]["global_isolated_voxel_filter"].asBool(),
          "point filter profile should be visible in status");
  Require(status["services"]["lidar_driver"]["enabled"].asBool() &&
              status["services"]["unitree_slam"]["enabled"].asBool(),
          "mock sensing services start warm by default");

  g1_web::PerceptionRequest request;
  request.request_key = "slam-request-0001";
  request.command = "navigate";
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "confirmation_required",
          "SLAM command requires explicit confirmation");

  request.confirmed = true;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "navigation must stay locked until a map is loaded and localized");

  request.request_key = "slam-relocate-before-load";
  request.command = "initialize_pose";
  request.map_name = "test1.pcd";
  request.pose = {};
  request.pose.x = status["pose"]["x"].asDouble();
  request.pose.y = status["pose"]["y"].asDouble();
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "relocation must stay locked until the selected map is loaded");

  request.request_key = "slam-load-map-test1";
  request.command = "load_map";
  const auto loaded = perception.Submit(request);
  Require(loaded.accepted,
          "map loading should automatically start SLAM dependencies");
  Json::Value localized = Parse(perception.SerializeStatus());
  Require(localized["mode"].asString() == "localizing" &&
              localized["map_loaded"].asBool() &&
              localized["map_name"].asString() == "test1.pcd" &&
              localized["services"]["lidar_driver"]["enabled"].asBool() &&
              localized["services"]["unitree_slam"]["enabled"].asBool(),
          "successful map loading records the loaded map and starts localization");
  std::string topic_map_pcd;
  Require(perception.GetTopicGlobalMapPcd("test1.pcd", topic_map_pcd) &&
              topic_map_pcd.rfind("# .PCD v0.7", 0) == 0 &&
              topic_map_pcd.find("POINTS ") != std::string::npos,
          "localized global-map topic should be available as the preferred PCD download source");

  request.request_key = "slam-relocate-after-load";
  request.command = "initialize_pose";
  request.pose.x += 0.1;
  Require(perception.Submit(request).accepted,
          "relocation is accepted after the same map has been loaded");

  request.request_key = "slam-request-0003";
  request.command = "navigate";
  request.pose.x = localized["pose"]["x"].asDouble() + 11.0;
  request.pose.y = localized["pose"]["y"].asDouble();
  Require(perception.Submit(request).error == "goal_over_10m",
          "official ten metre goal limit must be enforced");

  request.request_key = "slam-request-0004";
  request.pose.x = localized["pose"]["x"].asDouble() + 1.0;
  const auto accepted = perception.Submit(request);
  Require(accepted.accepted && accepted.request_id > 0,
          "nearby mock navigation goal should be accepted after localization");
  const auto duplicate = perception.Submit(request);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == accepted.request_id,
          "duplicate SLAM request key must not execute twice");

  request.request_key = "slam-request-pause";
  request.command = "pause_navigation";
  Require(perception.Submit(request).accepted,
          "pause is accepted only while navigation is active");
  Json::Value paused_status = Parse(perception.SerializeStatus());
  Require(paused_status["mode"].asString() == "paused" &&
              paused_status["paused"].asBool() &&
              paused_status["target_set"].asBool() &&
              paused_status["map_loaded"].asBool(),
          "accepted pause must immediately expose paused state without dropping the active goal");

  request.request_key = "slam-request-pause-again";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "a second pause is rejected after the first pause already changed state");

  request.request_key = "slam-request-resume";
  request.command = "resume_navigation";
  Require(perception.Submit(request).accepted,
          "resume is accepted only from the paused state");
  Json::Value resumed_status = Parse(perception.SerializeStatus());
  Require(resumed_status["mode"].asString() == "navigating" &&
              !resumed_status["paused"].asBool() &&
              resumed_status["target_set"].asBool() &&
              resumed_status["map_loaded"].asBool(),
          "accepted resume must immediately return to the same active navigation goal");

  request.request_key = "slam-request-0005";
  request.command = "start_mapping";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "mapping must not start while navigation is active");

  request.request_key = "slam-request-cancel";
  request.command = "cancel_navigation";
  Require(perception.Submit(request).accepted,
          "cancel should stop the active navigation task");
  Json::Value cancelled_status = Parse(perception.SerializeStatus());
  Require(cancelled_status["mode"].asString() == "localizing" &&
              !cancelled_status["paused"].asBool() &&
              !cancelled_status["target_set"].asBool() &&
              cancelled_status["map_loaded"].asBool(),
          "navigation cancel must clear the active goal while preserving the loaded map");

  request.request_key = "slam-request-after-cancel";
  request.command = "navigate";
  Require(perception.Submit(request).accepted,
          "a new navigation goal should be accepted after cancellation");

  request.request_key = "slam-request-0006";
  request.command = "stop_slam";
  Require(perception.Submit(request).accepted, "mock SLAM should stop");

  request.request_key = "slam-request-0007";
  request.command = "start_mapping";
  Require(perception.Submit(request).accepted, "mock mapping should start");
  Json::Value running = Parse(perception.SerializeStatus());
  Require(running["services"]["lidar_driver"]["enabled"].asBool() &&
              running["services"]["unitree_slam"]["enabled"].asBool(),
          "mapping starts lidar and SLAM dependencies");
  Require(running["lidar_inputs"]["ready"].asBool() &&
              running["topics"]["raw_lidar_imu"].asString() ==
                  "rt/utlidar/imu_livox_mid360",
          "SLAM status must expose the official lidar IMU input");

  request.request_key = "slam-request-0008";
  request.command = "initialize_pose";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "map loading and relocation stay locked during mapping");

  request.request_key = "slam-request-invalid-map-name";
  request.command = "finish_mapping";
  request.map_name = "office.pcd";
  Require(perception.Submit(request).error == "invalid_map_name",
          "map saving must reject names outside test1.pcd through test10.pcd");

  request.request_key = "slam-request-0009";
  request.map_name = "test1.pcd";
  Require(perception.Submit(request).accepted,
          "map saving is accepted only while mapping with an allowed PCD name");

  request.request_key = "slam-request-0010";
  Require(perception.Submit(request).error == "invalid_slam_state",
          "map saving locks again after mapping finishes");

  request.request_key = "slam-request-0011";
  request.command = "stop_slam";
  Require(perception.Submit(request).accepted, "mock SLAM should stop");
  Json::Value stopped_status = Parse(perception.SerializeStatus());
  Require(stopped_status["services"]["lidar_driver"]["enabled"].asBool() &&
              !stopped_status["services"]["unitree_slam"]["enabled"].asBool(),
          "SLAM stop keeps raw lidar and closes SLAM");
  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  stopped_status = Parse(perception.SerializeStatus());
  const Json::Value stopped_frame = Parse(perception.SerializeFrame());
  Require(stopped_status["mode"].asString() == "stopped" &&
              !stopped_status["map_loaded"].asBool() &&
              stopped_status["trajectory_points"].asUInt64() == 0 &&
              stopped_status["global_points"].asUInt64() == 0 &&
              stopped_status["point_source"].asString() == "utlidar_mid360" &&
              stopped_frame["live_points_count"].asUInt64() > 100,
          "SLAM stop clears map and restores raw lidar without task trail");

  request.request_key = "slam-relocate-after-stop";
  request.command = "initialize_pose";
  request.map_name = "test1.pcd";
  request.pose = {};
  request.pose.qw = 1.0;
  Require(perception.Submit(request).error == "invalid_slam_state",
          "stopping SLAM clears the loaded-map interlock");

  request.request_key = "slam-reload-after-stop";
  request.command = "load_map";
  Require(perception.Submit(request).accepted,
          "map loading can restart localization after SLAM was stopped");

  request.request_key = "slam-relocate-after-reload";
  request.command = "initialize_pose";
  Require(perception.Submit(request).accepted,
          "relocation unlocks again after the map reload succeeds");

  request.request_key = "slam-nav-trail-after-reload";
  request.command = "navigate";
  request.pose.x = 0.35;
  request.pose.y = 0.0;
  Require(perception.Submit(request).accepted,
          "navigation starts from the loaded map without another reload");
  std::this_thread::sleep_for(std::chrono::milliseconds(260));
  const Json::Value navigating_status = Parse(perception.SerializeStatus());
  const Json::Value navigating_frame = Parse(perception.SerializeFrame());
  Require(navigating_status["mode"].asString() == "navigating" &&
              navigating_status["map_loaded"].asBool() &&
              navigating_frame["trajectory_count"].asUInt64() >= 2,
          "navigation keeps the loaded map and publishes a visible travel trail");

  Json::Value arrived_status;
  for (int attempt = 0; attempt < 20; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arrived_status = Parse(perception.SerializeStatus());
    if (arrived_status["mode"].asString() == "arrived") break;
  }
  Require(arrived_status["mode"].asString() == "arrived" &&
              arrived_status["map_loaded"].asBool() &&
              arrived_status["pose_age_ms"].asInt64() <= 250 &&
              arrived_status["live_points"].asUInt64() > 100 &&
              !arrived_status["target_set"].asBool(),
          "arrival must keep localization, map and fresh pose alive");

  request.request_key = "slam-nav-again-without-reload";
  request.pose.x = arrived_status["pose"]["x"].asDouble() + 0.15;
  request.pose.y = arrived_status["pose"]["y"].asDouble();
  Require(perception.Submit(request).accepted,
          "a new navigation goal is accepted after arrival without reloading the map");
  perception.Stop();
}

void TestMockCameraCommands() {
  g1_web::CameraOptions options;
  options.mock = true;
  g1_web::CameraService camera(options);
  std::string error;
  Require(camera.Start(error), "mock camera should start");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  Require(camera.GetFrame("rgb").available, "mock RGB frame available");
  const Json::Value low_bandwidth = Parse(camera.SerializeStatus());
  Require(low_bandwidth["running"].asBool() &&
              low_bandwidth["bandwidth_profile"].asString() ==
                  "wifi_low_bandwidth" &&
              low_bandwidth["output_width"].asUInt() == 480 &&
              low_bandwidth["output_height"].asUInt() == 360 &&
              low_bandwidth["rgb_jpeg_quality"].asInt() == 55 &&
              low_bandwidth["depth_jpeg_quality"].asInt() == 45,
          "camera should default to the Wi-Fi low-bandwidth JPEG profile");
  Require(low_bandwidth["rgb"]["jpeg_bytes"].asUInt64() > 0,
          "camera status should expose compressed frame size");

  g1_web::CameraRequest request;
  request.request_key = "camera-request-0001";
  request.command = "stop";
  Require(camera.Submit(request).error == "confirmation_required",
          "camera command requires confirmation");
  request.confirmed = true;
  const auto stopped = camera.Submit(request);
  Require(stopped.accepted, "camera stop should be accepted");
  Require(!Parse(camera.SerializeStatus())["rgb"]["configured"].asBool(),
          "stopped camera should become unconfigured");
  Require(!Parse(camera.SerializeStatus())["running"].asBool(),
          "stopped camera should report capture is not running");

  request.request_key = "camera-request-0002";
  request.command = "start_v4l2";
  request.rgb_source = "/tmp/video0";
  Require(camera.Submit(request).error == "invalid_camera_source",
          "camera source outside /dev/videoN must be rejected");
  request.request_key = "camera-request-0003";
  request.rgb_source.clear();
  request.depth_source.clear();
  const auto started = camera.Submit(request);
  Require(started.accepted, "mock runtime camera auto-detect start should succeed");
  const Json::Value auto_status = Parse(camera.SerializeStatus());
  Require(auto_status["auto_detect"].asBool(),
          "empty camera sources should enable automatic device detection");
  Require(auto_status["first_person_service"]["paused_by_web"].asBool(),
          "camera start pauses first-person service in mock mode");
  const auto duplicate = camera.Submit(request);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == started.request_id,
          "duplicate camera request must not restart capture");
  camera.Stop();
  Require(!Parse(camera.SerializeStatus())["first_person_service"]
               ["paused_by_web"].asBool(),
          "camera stop restores first-person service in mock mode");
}

void TestConcurrentSnapshotAccess() {
  g1_web::SnapshotStore store;
  std::atomic<bool> failed{false};
  std::thread writer([&store] {
    for (int i = 0; i < 500; ++i) {
      store.PopulateMock(static_cast<double>(i) / 20.0);
    }
  });
  std::thread reader([&store, &failed] {
    for (int i = 0; i < 500; ++i) {
      const auto json = g1_web::SerializeSnapshot(store.GetSnapshot());
      if (json.empty() || json.front() != '{') {
        failed.store(true);
      }
    }
  });
  writer.join();
  reader.join();
  Require(!failed.load(), "concurrent reads should remain valid");
}

void TestMockVoiceService() {
  const std::filesystem::path customer_config_path =
      std::filesystem::current_path() / "config" / "customer_voice.json";
  std::error_code cleanup_error;
  std::filesystem::remove(customer_config_path, cleanup_error);

  g1_web::SnapshotStore store;
  g1_web::VoiceService voice(store, true);
  std::string error;
  Require(voice.Start(error), "mock voice service should start");
  Require(error.empty(), "mock voice start should not report an error");

  Require(!voice.EnqueueTts("   ", 0).accepted,
          "empty TTS should be rejected");
  Require(!voice.EnqueueTts("hello", 7).accepted,
          "invalid speaker should be rejected");
  const auto invalid_backend = voice.EnqueueTts("hello", -1, "unknown");
  Require(!invalid_backend.accepted &&
              invalid_backend.error == "invalid_tts_backend",
          "invalid explicit TTS backend should be rejected");
  const auto mixed =
      voice.EnqueueTts("你好 Hello Unitree，欢迎使用 G1", -1, "kokoro");
  Require(mixed.accepted && mixed.speaker_id == -1 &&
              mixed.backend == "kokoro",
          "mixed Chinese and English TTS should accept explicit Kokoro playback");
  Require(!voice.EnqueueTts("你好 Hello", 0).accepted,
          "mixed TTS with a forced single-language speaker should be rejected");
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  Require(store.GetSnapshot().voice.tts.request_id == mixed.request_id &&
              store.GetSnapshot().voice.tts.state == "succeeded" &&
              store.GetSnapshot().voice.tts.speaker_id == -1,
          "mixed TTS should complete as one logical request in mock mode");

  const auto asr_off = voice.SetAsrEnabled(false);
  Require(asr_off.accepted &&
              !store.GetSnapshot().voice.asr_subscribed,
          "mock ASR receiver should be switchable off");
  const auto asr_on = voice.SetAsrEnabled(true);
  Require(asr_on.accepted && store.GetSnapshot().voice.asr_subscribed,
          "mock ASR receiver should be switchable on");
  Require(!voice.SetVolume(101).accepted,
          "volume above 100 must be rejected");
  const auto volume = voice.SetVolume(37);
  Require(volume.accepted && store.GetSnapshot().voice.volume_pct == 37,
          "mock volume change should be reflected in telemetry");

  const auto initial_voice = store.GetSnapshot().voice;
  Require(initial_voice.llm.mode == "builtin" &&
              !initial_voice.chat_go_closed &&
              initial_voice.llm.builtin_api_available &&
              initial_voice.llm.builtin_response_subscribed,
          "builtin LLM mode should expose chat_go DDS interaction by default");
  const auto invalid_mode = voice.ChatWithLlm("auto", "不能自动选择模式");
  Require(!invalid_mode.accepted &&
              invalid_mode.error == "invalid_llm_mode",
          "LLM chat requests must name an explicit supported mode");
  const auto builtin_reply =
      voice.ChatWithLlm("builtin", "你好，笨笨同学");
  Require(builtin_reply.accepted && !builtin_reply.pending &&
              builtin_reply.mode == "builtin" &&
              !builtin_reply.response.empty() &&
              store.GetSnapshot().voice.llm.request_state == "succeeded" &&
              store.GetSnapshot().voice.llm.last_user_message ==
                  "你好，笨笨同学",
          "mock builtin LLM should accept console text and expose a response");
  const auto customer_mode = voice.SetLlmMode(
      "customer", "https://customer.example/v1",
      "test-key", "customer-model");
  if (initial_voice.llm.customer_api_available) {
    Require(customer_mode.accepted &&
                store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "customer" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_api_key_configured,
            "customer LLM mode should normalize a base URL and retain API config");
    const auto serialized_voice =
        g1_web::SerializeVoiceStatus(store.GetSnapshot());
    Require(serialized_voice.find("customer_api_key_configured") !=
                std::string::npos &&
                serialized_voice.find("test-key") == std::string::npos,
            "telemetry may expose key presence but must never expose the API key");
    const auto preserved_key_mode = voice.SetLlmMode(
        "customer", "https://customer.example/v1", "",
        "customer-model-refresh", true);
    Require(preserved_key_mode.accepted &&
                store.GetSnapshot().voice.llm.customer_api_key_configured &&
                store.GetSnapshot().voice.llm.customer_model ==
                    "customer-model-refresh",
            "page refresh should be able to reuse an in-memory key without receiving it back");
    const auto wrong_endpoint_preserve = voice.SetLlmMode(
        "customer", "https://other.example/v1", "", "other-model", true);
    Require(!wrong_endpoint_preserve.accepted &&
                wrong_endpoint_preserve.error ==
                    "customer_api_key_endpoint_changed" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_api_key_configured,
            "an in-memory key must not be silently reused for a different API endpoint");
    store.PopulateMock(2.0);
    Require(store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "customer",
            "mock telemetry refresh must not overwrite the selected LLM mode");
    const auto stale_builtin_reply =
        voice.ChatWithLlm("builtin", "这条消息不能发布到内置 DDS");
    Require(!stale_builtin_reply.accepted &&
                stale_builtin_reply.error == "llm_mode_mismatch" &&
                store.GetSnapshot().voice.llm.last_user_message ==
                    "你好，笨笨同学",
            "customer mode must reject stale builtin requests before DDS publish");
    g1_web::CustomerVoiceConfig customer_config;
    customer_config.role_prompt = "你是展厅里的 G1 接待机器人，回答简洁。";
    customer_config.wake_word = "小兵小兵";
    customer_config.wake_enabled = true;
    customer_config.tts_backend = "kokoro";
    customer_config.qa_entries.push_back(
        {"你叫什么名字", "我是展厅里的 G1 接待机器人。"});
    const auto configured = voice.SetCustomerVoiceConfig(customer_config);
    Require(configured.accepted &&
                store.GetSnapshot().voice.llm.customer_role_prompt ==
                    customer_config.role_prompt &&
                store.GetSnapshot().voice.llm.customer_wake_enabled &&
                store.GetSnapshot().voice.llm.customer_wake_word ==
                    customer_config.wake_word &&
                store.GetSnapshot().voice.llm.customer_qa_count == 1 &&
                store.GetSnapshot().voice.llm.customer_tts_backend ==
                    "kokoro",
            "customer role, wake word, QA library, and TTS backend should be configurable");
    const auto builtin_before_saved_switch =
        voice.SetLlmMode("builtin", "", "", "");
    Require(builtin_before_saved_switch.accepted,
            "switching away from customer mode should succeed before saved-config reuse");
    const auto saved_customer_switch =
        voice.SetLlmMode("customer", "", "", "");
    Require(saved_customer_switch.accepted &&
                store.GetSnapshot().voice.llm.mode == "customer" &&
                store.GetSnapshot().voice.llm.customer_api_url ==
                    "https://customer.example/v1/chat/completions" &&
                store.GetSnapshot().voice.llm.customer_model ==
                    "customer-model-refresh",
            "customer mode switching without API fields should reuse the saved customer API config");
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9001,"text":"小兵，小兵","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Require(store.GetSnapshot().voice.tts.text == "我在" &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "speaking only the configured wake phrase should answer 我在");
    const auto wake_ack_request_id = store.GetSnapshot().voice.tts.request_id;
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9001,"text":"小兵，小兵","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.tts.request_id == wake_ack_request_id,
            "the same ASR index must not trigger the wake acknowledgement twice");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9002,"text":"你叫什么名字？","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa" &&
                store.GetSnapshot().voice.llm.last_response ==
                    "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.tts.text ==
                    "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "speech after a wake acknowledgement should route through QA/LLM without repeating the wake phrase");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9003,"text":"我是展厅里的 G1 接待机器人。","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？",
            "recent robot TTS must not feed back through ASR as a follow-up question");

    g1_web::VoiceServiceTestAccess::ExpireWakeSession(voice);
    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9004,"text":"这句不应该自动发送","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？",
            "speech after the wake follow-up window expires must not be auto-submitted");

    g1_web::VoiceServiceTestAccess::HandleAudioMessage(
        voice,
        R"({"index":9005,"text":"小兵，小兵，你叫什么名字？","is_final":false})");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    Require(store.GetSnapshot().voice.llm.last_user_message ==
                "你叫什么名字？" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa" &&
                store.GetSnapshot().voice.llm.last_response ==
                    "我是展厅里的 G1 接待机器人。",
            "wake phrase punctuation should be ignored and same-utterance trailing speech should route through QA/LLM");

    const auto qa_reply =
        voice.ChatWithLlm("customer", "你叫什么名字？");
    Require(qa_reply.accepted && qa_reply.response_source == "qa" &&
                qa_reply.response == "我是展厅里的 G1 接待机器人。" &&
                store.GetSnapshot().voice.llm.last_response_source == "qa",
            "customer QA library should answer matching questions before the model path");

    g1_web::CustomerVoiceConfig duplicate_qa_config = customer_config;
    duplicate_qa_config.qa_entries = {
        {"您好", "旧回答"},
        {"您好", "新回答"},
        {"天气怎么样", "今天天气很好"},
        {"您好", "最终回答"},
    };
    const auto deduplicated = voice.SetCustomerVoiceConfig(duplicate_qa_config);
    Require(deduplicated.accepted &&
                deduplicated.config.qa_entries.size() == 2 &&
                store.GetSnapshot().voice.llm.customer_qa_count == 2,
            "saving QA should replace duplicate questions instead of appending them");
    const auto latest_duplicate_reply = voice.ChatWithLlm("customer", "您好");
    Require(latest_duplicate_reply.accepted &&
                latest_duplicate_reply.response_source == "qa" &&
                latest_duplicate_reply.response == "最终回答",
            "the last answer for a repeated QA question should replace older answers");
    const auto restored_customer_config =
        voice.SetCustomerVoiceConfig(customer_config);
    Require(restored_customer_config.accepted &&
                restored_customer_config.config.qa_entries.size() == 1,
            "saving the original QA library should fully replace the temporary library");
    const auto customer_reply =
        voice.ChatWithLlm("customer", "你好，客户模型");
    Require(customer_reply.accepted && !customer_reply.pending &&
                customer_reply.request_id == 0 &&
                customer_reply.mode == "customer" &&
                customer_reply.response_source == "llm" &&
                customer_reply.response.find("模拟客户大模型回复") == 0 &&
                customer_reply.tts_request_id == 0 &&
                store.GetSnapshot().voice.llm.request_state == "succeeded",
            "mock customer mode must use the customer model path when QA does not match");
    const auto spoken_customer_reply =
        voice.ChatWithLlm("customer", "请自动播报", true);
    Require(spoken_customer_reply.accepted &&
                spoken_customer_reply.tts_request_id > 0 &&
                spoken_customer_reply.tts_error.empty(),
            "customer replies should enqueue automatic TTS when requested");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    Require(store.GetSnapshot().voice.tts.request_id ==
                spoken_customer_reply.tts_request_id &&
                store.GetSnapshot().voice.tts.state == "succeeded",
            "automatic customer reply TTS should complete in mock mode");
    const auto builtin_mode = voice.SetLlmMode("builtin", "", "", "");
    Require(builtin_mode.accepted &&
                !store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "builtin",
            "switching back to builtin mode should re-enable chat_go");
    store.PopulateMock(3.0);
    Require(!store.GetSnapshot().voice.chat_go_closed &&
                store.GetSnapshot().voice.llm.mode == "builtin",
            "mock telemetry refresh must preserve builtin LLM mode");
    const auto stale_customer_reply =
        voice.ChatWithLlm("customer", "这条消息不能调用客户 API");
    Require(!stale_customer_reply.accepted &&
                stale_customer_reply.error == "llm_mode_mismatch" &&
                store.GetSnapshot().voice.llm.last_user_message ==
                    "请自动播报",
            "builtin mode must reject stale customer requests before HTTP call");
  } else {
    Require(!customer_mode.accepted &&
                customer_mode.error == "customer_api_unavailable",
            "customer mode should report unavailable without libcurl");
  }

  const auto accepted = voice.EnqueueTts("语音测试", 0);
  Require(accepted.accepted && accepted.request_id > 0,
          "valid TTS should enter the queue");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto snapshot = store.GetSnapshot();
  Require(snapshot.voice.tts.request_id == accepted.request_id,
          "TTS request id should be retained");
  Require(snapshot.voice.tts.state == "succeeded",
          "mock TTS should finish successfully");
  const auto english = voice.EnqueueTts("Hello from Unitree", -1);
  Require(english.accepted && english.speaker_id == 1,
          "automatic language selection should choose English speaker 1");
  Require(!voice.EnqueueTts("Hello from Unitree", -1).accepted,
          "a rapid duplicate TTS request must be rejected");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  Require(store.GetSnapshot().voice.tts.speaker_id == 1,
          "resolved English speaker should reach the TTS worker");

  const auto saved_config = voice.GetCustomerVoiceConfig();
  Require(saved_config.accepted && saved_config.config.api_key_configured &&
              !saved_config.config.api_key.empty() &&
              saved_config.config.api_key != "test-key",
          "customer config API must expose only a masked saved-key indicator");
  Require(std::filesystem::exists(customer_config_path),
          "customer API and enhancement config should persist to disk");
  const auto config_permissions =
      std::filesystem::status(customer_config_path).permissions();
  const auto forbidden_permissions =
      std::filesystem::perms::group_read |
      std::filesystem::perms::group_write |
      std::filesystem::perms::group_exec |
      std::filesystem::perms::others_read |
      std::filesystem::perms::others_write |
      std::filesystem::perms::others_exec;
  Require((config_permissions & forbidden_permissions) ==
              std::filesystem::perms::none &&
              (config_permissions & std::filesystem::perms::owner_read) !=
                  std::filesystem::perms::none &&
              (config_permissions & std::filesystem::perms::owner_write) !=
                  std::filesystem::perms::none,
          "customer config containing API key must be owner read/write only");
  std::ifstream persisted_stream(customer_config_path);
  std::stringstream persisted_buffer;
  persisted_buffer << persisted_stream.rdbuf();
  Require(persisted_buffer.str().find("test-key") != std::string::npos,
          "persisted customer config should retain the API key across restart");

  // Simulate a user or an older build directly editing a polluted JSON file.
  // GET must reload the disk source, deduplicate by question (last answer wins),
  // update runtime state, and repair the file instead of preserving duplicates.
  Json::CharReaderBuilder disk_reader_builder;
  Json::Value polluted_root;
  std::string disk_parse_errors;
  {
    std::ifstream disk_input(customer_config_path);
    Require(Json::parseFromStream(disk_reader_builder, disk_input,
                                  &polluted_root, &disk_parse_errors),
            "persisted customer config should be valid JSON before disk-sync test");
  }
  Json::Value polluted_entries(Json::arrayValue);
  for (int i = 0; i < 9; ++i) {
    Json::Value item(Json::objectValue);
    item["question"] = "您好";
    item["answer"] = i == 8 ? "磁盘最新回答" : "磁盘旧回答";
    polluted_entries.append(item);
  }
  Json::Value second_item(Json::objectValue);
  second_item["question"] = "从文件同步";
  second_item["answer"] = "已经同步";
  polluted_entries.append(second_item);
  polluted_root["qa_entries"] = polluted_entries;
  {
    Json::StreamWriterBuilder disk_writer_builder;
    disk_writer_builder["indentation"] = "  ";
    std::ofstream disk_output(customer_config_path,
                              std::ios::binary | std::ios::trunc);
    disk_output << Json::writeString(disk_writer_builder, polluted_root);
  }
  const auto disk_synced_config = voice.GetCustomerVoiceConfig();
  Require(disk_synced_config.accepted &&
              disk_synced_config.config.qa_entries.size() == 2 &&
              disk_synced_config.config.qa_entries[0].question == "您好" &&
              disk_synced_config.config.qa_entries[0].answer == "磁盘最新回答",
          "customer config GET should reload disk QA and collapse repeated questions");
  const auto disk_synced_customer_mode = voice.SetLlmMode(
      "customer", "https://customer.example/v1", "",
      "customer-model-refresh", true);
  Require(disk_synced_customer_mode.accepted,
          "disk-sync QA runtime test should reactivate the persisted customer mode");
  const auto disk_synced_reply = voice.ChatWithLlm("customer", "您好");
  Require(disk_synced_reply.accepted &&
              disk_synced_reply.response_source == "qa" &&
              disk_synced_reply.response == "磁盘最新回答",
          "disk-reloaded fixed answer should immediately become active at runtime");
  {
    Json::Value repaired_root;
    std::string repaired_parse_errors;
    std::ifstream repaired_input(customer_config_path);
    Require(Json::parseFromStream(disk_reader_builder, repaired_input,
                                  &repaired_root, &repaired_parse_errors) &&
                repaired_root["qa_entries"].isArray() &&
                repaired_root["qa_entries"].size() == 2,
            "loading a polluted QA file should repair duplicates on disk");
  }

  voice.Stop();

  g1_web::SnapshotStore restarted_store;
  g1_web::VoiceService restarted_voice(restarted_store, true);
  std::string restart_error;
  Require(restarted_voice.Start(restart_error),
          "restarted voice service should load persisted customer config");
  const auto restarted_snapshot = restarted_store.GetSnapshot().voice;
  Require(restarted_snapshot.llm.customer_api_url ==
              "https://customer.example/v1/chat/completions" &&
              restarted_snapshot.llm.customer_model ==
                  "customer-model-refresh" &&
              restarted_snapshot.llm.customer_api_key_configured,
          "restart should restore API URL, model, and saved API key");
  const auto loaded_customer_mode = restarted_voice.SetLlmMode(
      "customer", "https://customer.example/v1", "",
      "customer-model-refresh", true);
  Require(loaded_customer_mode.accepted,
          "persisted API key should be reusable after service restart");
  const auto replacement_mode = restarted_voice.SetLlmMode(
      "customer", "https://customer.example/v1", "replacement-key",
      "customer-model-v2", false);
  Require(replacement_mode.accepted,
          "entering a new API key should replace the persisted old key");
  std::ifstream replaced_stream(customer_config_path);
  std::stringstream replaced_buffer;
  replaced_buffer << replaced_stream.rdbuf();
  Require(replaced_buffer.str().find("replacement-key") != std::string::npos &&
              replaced_buffer.str().find("test-key") == std::string::npos,
          "persisted config should atomically overwrite the previous API key");
  restarted_voice.Stop();
  std::filesystem::remove(customer_config_path, cleanup_error);
}

void TestMockControlSafety() {
  g1_web::SnapshotStore store;
  store.PopulateMock(0.0);
  g1_web::ControlService control(store, true);
  std::string error;
  Require(control.Start(error), "mock control service should start");
  Require(error.empty(), "mock control start should not report an error");

  const auto excessive_motion =
      control.SubmitVelocity(0.51F, 0.0F, 0.0F, 0, true);
  Require(!excessive_motion.accepted &&
              excessive_motion.error == "velocity_out_of_range",
          "velocity above the low preset limit must be rejected");

  const auto invalid_speed_mode =
      control.SubmitVelocity(0.5F, 0.0F, 0.0F, 2, true);
  Require(!invalid_speed_mode.accepted &&
              invalid_speed_mode.error == "invalid_speed_mode",
          "unexposed speed modes must be rejected");

  const auto conventional_medium =
      control.SubmitVelocity(1.0F, 0.35F, 0.8F, 1, true);
  Require(conventional_medium.accepted,
          "regular medium preset should keep its existing limits");
  const auto excessive_regular_lateral =
      control.SubmitVelocity(0.0F, 0.36F, 0.0F, 1, true);
  Require(!excessive_regular_lateral.accepted &&
              excessive_regular_lateral.error == "velocity_out_of_range",
          "regular medium lateral limit must not increase");

  const auto walkrun_required =
      control.SubmitVelocity(3.0F, 0.0F, 0.0F, 3, true);
  Require(!walkrun_required.accepted &&
              walkrun_required.error == "speed_mode_requires_walkrun",
          "high preset should require walk-run mode");

  const auto forward_motion =
      control.SubmitVelocity(0.5F, 0.0F, 0.0F, 0, true);
  Require(forward_motion.accepted,
          "safe low forward velocity should be accepted in locomotion FSM");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  Require(store.GetSnapshot().control.motion.state == "active",
          "mock velocity should become active");

  const auto stop_motion =
      control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);
  Require(stop_motion.accepted, "mock stop should always be accepted");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  Require(store.GetSnapshot().control.motion.state == "stopped",
          "mock stop should immediately clear motion");

  unitree_hg::msg::dds_::SportModeState_ walkrun;
  walkrun.fsm_id(802);
  walkrun.fsm_mode(0);
  walkrun.task_id(0);
  walkrun.task_time(0.0F);
  store.UpdateSportMode(walkrun);
  const auto walkrun_low =
      control.SubmitVelocity(0.5F, 0.4F, 1.1F, 0, true);
  Require(walkrun_low.accepted,
          "walk-run low preset should use the boosted movement limits");
  const auto walkrun_medium =
      control.SubmitVelocity(1.0F, 0.6F, 1.3F, 1, true);
  Require(walkrun_medium.accepted,
          "walk-run medium preset should use the boosted movement limits");
  const auto excessive_walkrun_medium =
      control.SubmitVelocity(0.0F, 0.61F, 0.0F, 1, true);
  Require(!excessive_walkrun_medium.accepted &&
              excessive_walkrun_medium.error == "velocity_out_of_range",
          "walk-run medium lateral limit must still be enforced");
  const auto maximum_forward =
      control.SubmitVelocity(3.0F, 0.0F, 0.0F, 3, true);
  Require(maximum_forward.accepted,
          "official 3.0 m/s high preset should be accepted in walk-run FSM");
  const auto turn_left =
      control.SubmitVelocity(0.0F, 0.0F, 1.3F, 1, true);
  Require(turn_left.accepted,
          "boosted walk-run medium yaw should be accepted for A/D turning");
  const auto maximum_turn =
      control.SubmitVelocity(0.0F, 0.0F, 1.5F, 3, true);
  Require(maximum_turn.accepted,
          "high preset should accept the 1.5 rad/s yaw ceiling");
  const auto excessive_turn =
      control.SubmitVelocity(0.0F, 0.0F, 1.51F, 3, true);
  Require(!excessive_turn.accepted &&
              excessive_turn.error == "velocity_out_of_range",
          "yaw velocity above the server ceiling must be rejected");
  control.SubmitVelocity(0.0F, 0.0F, 0.0F, 0, false);

  const auto wrong_confirmation = control.Submit(
      "request-key-0001", "mode", "damp", 0, false);
  Require(!wrong_confirmation.accepted &&
              wrong_confirmation.error == "confirmation_required",
          "mode command requires explicit confirmation");

  const auto damp = control.Submit(
      "request-key-0002", "mode", "damp", 0, true);
  Require(damp.accepted, "mock damping command should be accepted");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  Require(store.GetSnapshot().control.fsm_id == 1,
          "mock damping command should reach fsm 1");

  const auto duplicate = control.Submit(
      "request-key-0002", "mode", "damp", 0, true);
  Require(duplicate.accepted && duplicate.duplicate &&
              duplicate.request_id == damp.request_id,
          "duplicate request key must not execute twice");

  const auto zero_torque = control.Submit(
      "request-key-0003", "mode", "zero_torque", 0, true);
  Require(zero_torque.accepted,
          "mock zero-torque command should be accepted");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto snapshot = store.GetSnapshot();
  Require(snapshot.control.fsm_id == 0,
          "mock zero-torque command should reach fsm 0");
  Require(snapshot.control.last_command.state == "succeeded",
          "mock mode command should finish successfully");

  Require(g1_web::ControlService::IsKnownCommand(
              "arm_action", "execute", 27),
          "documented handshake action should be allowlisted");
  Require(!g1_web::ControlService::IsKnownCommand(
              "arm_action", "execute", 999),
          "unknown arm action must be rejected");
  Require(g1_web::ControlService::IsKnownCommand(
              "arm_action", "execute_custom", 0),
          "recorded teach actions should use the custom action command");
  Require(!control.Submit("request-key-0004", "arm_action",
                          "execute_custom", 0, true, "")
               .accepted,
          "teach action execution requires a non-empty action name");
  unitree_hg::msg::dds_::SportModeState_ arm_ready;
  arm_ready.fsm_id(500);
  arm_ready.fsm_mode(0);
  arm_ready.task_id(0);
  arm_ready.task_time(0.0F);
  store.UpdateSportMode(arm_ready);
  const auto teach_action = control.Submit(
      "request-key-0005", "arm_action", "execute_custom", 0, true,
      "Waist_Drum_Dance");
  Require(teach_action.accepted,
          "mock recorded teach action should be accepted in a safe arm FSM");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto teach_snapshot = store.GetSnapshot();
  Require(teach_snapshot.control.task_id == 100 &&
              teach_snapshot.control.last_command.action_name ==
                  "Waist_Drum_Dance" &&
              teach_snapshot.control.last_command.state == "succeeded",
          "teach action name and custom task id should reach telemetry");
  for (const int fsm_id :
       {0, 1, 2, 3, 4, 500, 501, 702, 706, 801, 802}) {
    Require(g1_web::ControlService::IsKnownCommand(
                "mode", "set_fsm_id", fsm_id),
            "documented FSM id should be allowlisted");
  }
  Require(!g1_web::ControlService::IsKnownCommand(
              "mode", "set_fsm_id", 999),
          "unknown FSM id must be rejected");
  control.Stop();
}

void TestMockJointDebugger() {
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  const auto teach_store = std::filesystem::temp_directory_path() /
      ("g1_web_joint_teach_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) +
       ".json");
  g1_web::SnapshotStore store;
  store.PopulateMock(0.0);
  g1_web::ControlService control(store, true, web_root.string(),
                                 teach_store.string());
  std::string error;
  Require(control.Start(error), "mock joint debugger should start");

  const auto initial_status = Parse(control.SerializeJointDebugStatus());
  Require(initial_status["joints"].size() == 29,
          "joint debugger should expose all 29 mapped joints");
  Require(initial_status["joints"][0]["lower"].asFloat() < 0.0F &&
              initial_status["joints"][0]["upper"].asFloat() > 0.0F,
          "joint debugger should load limits from the selected URDF");

  unitree_hg::msg::dds_::LowState_ ab_state;
  ab_state.mode_pr() = 1;
  ab_state.mode_machine() = 2;
  store.UpdateLowState(ab_state);
  const auto ab_status = Parse(control.SerializeJointDebugStatus());
  Require(ab_status["upper_body_allowed"].asBool(),
          "arm_sdk mode must not require PR debug mode");

  const auto leg_in_arm_mode = control.ApplyJointDebug(
      "upper_body", {{0, 0.0F}}, true);
  Require(!leg_in_arm_mode.accepted &&
              leg_in_arm_mode.error == "upper_body_joint_not_allowed",
          "upper-body mode must reject leg joints");

  std::vector<std::pair<std::size_t, float>> upper_targets;
  for (std::size_t index = 12; index < 29; ++index) {
    upper_targets.emplace_back(index, 0.0F);
  }
  unitree_hg::msg::dds_::SportModeState_ upper_state;
  upper_state.fsm_mode(0);
  upper_state.task_id(0);
  upper_state.task_time(0.0F);
  upper_state.fsm_id(0);
  store.UpdateSportMode(upper_state);
  const auto zero_torque_upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(!zero_torque_upper.accepted &&
              zero_torque_upper.error == "upper_body_fsm_not_allowed" &&
              control.GetJointDebugTestStats().publish_count == 0,
          "zero-torque FSM must not publish rt/arm_sdk");
  upper_state.fsm_id(1);
  store.UpdateSportMode(upper_state);
  const auto damping_upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(!damping_upper.accepted &&
              damping_upper.error == "upper_body_fsm_not_allowed" &&
              control.GetJointDebugTestStats().publish_count == 0,
          "damping FSM must not publish rt/arm_sdk");

  upper_state.fsm_id(500);
  store.UpdateSportMode(upper_state);
  control.SetMockJointDebugAiSport(true);
  const auto upper =
      control.ApplyJointDebug("upper_body", upper_targets, true);
  Require(upper.accepted,
          "upper-body arm_sdk must not depend on service state");
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  const auto active_status = Parse(control.SerializeJointDebugStatus());
  Require(active_status["dds_state"].asString() == "active",
          "upper-body Apply should directly start the arm_sdk loop");
  const auto first_stats = control.GetJointDebugTestStats();
  Require(first_stats.publish_count > 0 &&
              first_stats.maximum_step_rad <= 0.0101F,
          "Apply should publish bounded interpolation steps, not jump");
  Require(first_stats.last_kp[12] == 300.0F &&
              first_stats.last_kd[12] == 3.0F &&
              first_stats.last_kp[15] == 80.0F &&
              first_stats.last_kd[15] == 3.0F &&
              first_stats.last_kp[19] == 40.0F &&
              first_stats.last_kd[19] == 1.5F,
          "upper-body mode should use high, low and wrist gains by motor id");

  upper_targets[0].second = 0.1F;
  Require(control.ApplyJointDebug("upper_body", upper_targets, true).accepted,
          "repeat Apply should update the existing loop");
  Require(control.GetJointDebugTestStats().loop_starts ==
              first_stats.loop_starts,
          "repeat Apply must not create another control loop");
  Require(control.StopJointDebug(true).accepted,
          "upper-body debug control should stop");

  upper_state.fsm_id(802);
  store.UpdateSportMode(upper_state);
  Require(control.ApplyJointDebug("upper_body", upper_targets, true).accepted,
          "walk-run FSM should allow upper-body arm_sdk control");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  upper_state.fsm_id(1);
  store.UpdateSportMode(upper_state);
  Json::Value stopped_status;
  for (int attempt = 0; attempt < 100; ++attempt) {
    stopped_status = Parse(control.SerializeJointDebugStatus());
    if (stopped_status["dds_state"].asString() == "idle") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(stopped_status["dds_state"].asString() == "idle" &&
              stopped_status["arm_sdk_weight"].asFloat() == 0.0F,
          "leaving an allowed FSM must stop the arm_sdk loop");
  const auto stopped_publish_count =
      control.GetJointDebugTestStats().publish_count;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  Require(control.GetJointDebugTestStats().publish_count ==
              stopped_publish_count,
          "arm_sdk must not publish after entering damping FSM");
  store.PopulateMock(0.0);

  std::vector<std::pair<std::size_t, float>> full_targets;
  for (std::size_t index = 0; index < 29; ++index) {
    full_targets.emplace_back(index, 0.0F);
  }
  control.SetMockJointDebugAiSport(true);
  const auto blocked_full =
      control.ApplyJointDebug("full_body", full_targets, true);
  Require(!blocked_full.accepted &&
              blocked_full.error == "debug_mode_required" &&
              !control.GetJointDebugTestStats().lowcmd_published,
          "ai_sport must absolutely block rt/lowcmd publication");

  auto invalid_targets = full_targets;
  invalid_targets[0].second = 99.0F;
  control.SetMockJointDebugAiSport(false);
  const auto out_of_range =
      control.ApplyJointDebug("full_body", invalid_targets, true);
  Require(!out_of_range.accepted &&
              out_of_range.error == "joint_out_of_range",
          "backend must reject targets outside URDF limits");

  Require(control.ApplyJointDebug("full_body", full_targets, true).accepted,
          "full-body mock control should start only after ai_sport stops");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto full_stats = control.GetJointDebugTestStats();
  Require(full_stats.lowcmd_published,
          "allowed full-body mode should exercise the mock lowcmd writer");
  Require(full_stats.last_kp[0] == 300.0F &&
              full_stats.last_kd[0] == 3.0F &&
              full_stats.last_kp[4] == 80.0F &&
              full_stats.last_kd[4] == 3.0F &&
              full_stats.last_kp[5] == 300.0F &&
              full_stats.last_kp[12] == 300.0F &&
              full_stats.last_kp[15] == 80.0F &&
              full_stats.last_kp[19] == 40.0F &&
              full_stats.last_kd[19] == 1.5F,
          "full-body mode should use official gains for legs, waist and arms");
  Require(control.StopJointDebug(true).accepted,
          "full-body mock control should stop deterministically");

  store.PopulateMock(0.0);
  upper_state.fsm_id(500);
  store.UpdateSportMode(upper_state);
  const auto before_record = control.GetJointDebugTestStats().publish_count;
  Require(control.StartJointTeachRecording("mock_wave", true).accepted,
          "mock upper-body teach recording should start");
  Json::Value recording_status = Parse(control.SerializeJointDebugStatus());
  Require(recording_status["teach_state"].asString() == "recording" &&
              recording_status["dds_state"].asString() == "released" &&
              recording_status["arm_sdk_weight"].asFloat() == 1.0F,
          "teach recording must take arm_sdk ownership immediately");
  for (int attempt = 0; attempt < 10; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    if (control.GetJointDebugTestStats().publish_count > before_record) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto after_release = control.GetJointDebugTestStats();
  Require(after_release.publish_count > before_record &&
              after_release.last_kp[12] == 0.0F &&
              after_release.last_kd[12] == 10.0F &&
              after_release.last_kp[13] == 300.0F &&
              after_release.last_kd[13] == 3.0F &&
              after_release.last_kp[14] == 300.0F &&
              after_release.last_kd[14] == 3.0F &&
              after_release.last_kp[15] == 0.0F &&
              after_release.last_kd[15] == 1.5F &&
              after_release.last_kp[19] == 0.0F &&
              after_release.last_kd[19] == 0.5F &&
              after_release.last_kp[28] == 0.0F &&
              after_release.last_kd[28] == 0.5F,
          "teach recording must free waist yaw, lock waist roll/pitch and damp arms/wrists");
  const auto frames_before_over_limit =
      recording_status["teach_recorded_frames"].asUInt();
  unitree_hg::msg::dds_::LowState_ over_limit_state;
  over_limit_state.mode_machine() = 2;
  for (auto& motor : over_limit_state.motor_state()) motor.q(0.0F);
  over_limit_state.motor_state().at(14).q(0.7F);
  over_limit_state.motor_state().at(20).q(2.0F);
  store.UpdateLowState(over_limit_state);
  store.UpdateSportMode(upper_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  recording_status = Parse(control.SerializeJointDebugStatus());
  const auto held_stats = control.GetJointDebugTestStats();
  Require(recording_status["teach_recorded_frames"].asUInt() >
              frames_before_over_limit,
          "teach recording should sample a measured URDF limit overshoot");
  Require(std::abs(held_stats.last_q[14] - 0.028F) < 1e-4F &&
              std::abs(held_stats.last_q[15]) < 1e-4F,
          "teach waist must hold its start pose while arm targets follow LowState");
  const auto frames_after_release =
      recording_status["teach_recorded_frames"].asUInt();
  for (int attempt = 0; attempt < 110; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  recording_status = Parse(control.SerializeJointDebugStatus());
  Require(recording_status["teach_state"].asString() == "recording" &&
              recording_status["last_error"].asString() !=
                  "control_lease_expired" &&
              recording_status["teach_recorded_frames"].asUInt() >
                  frames_after_release &&
              control.GetJointDebugTestStats().publish_count >
                  after_release.publish_count,
          "passive teach must keep DDS ownership without a browser heartbeat");
  Require(control.FinishJointTeachRecording(true, true).accepted,
          "teach recording should clamp measured limit overshoot, save and release");
  const auto saved_status = Parse(control.SerializeJointDebugStatus());
  Require(saved_status["teach_actions"].size() == 1 &&
              saved_status["teach_actions"][0]["name"].asString() ==
                  "mock_wave" &&
              saved_status["teach_actions"][0]["frames"].asUInt() >= 2 &&
              saved_status["dds_state"].asString() == "idle" &&
              saved_status["arm_sdk_weight"].asFloat() == 0.0F,
          "save must persist the recording and return arm_sdk ownership");
  const auto permissions = std::filesystem::status(teach_store).permissions();
  Require((permissions & (std::filesystem::perms::group_all |
                          std::filesystem::perms::others_all)) ==
              std::filesystem::perms::none,
          "teach action store must not grant group or other permissions");
  const auto reserved_binding =
      control.SetJointTeachRemoteBinding("mock_wave", "L2+B");
  Require(!reserved_binding.accepted &&
              reserved_binding.error ==
                  "joint_teach_remote_binding_not_allowed",
          "official G1 remote combinations must be rejected for custom actions");
  const auto mislabeled_f2 =
      control.SetJointTeachRemoteBinding("mock_wave", "F2+A");
  Require(!mislabeled_f2.accepted &&
              mislabeled_f2.error ==
                  "joint_teach_remote_binding_not_allowed",
          "the physical G1 remote exposes F1/F3, not F2");
  Require(control.SetJointTeachRemoteBinding("mock_wave", "F3+A").accepted,
          "an unused F1/F3 remote combination should bind to a teach action");
  const auto bound_status = Parse(control.SerializeJointDebugStatus());
  Require(bound_status["remote_control_ready"].asBool() &&
              bound_status["remote_binding_options"].size() == 16 &&
              bound_status["remote_binding_options"][4]["label"].asString() ==
                  "F1+UP(↑)" &&
              bound_status["remote_binding_options"][5]["label"].asString() ==
                  "F1+RIGHT(→)" &&
              bound_status["remote_binding_options"][6]["label"].asString() ==
                  "F1+DOWN(↓)" &&
              bound_status["remote_binding_options"][7]["label"].asString() ==
                  "F1+LEFT(←)" &&
              bound_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "status should expose safe remote bindings with arrow labels and the saved assignment");

  store.PopulateMock(0.0);
  store.UpdateSportMode(upper_state);
  Require(control.StartJointTeachRecording("mock_wave", true).accepted,
          "second teach recording should start for hold-after-save mode");
  for (int attempt = 0; attempt < 150; ++attempt) {
    store.PopulateMock(0.0);
    store.UpdateSportMode(upper_state);
    if (Parse(control.SerializeJointDebugStatus())["teach_state"].asString() ==
        "recording")
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  for (int attempt = 0; attempt < 6; ++attempt) {
    store.PopulateMock(static_cast<double>(attempt) * 0.02);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(control.FinishJointTeachRecording(true, false).accepted,
          "teach recording should save without returning arm_sdk ownership");
  for (int attempt = 0; attempt < 110; ++attempt) {
    store.PopulateMock(0.12);
    store.UpdateSportMode(upper_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto held_status = Parse(control.SerializeJointDebugStatus());
  const auto holding_stats = control.GetJointDebugTestStats();
  Require(held_status["teach_state"].asString() == "holding" &&
              held_status["dds_state"].asString() == "active" &&
              held_status["arm_sdk_weight"].asFloat() == 1.0F &&
              held_status["teach_record_name"].asString().empty() &&
              held_status["teach_recorded_frames"].asUInt() == 0 &&
              held_status["last_error"].asString() !=
                  "control_lease_expired" &&
              held_status["teach_actions"].size() == 1 &&
              held_status["teach_actions"][0]["hold_after_playback"].asBool() &&
              held_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "hold-after-save must survive browser heartbeat loss while keeping arm_sdk ownership and preserve its remote binding");
  Require(holding_stats.last_kp[12] == 300.0F &&
              holding_stats.last_kd[12] == 3.0F &&
              holding_stats.last_kp[13] == 300.0F &&
              holding_stats.last_kd[13] == 3.0F &&
              holding_stats.last_kp[15] == 80.0F &&
              holding_stats.last_kd[15] == 3.0F &&
              holding_stats.last_kp[19] == 40.0F &&
              holding_stats.last_kd[19] == 1.5F,
          "held teach pose must use the normal upper-body gains");
  control.SetMockJointDebugRemoteKeys(0);
  control.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto released_hold_status = Parse(control.SerializeJointDebugStatus());
  Require(released_hold_status["dds_state"].asString() == "idle" &&
              released_hold_status["arm_sdk_weight"].asFloat() == 0.0F &&
              released_hold_status["remote_last_binding"].asString() ==
                  "F3+A",
          "pressing the held action binding again must release arm_sdk control");
  control.Stop();

  g1_web::ControlService restored(store, true, web_root.string(),
                                  teach_store.string());
  Require(restored.Start(error), "saved teach action should reload");
  const auto restored_status = Parse(restored.SerializeJointDebugStatus());
  Require(restored_status["teach_actions"].size() == 1 &&
              restored_status["teach_actions"][0]["hold_after_playback"].asBool() &&
              restored_status["teach_actions"][0]["remote_binding"].asString() ==
                  "F3+A",
          "hold-after-playback and remote-binding metadata should persist across service restart");
  store.PopulateMock(0.0);
  store.UpdateSportMode(upper_state);
  restored.SetMockJointDebugRemoteKeys(0);
  restored.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto playback_start_status =
      Parse(restored.SerializeJointDebugStatus());
  Require(playback_start_status["teach_state"].asString() == "playing" &&
              playback_start_status["arm_sdk_weight"].asFloat() == 1.0F,
          "teach playback must take arm_sdk ownership immediately");
  Json::Value playback_hold_status;
  for (int attempt = 0; attempt < 250; ++attempt) {
    store.PopulateMock(0.12);
    store.UpdateSportMode(upper_state);
    const auto current = Parse(restored.SerializeJointDebugStatus());
    if (current["dds_state"].asString() == "active")
      Require(restored.HeartbeatJointDebug().accepted,
              "playback and hold should accept the browser heartbeat");
    playback_hold_status = current;
    if (current["teach_state"].asString() == "holding") break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  Require(playback_hold_status["teach_state"].asString() == "holding" &&
              playback_hold_status["dds_state"].asString() == "active" &&
              playback_hold_status["arm_sdk_weight"].asFloat() == 1.0F,
          "a hold-recorded action must keep its last pose after playback");
  restored.SetMockJointDebugRemoteKeys(0);
  restored.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto restored_released = Parse(restored.SerializeJointDebugStatus());
  Require(restored_released["dds_state"].asString() == "idle" &&
              restored_released["arm_sdk_weight"].asFloat() == 0.0F &&
              restored_released["remote_last_binding"].asString() == "F3+A",
          "pressing the held action binding again must restore control");
  Require(restored.DeleteJointTeachAction("mock_wave", true).accepted,
          "saved teach action should be deletable");
  Require(Parse(restored.SerializeJointDebugStatus())["teach_actions"].empty(),
          "deleted teach action should disappear from status");
  restored.Stop();
  std::error_code ignored;
  std::filesystem::remove(teach_store, ignored);
}

}  // namespace

int main() {
  TestJointNames();
  TestG1ModelCatalog();
  TestStaticAssets();
  TestFreshness();
  TestSerialization();
  TestPointCloudDecode();
  TestPointCloudWebFilter();
  TestLowObstacleSafetyDetector();
  TestMockNavigationBridge();
  TestMockPerceptionSafety();
  TestMockCameraCommands();
  TestConcurrentSnapshotAccess();
  TestMockVoiceService();
  TestMockControlSafety();
  TestMockJointDebugger();
  std::cout << "All UniRoboGui core tests passed.\n";
  return EXIT_SUCCESS;
}
