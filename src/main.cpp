#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include "g1_web/control_service.hpp"
#include "g1_web/camera_service.hpp"
#include "g1_web/http_server.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/robot_registry.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/unitree_data_source.hpp"
#include "g1_web/voice_service.hpp"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

std::string RegisteredProductsText() {
  std::string result;
  for (const auto* profile : g1_web::RobotRegistry::All()) {
    if (!result.empty()) result += ", ";
    result += profile->identity.product_id;
  }
  return result.empty() ? "<none>" : result;
}

struct Options {
  g1_web::RuntimeOptions runtime;
  std::string bind_address{"0.0.0.0"};
  std::uint16_t port{8080};
  unsigned int publish_hz{10};
  std::string robot{"g1"};
};

void PrintUsage(const char* program) {
  std::cout
      << "用法: " << program << " [选项]\n"
      << "  --interface <网卡>   DDS 网卡，默认 eth0\n"
      << "  --bind <地址>        HTTP 监听地址，默认 0.0.0.0\n"
      << "  --port <端口>        HTTP 端口，默认 8080\n"
      << "  --publish-hz <频率>  WebSocket 推送频率，默认 10\n"
      << "  --web-root <目录>    静态网页目录\n"
      << "  --robot <产品>       机器人产品，已注册: " << RegisteredProductsText()
      << "；缺省为 g1\n"
      << "  --device <附件>      声明已确认安装的附件，可重复；如 mid360\n"
      << "  --mock               使用模拟数据，不初始化 DDS\n"
      << "  --enable-navigation  允许 Web 调用真实 SLAM 位姿导航（默认禁用）\n"
      << "  --realsense          使用 librealsense2 同时获取 D435i RGB/深度\n"
      << "  --rgb-camera <设备>  RGB V4L2 设备，如 /dev/video0\n"
      << "  --depth-camera <设备> 深度/IR V4L2 设备，如 /dev/video2\n"
      << "  --help               显示帮助\n";
}

bool ParseUnsigned(const std::string& value, unsigned long minimum,
                   unsigned long maximum, unsigned long& result) {
  try {
    std::size_t used = 0;
    const auto parsed = std::stoul(value, &used);
    if (used != value.size() || parsed < minimum || parsed > maximum) {
      return false;
    }
    result = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool CapabilityAvailable(const g1_web::RobotProfile& profile,
                         g1_web::CapabilityKey key) {
  for (const auto& capability : profile.capabilities) {
    if (capability.key == key) {
      return capability.implemented && capability.available;
    }
  }
  return false;
}

[[noreturn]] void RestartWithRobot(int argc, char** argv,
                                   const std::string& product_id) {
  std::vector<std::string> arguments;
  arguments.reserve(static_cast<std::size_t>(argc) + 2);
  bool replaced = false;
  for (int i = 0; i < argc; ++i) {
    if (i > 0 && std::string(argv[i]) == "--robot") {
      arguments.emplace_back("--robot");
      arguments.push_back(product_id);
      if (i + 1 < argc) ++i;
      replaced = true;
      continue;
    }
    arguments.emplace_back(argv[i]);
  }
  if (!replaced) {
    arguments.emplace_back("--robot");
    arguments.push_back(product_id);
  }

  std::vector<char*> raw_arguments;
  raw_arguments.reserve(arguments.size() + 1);
  for (auto& argument : arguments) raw_arguments.push_back(argument.data());
  raw_arguments.push_back(nullptr);
  ::execv("/proc/self/exe", raw_arguments.data());
  std::perror("execv /proc/self/exe");
  std::exit(EXIT_FAILURE);
}

bool ParseOptions(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--help") {
      PrintUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }
    if (argument == "--mock") {
      options.runtime.mock = true;
      continue;
    }
    if (argument == "--enable-navigation") {
      options.runtime.enable_navigation = true;
      continue;
    }
    if (argument == "--realsense") {
      options.runtime.realsense = true;
      continue;
    }
    if (i + 1 >= argc) {
      std::cerr << "缺少参数值: " << argument << '\n';
      return false;
    }
    const std::string value(argv[++i]);
    if (argument == "--interface") {
      options.runtime.network_interface = value;
    } else if (argument == "--bind") {
      options.bind_address = value;
    } else if (argument == "--web-root") {
      options.runtime.web_root = value;
    } else if (argument == "--robot") {
      options.robot = value;
    } else if (argument == "--device") {
      if (value.empty()) {
        std::cerr << "附件标识不能为空\n";
        return false;
      }
      options.runtime.enabled_devices.insert(value);
    } else if (argument == "--rgb-camera") {
      options.runtime.rgb_camera = value;
    } else if (argument == "--depth-camera") {
      options.runtime.depth_camera = value;
    } else if (argument == "--port") {
      unsigned long parsed = 0;
      if (!ParseUnsigned(value, 1, 65535, parsed)) {
        std::cerr << "无效端口: " << value << '\n';
        return false;
      }
      options.port = static_cast<std::uint16_t>(parsed);
    } else if (argument == "--publish-hz") {
      unsigned long parsed = 0;
      if (!ParseUnsigned(value, 1, 60, parsed)) {
        std::cerr << "推送频率必须在 1 到 60 Hz 之间: " << value << '\n';
        return false;
      }
      options.publish_hz = static_cast<unsigned int>(parsed);
    } else {
      std::cerr << "未知参数: " << argument << '\n';
      return false;
    }
  }
  if (options.runtime.realsense &&
      (!options.runtime.rgb_camera.empty() ||
       !options.runtime.depth_camera.empty())) {
    std::cerr << "--realsense 不能与 --rgb-camera/--depth-camera 同时使用\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::cout.setf(std::ios::unitbuf);
  std::cerr.setf(std::ios::unitbuf);

  Options options;
  if (!ParseOptions(argc, argv, options)) {
    PrintUsage(argv[0]);
    return EXIT_FAILURE;
  }

  g1_web::RobotRuntimeBundle runtime;
  try {
    runtime = g1_web::RobotRegistry::CreateRuntime(options.robot,
                                                    options.runtime);
  } catch (const std::invalid_argument&) {
    std::cerr << "[ERROR] 未注册机器人产品: " << options.robot
              << "（当前已注册: " << RegisteredProductsText() << "）\n";
    return EXIT_FAILURE;
  } catch (const std::exception& exception) {
    std::cerr << "[ERROR] 机器人运行时创建失败: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
  auto& active_profile = runtime.profile;
  std::cout << "[INFO] 当前机器人产品: "
            << active_profile.identity.display_name << " ("
            << active_profile.identity.product_id << ")\n";

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  g1_web::SnapshotStore store;
  g1_web::UnitreeDataSource unitree_source(store);
  g1_web::MockDataSource mock_source(store, active_profile);
  g1_web::VoiceService voice_service(store, std::move(runtime.audio),
                                     options.runtime.mock);
  g1_web::ControlService control_service(
      store, std::move(runtime.locomotion), active_profile,
      std::move(runtime.joint_debug), options.runtime.mock,
      options.runtime.web_root);
  g1_web::PerceptionService perception_service(
      options.runtime.mock, options.runtime.enable_navigation,
      *runtime.devices);
  g1_web::CameraOptions camera_options;
  camera_options.mock = options.runtime.mock;
  camera_options.realsense = options.runtime.realsense;
  camera_options.rgb_source = options.runtime.rgb_camera;
  camera_options.depth_source = options.runtime.depth_camera;
  g1_web::CameraService camera_service(camera_options, *runtime.devices);

  if (options.runtime.mock) {
    mock_source.Start();
    std::cout << "[INFO] 已启用模拟数据源\n";
  } else {
    std::string dds_error;
    if (!unitree_source.Start(options.runtime.network_interface, dds_error,
                              runtime.telemetry)) {
      std::cerr << "[WARN] DDS 初始化失败，Web 服务将以降级状态启动: "
                << dds_error << '\n';
    } else {
      std::cout << "[INFO] DDS 只读订阅已启动，网卡: "
                << options.runtime.network_interface << '\n';
    }
  }

  g1_web::ApplyEffectiveDeviceCapabilities(
      active_profile, perception_service.ProbeDeviceCapabilities());
  if (runtime.devices->Camera().streams.empty()) {
    g1_web::ApplyEffectiveDeviceCapabilities(
        active_profile, camera_service.DeviceCapabilities());
  }

  const bool audio_enabled =
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kAudio);
  if (audio_enabled) {
    std::string voice_error;
    if (!voice_service.Start(voice_error)) {
      std::cerr << "[WARN] 语音服务处于降级状态: " << voice_error << '\n';
    } else {
      std::cout << "[INFO] ASR/TTS 已启动，默认启用 chat_go 笨笨同学；客户大模型可按需切换\n";
    }
  } else {
    std::cout << "[INFO] 当前产品语音能力保持禁用\n";
  }

  const bool control_enabled =
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kLocomotion) ||
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kJointDebug) ||
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kJointTeach);
  if (control_enabled) {
    std::string control_error;
    if (!control_service.Start(control_error)) {
      std::cerr << "[WARN] 控制服务处于禁用状态: "
                << control_error << '\n';
    } else {
      std::cout << "[INFO] 控制服务已启用；访问认证已关闭\n";
    }
  } else {
    std::cout << "[INFO] 当前产品实体控制能力保持禁用\n";
  }

  const bool perception_enabled =
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kLidar) ||
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kSlam);
  if (perception_enabled) {
    std::string perception_error;
    if (!perception_service.Start(perception_error)) {
      std::cerr << "[WARN] SLAM 感知服务处于降级状态: "
                << perception_error << '\n';
    } else {
      std::cout << "[INFO] SLAM 点云与里程计订阅已启用；真实导航 "
                << (options.runtime.enable_navigation ? "已显式开启" : "保持禁用")
                << '\n';
    }
  } else {
    std::cout << "[INFO] 当前产品 SLAM/LiDAR 能力未验证，保持禁用\n";
  }

  const bool camera_enabled =
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kCameraRgb) ||
      CapabilityAvailable(active_profile, g1_web::CapabilityKey::kCameraDepth);
  if (camera_enabled) {
    std::string camera_error;
    if (!camera_service.Start(camera_error)) {
      if (camera_error == "camera_sources_not_configured") {
        std::cout << "[INFO] 摄像头服务待命；页面启动时自动检测当前设备\n";
      } else {
        std::cerr << "[WARN] 摄像头服务处于降级状态: "
                  << camera_error << '\n';
      }
    } else {
      std::cout << "[INFO] RGB/深度摄像头服务已启动\n";
    }
  } else {
    std::cout << "[INFO] 当前产品摄像头能力未验证，保持禁用\n";
  }

  std::mutex robot_switch_mutex;
  std::string requested_robot;
  std::chrono::steady_clock::time_point robot_switch_after{};
  auto request_robot_switch = [&](const std::string& product_id) {
    std::lock_guard<std::mutex> lock(robot_switch_mutex);
    if (!requested_robot.empty()) return;
    requested_robot = product_id;
    robot_switch_after =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  };
  std::function<void(const std::string&)> robot_switch_handler;
  if (options.runtime.mock) {
    robot_switch_handler = request_robot_switch;
  }

  g1_web::HttpServer server(store, voice_service, control_service,
                            perception_service, camera_service,
                            active_profile, options.bind_address,
                            options.port, options.runtime.web_root,
                            options.publish_hz, std::move(robot_switch_handler));
  std::string server_error;
  if (!server.Start(server_error)) {
    std::cerr << "[ERROR] Web 服务启动失败: " << server_error << '\n';
    control_service.Stop();
    camera_service.Stop();
    perception_service.Stop();
    voice_service.Stop();
    mock_source.Stop();
    unitree_source.Stop();
    return EXIT_FAILURE;
  }

  std::cout << "[INFO] 机器人信息面板已启动: http://"
            << options.bind_address << ':' << options.port << '\n'
            << "[INFO] WebSocket 推送频率: " << options.publish_hz
            << " Hz\n"
            << "[INFO] 按 Ctrl+C 安全停止\n";

  std::string next_robot;
  while (!g_stop_requested) {
    {
      std::lock_guard<std::mutex> lock(robot_switch_mutex);
      if (!requested_robot.empty() &&
          std::chrono::steady_clock::now() >= robot_switch_after) {
        next_robot = requested_robot;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!next_robot.empty()) {
    std::cout << "\n[INFO] 正在切换机器人产品: " << next_robot << "\n";
  } else {
    std::cout << "\n[INFO] 正在停止服务...\n";
  }
  server.Stop();
  std::cout << "[INFO] HTTP 服务已停止\n";
  camera_service.Stop();
  std::cout << "[INFO] 摄像头服务已停止\n";
  perception_service.Stop();
  std::cout << "[INFO] SLAM 感知服务已停止\n";
  control_service.Stop();
  std::cout << "[INFO] 控制服务已停止\n";
  voice_service.Stop();
  std::cout << "[INFO] 语音服务已停止\n";
  mock_source.Stop();
  unitree_source.Stop();
  std::cout << "[INFO] 服务已安全停止\n";
  if (!next_robot.empty()) RestartWithRobot(argc, argv, next_robot);
  return EXIT_SUCCESS;
}
