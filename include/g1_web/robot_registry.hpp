#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "g1_web/audio_capability.hpp"
#include "g1_web/device_capability.hpp"
#include "g1_web/joint_debug_policy.hpp"
#include "g1_web/locomotion.hpp"
#include "g1_web/robot_profile.hpp"

namespace g1_web {

enum class TelemetrySourceKey {
  kLowState,
  kBms,
  kSecondaryImu,
  kMainBoard,
  kOdometry,
  kSportMode,
};

enum class TelemetrySourceKind {
  kHgLowState,
  kHgBmsState,
  kHgMainBoardState,
  kHgImuState,
  kGo2SportModeState,
  kHgSportModeState,
};

struct TelemetrySourceSpec {
  TelemetrySourceKey key;
  TelemetrySourceKind kind;
  std::string topic;
  bool required_for_health{false};
};

struct TelemetrySubscriptionPlan {
  std::vector<TelemetrySourceSpec> sources;
};

const char* TelemetrySourceKeyName(TelemetrySourceKey key);

struct RuntimeOptions {
  bool mock{false};
  std::string network_interface{"eth0"};
  std::string web_root{"/home/unitree/UniRoboGui/web"};
  bool enable_navigation{false};
  bool realsense{false};
  std::string rgb_camera;
  std::string depth_camera;
  std::set<std::string> enabled_devices;
};

struct RobotRuntimeBundle {
  RobotProfile profile;
  TelemetrySubscriptionPlan telemetry;
  std::unique_ptr<ILocomotion> locomotion;
  std::unique_ptr<IAudioCapability> audio;
  std::unique_ptr<IJointDebugPolicy> joint_debug;
  std::unique_ptr<IDeviceCapabilityPolicy> devices;
};

class RobotRegistry {
 public:
  static const RobotProfile* Find(const std::string& product_id);
  static std::vector<const RobotProfile*> All();
  static RobotRuntimeBundle CreateRuntime(const std::string& product_id,
                                          const RuntimeOptions& options);
};

}  // namespace g1_web
