#include "g1_web/control_service.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

#include <json/json.h>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "g1_web/g1_model_catalog.hpp"
#include "g1_web/joint_names.hpp"

namespace g1_web {
namespace {

constexpr char kArmTopic[] = "rt/arm_sdk";
constexpr char kLowCmdTopic[] = "rt/lowcmd";
constexpr char kRemoteTopic[] = "rt/wirelesscontroller";
constexpr char kSportService[] = "ai_sport";
constexpr float kArmPeriodSeconds = 0.02F;
constexpr float kBodyPeriodSeconds = 0.002F;
constexpr float kArmMaximumVelocity = 0.5F;
constexpr float kBodyMaximumVelocity = 0.25F;
constexpr float kWeightRate = 0.5F;
constexpr float kTeachWaistYawKd = 10.0F;
constexpr float kTeachArmKd = 1.5F;
constexpr float kTeachWristKd = 0.5F;
constexpr auto kTeachSamplePeriod = std::chrono::milliseconds(50);
constexpr std::size_t kTeachJointCount = 17;
constexpr std::size_t kMaximumTeachFrames = 2400;

struct RemoteBinding {
  const char* id;
  std::uint16_t mask;
};

// The current G1 handset is labeled F1/F3. SDK2 still exposes the second
// function-key bit as F2, so F3 below intentionally maps to bit 7.
constexpr std::uint16_t kRemoteStartMask = 1U << 2U;
constexpr std::array<RemoteBinding, 16> kRemoteBindings{{
    {"F1+A", (1U << 6U) | (1U << 8U)},
    {"F1+B", (1U << 6U) | (1U << 9U)},
    {"F1+X", (1U << 6U) | (1U << 10U)},
    {"F1+Y", (1U << 6U) | (1U << 11U)},
    {"F1+UP", (1U << 6U) | (1U << 12U)},
    {"F1+RIGHT", (1U << 6U) | (1U << 13U)},
    {"F1+DOWN", (1U << 6U) | (1U << 14U)},
    {"F1+LEFT", (1U << 6U) | (1U << 15U)},
    {"F3+A", (1U << 7U) | (1U << 8U)},
    {"F3+B", (1U << 7U) | (1U << 9U)},
    {"F3+X", (1U << 7U) | (1U << 10U)},
    {"F3+Y", (1U << 7U) | (1U << 11U)},
    {"F3+UP", (1U << 7U) | (1U << 12U)},
    {"F3+RIGHT", (1U << 7U) | (1U << 13U)},
    {"F3+DOWN", (1U << 7U) | (1U << 14U)},
    {"F3+LEFT", (1U << 7U) | (1U << 15U)},
}};

std::uint16_t RemoteBindingMask(const std::string& id) {
  const auto binding = std::find_if(
      kRemoteBindings.begin(), kRemoteBindings.end(),
      [&](const auto& item) { return id == item.id; });
  return binding == kRemoteBindings.end() ? 0 : binding->mask;
}

std::string RemoteBindingLabel(const std::string& id) {
  if (id.find("+UP") != std::string::npos) return id + "(↑)";
  if (id.find("+RIGHT") != std::string::npos) return id + "(→)";
  if (id.find("+DOWN") != std::string::npos) return id + "(↓)";
  if (id.find("+LEFT") != std::string::npos) return id + "(←)";
  return id;
}

// unitreerobotics/xr_teleoperate G1_29_ArmController motor classes:
// high 300/3, weak 80/3 (ankle pitch, shoulders, elbows), wrist 40/1.5.
constexpr std::array<float, kNamedJointCount> kKp{{
    300, 300, 300, 300, 80, 300, 300, 300, 300, 300, 80, 300,
    300, 300, 300, 80, 80, 80, 80, 40, 40, 40, 80, 80,
    80, 80, 40, 40, 40}};
constexpr std::array<float, kNamedJointCount> kKd{{
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 1.5F, 1.5F, 1.5F, 3, 3, 3, 3, 1.5F, 1.5F,
    1.5F}};

constexpr float TeachDampingKd(std::size_t index) {
  if (index == 12) return kTeachWaistYawKd;
  if ((index >= 19 && index <= 21) || index >= 26) return kTeachWristKd;
  return kTeachArmKd;
}

std::uint32_t Crc32Core(std::uint32_t* ptr, std::uint32_t len) {
  std::uint32_t crc = 0xFFFFFFFF;
  constexpr std::uint32_t polynomial = 0x04c11db7;
  for (std::uint32_t i = 0; i < len; ++i) {
    std::uint32_t xbit = 1U << 31;
    const std::uint32_t data = ptr[i];
    for (std::uint32_t bit = 0; bit < 32; ++bit) {
      crc = (crc & 0x80000000U) ? (crc << 1U) ^ polynomial : crc << 1U;
      if (data & xbit) crc ^= polynomial;
      xbit >>= 1U;
    }
  }
  return crc;
}

std::string Attribute(const std::string& tag, const char* name) {
  const std::regex pattern(std::string("\\b") + name +
                           R"(\s*=\s*["']([^"']+)["'])");
  std::smatch match;
  return std::regex_search(tag, match, pattern) ? match[1].str() : "";
}

bool LowStateFresh(const RobotSnapshot& snapshot) {
  const auto& source =
      snapshot.sources[static_cast<std::size_t>(SourceId::kLowState)];
  return source.received &&
         SteadyClock::now() - source.last_update <= std::chrono::seconds(1);
}

bool SportStateFresh(const ControlData& control) {
  return control.sport_state_received &&
         SteadyClock::now() - control.sport_state_last_update <=
             std::chrono::seconds(1);
}

bool UpperBodyFsmAllowed(std::uint32_t fsm_id) {
  return fsm_id == 500 || fsm_id == 501 || fsm_id == 801 ||
         fsm_id == 802;
}

std::string WriteJson(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

}  // namespace

class ControlService::JointDebugImpl {
 public:
  struct Limit {
    float lower{0.0F};
    float upper{0.0F};
    bool movable{false};
  };

  struct TeachAction {
    std::string name;
    std::uint8_t mode_machine{0};
    bool hold_after_playback{false};
    std::string remote_binding;
    std::vector<std::array<float, kTeachJointCount>> frames;
  };

  JointDebugImpl(ControlService& owner, SnapshotStore& store, bool mock,
                 std::string web_root, std::string teach_store)
      : owner(owner), store(store), mock(mock), web_root(std::move(web_root)),
        teach_store(teach_store.empty()
                        ? (std::filesystem::path(this->web_root).parent_path() /
                           "config/joint_teach_actions.json")
                              .string()
                        : std::move(teach_store)) {}

  ~JointDebugImpl() { Stop(); }

  void Start() {
    std::lock_guard<std::mutex> lock(mutex);
    if (initialized) return;
    try {
      if (!mock) {
        robot_state =
            std::make_unique<unitree::robot::b2::RobotStateClient>();
        robot_state->SetTimeout(2.0F);
        robot_state->Init();
        arm_publisher = std::make_shared<unitree::robot::ChannelPublisher<
            unitree_hg::msg::dds_::LowCmd_>>(kArmTopic);
        arm_publisher->InitChannel();
        lowcmd_publisher = std::make_shared<unitree::robot::ChannelPublisher<
            unitree_hg::msg::dds_::LowCmd_>>(kLowCmdTopic);
        lowcmd_publisher->InitChannel();
        try {
          remote_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_go::msg::dds_::WirelessController_>>(kRemoteTopic);
          remote_subscriber->InitChannel(
              [this](const void* data) {
                HandleRemoteKeys(static_cast<const unitree_go::msg::dds_::
                    WirelessController_*>(data)->keys());
              },
              1);
          remote_control_ready = true;
          remote_control_error.clear();
        } catch (const std::exception& error) {
          remote_subscriber.reset();
          remote_control_ready = false;
          remote_control_error = error.what();
        } catch (...) {
          remote_subscriber.reset();
          remote_control_ready = false;
          remote_control_error = "remote_control_initialization_failed";
        }
      } else {
        remote_control_ready = true;
        remote_control_error.clear();
      }
      try {
        LoadTeachActions();
        teach_store_error.clear();
      } catch (const std::exception& error) {
        teach_store_error = error.what();
      }
      initialized = true;
      initialization_error.clear();
    } catch (const std::exception& error) {
      initialization_error = error.what();
    } catch (...) {
      initialization_error = "joint_debug_initialization_failed";
    }
  }

  void Stop() {
    if (remote_subscriber) {
      remote_subscriber->CloseChannel();
      remote_subscriber.reset();
    }
    remote_control_ready = false;
    RequestStop();
    if (worker.joinable()) worker.join();
    std::lock_guard<std::mutex> lock(mutex);
    active = false;
    stopping = false;
    mode = "idle";
    arm_publisher.reset();
    lowcmd_publisher.reset();
    robot_state.reset();
    initialized = false;
  }

  void RequestStop() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!active) return;
      stopping = true;
    }
    cv.notify_all();
  }

  void JoinStopped() {
    if (worker.joinable()) worker.join();
  }

  bool IsActive() const {
    std::lock_guard<std::mutex> lock(mutex);
    return active;
  }

  void HandleRemoteKeys(std::uint16_t keys) {
    std::string action_name;
    std::string binding_id;
    bool release_hold = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      const std::uint16_t previous = remote_keys;
      remote_keys = keys;
      release_hold = teach_state == "holding" &&
                     (keys & kRemoteStartMask) != 0 &&
                     (previous & kRemoteStartMask) == 0;
      if (teach_state == "holding" && !release_hold &&
          !holding_action_name.empty()) {
        const auto action = std::find_if(
            teach_actions.begin(), teach_actions.end(),
            [&](const auto& item) { return item.name == holding_action_name; });
        if (action != teach_actions.end()) {
          const auto mask = RemoteBindingMask(action->remote_binding);
          if (mask != 0 && keys == mask && (previous & mask) != mask) {
            release_hold = true;
            binding_id = action->remote_binding;
          }
        }
      }
      if (!release_hold) {
        for (const auto& action : teach_actions) {
          const auto mask = RemoteBindingMask(action.remote_binding);
          if (mask != 0 && keys == mask && (previous & mask) != mask) {
            action_name = action.name;
            binding_id = action.remote_binding;
            break;
          }
        }
      }
    }
    if (release_hold) {
      const auto result = owner.StopJointDebug(true);
      std::lock_guard<std::mutex> lock(mutex);
      remote_last_action = "release_control";
      remote_last_binding = binding_id.empty() ? "START" : binding_id;
      if (!result.accepted) last_error = result.error;
      return;
    }
    if (action_name.empty()) return;
    const auto result = owner.PlayJointTeachAction(action_name, true);
    std::lock_guard<std::mutex> lock(mutex);
    remote_last_action = action_name;
    remote_last_binding = binding_id;
    if (!result.accepted) last_error = result.error;
  }

  bool RefreshAiSport(std::string& error) {
    if (mock) {
      std::lock_guard<std::mutex> lock(mutex);
      ai_sport_found = true;
      ai_sport_active = mock_ai_sport_active;
      error.clear();
      return true;
    }
    std::lock_guard<std::mutex> service_lock(service_mutex);
    std::vector<unitree::robot::b2::ServiceState> services;
    const auto result = robot_state->ServiceList(services);
    if (result != 0) {
      error = "service_state_unavailable";
      return false;
    }
    bool next_sport_found = false;
    bool next_sport_active = false;
    for (const auto& service : services) {
      if (service.name == kSportService) {
        next_sport_found = true;
        next_sport_active = service.status == 0;
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      ai_sport_found = next_sport_found;
      ai_sport_active = next_sport_active;
    }
    error.clear();
    return true;
  }

  bool LoadLimits(std::uint8_t mode_machine, std::string& error) {
    std::lock_guard<std::mutex> lock(mutex);
    if (limits_loaded && limits_mode == mode_machine) return true;
    const auto* model = FindG1Model(mode_machine);
    if (!model) {
      error = "unsupported_model";
      return false;
    }
    const std::filesystem::path path =
        std::filesystem::path(web_root) / "assets/unitree/g1_description" /
        std::string(model->urdf_file);
    std::ifstream input(path);
    if (!input) {
      error = "urdf_unavailable";
      return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string xml = buffer.str();
    std::array<Limit, kNamedJointCount> parsed{};
    std::size_t found = 0;
    std::size_t cursor = 0;
    while ((cursor = xml.find("<joint ", cursor)) != std::string::npos) {
      const auto tag_end = xml.find('>', cursor);
      const auto close = xml.find("</joint>", tag_end);
      if (tag_end == std::string::npos || close == std::string::npos) break;
      const std::string tag = xml.substr(cursor, tag_end - cursor + 1);
      const std::string name = Attribute(tag, "name");
      for (std::size_t index = 0; index < JointNames().size(); ++index) {
        if (name != std::string(JointNames()[index].name) + "_joint") continue;
        const std::string type = Attribute(tag, "type");
        if (type == "revolute") {
          const auto limit_begin = xml.find("<limit", tag_end);
          if (limit_begin == std::string::npos || limit_begin > close) break;
          const auto limit_end = xml.find('>', limit_begin);
          const std::string limit_tag =
              xml.substr(limit_begin, limit_end - limit_begin + 1);
          try {
            parsed[index].lower = std::stof(Attribute(limit_tag, "lower"));
            parsed[index].upper = std::stof(Attribute(limit_tag, "upper"));
          } catch (...) {
            error = "invalid_urdf_limit";
            return false;
          }
          parsed[index].movable =
              std::isfinite(parsed[index].lower) &&
              std::isfinite(parsed[index].upper) &&
              parsed[index].lower <= parsed[index].upper;
        }
        ++found;
        break;
      }
      cursor = close + 8;
    }
    if (found != kNamedJointCount) {
      error = "urdf_joint_mapping_incomplete";
      return false;
    }
    limits = parsed;
    limits_loaded = true;
    limits_mode = mode_machine;
    error.clear();
    return true;
  }

  void LoadTeachActions() {
    teach_actions.clear();
    std::ifstream input(teach_store);
    if (!input) return;
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errors;
    if (!Json::parseFromStream(builder, input, &root, &errors) ||
        !root.isObject() || root.get("schema_version", 0).asInt() != 1 ||
        !root["actions"].isArray()) {
      throw std::runtime_error("invalid_joint_teach_store");
    }
    for (const auto& item : root["actions"]) {
      if (!item.isObject() || !item["name"].isString() ||
          !ControlService::IsValidActionName(item["name"].asString()) ||
          !item["mode_machine"].isUInt() ||
          item["mode_machine"].asUInt() > 255 ||
          (item.isMember("hold_after_playback") &&
           !item["hold_after_playback"].isBool()) ||
          (item.isMember("remote_binding") &&
           !item["remote_binding"].isString()) ||
          !item["frames"].isArray() ||
          item["frames"].empty() ||
          item["frames"].size() > kMaximumTeachFrames) {
        throw std::runtime_error("invalid_joint_teach_store");
      }
      TeachAction action;
      action.name = item["name"].asString();
      action.mode_machine =
          static_cast<std::uint8_t>(item["mode_machine"].asUInt());
      action.hold_after_playback =
          item.get("hold_after_playback", false).asBool();
      action.remote_binding = item.get("remote_binding", "").asString();
      if (action.remote_binding.rfind("F2+", 0) == 0)
        action.remote_binding.replace(0, 2, "F3");
      if ((!action.remote_binding.empty() &&
           RemoteBindingMask(action.remote_binding) == 0) ||
          (!action.remote_binding.empty() &&
           std::any_of(teach_actions.begin(), teach_actions.end(),
                       [&](const auto& saved) {
                         return saved.remote_binding == action.remote_binding;
                       }))) {
        throw std::runtime_error("invalid_joint_teach_store");
      }
      for (const auto& saved_frame : item["frames"]) {
        if (!saved_frame.isArray() ||
            saved_frame.size() != kTeachJointCount) {
          throw std::runtime_error("invalid_joint_teach_store");
        }
        std::array<float, kTeachJointCount> frame{};
        for (Json::ArrayIndex index = 0; index < saved_frame.size(); ++index) {
          const float value = saved_frame[index].asFloat();
          if (!saved_frame[index].isNumeric() || !std::isfinite(value))
            throw std::runtime_error("invalid_joint_teach_store");
          frame[index] = value;
        }
        action.frames.push_back(frame);
      }
      teach_actions.push_back(std::move(action));
    }
  }

  bool SaveTeachActions(std::string& error) {
    Json::Value root(Json::objectValue);
    root["schema_version"] = 1;
    root["sample_period_ms"] =
        static_cast<Json::UInt>(kTeachSamplePeriod.count());
    root["actions"] = Json::Value(Json::arrayValue);
    for (const auto& action : teach_actions) {
      Json::Value item(Json::objectValue);
      item["name"] = action.name;
      item["mode_machine"] = action.mode_machine;
      item["hold_after_playback"] = action.hold_after_playback;
      item["remote_binding"] = action.remote_binding;
      item["frames"] = Json::Value(Json::arrayValue);
      for (const auto& frame : action.frames) {
        Json::Value saved_frame(Json::arrayValue);
        for (float value : frame) saved_frame.append(value);
        item["frames"].append(saved_frame);
      }
      root["actions"].append(item);
    }
    try {
      const std::filesystem::path path(teach_store);
      std::filesystem::create_directories(path.parent_path());
      const auto temporary = path.string() + ".tmp";
      std::ofstream output(temporary, std::ios::trunc);
      if (!output) throw std::runtime_error("open_failed");
      output << WriteJson(root) << '\n';
      output.close();
      if (!output) throw std::runtime_error("write_failed");
      std::filesystem::permissions(
          temporary,
          std::filesystem::perms::owner_read |
              std::filesystem::perms::owner_write,
          std::filesystem::perm_options::replace);
      std::filesystem::rename(temporary, path);
      teach_store_error.clear();
      error.clear();
      return true;
    } catch (...) {
      std::error_code ignored;
      std::filesystem::remove(teach_store + ".tmp", ignored);
      teach_store_error = "joint_teach_store_failed";
      error = "joint_teach_store_failed";
      return false;
    }
  }

  ControlService& owner;
  SnapshotStore& store;
  const bool mock;
  const std::string web_root;
  const std::string teach_store;
  mutable std::mutex mutex;
  std::mutex service_mutex;
  std::condition_variable cv;
  bool initialized{false};
  std::string initialization_error;
  std::string teach_store_error;
  bool remote_control_ready{false};
  std::string remote_control_error;
  std::uint16_t remote_keys{0};
  std::string remote_last_action;
  std::string remote_last_binding;
  bool active{false};
  bool stopping{false};
  std::string mode{"idle"};
  std::string last_error;
  bool ai_sport_found{false};
  bool ai_sport_active{true};
  bool mock_ai_sport_active{false};
  bool limits_loaded{false};
  std::uint8_t limits_mode{0};
  std::array<Limit, kNamedJointCount> limits{};
  std::array<float, kNamedJointCount> current{};
  std::array<float, kNamedJointCount> target{};
  std::array<bool, kNamedJointCount> selected{};
  float weight{0.0F};
  SteadyClock::time_point last_heartbeat{};
  JointDebugTestStats stats{};
  std::string teach_state{"idle"};
  std::string record_name;
  std::uint8_t record_mode_machine{0};
  SteadyClock::time_point last_record_sample{};
  std::vector<std::array<float, kTeachJointCount>> record_frames;
  std::vector<TeachAction> teach_actions;
  std::vector<std::array<float, kTeachJointCount>> playback_frames;
  std::size_t playback_index{0};
  bool playback_started{false};
  bool playback_hold_after{false};
  std::string playback_action_name;
  std::string holding_action_name;
  SteadyClock::time_point playback_next_frame{};
  std::thread worker;
  std::shared_ptr<unitree::robot::ChannelPublisher<
      unitree_hg::msg::dds_::LowCmd_>> arm_publisher;
  std::shared_ptr<unitree::robot::ChannelPublisher<
      unitree_hg::msg::dds_::LowCmd_>> lowcmd_publisher;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::WirelessController_>> remote_subscriber;
  std::unique_ptr<unitree::robot::b2::RobotStateClient> robot_state;
};

ControlService::ControlService(SnapshotStore& store, bool mock,
                               std::string web_root,
                               std::string joint_teach_store)
    : store_(store), mock_(mock),
      joint_debug_(CreateJointDebugImpl(*this, store, mock, std::move(web_root),
                                        std::move(joint_teach_store))) {}

ControlService::~ControlService() { Stop(); }

std::unique_ptr<ControlService::JointDebugImpl>
ControlService::CreateJointDebugImpl(ControlService& owner,
                                     SnapshotStore& store, bool mock,
                                     std::string web_root,
                                     std::string joint_teach_store) {
  return std::make_unique<JointDebugImpl>(
      owner, store, mock, std::move(web_root), std::move(joint_teach_store));
}

void ControlService::StartJointDebugImpl(JointDebugImpl& impl) { impl.Start(); }

void ControlService::StopJointDebugImpl(JointDebugImpl& impl) { impl.Stop(); }

bool ControlService::JointDebugActive() const {
  return joint_debug_ && joint_debug_->IsActive();
}

void ControlService::SetMockJointDebugAiSport(bool ai_sport_active) {
  if (!joint_debug_ || !mock_) return;
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  joint_debug_->mock_ai_sport_active = ai_sport_active;
}

void ControlService::SetMockJointDebugRemoteKeys(std::uint16_t keys) {
  if (!joint_debug_ || !mock_) return;
  joint_debug_->HandleRemoteKeys(keys);
}

JointDebugTestStats ControlService::GetJointDebugTestStats() const {
  if (!joint_debug_) return {};
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  return joint_debug_->stats;
}

std::string ControlService::SerializeJointDebugStatus() {
  Json::Value root(Json::objectValue);
  root["schema_version"] = 1;
  if (!joint_debug_) {
    root["initialized"] = false;
    root["error"] = "control_not_ready";
    return WriteJson(root);
  }
  std::string service_error;
  if (joint_debug_->initialized) joint_debug_->RefreshAiSport(service_error);
  const auto snapshot = store_.GetSnapshot();
  std::string limit_error;
  const bool limits_ready =
      joint_debug_->LoadLimits(snapshot.mode_machine, limit_error);
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  const bool all_joints_movable = limits_ready && std::all_of(
      joint_debug_->limits.begin(), joint_debug_->limits.end(),
      [](const JointDebugImpl::Limit& limit) { return limit.movable; });
  root["initialized"] = joint_debug_->initialized;
  root["mock"] = mock_;
  root["initialization_error"] = joint_debug_->initialization_error;
  root["teach_store_error"] = joint_debug_->teach_store_error;
  root["remote_control_ready"] = joint_debug_->remote_control_ready;
  root["remote_control_error"] = joint_debug_->remote_control_error;
  root["remote_keys"] = Json::UInt(joint_debug_->remote_keys);
  root["remote_last_action"] = joint_debug_->remote_last_action;
  root["remote_last_binding"] = joint_debug_->remote_last_binding;
  Json::Value remote_binding_options(Json::arrayValue);
  for (const auto& binding : kRemoteBindings) {
    Json::Value item(Json::objectValue);
    item["id"] = binding.id;
    item["label"] = RemoteBindingLabel(binding.id);
    remote_binding_options.append(item);
  }
  root["remote_binding_options"] = remote_binding_options;
  root["mode"] = joint_debug_->mode;
  root["dds_state"] =
      !joint_debug_->active
          ? "idle"
          : joint_debug_->teach_state == "releasing"
                ? "releasing"
                : joint_debug_->teach_state == "recording"
                      ? "released"
                      : joint_debug_->stopping ? "stopping" : "active";
  root["arm_sdk_weight"] = joint_debug_->weight;
  root["ai_sport_found"] = joint_debug_->ai_sport_found;
  root["ai_sport_active"] = joint_debug_->ai_sport_active;
  root["debug_mode_detected"] =
      joint_debug_->ai_sport_found && !joint_debug_->ai_sport_active;
  root["lowstate_fresh"] = LowStateFresh(snapshot);
  root["sport_state_fresh"] = SportStateFresh(snapshot.control);
  root["fsm_id"] = Json::UInt(snapshot.control.fsm_id);
  root["fsm_mode"] = Json::UInt(snapshot.control.fsm_mode);
  root["upper_body_fsm_allowed"] =
      SportStateFresh(snapshot.control) &&
      UpperBodyFsmAllowed(snapshot.control.fsm_id);
  root["mode_pr"] = snapshot.mode_pr;
  root["mode_machine"] = snapshot.mode_machine;
  root["upper_body_allowed"] =
      joint_debug_->initialized && LowStateFresh(snapshot) &&
      SportStateFresh(snapshot.control) &&
      UpperBodyFsmAllowed(snapshot.control.fsm_id) && limits_ready;
  root["full_body_allowed"] =
      joint_debug_->initialized && LowStateFresh(snapshot) &&
      snapshot.mode_pr == 0 && limits_ready &&
      all_joints_movable && service_error.empty() &&
      joint_debug_->ai_sport_found && !joint_debug_->ai_sport_active;
  root["error"] = !service_error.empty() ? service_error : limit_error;
  root["last_error"] = joint_debug_->last_error;
  root["teach_state"] = joint_debug_->teach_state;
  root["teach_record_name"] = joint_debug_->record_name;
  root["teach_recorded_frames"] = static_cast<Json::UInt>(
      joint_debug_->record_frames.size());
  Json::Value teach_actions(Json::arrayValue);
  for (const auto& action : joint_debug_->teach_actions) {
    Json::Value item(Json::objectValue);
    item["name"] = action.name;
    item["mode_machine"] = action.mode_machine;
    item["duration_s"] =
        static_cast<double>(action.frames.size() - 1) *
        static_cast<double>(kTeachSamplePeriod.count()) / 1000.0;
    item["hold_after_playback"] = action.hold_after_playback;
    item["remote_binding"] = action.remote_binding;
    item["frames"] = static_cast<Json::UInt>(action.frames.size());
    teach_actions.append(item);
  }
  root["teach_actions"] = teach_actions;
  Json::Value joints(Json::arrayValue);
  for (std::size_t index = 0; index < kNamedJointCount; ++index) {
    Json::Value joint(Json::objectValue);
    joint["index"] = static_cast<Json::UInt>(index);
    joint["name"] = std::string(JointNames()[index].name);
    joint["name_zh"] = std::string(JointNames()[index].name_zh);
    joint["movable"] = limits_ready && joint_debug_->limits[index].movable;
    joint["upper_body"] = index >= 12;
    if (limits_ready && joint_debug_->limits[index].movable) {
      joint["lower"] = joint_debug_->limits[index].lower;
      joint["upper"] = joint_debug_->limits[index].upper;
    }
    joint["current"] = snapshot.motors[index].q;
    joints.append(joint);
  }
  root["joints"] = joints;
  root["control_hz"] = joint_debug_->mode == "full_body" ? 500 : 50;
  root["maximum_velocity_rad_s"] =
      joint_debug_->mode == "full_body" ? kBodyMaximumVelocity
                                         : kArmMaximumVelocity;
  return WriteJson(root);
}

JointDebugSubmitResult ControlService::ApplyJointDebug(
    const std::string& requested_mode,
    const std::vector<std::pair<std::size_t, float>>& targets,
    bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!running_.load() || !joint_debug_ || !joint_debug_->initialized) {
    result.error = "control_not_ready";
    return result;
  }
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (requested_mode != "upper_body" && requested_mode != "full_body") {
    result.error = "invalid_control_mode";
    return result;
  }
  if (targets.empty() ||
      (requested_mode == "full_body" && targets.size() != kNamedJointCount)) {
    result.error = "invalid_joint_request";
    return result;
  }
  if (motion_active_.load()) {
    result.error = "control_busy";
    return result;
  }
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    if (command_running_ || !queue_.empty()) {
      result.error = "control_busy";
      return result;
    }
  }

  const auto snapshot = store_.GetSnapshot();
  if (!LowStateFresh(snapshot)) {
    result.error = "lowstate_unavailable";
    return result;
  }
  if (requested_mode == "upper_body" &&
      !SportStateFresh(snapshot.control)) {
    result.error = "sport_state_stale";
    return result;
  }
  if (requested_mode == "upper_body" &&
      !UpperBodyFsmAllowed(snapshot.control.fsm_id)) {
    result.error = "upper_body_fsm_not_allowed";
    return result;
  }
  if (requested_mode == "full_body" && snapshot.mode_pr != 0) {
    result.error = "pr_mode_required";
    return result;
  }
  std::string error;
  if (!joint_debug_->LoadLimits(snapshot.mode_machine, error)) {
    result.error = error;
    return result;
  }
  if (requested_mode == "full_body" &&
      !joint_debug_->RefreshAiSport(error)) {
    result.error = error;
    return result;
  }
  if (requested_mode == "full_body") {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    if (!joint_debug_->ai_sport_found || joint_debug_->ai_sport_active) {
      result.error = "debug_mode_required";
      return result;
    }
  }

  std::array<bool, kNamedJointCount> seen{};
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    for (const auto& [index, value] : targets) {
      if (index >= kNamedJointCount || seen[index] || !std::isfinite(value)) {
        result.error = "invalid_joint_request";
        return result;
      }
      seen[index] = true;
      if (requested_mode == "upper_body" && index < 12) {
        result.error = "upper_body_joint_not_allowed";
        return result;
      }
      const auto& limit = joint_debug_->limits[index];
      if (!limit.movable || value < limit.lower || value > limit.upper) {
        result.error = "joint_out_of_range";
        return result;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    if (joint_debug_->active && joint_debug_->mode != requested_mode) {
      result.error = "control_busy";
      return result;
    }
  }
  bool start_worker = false;
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    for (std::size_t index = 0; index < kNamedJointCount; ++index) {
      if (!joint_debug_->active) {
        joint_debug_->current[index] = snapshot.motors[index].q;
        joint_debug_->target[index] = snapshot.motors[index].q;
      }
      joint_debug_->selected[index] = false;
    }
    for (const auto& [index, value] : targets) {
      joint_debug_->target[index] = value;
      joint_debug_->selected[index] = true;
    }
    if (!joint_debug_->active) {
      joint_debug_->active = true;
      joint_debug_->stopping = false;
      joint_debug_->mode = requested_mode;
      joint_debug_->last_error.clear();
      const bool instant_teach_takeover =
          requested_mode == "upper_body" &&
          (joint_debug_->teach_state == "recording" ||
           joint_debug_->teach_state == "playing");
      joint_debug_->weight = instant_teach_takeover ? 1.0F : 0.0F;
      joint_debug_->last_heartbeat = SteadyClock::now();
      ++joint_debug_->stats.loop_starts;
      start_worker = true;
    }
  }
  if (start_worker) {
    if (joint_debug_->worker.joinable()) joint_debug_->worker.join();
    joint_debug_->worker = std::thread([impl = joint_debug_.get()] {
      auto next_service_check = SteadyClock::now();
      while (true) {
        std::string mode;
        std::string teach_state;
        bool stopping = false;
        {
          std::lock_guard<std::mutex> lock(impl->mutex);
          if (!impl->active) break;
          mode = impl->mode;
          teach_state = impl->teach_state;
          stopping = impl->stopping;
          const bool heartbeat_required =
              teach_state != "releasing" && teach_state != "recording" &&
              teach_state != "holding";
          if (!stopping && heartbeat_required &&
              SteadyClock::now() - impl->last_heartbeat >
                  std::chrono::seconds(2)) {
            impl->last_error = "control_lease_expired";
            impl->stopping = true;
            stopping = true;
          }
        }
        const float period = mode == "full_body" ? kBodyPeriodSeconds
                                                  : kArmPeriodSeconds;
        const float maximum_step =
            (mode == "full_body" ? kBodyMaximumVelocity
                                  : kArmMaximumVelocity) * period;
        const auto snapshot = impl->store.GetSnapshot();
        if (!stopping && mode == "upper_body" &&
            (!SportStateFresh(snapshot.control) ||
             !UpperBodyFsmAllowed(snapshot.control.fsm_id))) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->last_error = SportStateFresh(snapshot.control)
                                 ? "upper_body_fsm_not_allowed"
                                 : "sport_state_stale";
          impl->stopping = true;
          stopping = true;
        }
        if (!stopping && !LowStateFresh(snapshot)) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->last_error = "lowstate_unavailable";
          impl->stopping = true;
          stopping = true;
        }

        if (teach_state == "recording" && !stopping) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          if (impl->teach_state == "recording") {
            const auto now = SteadyClock::now();
            if (impl->record_frames.empty() ||
                now - impl->last_record_sample >= kTeachSamplePeriod) {
              std::array<float, kTeachJointCount> frame{};
              for (std::size_t index = 12; index < kNamedJointCount; ++index)
                frame[index - 12] = snapshot.motors[index].q;
              impl->record_frames.push_back(frame);
              impl->last_record_sample = now;
            }
            if (impl->record_frames.size() >= kMaximumTeachFrames) {
              impl->last_error = "joint_teach_duration_limit";
              impl->stopping = true;
              stopping = true;
            }
          }
        }

        if (mode == "full_body" &&
            SteadyClock::now() >= next_service_check) {
          std::string service_error;
          const bool refreshed = impl->RefreshAiSport(service_error);
          bool service_conflict = false;
          {
            std::lock_guard<std::mutex> lock(impl->mutex);
            service_conflict =
                !impl->ai_sport_found || impl->ai_sport_active;
          }
          if (!refreshed || service_conflict) {
            std::lock_guard<std::mutex> lock(impl->mutex);
            impl->last_error = service_error.empty()
                                   ? "debug_mode_required"
                                   : service_error;
            impl->stopping = true;
            stopping = true;
          }
          next_service_check = SteadyClock::now() +
                               std::chrono::milliseconds(500);
        }

        unitree_hg::msg::dds_::LowCmd_ command;
        command.mode_pr() = 0;
        command.mode_machine() = snapshot.mode_machine;
        bool finished = true;
        {
          std::lock_guard<std::mutex> lock(impl->mutex);
          if (impl->teach_state == "playing" && impl->playback_started &&
              SteadyClock::now() >= impl->playback_next_frame &&
              impl->playback_index < impl->playback_frames.size()) {
            const auto& frame =
                impl->playback_frames[impl->playback_index++];
            for (std::size_t index = 12; index < kNamedJointCount; ++index)
              impl->target[index] = frame[index - 12];
            impl->playback_next_frame =
                SteadyClock::now() + kTeachSamplePeriod;
          }
          for (std::size_t index = 0; index < kNamedJointCount; ++index) {
            if (!impl->selected[index]) continue;
            const float delta = std::clamp(
                impl->target[index] - impl->current[index],
                -maximum_step, maximum_step);
            impl->current[index] += delta;
            impl->stats.maximum_step_rad =
                std::max(impl->stats.maximum_step_rad, std::abs(delta));
            if (std::abs(impl->target[index] - impl->current[index]) > 1e-5F)
              finished = false;
          }
          if (mode == "upper_body") {
            const bool releasing = impl->teach_state == "releasing";
            const bool recording = impl->teach_state == "recording";
            impl->weight = std::clamp(
                impl->weight + (stopping ? -1.0F : 1.0F) *
                                   kWeightRate * period,
                0.0F, 1.0F);
            if (recording && !stopping) impl->weight = 1.0F;
            command.motor_cmd().at(29).q(impl->weight);
            if (!stopping && releasing && impl->weight >= 1.0F) {
              impl->teach_state = "recording";
              impl->last_record_sample = {};
            } else if (!stopping && impl->teach_state == "playing" && finished &&
                !impl->playback_started && impl->weight >= 1.0F) {
              impl->playback_started = true;
              impl->playback_next_frame =
                  SteadyClock::now() + kTeachSamplePeriod;
            } else if (!stopping && impl->teach_state == "playing" &&
                       impl->playback_started && finished &&
                       impl->playback_index >=
                           impl->playback_frames.size()) {
              if (impl->playback_hold_after) {
                impl->teach_state = "holding";
                impl->holding_action_name = impl->playback_action_name;
                impl->playback_started = false;
                impl->playback_frames.clear();
              } else {
                impl->teach_state = "idle";
                impl->holding_action_name.clear();
                impl->stopping = true;
                stopping = true;
              }
            }
            finished = stopping && impl->weight <= 0.0F;
          }
          const bool passive_teach =
              mode == "upper_body" && impl->teach_state == "recording";
          for (std::size_t index = 0; index < kNamedJointCount; ++index) {
            if (mode == "upper_body" && !impl->selected[index]) continue;
            const bool waist_locked_teach =
                passive_teach && index >= 13 && index <= 14;
            auto& motor = command.motor_cmd().at(index);
            motor.mode() = stopping && mode == "full_body" ? 0 : 1;
            motor.q() = passive_teach
                            ? (waist_locked_teach ? impl->current[index]
                                                  : snapshot.motors[index].q)
                            : impl->current[index];
            motor.dq() = 0.0F;
            motor.kp() = stopping && mode == "full_body"
                             ? 0.0F
                             : waist_locked_teach ? kKp[index]
                                                  : passive_teach ? 0.0F : kKp[index];
            motor.kd() = stopping && mode == "full_body"
                             ? 0.0F
                             : waist_locked_teach ? kKd[index]
                                                  : passive_teach ? TeachDampingKd(index)
                                                                  : kKd[index];
            motor.tau() = 0.0F;
            impl->stats.last_q[index] = motor.q();
            impl->stats.last_kp[index] = motor.kp();
            impl->stats.last_kd[index] = motor.kd();
          }
          ++impl->stats.publish_count;
          if (mode == "full_body") impl->stats.lowcmd_published = true;
        }
        if (mode == "full_body") {
          command.crc() = Crc32Core(
              reinterpret_cast<std::uint32_t*>(&command),
              (sizeof(command) >> 2U) - 1U);
        }
        const bool published = impl->mock ||
            (mode == "full_body" ? impl->lowcmd_publisher->Write(command)
                                 : impl->arm_publisher->Write(command));
        if (!published) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->last_error = "dds_publish_failed";
          impl->stopping = true;
          stopping = true;
        }
        if (stopping && (mode == "upper_body" ? finished : true)) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->active = false;
          impl->stopping = false;
          impl->mode = "idle";
          impl->teach_state = "idle";
          impl->holding_action_name.clear();
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int>(period * 1000000.0F)));
      }
    });
  }
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::StartJointTeachRecording(
    const std::string& name, bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (!IsValidActionName(name)) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  if (!joint_debug_ || joint_debug_->IsActive()) {
    result.error = "control_busy";
    return result;
  }
  const auto snapshot = store_.GetSnapshot();
  std::vector<std::pair<std::size_t, float>> targets;
  for (std::size_t index = 12; index < kNamedJointCount; ++index)
    targets.emplace_back(index, snapshot.motors[index].q);
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    joint_debug_->teach_state = "recording";
    joint_debug_->record_name = name;
    joint_debug_->record_mode_machine = snapshot.mode_machine;
    joint_debug_->last_record_sample = {};
    joint_debug_->record_frames.clear();
  }
  result = ApplyJointDebug("upper_body", targets, true);
  if (!result.accepted) {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    joint_debug_->teach_state = "idle";
    joint_debug_->record_name.clear();
    joint_debug_->record_frames.clear();
  }
  return result;
}

JointDebugSubmitResult ControlService::FinishJointTeachRecording(
    bool confirmed, bool release_control) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (!joint_debug_) {
    result.error = "control_not_ready";
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    if (joint_debug_->record_name.empty()) {
      result.error = "joint_teach_not_recording";
      return result;
    }
    if (joint_debug_->active && joint_debug_->teach_state != "recording") {
      result.error = "control_busy";
      return result;
    }
  }
  if (release_control && joint_debug_->IsActive()) {
    joint_debug_->RequestStop();
    joint_debug_->JoinStopped();
  }
  const auto snapshot = store_.GetSnapshot();
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  if (!release_control &&
      (!joint_debug_->active || joint_debug_->stopping ||
       joint_debug_->teach_state != "recording")) {
    result.error = "control_busy";
    return result;
  }
  if (joint_debug_->record_frames.size() < 2) {
    result.error = "joint_teach_too_short";
    if (release_control) {
      joint_debug_->record_name.clear();
      joint_debug_->record_frames.clear();
    }
    return result;
  }
  JointDebugImpl::TeachAction saved;
  saved.name = joint_debug_->record_name;
  saved.mode_machine = joint_debug_->record_mode_machine;
  saved.hold_after_playback = !release_control;
  saved.frames = joint_debug_->record_frames;
  for (auto& frame : saved.frames) {
    for (std::size_t index = 12; index < kNamedJointCount; ++index) {
      const auto& limit = joint_debug_->limits[index];
      float& value = frame[index - 12];
      if (!limit.movable || !std::isfinite(value)) {
        result.error = "joint_out_of_range";
        if (release_control) {
          joint_debug_->record_name.clear();
          joint_debug_->record_frames.clear();
        }
        return result;
      }
      value = std::clamp(value, limit.lower, limit.upper);
    }
  }
  const auto hold_frame = saved.frames.back();
  const auto previous = joint_debug_->teach_actions;
  const auto existing = std::find_if(
      joint_debug_->teach_actions.begin(), joint_debug_->teach_actions.end(),
      [&](const auto& action) { return action.name == saved.name; });
  if (existing != joint_debug_->teach_actions.end())
    saved.remote_binding = existing->remote_binding;
  if (existing == joint_debug_->teach_actions.end())
    joint_debug_->teach_actions.push_back(std::move(saved));
  else
    *existing = std::move(saved);
  if (!joint_debug_->SaveTeachActions(result.error)) {
    joint_debug_->teach_actions = previous;
    return result;
  }
  if (release_control) {
    joint_debug_->teach_state = "idle";
    joint_debug_->holding_action_name.clear();
  } else {
    for (std::size_t index = 12; index < kNamedJointCount; ++index) {
      joint_debug_->current[index] = snapshot.motors[index].q;
      joint_debug_->target[index] = hold_frame[index - 12];
      joint_debug_->selected[index] = true;
    }
    joint_debug_->teach_state = "holding";
    joint_debug_->holding_action_name = joint_debug_->record_name;
    joint_debug_->last_heartbeat = SteadyClock::now();
  }
  joint_debug_->record_name.clear();
  joint_debug_->record_frames.clear();
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::PlayJointTeachAction(
    const std::string& name, bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (!IsValidActionName(name) || !joint_debug_) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  if (joint_debug_->IsActive()) {
    result.error = "control_busy";
    return result;
  }
  const auto snapshot = store_.GetSnapshot();
  std::string limit_error;
  if (!joint_debug_->LoadLimits(snapshot.mode_machine, limit_error)) {
    result.error = limit_error;
    return result;
  }
  std::vector<std::array<float, kTeachJointCount>> frames;
  bool hold_after_playback = false;
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    const auto action = std::find_if(
        joint_debug_->teach_actions.begin(), joint_debug_->teach_actions.end(),
        [&](const auto& item) { return item.name == name; });
    if (action == joint_debug_->teach_actions.end()) {
      result.error = "joint_teach_action_not_found";
      return result;
    }
    if (action->mode_machine != snapshot.mode_machine) {
      result.error = "joint_teach_model_mismatch";
      return result;
    }
    frames = action->frames;
    hold_after_playback = action->hold_after_playback;
    for (const auto& frame : frames) {
      for (std::size_t index = 12; index < kNamedJointCount; ++index) {
        const auto& limit = joint_debug_->limits[index];
        const float value = frame[index - 12];
        if (!limit.movable || value < limit.lower || value > limit.upper) {
          result.error = "joint_out_of_range";
          return result;
        }
      }
    }
    joint_debug_->teach_state = "playing";
    joint_debug_->playback_frames = frames;
    joint_debug_->playback_index = 1;
    joint_debug_->playback_started = false;
    joint_debug_->playback_hold_after = hold_after_playback;
    joint_debug_->playback_action_name = name;
    joint_debug_->holding_action_name.clear();
  }
  std::vector<std::pair<std::size_t, float>> targets;
  for (std::size_t index = 12; index < kNamedJointCount; ++index)
    targets.emplace_back(index, frames.front()[index - 12]);
  result = ApplyJointDebug("upper_body", targets, true);
  if (!result.accepted) {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    joint_debug_->teach_state = "idle";
    joint_debug_->playback_frames.clear();
    joint_debug_->playback_hold_after = false;
    joint_debug_->playback_action_name.clear();
    joint_debug_->holding_action_name.clear();
  }
  return result;
}

JointDebugSubmitResult ControlService::DeleteJointTeachAction(
    const std::string& name, bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (!IsValidActionName(name) || !joint_debug_) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  if (joint_debug_->active) {
    result.error = "control_busy";
    return result;
  }
  const auto action = std::find_if(
      joint_debug_->teach_actions.begin(), joint_debug_->teach_actions.end(),
      [&](const auto& item) { return item.name == name; });
  if (action == joint_debug_->teach_actions.end()) {
    result.error = "joint_teach_action_not_found";
    return result;
  }
  const auto previous = joint_debug_->teach_actions;
  joint_debug_->teach_actions.erase(action);
  if (!joint_debug_->SaveTeachActions(result.error)) {
    joint_debug_->teach_actions = previous;
    return result;
  }
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::SetJointTeachRemoteBinding(
    const std::string& name, const std::string& binding) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!IsValidActionName(name) || !joint_debug_) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  if (!binding.empty() && RemoteBindingMask(binding) == 0) {
    result.error = "joint_teach_remote_binding_not_allowed";
    return result;
  }
  std::lock_guard<std::mutex> lock(joint_debug_->mutex);
  if (joint_debug_->active) {
    result.error = "control_busy";
    return result;
  }
  const auto action = std::find_if(
      joint_debug_->teach_actions.begin(), joint_debug_->teach_actions.end(),
      [&](const auto& item) { return item.name == name; });
  if (action == joint_debug_->teach_actions.end()) {
    result.error = "joint_teach_action_not_found";
    return result;
  }
  if (!binding.empty() &&
      std::any_of(joint_debug_->teach_actions.begin(),
                  joint_debug_->teach_actions.end(), [&](const auto& item) {
                    return item.name != name && item.remote_binding == binding;
                  })) {
    result.error = "joint_teach_remote_binding_in_use";
    return result;
  }
  const auto previous = action->remote_binding;
  action->remote_binding = binding;
  if (!joint_debug_->SaveTeachActions(result.error)) {
    action->remote_binding = previous;
    return result;
  }
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::StopJointDebug(bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  if (!confirmed) {
    result.error = "confirmation_required";
    return result;
  }
  if (!joint_debug_ || !joint_debug_->IsActive()) {
    result.accepted = true;
    return result;
  }
  joint_debug_->RequestStop();
  joint_debug_->JoinStopped();
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    joint_debug_->teach_state = "idle";
    joint_debug_->record_name.clear();
    joint_debug_->record_frames.clear();
    joint_debug_->playback_frames.clear();
    joint_debug_->playback_hold_after = false;
    joint_debug_->playback_action_name.clear();
    joint_debug_->holding_action_name.clear();
  }
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::HeartbeatJointDebug() {
  JointDebugSubmitResult result;
  if (!joint_debug_ || !joint_debug_->IsActive()) {
    result.error = "control_not_active";
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    joint_debug_->last_heartbeat = SteadyClock::now();
  }
  result.accepted = true;
  return result;
}

}  // namespace g1_web
