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


namespace g1_web {
namespace {

constexpr char kArmTopic[] = "rt/arm_sdk";
constexpr char kLowCmdTopic[] = "rt/lowcmd";
constexpr char kRemoteTopic[] = "rt/wirelesscontroller";
constexpr char kSportService[] = "ai_sport";
constexpr char kJointDebugArmOwner[] = "joint_debug.arm";
constexpr char kJointDebugWholeBodyOwner[] = "joint_debug.whole_body";
constexpr char kJointTeachOwner[] = "joint_teach";
constexpr float kBodyPeriodSeconds = 0.002F;
constexpr float kArmMaximumVelocity = 0.5F;
constexpr float kBodyMaximumVelocity = 0.25F;
constexpr auto kTeachSamplePeriod = std::chrono::milliseconds(50);
constexpr std::size_t kMaximumTeachFrames = 2400;

struct ResourceOwnerReleaseGuard {
  ResourceManager& manager;
  std::string owner;
  bool release{false};

  ~ResourceOwnerReleaseGuard() {
    if (release) manager.ReleaseOwner(owner);
  }
};

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

bool GroupIncluded(const JointDescriptor& joint,
                   const std::vector<std::string>& groups) {
  return std::find(groups.begin(), groups.end(), joint.display_group) !=
         groups.end();
}

std::vector<std::size_t> JointIndicesForGroups(
    const JointSchema& schema, const std::vector<std::string>& groups) {
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < schema.joints.size(); ++index) {
    if (GroupIncluded(schema.joints[index], groups)) indices.push_back(index);
  }
  return indices;
}

std::vector<std::size_t> ArmSdkJointIndicesForGroups(
    const JointSchema& schema, const IJointDebugPolicy& policy,
    const std::vector<std::string>& groups) {
  auto indices = JointIndicesForGroups(schema, groups);
  indices.erase(std::remove_if(indices.begin(), indices.end(),
                               [&](std::size_t index) {
                                 return !policy.ArmSdkJoint(
                                     schema.joints[index]);
                               }),
                indices.end());
  return indices;
}

std::vector<std::size_t> JointIndicesForMode(
    const JointSchema& schema, const IJointDebugPolicy& policy,
    const std::string& mode) {
  const auto groups = policy.GroupsForMode(mode);
  return policy.TransportForMode(mode) == JointDebugTransport::kArmSdk
             ? ArmSdkJointIndicesForGroups(schema, policy, groups)
             : JointIndicesForGroups(schema, groups);
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

std::string WriteJson(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

const char* TransportTopic(JointDebugTransport transport) {
  return transport == JointDebugTransport::kLowCmd ? kLowCmdTopic : kArmTopic;
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
    std::string product_id;
    std::string variant;
    bool legacy_identity{false};
    std::uint8_t mode_machine{0};
    bool hold_after_playback{false};
    std::string remote_binding;
    std::vector<std::vector<float>> frames;
  };

  JointDebugImpl(ControlService& owner, SnapshotStore& store,
                 IJointDebugPolicy& policy, bool mock,
                 std::string web_root, std::string teach_store)
      : owner(owner), store(store), policy(policy), mock(mock),
        web_root(std::move(web_root)),
        teach_store(teach_store.empty()
                        ? (std::filesystem::path(this->web_root).parent_path() /
                           "config/joint_teach_actions.json")
                              .string()
                        : std::move(teach_store)),
        upper_body_indices(JointIndicesForMode(
            policy.profile().joint_schema, policy, "upper_body")),
        full_body_indices(JointIndicesForMode(
            policy.profile().joint_schema, policy, "full_body")),
        teach_joint_indices(ArmSdkJointIndicesForGroups(
            policy.profile().joint_schema, policy, policy.TeachGroups())) {
    const auto joint_count = policy.profile().joint_schema.joints.size();
    limits.resize(joint_count);
    current.resize(joint_count);
    target.resize(joint_count);
    selected.resize(joint_count);
    const auto motor_slot_count = policy.profile().joint_schema.motor_slot_count;
    stats.last_q.resize(motor_slot_count);
    stats.last_kp.resize(motor_slot_count);
    stats.last_kd.resize(motor_slot_count);
  }

  ~JointDebugImpl() { Stop(); }

  void Start() {
    std::lock_guard<std::mutex> lock(mutex);
    if (initialized) return;
    try {
      teach_enabled = owner.safety_
                          .CheckCapability(owner.robot_profile_,
                                           CapabilityKey::kJointTeach, mock)
                          .allowed;
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
        if (teach_enabled && policy.SupportsTeachRemoteControl()) {
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
          remote_control_ready = false;
          remote_control_error =
              teach_enabled ? "joint_teach_remote_control_unsupported"
                            : "joint_teach_disabled";
        }
      } else {
        remote_control_ready =
            teach_enabled && policy.SupportsTeachRemoteControl();
        remote_control_error =
            !teach_enabled
                ? "joint_teach_disabled"
                : policy.SupportsTeachRemoteControl()
                      ? std::string{}
                      : "joint_teach_remote_control_unsupported";
      }
      if (teach_enabled) {
        try {
          LoadTeachActions();
          teach_store_error.clear();
        } catch (const std::exception& error) {
          teach_store_error = error.what();
        }
      } else {
        teach_actions.clear();
        teach_store_error.clear();
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
    std::string resource_owner;
    {
      std::lock_guard<std::mutex> lock(mutex);
      resource_owner = active_resource_owner;
      active_resource_owner.clear();
      active = false;
      stopping = false;
      mode = "idle";
      arm_publisher.reset();
      lowcmd_publisher.reset();
      robot_state.reset();
      initialized = false;
    }
    if (!resource_owner.empty()) owner.resources_.ReleaseOwner(resource_owner);
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
    if (!teach_enabled || !policy.SupportsTeachRemoteControl()) return;
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
    const auto* model = policy.ResolveModelVariant(mode_machine);
    if (!model) {
      error = "unsupported_model";
      return false;
    }
    std::filesystem::path asset_root(policy.profile().model_asset_root);
    if (asset_root.is_absolute()) asset_root = asset_root.relative_path();
    const std::filesystem::path path =
        std::filesystem::path(web_root) / asset_root / model->urdf_file;
    std::ifstream input(path);
    if (!input) {
      error = "urdf_unavailable";
      return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string xml = buffer.str();
    const auto& schema = policy.profile().joint_schema;
    std::vector<Limit> parsed(schema.joints.size());
    std::size_t found = 0;
    std::size_t cursor = 0;
    while ((cursor = xml.find("<joint ", cursor)) != std::string::npos) {
      const auto tag_end = xml.find('>', cursor);
      const auto close = xml.find("</joint>", tag_end);
      if (tag_end == std::string::npos || close == std::string::npos) break;
      const std::string tag = xml.substr(cursor, tag_end - cursor + 1);
      const std::string name = Attribute(tag, "name");
      for (std::size_t index = 0; index < schema.joints.size(); ++index) {
        if (name != schema.joints[index].urdf_joint_name) continue;
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
    if (found != schema.joints.size()) {
      error = "urdf_joint_mapping_incomplete";
      return false;
    }
    limits = std::move(parsed);
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
        !root.isObject() || !root["actions"].isArray()) {
      throw std::runtime_error("invalid_joint_teach_store");
    }
    const int schema_version = root.get("schema_version", 0).asInt();
    if (schema_version != 1 && schema_version != 2)
      throw std::runtime_error("invalid_joint_teach_store");
    if (schema_version == 1 &&
        (!root["sample_period_ms"].isUInt() ||
         root["sample_period_ms"].asUInt() != kTeachSamplePeriod.count())) {
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
          !item["frames"].isArray() || item["frames"].empty() ||
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
      if (schema_version == 2) {
        if (!item["product_id"].isString() ||
            item["product_id"].asString().empty() ||
            !item["variant"].isString() || item["variant"].asString().empty()) {
          throw std::runtime_error("invalid_joint_teach_store");
        }
        action.product_id = item["product_id"].asString();
        action.variant = item["variant"].asString();
      } else {
        action.product_id = policy.profile().identity.product_id;
        action.variant = policy.profile().identity.variant;
        action.legacy_identity = true;
      }
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
            saved_frame.size() != teach_joint_indices.size()) {
          throw std::runtime_error("invalid_joint_teach_store");
        }
        std::vector<float> frame(saved_frame.size());
        for (Json::ArrayIndex index = 0; index < saved_frame.size(); ++index) {
          const float value = saved_frame[index].asFloat();
          if (!saved_frame[index].isNumeric() || !std::isfinite(value))
            throw std::runtime_error("invalid_joint_teach_store");
          frame[index] = value;
        }
        action.frames.push_back(std::move(frame));
      }
      if (action.legacy_identity &&
          !policy.LegacyTeachActionCompatible(action.mode_machine,
                                              teach_joint_indices.size())) {
        throw std::runtime_error("invalid_joint_teach_store");
      }
      teach_actions.push_back(std::move(action));
    }
  }

  bool SaveTeachActions(std::string& error) {
    Json::Value root(Json::objectValue);
    root["schema_version"] = 2;
    root["sample_period_ms"] =
        static_cast<Json::UInt>(kTeachSamplePeriod.count());
    root["actions"] = Json::Value(Json::arrayValue);
    for (const auto& action : teach_actions) {
      Json::Value item(Json::objectValue);
      item["name"] = action.name;
      item["product_id"] = action.product_id;
      item["variant"] = action.variant;
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
  IJointDebugPolicy& policy;
  const bool mock;
  const std::string web_root;
  const std::string teach_store;
  mutable std::mutex mutex;
  std::mutex service_mutex;
  std::condition_variable cv;
  bool initialized{false};
  std::string initialization_error;
  std::string teach_store_error;
  bool teach_enabled{false};
  bool remote_control_ready{false};
  std::string remote_control_error;
  std::uint16_t remote_keys{0};
  std::string remote_last_action;
  std::string remote_last_binding;
  bool active{false};
  bool stopping{false};
  std::string mode{"idle"};
  std::string active_resource_owner;
  std::string last_error;
  bool ai_sport_found{false};
  bool ai_sport_active{true};
  bool mock_ai_sport_active{false};
  bool limits_loaded{false};
  std::uint8_t limits_mode{0};
  std::vector<std::size_t> upper_body_indices;
  std::vector<std::size_t> full_body_indices;
  std::vector<std::size_t> teach_joint_indices;
  std::vector<Limit> limits;
  std::vector<float> current;
  std::vector<float> target;
  std::vector<bool> selected;
  float weight{0.0F};
  SteadyClock::time_point last_heartbeat{};
  JointDebugTestStats stats{};
  std::string teach_state{"idle"};
  std::string record_name;
  std::uint8_t record_mode_machine{0};
  SteadyClock::time_point last_record_sample{};
  std::vector<std::vector<float>> record_frames;
  std::vector<TeachAction> teach_actions;
  std::vector<std::vector<float>> playback_frames;
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

ControlService::ControlService(
    SnapshotStore& store, std::unique_ptr<ILocomotion> locomotion,
    const RobotProfile& robot_profile,
    std::unique_ptr<IJointDebugPolicy> joint_debug_policy, bool mock,
    std::string web_root, std::string joint_teach_store)
    : store_(store), locomotion_(std::move(locomotion)),
      robot_profile_(robot_profile),
      joint_debug_policy_(std::move(joint_debug_policy)), mock_(mock) {
  const auto locomotion_capability = std::find_if(
      robot_profile_.capabilities.begin(), robot_profile_.capabilities.end(),
      [](const CapabilityDescriptor& capability) {
        return capability.key == CapabilityKey::kLocomotion;
      });
  if (!mock_ && locomotion_capability != robot_profile_.capabilities.end()) {
    const auto refresh =
        locomotion_capability->parameters.find("state_refresh_policy");
    poll_locomotion_state_ =
        refresh != locomotion_capability->parameters.end() &&
        refresh->second == "client_poll";
  }
  if (joint_debug_policy_) {
    joint_debug_ = CreateJointDebugImpl(
        *this, store, *joint_debug_policy_, mock, std::move(web_root),
        std::move(joint_teach_store));
  }
}

ControlService::~ControlService() { Stop(); }

std::unique_ptr<ControlService::JointDebugImpl>
ControlService::CreateJointDebugImpl(
    ControlService& owner, SnapshotStore& store,
    IJointDebugPolicy& joint_debug_policy, bool mock, std::string web_root,
    std::string joint_teach_store) {
  return std::make_unique<JointDebugImpl>(
      owner, store, joint_debug_policy, mock, std::move(web_root),
      std::move(joint_teach_store));
}

void ControlService::StartJointDebugImpl(JointDebugImpl& impl) { impl.Start(); }

void ControlService::StopJointDebugImpl(JointDebugImpl& impl) { impl.Stop(); }

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
  root["remote_binding_supported"] =
      joint_debug_->teach_enabled &&
      joint_debug_->policy.SupportsTeachRemoteControl();
  if (root["remote_binding_supported"].asBool()) {
    for (const auto& binding : kRemoteBindings) {
      Json::Value item(Json::objectValue);
      item["id"] = binding.id;
      item["label"] = RemoteBindingLabel(binding.id);
      remote_binding_options.append(item);
    }
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
  const bool arm_sdk_active =
      joint_debug_->active &&
      joint_debug_->policy.TransportForMode(joint_debug_->mode) ==
          JointDebugTransport::kArmSdk;
  const bool upper_body_fsm_allowed =
      arm_sdk_active
          ? joint_debug_->policy.UpperBodyActiveFsmAllowed(
                snapshot.control.fsm_id)
          : joint_debug_->policy.UpperBodyFsmAllowed(snapshot.control.fsm_id);
  root["upper_body_fsm_allowed"] =
      SportStateFresh(snapshot.control) && upper_body_fsm_allowed;
  root["mode_pr"] = snapshot.mode_pr;
  root["mode_machine"] = snapshot.mode_machine;
  root["product_id"] = robot_profile_.identity.product_id;
  root["variant"] = robot_profile_.identity.variant;
  Json::Value control_topics(Json::objectValue);
  const auto mode_allowed = [&](const std::string& candidate) {
    const auto groups = joint_debug_->policy.GroupsForMode(candidate);
    if (groups.empty()) return false;
    const auto transport = joint_debug_->policy.TransportForMode(candidate);
    control_topics[candidate] = TransportTopic(transport);
    if (!joint_debug_->initialized ||
        !LowStateFresh(snapshot) || !limits_ready) {
      return false;
    }
    const auto indices = JointIndicesForMode(
        robot_profile_.joint_schema, joint_debug_->policy, candidate);
    if (indices.empty()) return false;
    const bool selected_joints_movable = std::all_of(
        indices.begin(), indices.end(), [&](std::size_t index) {
          return joint_debug_->limits[index].movable;
        });
    if (!selected_joints_movable) return false;
    if (transport == JointDebugTransport::kArmSdk) {
      return SportStateFresh(snapshot.control) && upper_body_fsm_allowed;
    }
    return snapshot.mode_pr == 0 && service_error.empty() &&
           joint_debug_->ai_sport_found && !joint_debug_->ai_sport_active;
  };
  root["upper_body_allowed"] = mode_allowed("upper_body");
  root["full_body_allowed"] = mode_allowed("full_body") && all_joints_movable;
  root["head_allowed"] = mode_allowed("head");
  root["control_topics"] = control_topics;
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
    item["product_id"] = action.product_id;
    item["variant"] = action.variant;
    item["legacy_compatible"] = action.legacy_identity;
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
  const auto& schema = robot_profile_.joint_schema;
  for (std::size_t index = 0; index < schema.joints.size(); ++index) {
    const auto& descriptor = schema.joints[index];
    Json::Value joint(Json::objectValue);
    joint["index"] = static_cast<Json::UInt>(index);
    joint["name"] = descriptor.name;
    joint["name_zh"] = descriptor.name_zh;
    joint["display_group"] = descriptor.display_group;
    joint["motor_slot"] = static_cast<Json::UInt>(descriptor.motor_slot);
    joint["movable"] = limits_ready && joint_debug_->limits[index].movable;
    joint["upper_body"] =
        std::find(joint_debug_->upper_body_indices.begin(),
                  joint_debug_->upper_body_indices.end(), index) !=
        joint_debug_->upper_body_indices.end();
    Json::Value control_modes(Json::arrayValue);
    for (const auto* candidate : {"upper_body", "full_body", "head"}) {
      const auto indices = JointIndicesForMode(
          schema, joint_debug_->policy, candidate);
      if (std::find(indices.begin(), indices.end(), index) != indices.end())
        control_modes.append(candidate);
    }
    joint["control_modes"] = control_modes;
    if (limits_ready && joint_debug_->limits[index].movable) {
      joint["lower"] = joint_debug_->limits[index].lower;
      joint["upper"] = joint_debug_->limits[index].upper;
    }
    joint["current"] = snapshot.motors.at(descriptor.motor_slot).q;
    joints.append(joint);
  }
  root["joints"] = joints;
  const auto active_transport =
      joint_debug_->policy.TransportForMode(joint_debug_->mode);
  root["control_hz"] =
      active_transport == JointDebugTransport::kLowCmd
          ? 500
          : static_cast<int>(std::lround(
                1.0F / joint_debug_->policy.ArmSdkPeriodSeconds()));
  root["maximum_velocity_rad_s"] =
      active_transport == JointDebugTransport::kLowCmd ? kBodyMaximumVelocity
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
  const auto ready = safety_.CheckServiceReady(
      running_.load() && joint_debug_ && joint_debug_->initialized);
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto groups = joint_debug_->policy.GroupsForMode(requested_mode);
  if (groups.empty()) {
    result.error = "invalid_control_mode";
    return result;
  }
  const auto mode_indices = JointIndicesForMode(
      robot_profile_.joint_schema, joint_debug_->policy, requested_mode);
  const auto transport =
      joint_debug_->policy.TransportForMode(requested_mode);
  if (targets.empty() ||
      (requested_mode == "full_body" && targets.size() != mode_indices.size())) {
    result.error = "invalid_joint_request";
    return result;
  }
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    if (command_running_ || !queue_.empty()) {
      result.error = "control_busy";
      return result;
    }
  }

  std::string resource_owner;
  bool teach_control = false;
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    if (joint_debug_->active && joint_debug_->mode != requested_mode) {
      result.error = "control_busy";
      return result;
    }
    teach_control = requested_mode == "upper_body" &&
                    joint_debug_->teach_state != "idle";
    resource_owner = teach_control
                         ? kJointTeachOwner
                         : requested_mode == "full_body"
                               ? kJointDebugWholeBodyOwner
                               : kJointDebugArmOwner;
  }

  const auto primary_resource =
      teach_control ? ControlResource::JointTeach
                    : requested_mode == "full_body"
                          ? ControlResource::WholeBody
                          : ControlResource::Arm;
  const auto capability = safety_.CheckCapability(
      robot_profile_,
      teach_control ? CapabilityKey::kJointTeach
                    : requested_mode == "head" ? CapabilityKey::kHeadControl
                                               : CapabilityKey::kJointDebug,
      mock_);
  if (!capability.allowed) {
    result.error = capability.error;
    return result;
  }

  const auto acquired = resources_.Acquire(primary_resource, resource_owner);
  const auto primary = safety_.CheckResource(acquired);
  if (!primary.allowed) {
    result.error = primary.error;
    return result;
  }
  ResourceOwnerReleaseGuard resource_guard{
      resources_, resource_owner, !acquired.already_owned};
  if (teach_control) {
    const auto arm = safety_.CheckResource(
        resources_.Acquire(ControlResource::Arm, resource_owner));
    if (!arm.allowed) {
      result.error = arm.error;
      return result;
    }
  }
  if (transport == JointDebugTransport::kLowCmd) {
    const auto lowcmd = safety_.CheckResource(
        resources_.Acquire(ControlResource::LowCmd, resource_owner));
    if (!lowcmd.allowed) {
      result.error = lowcmd.error;
      return result;
    }
  }

  const auto snapshot = store_.GetSnapshot();
  const auto lowstate = safety_.CheckFreshness(
      LowStateFresh(snapshot), "lowstate_unavailable");
  if (!lowstate.allowed) {
    result.error = lowstate.error;
    return result;
  }
  if (transport == JointDebugTransport::kArmSdk) {
    const auto sport_state = safety_.CheckFreshness(
        SportStateFresh(snapshot.control), "sport_state_stale");
    if (!sport_state.allowed) {
      result.error = sport_state.error;
      return result;
    }
  }
  if (transport == JointDebugTransport::kArmSdk) {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    const bool continuing =
        joint_debug_->active && joint_debug_->mode == requested_mode;
    const bool fsm_allowed =
        continuing
            ? joint_debug_->policy.UpperBodyActiveFsmAllowed(
                  snapshot.control.fsm_id)
            : joint_debug_->policy.UpperBodyFsmAllowed(snapshot.control.fsm_id);
    if (!fsm_allowed) {
      result.error = "upper_body_fsm_not_allowed";
      return result;
    }
  }
  if (transport == JointDebugTransport::kLowCmd && snapshot.mode_pr != 0) {
    result.error = "pr_mode_required";
    return result;
  }
  std::string error;
  if (!joint_debug_->LoadLimits(snapshot.mode_machine, error)) {
    result.error = error;
    return result;
  }
  if (transport == JointDebugTransport::kLowCmd &&
      !joint_debug_->RefreshAiSport(error)) {
    result.error = error;
    return result;
  }
  if (transport == JointDebugTransport::kLowCmd) {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    if (!joint_debug_->ai_sport_found || joint_debug_->ai_sport_active) {
      result.error = "debug_mode_required";
      return result;
    }
  }

  std::vector<bool> seen(robot_profile_.joint_schema.joints.size());
  {
    std::lock_guard<std::mutex> lock(joint_debug_->mutex);
    for (const auto& [index, value] : targets) {
      if (index >= seen.size() || seen[index] || !std::isfinite(value)) {
        result.error = "invalid_joint_request";
        return result;
      }
      seen[index] = true;
      if (requested_mode != "full_body" &&
          std::find(mode_indices.begin(), mode_indices.end(), index) ==
              mode_indices.end()) {
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
    for (std::size_t index = 0;
         index < robot_profile_.joint_schema.joints.size(); ++index) {
      if (!joint_debug_->active) {
        const auto motor_slot =
            robot_profile_.joint_schema.joints[index].motor_slot;
        joint_debug_->current[index] = snapshot.motors.at(motor_slot).q;
        joint_debug_->target[index] = snapshot.motors.at(motor_slot).q;
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
      joint_debug_->active_resource_owner = resource_owner;
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
    try {
      joint_debug_->worker = std::thread([impl = joint_debug_.get()] {
      auto next_service_check = SteadyClock::now();
      try {
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
          if (!stopping && heartbeat_required) {
            const auto lease = impl->owner.safety_.CheckLease(
                SteadyClock::now() - impl->last_heartbeat <=
                std::chrono::seconds(2));
            if (!lease.allowed) {
              impl->last_error = lease.error;
              impl->stopping = true;
              stopping = true;
            }
          }
        }
        const auto transport = impl->policy.TransportForMode(mode);
        const float period =
            transport == JointDebugTransport::kLowCmd
                ? kBodyPeriodSeconds
                : impl->policy.ArmSdkPeriodSeconds();
        const float maximum_step =
            (transport == JointDebugTransport::kLowCmd ? kBodyMaximumVelocity
                                                       : kArmMaximumVelocity) *
            period;
        const auto snapshot = impl->store.GetSnapshot();
        if (!stopping && transport == JointDebugTransport::kArmSdk) {
          const auto sport_state = impl->owner.safety_.CheckFreshness(
              SportStateFresh(snapshot.control), "sport_state_stale");
          if (!sport_state.allowed ||
              !impl->policy.UpperBodyActiveFsmAllowed(
                  snapshot.control.fsm_id)) {
            std::lock_guard<std::mutex> lock(impl->mutex);
            impl->last_error = sport_state.allowed
                                   ? "upper_body_fsm_not_allowed"
                                   : sport_state.error;
            impl->stopping = true;
            stopping = true;
          }
        }
        if (!stopping) {
          const auto lowstate = impl->owner.safety_.CheckFreshness(
              LowStateFresh(snapshot), "lowstate_unavailable");
          if (!lowstate.allowed) {
            std::lock_guard<std::mutex> lock(impl->mutex);
            impl->last_error = lowstate.error;
            impl->stopping = true;
            stopping = true;
          }
        }

        if (teach_state == "recording" && !stopping) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          if (impl->teach_state == "recording") {
            const auto now = SteadyClock::now();
            if (impl->record_frames.empty() ||
                now - impl->last_record_sample >= kTeachSamplePeriod) {
              std::vector<float> frame(impl->teach_joint_indices.size());
              for (std::size_t position = 0;
                   position < impl->teach_joint_indices.size(); ++position) {
                const auto joint_index = impl->teach_joint_indices[position];
                const auto motor_slot = impl->policy.profile()
                                            .joint_schema.joints[joint_index]
                                            .motor_slot;
                frame[position] = snapshot.motors.at(motor_slot).q;
              }
              impl->record_frames.push_back(std::move(frame));
              impl->last_record_sample = now;
            }
            if (impl->record_frames.size() >= kMaximumTeachFrames) {
              impl->last_error = "joint_teach_duration_limit";
              impl->stopping = true;
              stopping = true;
            }
          }
        }

        if (transport == JointDebugTransport::kLowCmd &&
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
            for (std::size_t position = 0;
                 position < impl->teach_joint_indices.size(); ++position) {
              impl->target[impl->teach_joint_indices[position]] = frame[position];
            }
            impl->playback_next_frame =
                SteadyClock::now() + kTeachSamplePeriod;
          }
          for (std::size_t index = 0; index < impl->current.size(); ++index) {
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
          if (transport == JointDebugTransport::kArmSdk) {
            const bool releasing = impl->teach_state == "releasing";
            const bool recording = impl->teach_state == "recording";
            impl->weight = std::clamp(
                impl->weight + (stopping ? -1.0F : 1.0F) *
                                   impl->policy.ArmSdkWeightRatePerSecond() *
                                   period,
                0.0F, 1.0F);
            if (recording && !stopping) impl->weight = 1.0F;
            if (impl->policy.ArmSdkWeightUsesModePr()) {
              command.mode_pr() = static_cast<std::uint8_t>(
                  std::lround(std::clamp(impl->weight, 0.0F, 1.0F) * 100.0F));
            } else {
              const auto weight_slot = impl->policy.ArmSdkWeightMotorSlot();
              if (!weight_slot)
                throw std::runtime_error("arm_sdk_weight_unavailable");
              command.motor_cmd().at(*weight_slot).q(impl->weight);
            }
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
              transport == JointDebugTransport::kArmSdk &&
              impl->teach_state == "recording";
          const auto& schema = impl->policy.profile().joint_schema;
          for (std::size_t index = 0; index < schema.joints.size(); ++index) {
            const auto& descriptor = schema.joints[index];
            const bool arm_sdk_support =
                transport == JointDebugTransport::kArmSdk &&
                impl->policy.ArmSdkJoint(descriptor);
            if (mode != "full_body" && !impl->selected[index] &&
                !arm_sdk_support) {
              continue;
            }
            const bool passive_selected =
                passive_teach && impl->selected[index];
            const auto gain =
                passive_selected
                    ? impl->policy.RecordingGainForMode(descriptor, mode)
                    : impl->policy.NormalGainForMode(descriptor, mode);
            auto& motor = command.motor_cmd().at(descriptor.motor_slot);
            motor.mode() =
                stopping && transport == JointDebugTransport::kLowCmd ? 0 : 1;
            motor.q() = passive_selected
                            ? (gain.hold_record_start_pose
                                   ? impl->current[index]
                                   : snapshot.motors.at(descriptor.motor_slot).q)
                            : impl->current[index];
            motor.dq() = 0.0F;
            motor.kp() = stopping && transport == JointDebugTransport::kLowCmd
                             ? 0.0F
                             : gain.kp;
            motor.kd() = stopping && transport == JointDebugTransport::kLowCmd
                             ? 0.0F
                             : gain.kd;
            motor.tau() = 0.0F;
            impl->stats.last_q[descriptor.motor_slot] = motor.q();
            impl->stats.last_kp[descriptor.motor_slot] = motor.kp();
            impl->stats.last_kd[descriptor.motor_slot] = motor.kd();
          }
          ++impl->stats.publish_count;
          if (transport == JointDebugTransport::kLowCmd)
            impl->stats.lowcmd_published = true;
        }
        if (transport == JointDebugTransport::kLowCmd) {
          command.crc() = Crc32Core(
              reinterpret_cast<std::uint32_t*>(&command),
              (sizeof(command) >> 2U) - 1U);
        }
        const bool published = impl->mock ||
            (transport == JointDebugTransport::kLowCmd
                 ? impl->lowcmd_publisher->Write(command)
                 : impl->arm_publisher->Write(command));
        if (!published) {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->last_error = "dds_publish_failed";
          impl->stopping = true;
          stopping = true;
        }
        if (stopping &&
            (transport == JointDebugTransport::kArmSdk ? finished : true)) {
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
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(impl->mutex);
        impl->last_error = error.what();
        impl->active = false;
        impl->stopping = false;
        impl->mode = "idle";
        impl->teach_state = "idle";
        impl->holding_action_name.clear();
      } catch (...) {
        std::lock_guard<std::mutex> lock(impl->mutex);
        impl->last_error = "joint_debug_worker_exception";
        impl->active = false;
        impl->stopping = false;
        impl->mode = "idle";
        impl->teach_state = "idle";
        impl->holding_action_name.clear();
      }
      std::string resource_owner;
      {
        std::lock_guard<std::mutex> lock(impl->mutex);
        resource_owner = impl->active_resource_owner;
        impl->active_resource_owner.clear();
      }
      if (!resource_owner.empty())
        impl->owner.safety_.CleanupControlFailure(
            impl->owner.resources_, resource_owner);
      });
    } catch (...) {
      std::lock_guard<std::mutex> lock(joint_debug_->mutex);
      joint_debug_->active = false;
      joint_debug_->stopping = false;
      joint_debug_->mode = "idle";
      joint_debug_->active_resource_owner.clear();
      result.error = "joint_debug_worker_start_failed";
      return result;
    }
  }
  resource_guard.release = false;
  result.accepted = true;
  return result;
}

JointDebugSubmitResult ControlService::StartJointTeachRecording(
    const std::string& name, bool confirmed) {
  std::lock_guard<std::recursive_mutex> request_lock(
      joint_debug_request_mutex_);
  JointDebugSubmitResult result;
  const auto ready = safety_.CheckServiceReady(
      running_.load() && joint_debug_ && joint_debug_->initialized);
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto capability = safety_.CheckCapability(
      robot_profile_, CapabilityKey::kJointTeach, mock_);
  if (!capability.allowed) {
    result.error = capability.error;
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
  for (const auto index : joint_debug_->teach_joint_indices) {
    const auto motor_slot = robot_profile_.joint_schema.joints[index].motor_slot;
    targets.emplace_back(index, snapshot.motors.at(motor_slot).q);
  }
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
  const auto ready = safety_.CheckServiceReady(
      running_.load() && joint_debug_ && joint_debug_->initialized);
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto capability = safety_.CheckCapability(
      robot_profile_, CapabilityKey::kJointTeach, mock_);
  if (!capability.allowed) {
    result.error = capability.error;
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
  saved.product_id = robot_profile_.identity.product_id;
  saved.variant = robot_profile_.identity.variant;
  saved.mode_machine = joint_debug_->record_mode_machine;
  saved.hold_after_playback = !release_control;
  saved.frames = joint_debug_->record_frames;
  for (auto& frame : saved.frames) {
    for (std::size_t position = 0;
         position < joint_debug_->teach_joint_indices.size(); ++position) {
      const auto index = joint_debug_->teach_joint_indices[position];
      const auto& limit = joint_debug_->limits[index];
      float& value = frame[position];
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
    for (std::size_t position = 0;
         position < joint_debug_->teach_joint_indices.size(); ++position) {
      const auto index = joint_debug_->teach_joint_indices[position];
      const auto motor_slot = robot_profile_.joint_schema.joints[index].motor_slot;
      joint_debug_->current[index] = snapshot.motors.at(motor_slot).q;
      joint_debug_->target[index] = hold_frame[position];
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
  const auto ready = safety_.CheckServiceReady(
      running_.load() && joint_debug_ && joint_debug_->initialized);
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto capability = safety_.CheckCapability(
      robot_profile_, CapabilityKey::kJointTeach, mock_);
  if (!capability.allowed) {
    result.error = capability.error;
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
  std::vector<std::vector<float>> frames;
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
    if (action->product_id != robot_profile_.identity.product_id) {
      result.error = "joint_teach_product_mismatch";
      return result;
    }
    if (action->variant != robot_profile_.identity.variant) {
      result.error = "joint_teach_variant_mismatch";
      return result;
    }
    if (action->mode_machine != snapshot.mode_machine) {
      result.error = "joint_teach_model_mismatch";
      return result;
    }
    frames = action->frames;
    hold_after_playback = action->hold_after_playback;
    for (const auto& frame : frames) {
      for (std::size_t position = 0;
           position < joint_debug_->teach_joint_indices.size(); ++position) {
        const auto index = joint_debug_->teach_joint_indices[position];
        const auto& limit = joint_debug_->limits[index];
        const float value = frame[position];
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
  for (std::size_t position = 0;
       position < joint_debug_->teach_joint_indices.size(); ++position) {
    targets.emplace_back(joint_debug_->teach_joint_indices[position],
                         frames.front()[position]);
  }
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
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
    return result;
  }
  const auto capability = safety_.CheckCapability(
      robot_profile_, CapabilityKey::kJointTeach, mock_);
  if (!capability.allowed) {
    result.error = capability.error;
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
  const auto capability = safety_.CheckCapability(
      robot_profile_, CapabilityKey::kJointTeach, mock_);
  if (!capability.allowed) {
    result.error = capability.error;
    return result;
  }
  if (!IsValidActionName(name) || !joint_debug_) {
    result.error = "invalid_teach_action_name";
    return result;
  }
  if (!joint_debug_->policy.SupportsTeachRemoteControl() ||
      (!binding.empty() && RemoteBindingMask(binding) == 0)) {
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
  const auto confirmation = safety_.CheckConfirmation(confirmed);
  if (!confirmation.allowed) {
    result.error = confirmation.error;
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
  const auto ready = safety_.CheckServiceReady(
      running_.load() && joint_debug_ && joint_debug_->initialized);
  if (!ready.allowed) {
    result.error = ready.error;
    return result;
  }
  if (!joint_debug_->IsActive()) {
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
