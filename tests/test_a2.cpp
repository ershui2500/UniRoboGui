#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <functional>
#include <thread>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>
#include <json/json.h>
#include "g1_web/a2_joint_debug_policy.hpp"
#include "g1_web/a2_locomotion_adapter.hpp"
#include "g1_web/a2_device_capability.hpp"
#include "g1_web/camera_service.hpp"
#include "g1_web/control_service.hpp"
#include "g1_web/json_serializer.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/robot_registry.hpp"
#include "g1_web/remote_keys.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/unitree_data_source.hpp"

namespace {
void Check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
Json::Value Parse(const std::string& text) {
  Json::Value root; Json::CharReaderBuilder builder; std::string error;
  std::istringstream stream(text);
  Check(Json::parseFromStream(builder, stream, &root, &error), "JSON parse");
  return root;
}
void WaitFor(const std::function<bool()>& predicate, const char* message) {
  const auto deadline = g1_web::SteadyClock::now() + std::chrono::seconds(3);
  while (g1_web::SteadyClock::now() < deadline) {
    if (predicate()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Check(false, message);
}

void TestFailedLeaseStopRetainsInterlock(g1_web::RobotProfile profile) {
  using namespace g1_web;
  for (auto& capability : profile.capabilities) {
    if (capability.key == CapabilityKey::kLocomotion) {
      capability.parameters["real_control_policy"] = "operator_validation";
    }
  }
  std::atomic<int> stops{0};
  std::atomic<bool> allow_stop{false};
  auto adapter = CreateA2LocomotionAdapterForTesting(
      [&](const std::string& command, int, float, float, float) {
        if (command == "stop_move") {
          ++stops;
          return allow_stop.load() ? 0 : 3104;
        }
        return 0;
      });
  SnapshotStore store;
  ControlService service(store, std::move(adapter), profile, nullptr, false);
  std::string error;
  Check(service.Start(error), "injected transport service starts without DDS");
  unitree_hg::msg::dds_::SportModeState_ state;
  state.fsm_id(3);
  store.UpdateSportMode(state);
  Check(service.SubmitVelocity(.2F, 0, 0, 0, true).accepted,
        "injected velocity accepted before lease expires");
  WaitFor([&] { return store.GetSnapshot().control.motion.state == "active"; },
          "injected velocity reaches active state");
  WaitFor([&] {
    const auto motion = store.GetSnapshot().control.motion;
    return stops.load() > 0 && motion.state == "failed";
  }, "failed watchdog stop is reported");
  Check(store.GetSnapshot().control.motion.active,
        "failed watchdog stop retains active state until SDK confirms stop");
  Check(!service.SubmitVelocity(.1F, 0, 0, 0, true).accepted,
        "failed watchdog stop prevents new active velocity");
  WaitFor([&] { return stops.load() >= 2; },
          "failed watchdog stop is retried");
  Check(store.GetSnapshot().control.motion.active,
        "retry failure does not release active motion interlock");
  allow_stop.store(true);
  WaitFor([&] {
    const auto motion = store.GetSnapshot().control.motion;
    return motion.state == "stopped" && !motion.active;
  }, "successful retry finally releases stopped state");
  service.Stop();
}

void TestRenewalDuringBlockedStop(g1_web::RobotProfile profile) {
  using namespace g1_web;
  for (auto& capability : profile.capabilities) {
    if (capability.key == CapabilityKey::kLocomotion) {
      capability.parameters["real_control_policy"] = "operator_validation";
    }
  }
  std::atomic<int> moves{0};
  std::atomic<int> stops{0};
  std::atomic<bool> stop_entered{false};
  std::atomic<bool> renewed_move_checked{false};
  std::atomic<bool> renewed_move_interlocked{false};
  std::mutex stop_mutex;
  std::condition_variable stop_cv;
  bool release_stop = false;
  ControlService* service_ptr = nullptr;
  auto adapter = CreateA2LocomotionAdapterForTesting(
      [&](const std::string& command, int, float, float, float) {
        if (command == "stop_move") ++stops;
        if (command == "stop_move" && !stop_entered.exchange(true)) {
          std::unique_lock<std::mutex> lock(stop_mutex);
          Check(stop_cv.wait_for(lock, std::chrono::seconds(3), [&] { return release_stop; }),
                "test releases delayed stop without a deadlock");
        }
        if (command == "move" && ++moves == 2) {
          // This executes after the old stop has completed and released its owner.
          const auto attempt = service_ptr->Submit(
              "a2-while-renewed-move", "mode", "run", 0, true);
          renewed_move_interlocked.store(!attempt.accepted && attempt.error == "motion_active");
          renewed_move_checked.store(true);
        }
        return 0;
      });
  SnapshotStore store;
  ControlService service(store, std::move(adapter), profile, nullptr, false);
  service_ptr = &service;
  std::string error;
  Check(service.Start(error), "delayed-stop test starts injected transport without DDS");
  unitree_hg::msg::dds_::SportModeState_ state;
  state.fsm_id(3);
  store.UpdateSportMode(state);
  Check(service.SubmitVelocity(.2F, 0, 0, 0, true).accepted,
        "initial delayed-stop velocity accepted");
  WaitFor([&] { return stop_entered.load(); }, "lease expiry enters blocked StopMove");
  const auto blocked_mode = service.Submit("a2-during-stop", "mode", "run", 0, true);
  Check(!blocked_mode.accepted && blocked_mode.error == "motion_active",
        "in-flight StopMove retains locomotion resource");
  store.UpdateSportMode(state);
  Check(service.SubmitVelocity(.2F, 0, 0, 0, true).accepted,
        "renewal can queue behind in-flight stop");
  {
    const std::lock_guard<std::mutex> lock(stop_mutex);
    release_stop = true;
  }
  stop_cv.notify_one();
  WaitFor([&] { return renewed_move_checked.load(); },
          "queued renewal executes after blocked stop completes");
  Check(renewed_move_interlocked.load(),
        "renewed Move reacquires resource before SDK dispatch");
  const int stops_before_shutdown = stops.load();
  service.Stop();
  Check(stops.load() > stops_before_shutdown,
        "service shutdown sends final StopMove for renewed active motion");
}

void TestControlServiceSafety(const g1_web::RobotProfile& profile) {
  using namespace g1_web;
  SnapshotStore store;
  ControlService service(store, CreateA2LocomotionAdapter(true), profile,
                         nullptr, true);
  std::string error;
  Check(service.Start(error), "A2 control service starts with mock adapter");
  const auto set_fsm = [&](std::uint32_t fsm) {
    unitree_hg::msg::dds_::SportModeState_ state;
    state.fsm_id(fsm);
    state.fsm_mode(0);
    store.UpdateSportMode(state);
  };
  unsigned request_number = 0;
  const auto submit = [&](const char* command) {
    return service.Submit("a2-safety-" + std::to_string(++request_number),
                          "mode", command, 0, true);
  };
  const auto finish_command = [&](const char* command) {
    ControlSubmitResult result;
    WaitFor([&] {
      result = submit(command);
      Check(result.accepted || result.error == "control_busy",
            "allowed mode may only wait for previous command cleanup");
      return result.accepted;
    }, "allowed mode accepted");
    WaitFor([&] {
      const auto last = store.GetSnapshot().control.last_command;
      return last.request_id == result.request_id && last.state == "succeeded";
    }, "mock mode command succeeds");
  };

  auto stale = store.GetSnapshot().control;
  stale.sport_state_last_update = SteadyClock::now() - std::chrono::seconds(2);
  store.SetControlState(stale);
  Check(service.SubmitVelocity(.1F, 0, 0, 0, true).error == "sport_state_stale",
        "velocity submission rejects stale state");
  Check(submit("walk").error == "sport_state_stale",
        "mode submission rejects stale state");
  set_fsm(3);
  Check(!service.Submit("a2-unconfirmed", "mode", "walk", 0, false).accepted,
        "mode submission requires confirmation");
  for (std::uint32_t fsm : {2U, 3U, 4U, 5U, 99U}) {
    set_fsm(fsm);
    Check(submit("recovery_stand").error == "mode_transition_not_allowed",
          "recovery rejects every non-passive non-down source");
  }
  for (std::uint32_t fsm : {0U, 1U}) {
    set_fsm(fsm);
    finish_command("recovery_stand");
    Check(store.GetSnapshot().control.fsm_id == 3,
          "recovery mock reaches walking balance state");
  }
  for (std::uint32_t fsm : {0U, 1U, 2U, 6U, 99U}) {
    for (const char* command : {"walk", "run", "climb"}) {
      set_fsm(fsm);
      Check(submit(command).error == "mode_transition_not_allowed",
            "gait switch rejects sources outside walking running climbing");
    }
  }
  for (std::uint32_t fsm : {3U, 4U, 5U}) {
    for (const char* command : {"walk", "run", "climb"}) {
      set_fsm(fsm);
      finish_command(command);
    }
  }
  // Wait for the command worker to release its resource before exercising leases.
  set_fsm(3);
  WaitFor([&] {
    return service.SubmitVelocity(.2F, 0, 0, 0, true).accepted;
  }, "initial velocity accepted");
  WaitFor([&] { return store.GetSnapshot().control.motion.state == "active"; },
          "velocity becomes active");
  for (int renewal = 0; renewal < 6; ++renewal) {
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    Check(store.GetSnapshot().control.motion.active,
          "regular renewals keep motion active past original lease");
    set_fsm(3);
    Check(service.SubmitVelocity(.2F, 0, 0, 0, true).accepted,
          "velocity lease renewal accepted");
    WaitFor([&] { return store.GetSnapshot().control.motion.state == "active"; },
            "renewed velocity becomes active");
  }
  WaitFor([&] { return store.GetSnapshot().control.motion.state == "stopped"; },
          "missing renewal expires backend lease");
  const auto stopped = store.GetSnapshot().control.motion;
  Check(!stopped.active && stopped.vx_m_s == 0 && stopped.vy_m_s == 0 &&
            stopped.vyaw_rad_s == 0,
        "expired velocity lease publishes all-zero stopped state");
  service.Stop();

  RuntimeOptions readonly_options;
  readonly_options.mock = true;
  readonly_options.read_only = true;
  auto readonly = RobotRegistry::CreateRuntime("a2", readonly_options);
  SnapshotStore readonly_store;
  ControlService readonly_service(readonly_store, std::move(readonly.locomotion),
                                  readonly.profile, nullptr, true);
  Check(readonly_service.Start(error), "readonly mock service initializes for rejection checks");
  Check(!readonly_service.SubmitVelocity(.1F, 0, 0, 0, true).accepted &&
            !readonly_service.Submit("a2-readonly", "mode", "walk", 0, true).accepted,
        "readonly capability rejects motion and mode commands");
  readonly_service.Stop();
}

void TestJointDebug(const g1_web::RobotProfile& profile) {
  using namespace g1_web;
  const auto web_root =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
  auto policy = CreateA2JointDebugPolicy(profile);
  Check(policy->GroupsForMode("full_body") ==
            std::vector<std::string>({"front_right_leg", "front_left_leg",
                                      "rear_right_leg", "rear_left_leg"}) &&
            policy->GroupsForMode("upper_body").empty() &&
            policy->TransportForMode("full_body") ==
                JointDebugTransport::kLowCmd &&
            policy->UsesMotionSwitcherForLowCmd() &&
            !policy->SupportsMotionModeSwitch() &&
            policy->LowCmdBlockingServices() ==
                std::vector<std::string>({"ai_sport"}) &&
            policy->TeachGroups() == policy->GroupsForMode("full_body") &&
            policy->SupportsTeachRemoteControl(),
        "A2 policy exposes guarded full-body HG LowCmd and Mock teaching without automatic switching");
  const auto gain = policy->NormalGain(profile.joint_schema.joints.front());
  Check(gain.kp == 0.0F && gain.kd == 0.0F &&
            policy->ResolveModelVariant(1) != nullptr &&
            policy->ResolveModelVariant(0) == nullptr,
        "A2 mock-only policy keeps zero gains and exact model selection");
  std::vector<std::pair<std::size_t, float>> targets;
  for (std::size_t index = 0; index < 12; ++index) {
    targets.emplace_back(index, index % 3 == 1 ? 0.8F
                                  : index % 3 == 2 ? -1.5F
                                                   : 0.0F);
  }

  SnapshotStore empty_store;
  ControlService empty_control(
      empty_store, CreateA2LocomotionAdapter(true), profile,
      CreateA2JointDebugPolicy(profile), true, web_root.string(), (std::filesystem::temp_directory_path() /
                             "unirobo_a2_empty_test_store.json").string());
  std::string error;
  Check(empty_control.Start(error), "A2 empty Mock Joint Debug starts");
  Check(empty_control.ApplyJointDebug("full_body", targets, true).error ==
            "lowstate_unavailable",
        "A2 rejects Joint Debug without fresh LowState");
  empty_control.Stop();

  SnapshotStore store;
  unitree_hg::msg::dds_::LowState_ low;
  low.mode_machine(1);
  for (std::size_t index = 0; index < 12; ++index) {
    low.motor_state()[index].mode(1);
    low.motor_state()[index].q(index % 3 == 1 ? 0.8F
                                  : index % 3 == 2 ? -1.5F
                                                   : 0.0F);
  }
  store.UpdateLowState(low);
  ControlService control(store, CreateA2LocomotionAdapter(true), profile,
                         CreateA2JointDebugPolicy(profile), true,
                         web_root.string(), (std::filesystem::temp_directory_path() /
                             "unirobo_a2_empty_test_store.json").string());
  error.clear();
  Check(control.Start(error), "A2 mock Joint Debug starts");
  auto status = Parse(control.SerializeJointDebugStatus());
  Check(status["initialized"].asBool() && status["joints"].size() == 12 &&
            status["full_body_allowed"].asBool() &&
            !status["upper_body_allowed"].asBool() &&
            status["control_topics"]["full_body"].asString() ==
                "rt/lowcmd" &&
            !status["motion_mode_switch_supported"].asBool() &&
            status["remote_binding_supported"].asBool() &&
            status["teach_actions"].empty(),
        "A2 status exposes twelve full-body joints and Mock teaching without switching");
  auto duplicate_targets = targets;
  duplicate_targets[1].first = 0;
  auto out_of_range_targets = targets;
  out_of_range_targets[0].second = 99.0F;
  Check(control.ApplyJointDebug("upper_body", targets, true).error ==
            "invalid_control_mode" &&
            control.ApplyJointDebug("full_body", targets, false).error ==
                "confirmation_required" &&
            control.ApplyJointDebug("full_body", duplicate_targets, true)
                    .error == "invalid_joint_request" &&
            control.ApplyJointDebug("full_body", out_of_range_targets, true).error ==
                "joint_out_of_range" &&
            control.SwitchJointDebugMode(true, true).error ==
                "motion_mode_switch_unsupported" &&
            !control.StartJointTeachRecording("", true).accepted,
        "A2 rejects unsupported modes, invalid targets, automatic switching and invalid teaching names");
  control.SetMockJointDebugAiSport(true);
  Check(control.ApplyJointDebug("full_body", targets, true).error ==
            "debug_mode_required",
        "A2 rejects Joint Debug while ai_sport remains active");
  control.SetMockJointDebugAiSport(false);
  low.mode_pr(1);
  store.UpdateLowState(low);
  Check(control.ApplyJointDebug("full_body", targets, true).error ==
            "pr_mode_required",
        "A2 requires processed joint coordinates for HG LowCmd");
  low.mode_pr(0);
  store.UpdateLowState(low);
  targets[0].second = 0.1F;
  low.motor_state()[0].q(std::numeric_limits<float>::quiet_NaN());
  store.UpdateLowState(low);
  Check(!Parse(control.SerializeJointDebugStatus())["full_body_allowed"].asBool() &&
            control.ApplyJointDebug("full_body", targets, true).error ==
                "joint_state_invalid",
        "A2 refuses non-finite live joint positions before LowCmd starts");
  low.motor_state()[0].q(0.0F);
  store.UpdateLowState(low);
  Check(control.ApplyJointDebug("full_body", targets, true).accepted,
        "A2 accepts a bounded confirmed Mock target");
  WaitFor([&] { return control.GetJointDebugTestStats().lowcmd_published; },
          "A2 Mock publishes through the shared HG LowCmd loop");
  Check(control.HeartbeatJointDebug().accepted,
        "A2 Mock Joint Debug heartbeat renews the lease");
  const auto stats = control.GetJointDebugTestStats();
  Check(std::all_of(stats.last_kp.begin(), stats.last_kp.end(),
                    [](float value) { return value == 0.0F; }) &&
            std::all_of(stats.last_kd.begin(), stats.last_kd.end(),
                        [](float value) { return value == 0.0F; }) &&
            std::all_of(stats.last_q.begin() + 12, stats.last_q.end(),
                        [](float value) { return value == 0.0F; }),
        "A2 Mock keeps zero gains and leaves reserved motor slots untouched");
  low.motor_state()[0].q(std::numeric_limits<float>::quiet_NaN());
  store.UpdateLowState(low);
  WaitFor([&] {
    const auto next = Parse(control.SerializeJointDebugStatus());
    return next["dds_state"].asString() == "idle" &&
           next["last_error"].asString() == "joint_state_invalid";
  }, "A2 Mock releases LowCmd if a live joint position becomes invalid");
  low.motor_state()[0].q(0.0F);
  store.UpdateLowState(low);
  Check(control.ApplyJointDebug("full_body", targets, true).accepted,
        "A2 Mock can resume only after receiving valid LowState");
  low.mode_pr(1);
  store.UpdateLowState(low);
  WaitFor([&] {
    const auto next = Parse(control.SerializeJointDebugStatus());
    return next["dds_state"].asString() == "idle" &&
           next["last_error"].asString() == "pr_mode_required";
  }, "A2 Mock releases control if LowState leaves processed-coordinate mode");
  low.mode_pr(0);
  store.UpdateLowState(low);
  Check(control.StopJointDebug(true).accepted,
        "A2 Mock Joint Debug releases deterministically");
  control.Stop();

  const auto teach_store = std::filesystem::temp_directory_path() /
      ("a2_teach_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  SnapshotStore teach_snapshot;
  teach_snapshot.UpdateLowState(low);
  ControlService teach_control(teach_snapshot, CreateA2LocomotionAdapter(true), profile,
                               CreateA2JointDebugPolicy(profile), true,
                               web_root.string(), teach_store.string());
  Check(teach_control.Start(error), "A2 Mock teaching starts");
  const auto remote_policy = CreateA2JointDebugPolicy(profile);
  Check(remote_policy->TeachRemoteDataSource() == TeachRemoteSource::kWirelessController &&
            !remote_policy->TeachRemoteRequiresActiveSession() &&
            remote_policy->TeachRemoteSecondaryKeyLabel() == "F3",
        "A2 uses WirelessController and permits interlocked idle playback");
  Check(teach_control.StartJointTeachRecording("a2_teach", true).accepted,
        "A2 Mock starts recording");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  Check(teach_control.FinishJointTeachRecording(true, true).accepted,
        "A2 Mock saves recording");
  Check(teach_control.SetJointTeachRemoteBinding("a2_teach", "F3+A").accepted,
        "A2 Mock saves remote binding");
  const auto taught = Parse(teach_control.SerializeJointDebugStatus());
  Check(taught["teach_actions"].size() == 1 &&
            taught["teach_actions"][0]["frames"].asUInt() >= 2 &&
            taught["teach_actions"][0]["remote_binding"].asString() == "F3+A",
        "A2 Mock recording and binding are retained");
  Check(teach_control.StopJointDebug(true).accepted, "A2 remote test starts from idle");
  teach_control.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  teach_control.SetMockJointDebugRemoteKeys(0);
  const auto triggered = Parse(teach_control.SerializeJointDebugStatus());
  Check(triggered["remote_last_action"].asString() == "a2_teach" &&
            triggered["last_error"].asString().empty() &&
            triggered["dds_state"].asString() == "active" &&
            triggered["remote_message_count"].asUInt64() == 2,
        "A2 Mock remote binding starts playback from idle");
  teach_control.SetMockJointDebugRemoteKeys(1U << 2U);
  const auto released = Parse(teach_control.SerializeJointDebugStatus());
  Check(released["dds_state"].asString() == "idle" &&
            released["remote_last_action"].asString() == "release_control" &&
            released["remote_last_binding"].asString() == "START",
        "START releases active A2 playback before final-pose holding");
  teach_control.SetMockJointDebugRemoteKeys(0);
  teach_control.SetMockJointDebugAiSport(true);
  teach_control.SetMockJointDebugRemoteKeys((1U << 7U) | (1U << 8U));
  const auto blocked_remote = Parse(teach_control.SerializeJointDebugStatus());
  Check(blocked_remote["dds_state"].asString() == "idle" &&
            !blocked_remote["last_error"].asString().empty(),
        "A2 remote playback rejects an active high-level service");
  teach_control.Stop();
  std::filesystem::remove(teach_store);

  SnapshotStore lease_store;
  lease_store.UpdateLowState(low);
  ControlService lease_control(
      lease_store, CreateA2LocomotionAdapter(true), profile,
      CreateA2JointDebugPolicy(profile), true, web_root.string(), (std::filesystem::temp_directory_path() /
                             "unirobo_a2_empty_test_store.json").string());
  error.clear();
  Check(lease_control.Start(error), "A2 lease-validation service starts");
  targets[0].second = 0.0F;
  Check(lease_control.ApplyJointDebug("full_body", targets, true).accepted,
        "A2 Mock Joint Debug starts for lease validation");
  WaitFor([&] {
    lease_store.UpdateLowState(low);
    const auto next = Parse(lease_control.SerializeJointDebugStatus());
    return next["dds_state"].asString() == "idle" &&
           next["last_error"].asString() == "control_lease_expired";
  }, "A2 Mock Joint Debug lease expiry releases control");
  lease_control.Stop();
}

}
void TestSlamControlFeedback() {
  using namespace g1_web;
  for (const auto* command : {"navigate", "pause_navigation", "resume_navigation", "cancel_navigation"}) {
    Check(IsUnverifiedSlamControlResponse(command, 3203), "known control RPC mismatch permits dispatch");
    Check(!IsUnverifiedSlamControlResponse(command, 3104), "timeouts remain errors");
    Check(!IsUnverifiedSlamControlResponse(command, 0), "normal success is not unverified");
  }
  Check(!IsUnverifiedSlamControlResponse("load_map", 3203), "localization still requires actual data");
  Check(!IsUnverifiedSlamControlResponse("unknown", 3203), "unknown commands remain rejected");
}

int main() {
  TestSlamControlFeedback();
  using namespace g1_web;
  std::array<std::uint8_t, 40> remote{};
  remote[2] = 0x80; remote[3] = 0x01; remote[4] = 0xff;
  Check(DecodeRaw40RemoteKeys(remote) == 0x180, "remote key word uses bytes 2 and 3");
  RuntimeOptions options; options.mock = true;
  auto missing_attachment = RobotRegistry::CreateRuntime("a2", options);
  const auto missing_slam = std::find_if(missing_attachment.profile.capabilities.begin(),
      missing_attachment.profile.capabilities.end(), [](const auto& capability) {
        return capability.key == CapabilityKey::kSlam;
      });
  Check(missing_slam != missing_attachment.profile.capabilities.end() &&
            !missing_slam->available && missing_slam->reason == "required_attachment_disabled",
        "A2 SLAM requires an explicit JT128 declaration");
  options.enabled_devices.insert("jt128");
  auto runtime = RobotRegistry::CreateRuntime("a2", options);
  const auto& profile = runtime.profile;
  const auto& perception_policy = runtime.devices->Perception();
  Check(perception_policy.lidar_model == "JT128" &&
            perception_policy.navigation_max_distance_m == 30.0 &&
            perception_policy.dependencies.size() == 1 &&
            perception_policy.dependencies[0].name == "unitree_slam" &&
            perception_policy.dependencies[0].lifecycle == DependencyLifecycle::kExternalRequired &&
            !perception_policy.raw_input_required &&
            perception_policy.mapping_points_topic == "rt/unitree/slam_mapping/points" &&
            perception_policy.mapping_odom_topic == "rt/unitree/slam_mapping/odom" &&
            perception_policy.relocation_points_topic == "rt/unitree/slam_relocation/points" &&
            perception_policy.relocation_odom_topic == "rt/unitree/slam_relocation/odom" &&
            perception_policy.global_map_topic == "rt/unitree/slam_relocation/global_map" &&
            perception_policy.slam_info_topic == "rt/slam_info" &&
            perception_policy.slam_key_info_topic == "rt/slam_key_info" &&
            runtime.devices->DetectHardware(CapabilityKey::kSlam) == HardwarePresence::kPresent &&
            runtime.devices->DetectHardware(CapabilityKey::kLidar) == HardwarePresence::kUnknown,
        "A2 configures documented external SLAM topics");
  Check(perception_policy.raw_lidar_points_topic == "rt/unitree/slam_lidar/points" &&
        perception_policy.raw_lidar_imu_topic == "rt/unitree/slam_lidar/imu1" &&
        perception_policy.raw_lidar_frame == "base_link" &&
        perception_policy.raw_lidar_to_base[2] == 1 &&
        std::abs(perception_policy.raw_lidar_to_base[3] - 0.33767F) < 1e-6 &&
        perception_policy.low_obstacle_source.empty(), "A2 preview uses its own transform without enabling obstacle control");
  PerceptionService perception(true, true, *runtime.devices);
  std::string perception_error;
  Check(perception.Start(perception_error), "A2 mock perception starts");
  const auto slam_status = Parse(perception.SerializeStatus());
  Check(slam_status["navigation_max_distance_m"].asDouble() == 30.0,
        "A2 status advertises the official 30 m navigation bound");
  PerceptionRequest nav_request;
  nav_request.request_key = "a2-map-load-0001";
  nav_request.command = "load_map";
  nav_request.map_name = "test1.pcd";
  nav_request.confirmed = true;
  Check(perception.Submit(nav_request).accepted, "A2 mock map loads");
  nav_request.command = "navigate";
  nav_request.request_key = "a2-goal-over-30m";
  nav_request.pose.x = 32.0;
  nav_request.pose.y = 0.0;
  Check(perception.Submit(nav_request).error == "goal_over_limit",
        "A2 goal beyond 30 m is rejected");
  nav_request.request_key = "a2-goal-within-30m";
  nav_request.pose.x = 29.0;
  Check(perception.Submit(nav_request).accepted, "A2 goal within 30 m mock limit accepted");
  perception.Stop();
  Check(profile.joint_schema.motor_slot_count == 35 && profile.joint_schema.joints.size() == 12, "A2 topology");
  for (const auto& capability : profile.capabilities) {
    if (capability.key == CapabilityKey::kLocomotion) {
      Check(capability.parameters.at("interaction_action_commands") == "front_flip,back_flip" &&
                capability.parameters.at("mode_sources").find("front_flip:3;back_flip:3") != std::string::npos,
            "A2 built-in actions are declared with restricted source FSM");
    }
  }
  Check(runtime.devices && runtime.locomotion && runtime.audio && runtime.joint_debug,
        "mock bundle includes A2 Joint Debug policy");
  Check(runtime.audio->Features().tts && runtime.audio->Features().asr &&
            runtime.audio->Features().volume && runtime.audio->Features().play_stream &&
            !runtime.audio->Features().builtin_llm, "A2 native audio features");
  std::string audio_error;
  Check(runtime.audio->Start(audio_error), "A2 mock audio starts");
  std::uint8_t audio_volume = 0;
  Check(runtime.audio->GetVolume(audio_volume) == 0 && audio_volume == 82 &&
            runtime.audio->TtsMaker("你好", 0) == 0, "A2 mock volume and TTS");
  runtime.audio->Stop();
  const auto* rgb = FindCameraStream(runtime.devices->Camera(), CameraStreamRole::kRgb);
  Check(rgb && rgb->transport == CameraTransport::kRtpH264Udp &&
            rgb->fixed_source == "udp://230.1.1.1:1720" &&
            rgb->width == 1280 && rgb->height == 720 && rgb->fps == 15 &&
            rgb->multicast_interface == "eth0" &&
            !runtime.devices->Camera().allow_client_source_override,
        "A2 RGB uses the official fixed native stream");
  Check(BuildFixedRtpH264UdpPipeline(*rgb).find("port=1720 multicast-iface=eth0") != std::string::npos,
        "A2 RGB uses the existing fixed H264 receiver");
  for (std::size_t i = 0; i < 12; ++i) Check(profile.joint_schema.joints[i].motor_slot == i, "motor slots");
  for (const auto& capability : profile.capabilities) {
    if (capability.key == CapabilityKey::kCameraRgb) {
      Check(capability.implemented && capability.available &&
                capability.verification_level == VerificationLevel::kMockVerified,
            "A2 RGB is implemented but not declared real verified");
    } else if (capability.key == CapabilityKey::kAudio) {
      Check(capability.implemented && capability.available &&
                capability.verification_level == VerificationLevel::kMockVerified &&
                capability.parameters.at("tts_backends") == "kokoro,unitree" &&
                capability.parameters.at("play_stream_transport") == "a2_audio_client" &&
                capability.parameters.at("default_tts_backend") == "unitree" &&
                capability.parameters.at("builtin_llm") == "false" &&
                capability.parameters.count("ui_placeholder") == 0,
            "A2 audio is enabled without claiming real verification or builtin LLM");
    } else if (capability.key == CapabilityKey::kSlam) {
      Check(capability.implemented && capability.available &&
                capability.verification_level == VerificationLevel::kMockVerified &&
                capability.parameters.at("required_devices") == "jt128" &&
                capability.parameters.at("navigation_max_distance_m") == "30" &&
                capability.parameters.count("ui_placeholder") == 0,
            "A2 SLAM is enabled only with the declared JT128 and remains mock verified");
    } else if (capability.key == CapabilityKey::kLidar) {
      Check(capability.implemented && capability.available &&
                capability.parameters.at("raw_preview_only") == "true",
            "A2 JT128 preview is implemented without motion control");
    } else if (capability.key == CapabilityKey::kJointDebug) {
      Check(capability.implemented && capability.available &&
                capability.verification_level == VerificationLevel::kMockVerified &&
                capability.parameters.at("control_modes") == "full_body" &&
                capability.parameters.at("control_topic") == "rt/lowcmd" &&
                capability.parameters.at("real_control_policy") == "operator_validation",
            "A2 Joint Debug is available only as a Mock-verified full-body capability");
    } else if (capability.key == CapabilityKey::kJointTeach) {
      Check(capability.implemented && capability.available &&
                capability.parameters.at("real_control_policy") == "operator_validation",
            "A2 teaching requires operator validation");
    } else if (capability.key != CapabilityKey::kTelemetry && capability.key != CapabilityKey::kLocomotion) {
      Check(!capability.implemented && !capability.available, "unsupported capability");
      if (capability.key == CapabilityKey::kCameraDepth) {
        Check(capability.parameters.count("ui_placeholder") == 0, "A2 depth panel is hidden");
      }
    }
  }
  for (bool readonly : {false, true}) {
    options.mock = false; options.read_only = readonly;
    auto real = RobotRegistry::CreateRuntime("a2", options);
    for (const auto& cap : real.profile.capabilities) {
      if (cap.key == CapabilityKey::kLocomotion) {
        Check(cap.available == !readonly &&
                  cap.verification_level == VerificationLevel::kMockVerified &&
                  cap.parameters.at("real_control_policy") == "operator_validation",
              "A2 operator validation is available except in read-only mode, without claiming control verification");
      } else if (cap.key == CapabilityKey::kSlam) {
        Check(cap.available == !readonly &&
                  cap.verification_level == VerificationLevel::kMockVerified &&
                  (readonly ? cap.reason == "readonly_runtime" : cap.reason.empty()),
              "A2 SLAM commands are closed in read-only mode");
      } else if (cap.key == CapabilityKey::kJointDebug || cap.key == CapabilityKey::kJointTeach) {
        Check(cap.implemented && !cap.available && real.joint_debug &&
                  cap.verification_level == VerificationLevel::kMockVerified &&
                  cap.parameters.at("real_control_policy") == "operator_validation",
              "A2 Joint Debug and teaching remain unavailable in every real runtime");
      }
    }
  }
  {
    RuntimeOptions unavailable_options;
    auto unavailable = RobotRegistry::CreateRuntime("a2", unavailable_options);
    SnapshotStore preview_store;
    unitree_hg::msg::dds_::LowState_ preview_low;
    preview_low.mode_machine(1);
    for (std::size_t i = 0; i < 12; ++i) preview_low.motor_state()[i].mode(1);
    preview_store.UpdateLowState(preview_low);
    const auto web = std::filesystem::path(__FILE__).parent_path().parent_path() / "web";
    ControlService preview(preview_store, CreateA2LocomotionAdapter(true),
                           unavailable.profile, std::move(unavailable.joint_debug),
                           true, web.string());
    std::string preview_error;
    Check(preview.Start(preview_error), "A2 unavailable preview initializes without output");
    const auto state = Parse(preview.SerializeJointDebugStatus());
    Check(!state["initialized"].asBool() && state["metadata_ready"].asBool() &&
              !state["control_available"].asBool() && state["joints"].size() == 12 &&
              !state["full_body_allowed"].asBool(),
          "A2 unavailable preview exposes joints but disallows control");
    Check(!preview.ApplyJointDebug("full_body", {{0, 0}}, true).accepted &&
              !preview.StartJointTeachRecording("preview", true).accepted &&
              !preview.PlayJointTeachAction("preview", true).accepted &&
              !preview.DeleteJointTeachAction("preview", true).accepted &&
              !preview.SetJointTeachRemoteBinding("preview", "F3+A").accepted &&
              !preview.GetJointDebugTestStats().lowcmd_published,
          "A2 unavailable preview rejects output and recording");
    preview.Stop();
  }
  const auto gains_path = std::filesystem::temp_directory_path() /
      ("a2_gains_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  {
    std::ofstream file(gains_path);
    file << R"({"remote_protocol":"unitree_raw40_keys_v1","joints":[)";
    for (int i = 0; i < 12; ++i) {
      if (i) file << ',';
      file << R"({"kp":2,"kd":1,"record_kp":0,"record_kd":0.5})";
    }
    file << R"(]})";
  }
  A2JointGains configured;
  Check(LoadA2JointGains(gains_path.string(), configured) &&
            configured.remote_protocol_confirmed && configured.normal[11].kp == 2 && configured.recording[11].kd == .5F,
        "A2 loads all twelve explicitly configured joint gains");
  options.mock = false; options.read_only = false;
  options.a2_joint_gains_file = gains_path.string();
  auto configured_runtime = RobotRegistry::CreateRuntime("a2", options);
  for (const auto& cap : configured_runtime.profile.capabilities) {
    if (cap.key == CapabilityKey::kJointDebug || cap.key == CapabilityKey::kJointTeach)
      Check(cap.available && cap.parameters.at("real_control_policy") == "operator_validation",
            "valid configuration opens operator validation");
  }
  Check(configured_runtime.joint_debug != nullptr, "configured A2 joint policy exists");
  { std::ofstream file(gains_path); file << R"({"joints":[{"kp":-1}]})"; }
  Check(!LoadA2JointGains(gains_path.string(), configured), "invalid gains rejected");
  auto invalid_runtime = RobotRegistry::CreateRuntime("a2", options);
  for (const auto& cap : invalid_runtime.profile.capabilities) {
    if (cap.key == CapabilityKey::kJointDebug || cap.key == CapabilityKey::kJointTeach)
      Check(!cap.available && cap.reason == "a2_joint_gains_invalid", "invalid configuration fails closed");
  }
  std::filesystem::remove(gains_path);
  auto adapter = CreateA2LocomotionAdapter(true);
  LocomotionInitialization state; std::string error;
  Check(adapter->Initialize(state, error) && state.fsm_id == 3, "default walking");
  const float limits[3][2][3] = {
      {{.8F,.5F,2.F},{1.5F,.5F,2.5F}},
      {{2.5F,.3F,1.5F},{4.F,.3F,1.F}},
      {{.6F,.3F,1.F},{.6F,.3F,1.F}}};
  ControlData control;
  for (unsigned fsm = 3; fsm <= 5; ++fsm) {
    control.fsm_id = fsm;
    for (int speed = 0; speed <= 1; ++speed) {
      const auto& v = limits[fsm-3][speed];
      for (float sign : {-1.F, 1.F}) {
        Check(adapter->ValidateVelocity(control, sign*v[0], sign*v[1], sign*v[2], speed).empty(), "boundary accepted");
      }
      Check(!adapter->ValidateVelocity(control, v[0]+.001F, 0, 0, speed).empty(), "x bound");
      Check(!adapter->ValidateVelocity(control, .1F, v[1]+.001F, 0, speed).empty(), "y bound");
      Check(!adapter->ValidateVelocity(control, .1F, 0, v[2]+.001F, speed).empty(), "yaw bound");
    }
  }
  control.fsm_id = 3;
  Check(!adapter->ValidateVelocity(control, std::numeric_limits<float>::quiet_NaN(),0,0,0).empty(), "NaN rejected");
  Check(!adapter->ValidateVelocity(control, .1F,0,std::numeric_limits<float>::infinity(),0).empty(), "Inf rejected");
  Check(!adapter->ValidateVelocity(control, .1F,0,0,3).empty(), "invalid speed");
  control.fsm_id = 4;
  Check(adapter->ValidateVelocity(control, 4,0,0,1).empty(), "running high");
  control.fsm_id = 5;
  Check(!adapter->ValidateVelocity(control, 4,0,0,1).empty(), "queued vector invalid after gait change");
  control.fsm_id = 99;
  Check(!adapter->ValidateVelocityExecution(control,0).empty(), "unknown FSM");
  std::vector<std::string> calls; int failure = 0;
  std::string failing_command = "speed_level";
  auto transport = CreateA2LocomotionAdapterForTesting(
      [&](const std::string& command, int argument, float, float, float) {
        calls.push_back(command + ":" + std::to_string(argument));
        return failure && command == failing_command ? failure : 0;
      });
  Check(transport->Initialize(state,error), "injected transport");
  for (const auto& command : {"damp","stand_up","stand_down","balance_stand","stop_move","recovery_stand","walk","run","climb"}) {
    Check(transport->ExecuteMode(command,0).api_result == 0, "allowed command");
  }
  Check(calls[6] == "walk:0" && calls[7] == "run:1" && calls[8] == "climb:2", "gait arguments");
  for (const auto& command : {"left_side_gait_on","left_side_gait_off","right_side_gait_on","right_side_gait_off",
                              "hand_stand_on","hand_stand_off","biped_stand_on","biped_stand_off","front_flip","back_flip"}) {
    Check(transport->ExecuteMode(command,0).api_result == 0, "A2 built-in action is mapped");
  }
  Check(transport->ExecuteMode("front_flip",1).api_result != 0 && transport->ExecuteMode("walk",1).api_result != 0,
        "reject invalid command arguments");
  calls.clear(); failure=3104;
  Check(transport->ApplyVelocity(.1F,0,0,1,true).api_result == 3104 && calls.size()==1, "SpeedLevel failure prevents Move");
  calls.clear(); failure=0;
  Check(transport->ApplyVelocity(.1F,0,0,1,true).api_result == 0 && calls == std::vector<std::string>{"speed_level:1","move:0"}, "SpeedLevel precedes Move");
  calls.clear(); failure = 4205; failing_command = "move";
  Check(transport->ApplyVelocity(.1F,0,0,1,true).api_result == 4205 &&
            calls == std::vector<std::string>{"speed_level:1","move:0"},
        "Move failure preserves original SDK result after successful SpeedLevel");
  failure = 0;
  calls.clear();
  bool guard_valid = true;
  auto guarded = CreateA2LocomotionAdapterForTesting(
      [&](const std::string& command, int, float, float, float) {
        calls.push_back(command);
        if (command == "speed_level") guard_valid = false;
        return 0;
      });
  Check(guarded->Initialize(state, error), "guarded injected transport initializes");
  const auto guard = [&] {
    return guard_valid ? std::string{} : std::string{"motion_precondition_changed"};
  };
  auto guarded_result = guarded->ApplyVelocityGuarded(.1F, 0, 0, 0, true, guard);
  Check(guarded_result.api_result != 0 && guarded_result.error == "motion_precondition_changed" &&
            calls == std::vector<std::string>{"speed_level"},
        "state change during SpeedLevel prevents Move and preserves guard error");
  calls.clear();
  guarded_result = guarded->ApplyVelocityGuarded(.1F, 0, 0, 0, true, guard);
  Check(guarded_result.api_result != 0 && guarded_result.error == "motion_precondition_changed" && calls.empty(),
        "initial guard rejection makes no transport calls");
  transport->ApplyVelocity(0,0,0,0,false);
  Check(calls == std::vector<std::string>{"stop_move:0"}, "stop bypasses speed setting");

  SnapshotStore store;
  unitree_go::msg::dds_::SportModeState_ sport;
  sport.mode(4); sport.error_code(1016);
  store.UpdateModeSportState(sport);
  Check(store.GetSnapshot().control.fsm_id == 4, "mode FSM");
  store.UpdateGo2SportMode(sport);
  Check(store.GetSnapshot().control.fsm_id == 1016, "GO2 error_code unchanged");
  unitree_hg::msg::dds_::BmsState_ battery;
  battery.soc(82); store.UpdateBms(battery);
  auto root = Parse(SerializeSnapshot(profile, store.GetSnapshot()));
  Check(root["batteries"].size()==2 && root["batteries"][0]["id"]=="2" && root["battery"]["soc_pct"]==82, "legacy primary battery");
  Check(root["batteries"][1]["state"]["received"]==false, "absent battery not filled");
  battery.soc(31); store.UpdateBms(battery,true);
  battery.soc(76); store.UpdateBms(battery);
  auto snapshot = store.GetSnapshot();
  snapshot.sources[static_cast<std::size_t>(SourceId::kBms)].last_update -= std::chrono::seconds(4);
  root=Parse(SerializeSnapshot(profile,snapshot));
  Check(root["batteries"][0]["state"]["status"]=="offline" && root["batteries"][1]["state"]["status"]=="online", "independent freshness");
  Check(root["batteries"][0]["soc_pct"]==76 && root["batteries"][1]["soc_pct"]==31, "independent updates");
  Check(Parse(SerializeSnapshot(*RobotRegistry::Find("g1"),snapshot))["batteries"].empty(), "old profiles retain single battery");
  MockDataSource mock(store,profile); mock.Start(); std::this_thread::sleep_for(std::chrono::milliseconds(120)); mock.Stop();
  snapshot=store.GetSnapshot();
  Check(!snapshot.sources[static_cast<std::size_t>(SourceId::kSecondaryImu)].received && !snapshot.faults.received, "only declared mock sources");
  Check(snapshot.sources[static_cast<std::size_t>(SourceId::kSecondaryBms)].received, "secondary mock source");
  TestJointDebug(profile);
  TestControlServiceSafety(profile);
  TestFailedLeaseStopRetainsInterlock(profile);
  TestRenewalDuringBlockedStop(profile);
  std::cout << "A2 profile, dispatch, velocity limits, state interpretation, batteries and control safety passed\n";
}
