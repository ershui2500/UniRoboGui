#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "g1_web/joint_debug_policy.hpp"
#include "g1_web/locomotion.hpp"
#include "g1_web/resource_manager.hpp"
#include "g1_web/safety_manager.hpp"
#include "g1_web/snapshot_store.hpp"

namespace g1_web {

struct ControlSubmitResult {
  bool accepted{false};
  bool duplicate{false};
  std::uint64_t request_id{0};
  std::string error;
};

struct MotionSubmitResult {
  bool accepted{false};
  std::uint64_t sequence{0};
  std::string error;
};

struct JointDebugSubmitResult {
  bool accepted{false};
  std::string error;
};

struct JointDebugTestStats {
  std::uint64_t loop_starts{0};
  std::uint64_t publish_count{0};
  float maximum_step_rad{0.0F};
  bool lowcmd_published{false};
  std::vector<float> last_q;
  std::vector<float> last_kp;
  std::vector<float> last_kd;
};

class ControlService {
 public:
  ControlService(SnapshotStore& store,
                 std::unique_ptr<ILocomotion> locomotion,
                 const RobotProfile& robot_profile,
                 std::unique_ptr<IJointDebugPolicy> joint_debug_policy,
                 bool mock, std::string web_root = {},
                 std::string joint_teach_store = {});
  ~ControlService();

  bool Start(std::string& error);
  void Stop();

  ControlSubmitResult Submit(const std::string& request_key,
                             const std::string& category,
                             const std::string& command,
                             std::int32_t argument,
                             bool confirmed,
                             const std::string& action_name = {});
  MotionSubmitResult SubmitVelocity(float vx_m_s, float vy_m_s,
                                    float vyaw_rad_s, std::int32_t speed_mode,
                                    bool active);
  std::string SerializeJointDebugStatus();
  JointDebugSubmitResult ApplyJointDebug(
      const std::string& mode,
      const std::vector<std::pair<std::size_t, float>>& targets,
      bool confirmed);
  JointDebugSubmitResult StopJointDebug(bool confirmed);
  JointDebugSubmitResult HeartbeatJointDebug();
  JointDebugSubmitResult StartJointTeachRecording(
      const std::string& name, bool confirmed);
  JointDebugSubmitResult FinishJointTeachRecording(bool confirmed,
                                                   bool release_control);
  JointDebugSubmitResult PlayJointTeachAction(
      const std::string& name, bool confirmed);
  JointDebugSubmitResult DeleteJointTeachAction(
      const std::string& name, bool confirmed);
  JointDebugSubmitResult SetJointTeachRemoteBinding(
      const std::string& name, const std::string& binding);
  JointDebugTestStats GetJointDebugTestStats() const;
  void SetMockJointDebugAiSport(bool ai_sport_active);
  void SetMockJointDebugRemoteKeys(std::uint16_t keys);

  bool IsKnownCommand(const std::string& category,
                      const std::string& command,
                      std::int32_t argument) const;

 private:
  struct Request {
    std::uint64_t request_id{0};
    std::string request_key;
    std::string category;
    std::string command;
    std::int32_t argument{0};
    std::string action_name;
    ControlResource resource{ControlResource::Locomotion};
    std::string resource_owner;
    bool release_resource{false};
  };

  struct VelocityRequest {
    std::uint64_t sequence{0};
    float vx_m_s{0.0F};
    float vy_m_s{0.0F};
    float vyaw_rad_s{0.0F};
    std::int32_t speed_mode{0};
    bool active{false};
    bool release_resource{false};
  };

  bool ValidatePreconditions(const Request& request,
                             std::string& error) const;
  void WorkerLoop();
  void MotionWorkerLoop();
  void StateWorkerLoop();
  void Execute(Request request);
  std::int32_t ExecuteReal(const Request& request,
                           std::string& error);
  void ExecuteMock(const Request& request);
  bool WaitForFsm(std::uint32_t target_fsm_id,
                  std::chrono::milliseconds timeout) const;
  static bool IsValidRequestKey(const std::string& request_key);
  static bool IsValidActionName(const std::string& action_name);
  static std::int64_t SystemTimeMs();

  class JointDebugImpl;
  static std::unique_ptr<JointDebugImpl> CreateJointDebugImpl(
      ControlService& owner, SnapshotStore& store,
      IJointDebugPolicy& joint_debug_policy, bool mock,
      std::string web_root, std::string joint_teach_store);
  static void StartJointDebugImpl(JointDebugImpl& impl);
  static void StopJointDebugImpl(JointDebugImpl& impl);

  SnapshotStore& store_;
  std::unique_ptr<ILocomotion> locomotion_;
  const RobotProfile& robot_profile_;
  std::unique_ptr<IJointDebugPolicy> joint_debug_policy_;
  SafetyManager safety_;
  bool mock_{false};
  bool poll_locomotion_state_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> motion_active_{false};
  std::atomic<std::uint64_t> next_request_id_{1};
  std::atomic<std::uint64_t> next_motion_sequence_{1};
  std::thread worker_;
  std::thread motion_worker_;
  std::thread state_worker_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Request> queue_;
  bool command_running_{false};
  bool arm_interrupt_running_{false};
  std::unordered_map<std::string, std::uint64_t> recent_requests_;
  std::deque<std::string> recent_request_order_;

  std::mutex motion_mutex_;
  std::condition_variable motion_cv_;
  VelocityRequest pending_velocity_{};
  bool velocity_pending_{false};

  ResourceManager resources_;
  std::recursive_mutex joint_debug_request_mutex_;
  std::unique_ptr<JointDebugImpl> joint_debug_;
};

}  // namespace g1_web
