#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_subscriber.hpp>

#include "g1_web/robot_profile.hpp"
#include "g1_web/snapshot_store.hpp"

namespace g1_web {

struct TelemetrySubscriptionPlan;

class UnitreeDataSource {
 public:
  explicit UnitreeDataSource(SnapshotStore& store);
  ~UnitreeDataSource();

  bool Start(const std::string& network_interface, std::string& error,
             const TelemetrySubscriptionPlan& plan);
  void Stop();

 private:
  SnapshotStore& store_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_hg::msg::dds_::LowState_>>
      low_state_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_hg::msg::dds_::BmsState_>>
      bms_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_hg::msg::dds_::IMUState_>>
      secondary_imu_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_hg::msg::dds_::MainBoardState_>>
      mainboard_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::SportModeState_>>
      odometry_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_hg::msg::dds_::SportModeState_>>
      sport_mode_;
};

class MockDataSource {
 public:
  explicit MockDataSource(SnapshotStore& store);
  MockDataSource(SnapshotStore& store, const RobotProfile& profile);
  ~MockDataSource();

  void Start();
  void Stop();

 private:
  SnapshotStore& store_;
  std::uint8_t mode_machine_{2};
  std::uint32_t initial_fsm_id_{500};
  std::vector<std::size_t> semantic_motor_slots_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace g1_web
