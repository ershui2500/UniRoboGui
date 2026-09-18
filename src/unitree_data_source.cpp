#include "g1_web/unitree_data_source.hpp"

#include <chrono>
#include <exception>

#include <unitree/robot/channel/channel_factory.hpp>

#include "g1_web/robot_registry.hpp"

namespace g1_web {

UnitreeDataSource::UnitreeDataSource(SnapshotStore& store) : store_(store) {}

UnitreeDataSource::~UnitreeDataSource() { Stop(); }

bool UnitreeDataSource::Start(const std::string& network_interface,
                             std::string& error,
                             const TelemetrySubscriptionPlan& plan) {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    for (const auto& source : plan.sources) {
      switch (source.kind) {
        case TelemetrySourceKind::kHgLowState:
          low_state_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::LowState_>>(source.topic);
          low_state_->InitChannel(
              [this](const void* data) {
                store_.UpdateLowState(
                    *static_cast<const unitree_hg::msg::dds_::LowState_*>(
                        data));
              },
              1);
          break;

        case TelemetrySourceKind::kHgBmsState:
          bms_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::BmsState_>>(source.topic);
          bms_->InitChannel(
              [this](const void* data) {
                store_.UpdateBms(
                    *static_cast<const unitree_hg::msg::dds_::BmsState_*>(
                        data));
              },
              1);
          break;

        case TelemetrySourceKind::kHgImuState:
          secondary_imu_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::IMUState_>>(source.topic);
          secondary_imu_->InitChannel(
              [this](const void* data) {
                store_.UpdateSecondaryImu(
                    *static_cast<const unitree_hg::msg::dds_::IMUState_*>(
                        data));
              },
              1);
          break;

        case TelemetrySourceKind::kHgMainBoardState:
          mainboard_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::MainBoardState_>>(source.topic);
          mainboard_->InitChannel(
              [this](const void* data) {
                store_.UpdateMainBoard(
                    *static_cast<
                        const unitree_hg::msg::dds_::MainBoardState_*>(data));
              },
              1);
          break;

        case TelemetrySourceKind::kGo2SportModeState:
          odometry_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_go::msg::dds_::SportModeState_>>(source.topic);
          odometry_->InitChannel(
              [this](const void* data) {
                store_.UpdateOdometry(
                    *static_cast<
                        const unitree_go::msg::dds_::SportModeState_*>(data));
              },
              1);
          break;

        case TelemetrySourceKind::kHgSportModeState:
          sport_mode_ = std::make_shared<unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::SportModeState_>>(source.topic);
          sport_mode_->InitChannel(
              [this](const void* data) {
                store_.UpdateSportMode(
                    *static_cast<
                        const unitree_hg::msg::dds_::SportModeState_*>(data));
              },
              1);
          break;
      }
    }

    store_.SetDdsStatus(true);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
  } catch (...) {
    error = "Unknown DDS initialization error";
  }

  Stop();
  store_.SetDdsStatus(false, error);
  return false;
}

void UnitreeDataSource::Stop() {
  if (sport_mode_) {
    sport_mode_->CloseChannel();
    sport_mode_.reset();
  }
  if (odometry_) {
    odometry_->CloseChannel();
    odometry_.reset();
  }
  if (mainboard_) {
    mainboard_->CloseChannel();
    mainboard_.reset();
  }
  if (secondary_imu_) {
    secondary_imu_->CloseChannel();
    secondary_imu_.reset();
  }
  if (bms_) {
    bms_->CloseChannel();
    bms_.reset();
  }
  if (low_state_) {
    low_state_->CloseChannel();
    low_state_.reset();
  }
}

MockDataSource::MockDataSource(SnapshotStore& store) : store_(store) {}

MockDataSource::MockDataSource(SnapshotStore& store, const RobotProfile& profile)
    : store_(store) {
  if (!profile.model_variants.empty()) {
    mode_machine_ =
        static_cast<std::uint8_t>(profile.model_variants.front().selector_value);
  }
  for (const auto& capability : profile.capabilities) {
    if (capability.key != CapabilityKey::kLocomotion) continue;
    const auto allowed_fsm = capability.parameters.find("allowed_fsm");
    if (allowed_fsm != capability.parameters.end()) {
      try {
        initial_fsm_id_ =
            static_cast<std::uint32_t>(std::stoul(allowed_fsm->second));
      } catch (...) {
      }
    }
    break;
  }
  semantic_motor_slots_.reserve(profile.joint_schema.joints.size());
  for (const auto& joint : profile.joint_schema.joints) {
    semantic_motor_slots_.push_back(joint.motor_slot);
  }
}

MockDataSource::~MockDataSource() { Stop(); }

void MockDataSource::Start() {
  if (running_.exchange(true)) {
    return;
  }
  unitree_hg::msg::dds_::SportModeState_ state;
  state.fsm_id(initial_fsm_id_);
  state.fsm_mode(0);
  state.task_id(0);
  state.task_time(0.0F);
  store_.UpdateSportMode(state);
  thread_ = std::thread([this] {
    const auto started = SteadyClock::now();
    while (running_.load()) {
      const double elapsed =
          std::chrono::duration<double>(SteadyClock::now() - started).count();
      store_.PopulateMock(elapsed, mode_machine_, semantic_motor_slots_);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
}

void MockDataSource::Stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

}  // namespace g1_web
