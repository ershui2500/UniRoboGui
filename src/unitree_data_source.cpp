#include "g1_web/unitree_data_source.hpp"

#include <chrono>
#include <exception>

#include <unitree/robot/channel/channel_factory.hpp>

namespace g1_web {

UnitreeDataSource::UnitreeDataSource(SnapshotStore& store) : store_(store) {}

UnitreeDataSource::~UnitreeDataSource() { Stop(); }

bool UnitreeDataSource::Start(const std::string& network_interface,
                             std::string& error) {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    low_state_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_hg::msg::dds_::LowState_>>("rt/lf/lowstate");
    low_state_->InitChannel(
        [this](const void* data) {
          store_.UpdateLowState(
              *static_cast<const unitree_hg::msg::dds_::LowState_*>(data));
        },
        1);

    bms_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_hg::msg::dds_::BmsState_>>("rt/lf/bmsstate");
    bms_->InitChannel(
        [this](const void* data) {
          store_.UpdateBms(
              *static_cast<const unitree_hg::msg::dds_::BmsState_*>(data));
        },
        1);

    secondary_imu_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_hg::msg::dds_::IMUState_>>("rt/lf/secondary_imu");
    secondary_imu_->InitChannel(
        [this](const void* data) {
          store_.UpdateSecondaryImu(
              *static_cast<const unitree_hg::msg::dds_::IMUState_*>(data));
        },
        1);

    mainboard_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_hg::msg::dds_::MainBoardState_>>("rt/lf/mainboardstate");
    mainboard_->InitChannel(
        [this](const void* data) {
          store_.UpdateMainBoard(
              *static_cast<const unitree_hg::msg::dds_::MainBoardState_*>(
                  data));
        },
        1);

    odometry_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>>("rt/odommodestate");
    odometry_->InitChannel(
        [this](const void* data) {
          store_.UpdateOdometry(
              *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
                  data));
        },
        1);

    sport_mode_ = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree_hg::msg::dds_::SportModeState_>>("rt/sportmodestate");
    sport_mode_->InitChannel(
        [this](const void* data) {
          store_.UpdateSportMode(
              *static_cast<
                  const unitree_hg::msg::dds_::SportModeState_*>(data));
        },
        1);

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

MockDataSource::~MockDataSource() { Stop(); }

void MockDataSource::Start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread([this] {
    const auto started = SteadyClock::now();
    while (running_.load()) {
      const double elapsed =
          std::chrono::duration<double>(SteadyClock::now() - started).count();
      store_.PopulateMock(elapsed);
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
