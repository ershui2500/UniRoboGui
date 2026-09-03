#include "g1_web/snapshot_store.hpp"

#include <algorithm>
#include <cmath>

namespace g1_web {
namespace {

std::int64_t SystemTimeMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

ImuData ToImu(const unitree_hg::msg::dds_::IMUState_& message) {
  ImuData result;
  result.quaternion = message.quaternion();
  result.gyroscope = message.gyroscope();
  result.accelerometer = message.accelerometer();
  result.rpy = message.rpy();
  result.temperature_raw = message.temperature();
  return result;
}

ImuData ToImu(const unitree_go::msg::dds_::IMUState_& message) {
  ImuData result;
  result.quaternion = message.quaternion();
  result.gyroscope = message.gyroscope();
  result.accelerometer = message.accelerometer();
  result.rpy = message.rpy();
  result.temperature_raw = message.temperature();
  return result;
}

}  // namespace

Freshness ClassifyFreshness(bool received, std::int64_t age_ms) {
  if (!received || age_ms > 3000) {
    return Freshness::kOffline;
  }
  if (age_ms > 1000) {
    return Freshness::kDelayed;
  }
  return Freshness::kOnline;
}

const char* FreshnessName(Freshness freshness) {
  switch (freshness) {
    case Freshness::kOnline:
      return "online";
    case Freshness::kDelayed:
      return "delayed";
    case Freshness::kOffline:
      return "offline";
  }
  return "offline";
}

const char* SourceName(SourceId source) {
  switch (source) {
    case SourceId::kLowState:
      return "low_state";
    case SourceId::kBms:
      return "bms";
    case SourceId::kSecondaryImu:
      return "secondary_imu";
    case SourceId::kMainBoard:
      return "mainboard";
    case SourceId::kOdometry:
      return "odometry";
    case SourceId::kCount:
      break;
  }
  return "unknown";
}

void SnapshotStore::SetDdsStatus(bool initialized, const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.dds_initialized = initialized;
  snapshot_.dds_error = error;
  ++snapshot_.sequence;
}

void SnapshotStore::TouchLocked(SourceId source) {
  auto& state = snapshot_.sources[static_cast<std::size_t>(source)];
  state.received = true;
  state.last_update = SteadyClock::now();
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateLowState(
    const unitree_hg::msg::dds_::LowState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.version = message.version();
  snapshot_.mode_pr = message.mode_pr();
  snapshot_.mode_machine = message.mode_machine();
  snapshot_.tick = message.tick();
  snapshot_.hip_imu = ToImu(message.imu_state());

  const auto& states = message.motor_state();
  for (std::size_t i = 0; i < states.size(); ++i) {
    const auto& source = states[i];
    auto& target = snapshot_.motors[i];
    target.mode = source.mode();
    target.q = source.q();
    target.dq = source.dq();
    target.ddq = source.ddq();
    target.tau_est = source.tau_est();
    target.temperature = source.temperature();
    target.voltage = source.vol();
    target.sensor_raw = source.sensor();
    target.state_raw = source.motorstate();
  }
  TouchLocked(SourceId::kLowState);
}

void SnapshotStore::UpdateBms(
    const unitree_hg::msg::dds_::BmsState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& battery = snapshot_.battery;
  battery.version_high = message.version_high();
  battery.version_low = message.version_low();
  battery.function_raw = message.fn();
  battery.cell_voltage_raw = message.cell_vol();
  battery.voltage_raw = message.bmsvoltage();
  battery.current_raw = message.current();
  battery.soc = message.soc();
  battery.soh = message.soh();
  battery.temperature_raw = message.temperature();
  battery.cycle = message.cycle();
  battery.manufacturer_date_raw = message.manufacturer_date();
  battery.state_raw = message.bmsstate();
  TouchLocked(SourceId::kBms);
}

void SnapshotStore::UpdateSecondaryImu(
    const unitree_hg::msg::dds_::IMUState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.torso_imu = ToImu(message);
  TouchLocked(SourceId::kSecondaryImu);
}

void SnapshotStore::UpdateMainBoard(
    const unitree_hg::msg::dds_::MainBoardState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.mainboard.fan_state_raw = message.fan_state();
  snapshot_.mainboard.temperature_raw = message.temperature();
  snapshot_.mainboard.value_raw = message.value();
  snapshot_.mainboard.state_raw = message.state();
  TouchLocked(SourceId::kMainBoard);
}

void SnapshotStore::UpdateOdometry(
    const unitree_go::msg::dds_::SportModeState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& odometry = snapshot_.odometry;
  odometry.error_code = message.error_code();
  odometry.mode_raw = message.mode();
  odometry.gait_type_raw = message.gait_type();
  odometry.progress = message.progress();
  odometry.position = message.position();
  odometry.velocity = message.velocity();
  odometry.body_height = message.body_height();
  odometry.yaw_speed = message.yaw_speed();
  odometry.imu = ToImu(message.imu_state());
  TouchLocked(SourceId::kOdometry);
}

void SnapshotStore::SetVoiceState(const VoiceData& voice) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice = voice;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateAsrSubscription(bool subscribed,
                                          std::int32_t api_result,
                                          const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.asr_subscribed = subscribed;
  snapshot_.voice.asr_control_api_result = api_result;
  snapshot_.voice.asr_control_error = error;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateVolume(std::int32_t api_result,
                                 std::int32_t volume_pct) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.volume_api_result = api_result;
  if (api_result == 0) {
    snapshot_.voice.volume_pct = volume_pct;
  }
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateAsr(const AsrData& asr) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.asr = asr;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdatePlayState(std::int32_t play_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.play_state_raw = play_state;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateTts(const TtsData& tts) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.tts = tts;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateLlmState(
    bool chat_go_found, const std::string& chat_go_service_name,
    bool chat_go_closed, std::int32_t chat_go_api_result,
    std::int32_t chat_go_status_raw, const LlmData& llm) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.voice.chat_go_found = chat_go_found;
  snapshot_.voice.chat_go_service_name = chat_go_service_name;
  snapshot_.voice.chat_go_closed = chat_go_closed;
  snapshot_.voice.chat_go_api_result = chat_go_api_result;
  snapshot_.voice.chat_go_status_raw = chat_go_status_raw;
  snapshot_.voice.llm = llm;
  ++snapshot_.sequence;
}

void SnapshotStore::SetControlState(const ControlData& control) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control = control;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateSportMode(
    const unitree_hg::msg::dds_::SportModeState_& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control.sport_state_received = true;
  snapshot_.control.sport_state_last_update = SteadyClock::now();
  snapshot_.control.fsm_id = message.fsm_id();
  snapshot_.control.fsm_mode = message.fsm_mode();
  snapshot_.control.task_id = message.task_id();
  snapshot_.control.task_time_s = message.task_time();
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateControlCommand(
    const ControlCommandData& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control.last_command = command;
  ++snapshot_.sequence;
}

void SnapshotStore::UpdateMotion(const MotionData& motion) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control.motion = motion;
  ++snapshot_.sequence;
}

void SnapshotStore::PopulateMock(double elapsed_seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.dds_initialized = true;
  snapshot_.dds_error.clear();
  snapshot_.version = {1, 0};
  snapshot_.mode_pr = 0;
  snapshot_.mode_machine = 2;
  snapshot_.tick = static_cast<std::uint32_t>(elapsed_seconds * 500.0);

  const float wave = static_cast<float>(std::sin(elapsed_seconds));
  snapshot_.hip_imu.quaternion = {1.0F, 0.0F, wave * 0.03F, 0.0F};
  snapshot_.hip_imu.rpy = {wave * 0.04F, wave * 0.02F, wave * 0.08F};
  snapshot_.hip_imu.gyroscope = {0.01F, 0.02F, wave * 0.08F};
  snapshot_.hip_imu.accelerometer = {0.0F, 0.0F, 9.81F};
  snapshot_.hip_imu.temperature_raw = 38;
  snapshot_.torso_imu = snapshot_.hip_imu;
  snapshot_.torso_imu.rpy[0] *= -0.5F;
  snapshot_.torso_imu.temperature_raw = 40;

  for (std::size_t i = 0; i < snapshot_.motors.size(); ++i) {
    auto& motor = snapshot_.motors[i];
    motor.mode = i < kNamedJointCount ? 1 : 0;
    motor.q = wave * 0.2F + static_cast<float>(i) * 0.002F;
    motor.dq = std::cos(elapsed_seconds) * 0.1F;
    motor.ddq = -wave * 0.1F;
    motor.tau_est = wave * 2.0F;
    motor.temperature = {42, 45};
    motor.voltage = 48.2F;
    motor.sensor_raw = {0, 0};
    motor.state_raw = 0;
  }

  auto& battery = snapshot_.battery;
  battery.version_high = 1;
  battery.version_low = 2;
  battery.soc = 82;
  battery.soh = 96;
  battery.current_raw = -1380;
  battery.cycle = 37;
  battery.manufacturer_date_raw = 24872;
  battery.voltage_raw = {76800, 38400, 38400};
  battery.temperature_raw.fill(31);
  for (std::size_t i = 0; i < battery.cell_voltage_raw.size(); ++i) {
    battery.cell_voltage_raw[i] =
        i < 20 ? static_cast<std::uint16_t>(3835 + (i % 4)) : 0;
  }

  snapshot_.odometry.error_code = 0;
  snapshot_.odometry.mode_raw = 1;
  snapshot_.odometry.gait_type_raw = 0;
  snapshot_.odometry.progress = 1.0F;
  snapshot_.odometry.position = {wave * 0.1F, 0.0F, 0.78F};
  snapshot_.odometry.velocity = {
      static_cast<float>(std::cos(elapsed_seconds) * 0.1), 0.0F, 0.0F};
  snapshot_.odometry.body_height = 0.78F;
  snapshot_.odometry.yaw_speed = wave * 0.05F;
  snapshot_.odometry.imu = snapshot_.hip_imu;

  snapshot_.mainboard.fan_state_raw = {1200, 1180, 0, 0, 0, 0};
  snapshot_.mainboard.temperature_raw = {46, 44, 39, 0, 0, 0};
  snapshot_.mainboard.value_raw = {12.1F, 5.0F, 3.3F, 0.0F, 0.0F, 0.0F};
  snapshot_.mainboard.state_raw = {0, 0, 0, 0, 0, 0};

  const bool initialize_mock_voice = !snapshot_.voice.initialized;
  if (initialize_mock_voice) {
    snapshot_.voice.initialized = true;
    snapshot_.voice.initialization_error.clear();
    snapshot_.voice.asr_subscribed = true;
    snapshot_.voice.chat_go_found = true;
    snapshot_.voice.chat_go_service_name = "chat_go";
    snapshot_.voice.chat_go_closed = false;
    snapshot_.voice.chat_go_api_result = 0;
    snapshot_.voice.chat_go_status_raw = 0;
    snapshot_.voice.llm.mode = "builtin";
    snapshot_.voice.llm.builtin_api_available = true;
    snapshot_.voice.llm.builtin_response_subscribed = true;
    snapshot_.voice.llm.customer_api_available = true;
    snapshot_.voice.vui_service_found = true;
    snapshot_.voice.vui_service_status_raw = 0;
    snapshot_.voice.volume_api_result = 0;
    snapshot_.voice.volume_pct = 82;
  }
  if (!snapshot_.voice.asr.received &&
      elapsed_seconds > 1.0) {
    snapshot_.voice.asr.received = true;
    snapshot_.voice.asr.index = 1;
    snapshot_.voice.asr.timestamp_raw = 29319303490ULL;
    snapshot_.voice.asr.text = "你好，我是 G1";
    snapshot_.voice.asr.angle_raw = 90;
    snapshot_.voice.asr.speaker_id_raw = 0;
    snapshot_.voice.asr.sense_raw = "unknown";
    snapshot_.voice.asr.confidence = 0.95;
    snapshot_.voice.asr.language = "zh-CN";
    snapshot_.voice.asr.is_final = true;
    snapshot_.voice.asr.raw_json =
        R"({"index":1,"text":"你好，我是 G1","confidence":0.95})";
    snapshot_.voice.asr.received_time_ms = SystemTimeMs();
  }

  const bool initialize_mock_control =
      !snapshot_.control.sport_state_received;
  snapshot_.control.sport_state_received = true;
  snapshot_.control.sport_state_last_update = SteadyClock::now();
  if (initialize_mock_control) {
    snapshot_.control.fsm_id = 500;
    snapshot_.control.fsm_mode = 0;
    snapshot_.control.task_id = 0;
    snapshot_.control.task_time_s = 0.0F;
  }

  const auto now = SteadyClock::now();
  for (auto& source : snapshot_.sources) {
    source.received = true;
    source.last_update = now;
  }
  ++snapshot_.sequence;
}

RobotSnapshot SnapshotStore::GetSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

}  // namespace g1_web
