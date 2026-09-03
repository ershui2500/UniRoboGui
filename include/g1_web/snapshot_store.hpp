#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/MainBoardState_.hpp>
#include <unitree/idl/hg/SportModeState_.hpp>

#include "g1_web/joint_names.hpp"

namespace g1_web {

using SteadyClock = std::chrono::steady_clock;

enum class SourceId : std::size_t {
  kLowState = 0,
  kBms,
  kSecondaryImu,
  kMainBoard,
  kOdometry,
  kCount
};

enum class Freshness {
  kOnline,
  kDelayed,
  kOffline,
};

struct SourceState {
  bool received{false};
  SteadyClock::time_point last_update{};
};

struct ImuData {
  std::array<float, 4> quaternion{};
  std::array<float, 3> gyroscope{};
  std::array<float, 3> accelerometer{};
  std::array<float, 3> rpy{};
  std::int32_t temperature_raw{0};
};

struct MotorData {
  std::uint8_t mode{0};
  float q{0.0F};
  float dq{0.0F};
  float ddq{0.0F};
  float tau_est{0.0F};
  std::array<std::int16_t, 2> temperature{};
  float voltage{0.0F};
  std::array<std::uint32_t, 2> sensor_raw{};
  std::uint32_t state_raw{0};
};

struct BatteryData {
  std::uint8_t version_high{0};
  std::uint8_t version_low{0};
  std::uint8_t function_raw{0};
  std::array<std::uint16_t, 40> cell_voltage_raw{};
  std::array<std::uint32_t, 3> voltage_raw{};
  std::int32_t current_raw{0};
  std::uint8_t soc{0};
  std::uint8_t soh{0};
  std::array<std::int16_t, 12> temperature_raw{};
  std::uint16_t cycle{0};
  std::uint16_t manufacturer_date_raw{0};
  std::array<std::uint32_t, 5> state_raw{};
};

struct OdometryData {
  std::uint32_t error_code{0};
  std::uint8_t mode_raw{0};
  std::uint8_t gait_type_raw{0};
  float progress{0.0F};
  std::array<float, 3> position{};
  std::array<float, 3> velocity{};
  float body_height{0.0F};
  float yaw_speed{0.0F};
  ImuData imu{};
};

struct MainBoardData {
  std::array<std::uint16_t, 6> fan_state_raw{};
  std::array<std::int16_t, 6> temperature_raw{};
  std::array<float, 6> value_raw{};
  std::array<std::uint32_t, 6> state_raw{};
};

struct AsrData {
  bool received{false};
  std::uint64_t index{0};
  std::uint64_t timestamp_raw{0};
  std::string text;
  std::int32_t angle_raw{0};
  std::int32_t speaker_id_raw{0};
  std::string sense_raw;
  double confidence{0.0};
  std::string language;
  bool is_final{false};
  std::string raw_json;
  std::int64_t received_time_ms{0};
};

struct TtsData {
  std::uint64_t request_id{0};
  std::string state{"idle"};
  std::string text;
  std::int32_t speaker_id{0};
  std::string backend{"unitree"};
  std::string backend_error;
  std::int32_t api_result{0};
  std::int64_t updated_time_ms{0};
};

struct LlmData {
  std::string mode{"builtin"};
  bool builtin_api_available{false};
  bool builtin_response_subscribed{false};
  std::string builtin_request_topic{"rt/api/gpt/request"};
  std::string builtin_response_topic{"rt/api/gpt/response"};
  bool customer_api_available{false};
  bool customer_api_configured{false};
  bool customer_api_key_configured{false};
  std::string customer_api_url;
  std::string customer_model;
  std::string customer_role_prompt;
  std::string customer_wake_word;
  bool customer_wake_enabled{false};
  std::uint32_t customer_qa_count{0};
  std::string customer_tts_backend{"unitree"};
  std::string last_response_source;
  std::string request_state{"idle"};
  std::int64_t request_id{0};
  std::int32_t response_status_code{0};
  std::string last_user_message;
  std::string last_response;
  std::string last_error;
  std::int64_t updated_time_ms{0};
};

struct VoiceData {
  bool initialized{false};
  std::string initialization_error;
  bool asr_subscribed{false};
  std::int32_t asr_control_api_result{0};
  std::string asr_control_error;
  bool chat_go_found{false};
  std::string chat_go_service_name;
  bool chat_go_closed{false};
  std::int32_t chat_go_api_result{0};
  std::int32_t chat_go_status_raw{-1};
  bool vui_service_found{false};
  std::int32_t vui_service_status_raw{-1};
  std::int32_t volume_api_result{0};
  std::int32_t volume_pct{-1};
  std::int32_t play_state_raw{-1};
  AsrData asr{};
  TtsData tts{};
  LlmData llm{};
};

struct ControlCommandData {
  std::uint64_t request_id{0};
  std::string request_key;
  std::string category;
  std::string command;
  std::int32_t argument{0};
  std::string action_name;
  std::string state{"idle"};
  std::int32_t api_result{0};
  std::string error;
  std::int64_t accepted_time_ms{0};
  std::int64_t completed_time_ms{0};
};

struct MotionData {
  bool active{false};
  float vx_m_s{0.0F};
  float vy_m_s{0.0F};
  float vyaw_rad_s{0.0F};
  std::int32_t speed_mode{0};
  std::uint64_t sequence{0};
  std::string state{"stopped"};
  std::int32_t api_result{0};
  std::string error;
  std::int64_t updated_time_ms{0};
};

struct ControlData {
  bool initialized{false};
  bool enabled{false};
  bool mock{false};
  std::string initialization_error;
  bool sport_state_received{false};
  SteadyClock::time_point sport_state_last_update{};
  std::uint32_t fsm_id{0};
  std::uint32_t fsm_mode{0};
  std::uint32_t task_id{0};
  float task_time_s{0.0F};
  std::int32_t action_list_api_result{-1};
  std::string action_list_raw;
  ControlCommandData last_command{};
  MotionData motion{};
};

struct RobotSnapshot {
  std::uint64_t sequence{0};
  bool dds_initialized{false};
  std::string dds_error;
  std::array<SourceState, static_cast<std::size_t>(SourceId::kCount)> sources{};

  std::array<std::uint32_t, 2> version{};
  std::uint8_t mode_pr{0};
  std::uint8_t mode_machine{0};
  std::uint32_t tick{0};
  ImuData hip_imu{};
  ImuData torso_imu{};
  std::array<MotorData, kMotorSlotCount> motors{};
  BatteryData battery{};
  OdometryData odometry{};
  MainBoardData mainboard{};
  VoiceData voice{};
  ControlData control{};
};

Freshness ClassifyFreshness(bool received, std::int64_t age_ms);
const char* FreshnessName(Freshness freshness);
const char* SourceName(SourceId source);

class SnapshotStore {
 public:
  void SetDdsStatus(bool initialized, const std::string& error = {});
  void UpdateLowState(const unitree_hg::msg::dds_::LowState_& message);
  void UpdateBms(const unitree_hg::msg::dds_::BmsState_& message);
  void UpdateSecondaryImu(const unitree_hg::msg::dds_::IMUState_& message);
  void UpdateMainBoard(const unitree_hg::msg::dds_::MainBoardState_& message);
  void UpdateOdometry(const unitree_go::msg::dds_::SportModeState_& message);
  void SetVoiceState(const VoiceData& voice);
  void UpdateAsrSubscription(bool subscribed, std::int32_t api_result,
                             const std::string& error = {});
  void UpdateVolume(std::int32_t api_result, std::int32_t volume_pct);
  void UpdateAsr(const AsrData& asr);
  void UpdatePlayState(std::int32_t play_state);
  void UpdateTts(const TtsData& tts);
  void UpdateLlmState(bool chat_go_found,
                      const std::string& chat_go_service_name,
                      bool chat_go_closed,
                      std::int32_t chat_go_api_result,
                      std::int32_t chat_go_status_raw,
                      const LlmData& llm);
  void SetControlState(const ControlData& control);
  void UpdateSportMode(
      const unitree_hg::msg::dds_::SportModeState_& message);
  void UpdateControlCommand(const ControlCommandData& command);
  void UpdateMotion(const MotionData& motion);
  void PopulateMock(double elapsed_seconds);

  RobotSnapshot GetSnapshot() const;

 private:
  void TouchLocked(SourceId source);

  mutable std::mutex mutex_;
  RobotSnapshot snapshot_;
};

}  // namespace g1_web
