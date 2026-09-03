#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/internal/internal_request_response.hpp>

#include "g1_web/snapshot_store.hpp"

namespace g1_web {

struct TtsEnqueueResult {
  bool accepted{false};
  std::uint64_t request_id{0};
  std::int32_t speaker_id{-1};
  std::string backend;
  std::string error;
};

struct VoiceActionResult {
  bool accepted{false};
  std::int32_t api_result{0};
  std::int32_t value{-1};
  std::string error;
};

struct LlmModeResult {
  bool accepted{false};
  std::string mode;
  std::int32_t api_result{0};
  std::string error;
};

struct LlmChatResult {
  bool accepted{false};
  bool pending{false};
  std::int64_t request_id{0};
  long http_status{0};
  std::uint64_t tts_request_id{0};
  std::string mode;
  std::string response;
  std::string response_source;
  std::string error;
  std::string tts_error;
};

struct CustomerQaEntry {
  std::string question;
  std::string answer;
};

struct CustomerVoiceConfig {
  std::string api_url;
  std::string api_key;
  std::string model;
  bool api_key_configured{false};
  bool preserve_api_key{false};
  bool update_api_config{false};
  std::string config_path;
  std::string role_prompt;
  std::string wake_word;
  bool wake_enabled{false};
  std::string tts_backend{"unitree"};
  std::vector<CustomerQaEntry> qa_entries;
};

struct CustomerVoiceConfigResult {
  bool accepted{false};
  CustomerVoiceConfig config;
  std::string error;
};

class VoiceService {
  friend struct VoiceServiceTestAccess;

 public:
  VoiceService(SnapshotStore& store, bool mock);
  ~VoiceService();

  bool Start(std::string& error);
  void Stop();
  TtsEnqueueResult EnqueueTts(const std::string& text,
                              std::int32_t speaker_id,
                              const std::string& backend = "");
  VoiceActionResult SetAsrEnabled(bool enabled);
  VoiceActionResult SetVolume(std::int32_t volume_pct);
  LlmModeResult SetLlmMode(const std::string& mode,
                           const std::string& api_url,
                           const std::string& api_key,
                           const std::string& model,
                           bool preserve_api_key = false);
  LlmChatResult ChatWithLlm(const std::string& mode,
                            const std::string& message,
                            bool auto_tts = false);
  CustomerVoiceConfigResult GetCustomerVoiceConfig();
  CustomerVoiceConfigResult SetCustomerVoiceConfig(
      const CustomerVoiceConfig& config);

 private:
  struct TtsRequest {
    std::uint64_t request_id{0};
    std::string text;
    std::int32_t speaker_id{0};
    std::string backend;
  };

  void HandleAudioMessage(const std::string& message);
  void WorkerLoop();
  VoiceData InitializeRealVoice(std::string& error);
  bool StartAsrSubscription(std::string& error);
  void StopAsrSubscription();
  bool StartBuiltinLlmChannels(std::string& error);
  void StopBuiltinLlmChannels();
  LlmChatResult ChatWithBuiltinLlm(const std::string& message);
  LlmChatResult ChatWithCustomerLlm(const std::string& message,
                                     bool auto_tts);
  void HandleBuiltinLlmResponse(
      const unitree::robot::Response& response);
  void CheckBuiltinLlmTimeout();
  void QueueCustomerWakeRequest(const AsrData& asr);
  std::string FindCustomerQaAnswer(const std::string& question) const;
  bool LoadCustomerVoiceConfig(std::string& error);
  bool SaveCustomerVoiceConfig(std::string& error) const;
  bool PlayWithLocalTts(const std::string& text, std::string& error);
  static std::string NormalizeQuestionForMatch(const std::string& text);
  static bool StripCustomerWakePrefix(const std::string& text,
                                      const std::string& wake_word,
                                      std::string& message);
  static bool ResolveSpeakerId(const std::string& text,
                               std::int32_t requested_speaker_id,
                               std::int32_t& resolved_speaker_id,
                               std::string& error);
  bool SwitchChatGo(bool enabled, std::int32_t& api_result,
                    std::int32_t& status_raw, std::string& error);
  void PublishLlmState(const LlmData& llm, std::int32_t api_result,
                       std::int32_t status_raw);
  static bool IsValidApiUrl(const std::string& url);
  static std::string NormalizeCustomerApiUrl(const std::string& url);
  static std::string ExtractLlmError(const std::string& body);
  static std::string ExtractLlmResponse(const std::string& body);
  static std::string ExtractBuiltinLlmResponse(const std::string& body);
  static std::string Trim(const std::string& text);
  static std::string NormalizeServiceName(const std::string& name);

  SnapshotStore& store_;
  bool mock_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> asr_enabled_{true};
  std::atomic<std::uint64_t> next_request_id_{1};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<TtsRequest> queue_;
  std::deque<std::string> customer_wake_queue_;
  bool tts_in_flight_{false};
  std::string last_tts_text_;
  std::int32_t last_tts_speaker_id_{-1};
  std::string last_tts_backend_;
  std::chrono::steady_clock::time_point last_tts_accepted_{};

  std::mutex play_state_mutex_;
  std::condition_variable play_state_cv_;
  std::uint64_t play_event_sequence_{0};
  std::uint64_t last_play_start_event_{0};
  std::uint64_t last_play_stop_event_{0};
  std::int32_t observed_play_state_{-1};

  std::mutex voice_mutex_;
  std::mutex llm_mutex_;
  std::string chat_go_service_name_{"chat_go"};
  bool chat_go_found_{false};
  bool chat_go_closed_{false};
  std::string llm_mode_{"builtin"};
  std::string customer_api_url_;
  std::string customer_api_key_;
  std::string customer_model_;
  std::string customer_role_prompt_;
  std::string customer_wake_word_;
  bool customer_wake_enabled_{false};
  std::string customer_tts_backend_{"kokoro"};
  std::vector<CustomerQaEntry> customer_qa_entries_;
  std::uint64_t last_customer_wake_asr_index_{0};
  std::chrono::steady_clock::time_point customer_wake_until_{};
  std::string customer_config_path_{"config/customer_voice.json"};
  std::int64_t pending_builtin_request_id_{0};

  std::unique_ptr<unitree::robot::g1::AudioClient> audio_client_;
  std::unique_ptr<unitree::robot::b2::RobotStateClient>
      robot_state_client_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      std_msgs::msg::dds_::String_>>
      audio_subscriber_;
  std::shared_ptr<unitree::robot::ChannelPublisher<
      unitree::robot::Request>>
      builtin_llm_publisher_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree::robot::Response>>
      builtin_llm_subscriber_;
};

}  // namespace g1_web
