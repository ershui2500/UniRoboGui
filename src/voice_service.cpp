#include "g1_web/voice_service.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <json/json.h>

#ifdef G1_WEB_HAS_CURL
#include <curl/curl.h>
#endif

namespace g1_web {
namespace {

constexpr const char* kBuiltinGptRequestTopic = "rt/api/gpt/request";
constexpr const char* kBuiltinGptResponseTopic = "rt/api/gpt/response";
constexpr const char* kSavedApiKeyMask = "••••••••••••";
constexpr std::int64_t kBuiltinGptApiId = 1001;
constexpr auto kCustomerWakeFollowupTimeout = std::chrono::seconds(15);
constexpr std::int64_t kCustomerWakeEchoGuardMs = 3000;

std::int64_t SystemTimeMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::int64_t SteadyTimeNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool DecodeUtf8CodePoint(const std::string& text, std::size_t& offset,
                         std::uint32_t& code_point) {
  const auto first = static_cast<unsigned char>(text[offset]);
  std::size_t length = 0;
  if (first < 0x80) {
    code_point = first;
    ++offset;
    return true;
  }
  if ((first & 0xE0) == 0xC0) {
    code_point = first & 0x1F;
    length = 2;
  } else if ((first & 0xF0) == 0xE0) {
    code_point = first & 0x0F;
    length = 3;
  } else if ((first & 0xF8) == 0xF0) {
    code_point = first & 0x07;
    length = 4;
  } else {
    return false;
  }
  if (offset + length > text.size()) return false;
  for (std::size_t index = 1; index < length; ++index) {
    const auto value = static_cast<unsigned char>(text[offset + index]);
    if ((value & 0xC0) != 0x80) return false;
    code_point = (code_point << 6) | (value & 0x3F);
  }
  offset += length;
  return true;
}

bool IsHanCodePoint(std::uint32_t value) {
  return (value >= 0x3400 && value <= 0x4DBF) ||
         (value >= 0x4E00 && value <= 0x9FFF) ||
         (value >= 0xF900 && value <= 0xFAFF);
}

enum class TtsLanguage { kNeutral, kChinese, kEnglish };

struct TtsSegment {
  std::string text;
  std::int32_t speaker_id{0};
};

TtsLanguage ClassifyTtsCodePoint(std::uint32_t code_point) {
  if (IsHanCodePoint(code_point)) return TtsLanguage::kChinese;
  if (code_point < 0x80 &&
      std::isalpha(static_cast<unsigned char>(code_point)) != 0) {
    return TtsLanguage::kEnglish;
  }
  return TtsLanguage::kNeutral;
}

std::string NormalizeNeutralForSpeaker(const std::string& text,
                                       std::int32_t speaker_id) {
  if (speaker_id != 1) return text;

  std::string normalized;
  for (std::size_t offset = 0; offset < text.size();) {
    const std::size_t token_begin = offset;
    std::uint32_t code_point = 0;
    if (!DecodeUtf8CodePoint(text, offset, code_point)) return text;
    switch (code_point) {
      case 0xFF0C: normalized += ","; break;   // ，
      case 0x3002: normalized += "."; break;   // 。
      case 0xFF01: normalized += "!"; break;   // ！
      case 0xFF1F: normalized += "?"; break;   // ？
      case 0xFF1A: normalized += ":"; break;   // ：
      case 0xFF1B: normalized += ";"; break;   // ；
      case 0x3001: normalized += ","; break;   // 、
      default:
        normalized.append(text, token_begin, offset - token_begin);
        break;
    }
  }
  return normalized;
}

bool IsTokenNeutral(std::uint32_t code_point) {
  return (code_point < 0x80 &&
          std::isdigit(static_cast<unsigned char>(code_point)) != 0) ||
         code_point == '-' || code_point == '_' || code_point == '+' ||
         code_point == '/' || code_point == '#' || code_point == '@' ||
         code_point == '%' || code_point == '&';
}

void AppendSwitchBoundary(TtsSegment& segment,
                          const std::string& pending_neutral) {
  std::string token_suffix;
  bool strong_stop = false;
  bool question_stop = false;
  bool exclamation_stop = false;
  for (std::size_t offset = 0; offset < pending_neutral.size();) {
    const std::size_t token_begin = offset;
    std::uint32_t code_point = 0;
    if (!DecodeUtf8CodePoint(pending_neutral, offset, code_point)) break;
    if (IsTokenNeutral(code_point)) {
      token_suffix.append(pending_neutral, token_begin, offset - token_begin);
    } else if (code_point == '.') {
      const bool decimal_point =
          token_begin > 0 && offset < pending_neutral.size() &&
          std::isdigit(static_cast<unsigned char>(pending_neutral[token_begin - 1])) != 0 &&
          std::isdigit(static_cast<unsigned char>(pending_neutral[offset])) != 0;
      if (decimal_point) token_suffix += ".";
      else strong_stop = true;
    }
    if (code_point == 0x3002) strong_stop = true;
    if (code_point == '?' || code_point == 0xFF1F) question_stop = true;
    if (code_point == '!' || code_point == 0xFF01) exclamation_stop = true;
  }
  segment.text += NormalizeNeutralForSpeaker(token_suffix, segment.speaker_id);

  if (segment.speaker_id == 1) {
    if (question_stop) segment.text += "?";
    else if (exclamation_stop) segment.text += "!";
    else if (strong_stop) segment.text += ".";
    else segment.text += ",";
  } else {
    if (question_stop) segment.text += "？";
    else if (exclamation_stop) segment.text += "！";
    else if (strong_stop) segment.text += "。";
    else segment.text += "，";
  }
}

std::vector<TtsSegment> SplitMixedTtsText(const std::string& text) {
  std::vector<TtsSegment> segments;
  std::string pending_neutral;
  for (std::size_t offset = 0; offset < text.size();) {
    const std::size_t token_begin = offset;
    std::uint32_t code_point = 0;
    if (!DecodeUtf8CodePoint(text, offset, code_point)) return {};
    const std::string token = text.substr(token_begin, offset - token_begin);
    const TtsLanguage language = ClassifyTtsCodePoint(code_point);
    if (language == TtsLanguage::kNeutral) {
      pending_neutral += token;
      continue;
    }

    const std::int32_t speaker_id =
        language == TtsLanguage::kEnglish ? 1 : 0;
    if (segments.empty()) {
      TtsSegment segment;
      segment.speaker_id = speaker_id;
      segment.text = NormalizeNeutralForSpeaker(pending_neutral, speaker_id);
      segment.text += token;
      pending_neutral.clear();
      segments.push_back(std::move(segment));
      continue;
    }

    if (segments.back().speaker_id == speaker_id) {
      segments.back().text +=
          NormalizeNeutralForSpeaker(pending_neutral, speaker_id);
      segments.back().text += token;
      pending_neutral.clear();
      continue;
    }

    AppendSwitchBoundary(segments.back(), pending_neutral);
    pending_neutral.clear();
    segments.push_back({token, speaker_id});
  }

  if (!pending_neutral.empty()) {
    if (segments.empty()) {
      segments.push_back({pending_neutral, 0});
    } else {
      segments.back().text +=
          NormalizeNeutralForSpeaker(pending_neutral,
                                     segments.back().speaker_id);
    }
  }
  return segments;
}

std::chrono::milliseconds EstimateTtsPlaybackDuration(const std::string& text) {
  std::size_t code_points = 0;
  for (std::size_t offset = 0; offset < text.size();) {
    std::uint32_t code_point = 0;
    if (!DecodeUtf8CodePoint(text, offset, code_point)) break;
    ++code_points;
  }
  const auto estimate = std::chrono::milliseconds(
      600 + static_cast<std::int64_t>(code_points) * 180);
  return std::min(std::chrono::milliseconds(12000),
                  std::max(std::chrono::milliseconds(900), estimate));
}

#ifdef G1_WEB_HAS_CURL
constexpr std::size_t kMaximumLlmResponseBytes = 1024 * 1024;
constexpr std::size_t kMaximumLocalTtsPcmBytes = 8 * 1024 * 1024;
constexpr std::size_t kUnitreeAudioChunkBytes = 96000;  // SDK example maximum: 3 s PCM
constexpr std::size_t kLocalTtsPrebufferBytes = 8000;   // 250 ms @ 16 kHz mono s16le
constexpr std::size_t kLocalTtsMinimumStreamBytes = 40000;  // 1.25 s incl. tail silence
constexpr auto kLocalTtsSendPace = std::chrono::milliseconds(100);
static_assert((kLocalTtsMinimumStreamBytes % 2) == 0,
              "local TTS minimum stream must contain whole s16le samples");

std::size_t CurlWriteCallback(char* data, std::size_t size,
                              std::size_t count, void* user_data) {
  const std::size_t bytes = size * count;
  auto* output = static_cast<std::string*>(user_data);
  if (!output || output->size() + bytes > kMaximumLlmResponseBytes) {
    return 0;
  }
  output->append(data, bytes);
  return bytes;
}

struct LocalTtsStreamContext {
  unitree::robot::g1::AudioClient* audio_client{nullptr};
  std::string app_name{"unirobo_kokoro"};
  std::string stream_id;
  std::string pending_pcm;
  std::size_t total_pcm_bytes{0};
  bool playback_started{false};
  std::chrono::steady_clock::time_point next_send_time{};
  std::int32_t api_result{0};
  std::string error;
};

bool FlushLocalTtsPcm(LocalTtsStreamContext& context, bool force) {
  if (!context.audio_client || context.api_result != 0) return false;
  if (!context.playback_started) {
    if (!force && context.pending_pcm.size() < kLocalTtsPrebufferBytes) {
      return true;
    }
    context.playback_started = true;
  }

  while (!context.pending_pcm.empty() && context.playback_started) {
    const std::size_t chunk_size =
        std::min(kUnitreeAudioChunkBytes, context.pending_pcm.size());
    if (context.next_send_time.time_since_epoch().count() != 0) {
      const auto now = std::chrono::steady_clock::now();
      if (now < context.next_send_time) {
        std::this_thread::sleep_until(context.next_send_time);
      }
    }
    std::vector<std::uint8_t> chunk(
        context.pending_pcm.begin(),
        context.pending_pcm.begin() + static_cast<std::ptrdiff_t>(chunk_size));
    context.api_result = context.audio_client->PlayStream(
        context.app_name, context.stream_id, std::move(chunk));
    if (context.api_result != 0) {
      context.error =
          "local_tts_play_stream_failed_" + std::to_string(context.api_result);
      return false;
    }
    context.next_send_time =
        std::chrono::steady_clock::now() + kLocalTtsSendPace;
    context.pending_pcm.erase(0, chunk_size);
  }
  return true;
}

std::size_t CurlPcmWriteCallback(char* data, std::size_t size,
                                 std::size_t count, void* user_data) {
  const std::size_t bytes = size * count;
  auto* context = static_cast<LocalTtsStreamContext*>(user_data);
  if (!context || !context->audio_client || bytes == 0 ||
      context->total_pcm_bytes + bytes > kMaximumLocalTtsPcmBytes) {
    if (context && context->error.empty()) {
      context->error = "local_tts_pcm_too_large";
    }
    return 0;
  }
  context->pending_pcm.append(data, bytes);
  context->total_pcm_bytes += bytes;
  if (!FlushLocalTtsPcm(*context, false)) return 0;
  return bytes;
}
#endif

}  // namespace

VoiceService::VoiceService(SnapshotStore& store, bool mock)
    : store_(store), mock_(mock) {}

VoiceService::~VoiceService() { Stop(); }

bool VoiceService::Start(std::string& error) {
  if (running_.load()) {
    error = "Voice service is already running";
    return false;
  }

  std::string config_error;
  LoadCustomerVoiceConfig(config_error);

  VoiceData voice;
  if (mock_) {
    voice.initialized = true;
    voice.asr_subscribed = true;
    voice.asr_control_api_result = 0;
    voice.chat_go_found = true;
    voice.chat_go_service_name = "chat_go";
    voice.chat_go_closed = false;
    voice.chat_go_status_raw = 0;
    voice.llm.mode = "builtin";
    voice.llm.builtin_api_available = true;
    voice.llm.builtin_response_subscribed = true;
    voice.llm.customer_api_available = true;
    chat_go_found_ = true;
    chat_go_service_name_ = "chat_go";
    chat_go_closed_ = false;
    llm_mode_ = "builtin";
    voice.vui_service_found = true;
    voice.vui_service_status_raw = 0;
    voice.volume_pct = 82;
    error.clear();
  } else {
    voice = InitializeRealVoice(error);
    if (!voice.initialized) {
      store_.SetVoiceState(voice);
      return false;
    }
  }

  voice.llm.customer_api_url = customer_api_url_;
  voice.llm.customer_model = customer_model_;
  voice.llm.customer_api_configured =
      !customer_api_url_.empty() && !customer_model_.empty();
  voice.llm.customer_api_key_configured = !customer_api_key_.empty();
  voice.llm.customer_role_prompt = customer_role_prompt_;
  voice.llm.customer_wake_word = customer_wake_word_;
  voice.llm.customer_wake_enabled = customer_wake_enabled_;
  voice.llm.customer_qa_count =
      static_cast<std::uint32_t>(customer_qa_entries_.size());
  voice.llm.customer_tts_backend = customer_tts_backend_;
  if (!config_error.empty()) {
    voice.llm.last_error = config_error;
  }
  store_.SetVoiceState(voice);

  asr_enabled_.store(voice.asr_subscribed);
  last_customer_wake_asr_index_ = 0;
  customer_wake_until_ = {};
  running_.store(true);
  std::string builtin_llm_error;
  if (!StartBuiltinLlmChannels(builtin_llm_error)) {
    LlmData llm = store_.GetSnapshot().voice.llm;
    llm.builtin_api_available = false;
    llm.builtin_response_subscribed = false;
    llm.last_error = builtin_llm_error;
    llm.updated_time_ms = SystemTimeMs();
    PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);
  }
  worker_ = std::thread([this] { WorkerLoop(); });
  return error.empty();
}

VoiceData VoiceService::InitializeRealVoice(std::string& error) {
  VoiceData voice;
  try {
    audio_client_ =
        std::make_unique<unitree::robot::g1::AudioClient>();
    audio_client_->SetTimeout(10.0F);
    audio_client_->Init();

    std::string asr_error;
    voice.asr_subscribed = StartAsrSubscription(asr_error);
    voice.asr_control_api_result = voice.asr_subscribed ? 0 : -1;
    voice.asr_control_error = asr_error;

    robot_state_client_ =
        std::make_unique<unitree::robot::b2::RobotStateClient>();
    robot_state_client_->SetTimeout(10.0F);
    robot_state_client_->Init();

    std::vector<unitree::robot::b2::ServiceState> services;
    const std::int32_t list_result =
        robot_state_client_->ServiceList(services);
    std::string chat_go_name;
    if (list_result == 0) {
      for (const auto& service : services) {
        const std::string normalized = NormalizeServiceName(service.name);
        if (normalized == "chatgo") {
          chat_go_name = service.name;
          voice.chat_go_found = true;
          voice.chat_go_service_name = service.name;
          voice.chat_go_status_raw = service.status;
        } else if (normalized == "vuiservice") {
          voice.vui_service_found = true;
          voice.vui_service_status_raw = service.status;
        }
      }
    }

    if (chat_go_name.empty()) {
      // ServiceList can be incomplete on some firmware. Use the documented
      // user-visible name for one explicit switch attempt.
      chat_go_name = "chat_go";
      voice.chat_go_service_name = chat_go_name;
    }
    chat_go_service_name_ = chat_go_name;
    chat_go_found_ = voice.chat_go_found;

    std::int32_t switched_status = voice.chat_go_status_raw;
    if (voice.chat_go_found && voice.chat_go_status_raw == 0) {
      voice.chat_go_api_result = 0;
      voice.chat_go_closed = false;
    } else {
      voice.chat_go_api_result = robot_state_client_->ServiceSwitch(
          chat_go_name, 1, switched_status);
      voice.chat_go_status_raw = switched_status;
      voice.chat_go_found =
          voice.chat_go_found || voice.chat_go_api_result == 0;
      voice.chat_go_closed =
          !(voice.chat_go_api_result == 0 && switched_status == 0);
    }
    chat_go_found_ = voice.chat_go_found;
    chat_go_closed_ = voice.chat_go_closed;
    llm_mode_ = "builtin";
    voice.llm.mode = "builtin";
#ifdef G1_WEB_HAS_CURL
    voice.llm.customer_api_available = true;
#else
    voice.llm.customer_api_available = false;
#endif

    std::uint8_t volume = 0;
    voice.volume_api_result = audio_client_->GetVolume(volume);
    if (voice.volume_api_result == 0) {
      voice.volume_pct = volume;
    }

    voice.initialized = true;
    if (voice.chat_go_closed) {
      error = "Failed to enable chat_go: ret=" +
              std::to_string(voice.chat_go_api_result) +
              ", status=" + std::to_string(voice.chat_go_status_raw);
      voice.initialization_error = error;
    } else if (voice.volume_api_result != 0) {
      error = "Audio service verification failed: GetVolume ret=" +
              std::to_string(voice.volume_api_result);
      voice.initialization_error = error;
    } else {
      error.clear();
      voice.initialization_error.clear();
    }
  } catch (const std::exception& exception) {
    error = exception.what();
    voice.initialized = false;
    voice.initialization_error = error;
    if (audio_subscriber_) {
      audio_subscriber_->CloseChannel();
      audio_subscriber_.reset();
    }
  } catch (...) {
    error = "Unknown voice initialization error";
    voice.initialized = false;
    voice.initialization_error = error;
    if (audio_subscriber_) {
      audio_subscriber_->CloseChannel();
      audio_subscriber_.reset();
    }
  }
  return voice;
}

void VoiceService::Stop() {
  running_.store(false);
  asr_enabled_.store(false);
  StopAsrSubscription();
  StopBuiltinLlmChannels();
  queue_cv_.notify_all();
  play_state_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  audio_client_.reset();
  robot_state_client_.reset();
  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    last_customer_wake_asr_index_ = 0;
    customer_wake_until_ = {};
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.clear();
  customer_wake_queue_.clear();
  tts_in_flight_ = false;
}

TtsEnqueueResult VoiceService::EnqueueTts(const std::string& text,
                                          std::int32_t speaker_id,
                                          const std::string& backend) {
  TtsEnqueueResult result;
  const std::string trimmed = Trim(text);
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (trimmed.empty()) {
    result.error = "text_is_empty";
    return result;
  }
  if (trimmed.size() > 1024) {
    result.error = "text_too_long";
    return result;
  }
  std::int32_t resolved_speaker_id = -1;
  if (!ResolveSpeakerId(trimmed, speaker_id, resolved_speaker_id,
                        result.error)) {
    return result;
  }
  result.speaker_id = resolved_speaker_id;

  std::string resolved_backend = backend;
  if (resolved_backend.empty()) {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    resolved_backend = customer_tts_backend_;
  }
  if (resolved_backend != "kokoro" && resolved_backend != "unitree") {
    result.error = "invalid_tts_backend";
    return result;
  }
  result.backend = resolved_backend;

  if (store_.GetSnapshot().voice.play_state_raw == 1) {
    result.error = "audio_busy";
    return result;
  }

  TtsRequest request;
  request.request_id = next_request_id_.fetch_add(1);
  request.text = trimmed;
  request.speaker_id = resolved_speaker_id;
  request.backend = resolved_backend;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (request.text == last_tts_text_ &&
        request.speaker_id == last_tts_speaker_id_ &&
        request.backend == last_tts_backend_ &&
        last_tts_accepted_.time_since_epoch().count() != 0 &&
        now - last_tts_accepted_ < std::chrono::seconds(3)) {
      result.error = "duplicate_tts_request";
      return result;
    }
    if (tts_in_flight_) {
      result.error = "tts_busy";
      return result;
    }
    if (queue_.size() >= 4) {
      result.error = "tts_queue_full";
      return result;
    }
    queue_.push_back(request);
    last_tts_text_ = request.text;
    last_tts_speaker_id_ = request.speaker_id;
    last_tts_backend_ = request.backend;
    last_tts_accepted_ = now;
  }

  TtsData tts;
  tts.request_id = request.request_id;
  tts.state = "queued";
  tts.text = request.text;
  tts.speaker_id = request.speaker_id;
  tts.backend = request.backend;
  tts.updated_time_ms = SystemTimeMs();
  store_.UpdateTts(tts);
  queue_cv_.notify_one();

  result.accepted = true;
  result.request_id = request.request_id;
  return result;
}

void VoiceService::WorkerLoop() {
  while (running_.load()) {
    TtsRequest request;
    std::string customer_wake_message;
    bool has_tts_request = false;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(
          lock, std::chrono::milliseconds(500),
          [this] {
            return !running_.load() || !queue_.empty() ||
                   !customer_wake_queue_.empty();
          });
      if (!running_.load()) {
        break;
      }
      if (!queue_.empty()) {
        request = std::move(queue_.front());
        queue_.pop_front();
        tts_in_flight_ = true;
        has_tts_request = true;
      } else if (!customer_wake_queue_.empty()) {
        customer_wake_message = std::move(customer_wake_queue_.front());
        customer_wake_queue_.pop_front();
      }
    }

    CheckBuiltinLlmTimeout();
    if (!customer_wake_message.empty()) {
      const auto wake_result = ChatWithCustomerLlm(customer_wake_message, true);
      {
        std::lock_guard<std::mutex> lock(llm_mutex_);
        if (llm_mode_ == "customer" && customer_wake_enabled_) {
          customer_wake_until_ = std::chrono::steady_clock::now() +
                                 kCustomerWakeFollowupTimeout;
        }
      }
      if (!wake_result.accepted) {
        const char* failure_prompt =
            wake_result.error == "customer_api_not_configured"
                ? "大模型还没有配置，请先完成配置"
                : "网络或大模型服务暂时不可用，请稍后再试";
        std::string tts_backend;
        {
          std::lock_guard<std::mutex> lock(llm_mutex_);
          tts_backend = customer_tts_backend_;
        }
        EnqueueTts(failure_prompt, -1, tts_backend);
      }
      continue;
    }
    if (!has_tts_request) continue;

    TtsData tts;
    tts.request_id = request.request_id;
    tts.state = "running";
    tts.text = request.text;
    tts.speaker_id = request.speaker_id;
    tts.backend = request.backend;
    tts.updated_time_ms = SystemTimeMs();
    store_.UpdateTts(tts);

    if (tts.backend == "kokoro" && !mock_) {
      std::string local_tts_error;
      if (PlayWithLocalTts(request.text, local_tts_error)) {
        tts.api_result = 0;
        tts.state = "succeeded";
        tts.updated_time_ms = SystemTimeMs();
        store_.UpdateTts(tts);
        std::lock_guard<std::mutex> lock(queue_mutex_);
        tts_in_flight_ = false;
        continue;
      }
      tts.backend = "unitree_fallback";
      tts.backend_error = local_tts_error;
      store_.UpdateTts(tts);
    }

    std::vector<TtsSegment> segments;
    if (request.speaker_id == -1) {
      segments = SplitMixedTtsText(request.text);
    } else {
      segments.push_back({request.text, request.speaker_id});
    }

    tts.api_result = 0;
    for (const auto& segment : segments) {
      if (!running_.load()) {
        tts.api_result = -1;
        break;
      }
      if (mock_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        continue;
      }

      std::uint64_t play_event_baseline = 0;
      {
        std::lock_guard<std::mutex> lock(play_state_mutex_);
        play_event_baseline = play_event_sequence_;
      }

      tts.api_result = audio_client_->TtsMaker(segment.text, segment.speaker_id);
      if (tts.api_result != 0) break;

      std::unique_lock<std::mutex> play_lock(play_state_mutex_);
      const bool saw_playing = play_state_cv_.wait_for(
          play_lock, std::chrono::seconds(3), [this, play_event_baseline] {
            return !running_.load() ||
                   last_play_start_event_ > play_event_baseline;
          });
      if (!running_.load()) {
        tts.api_result = -1;
        break;
      }

      if (!saw_playing) {
        play_lock.unlock();
        std::this_thread::sleep_for(EstimateTtsPlaybackDuration(segment.text));
        continue;
      }

      const std::uint64_t segment_start_event = last_play_start_event_;
      const bool finished = play_state_cv_.wait_for(
          play_lock, std::chrono::seconds(45), [this, segment_start_event] {
            return !running_.load() ||
                   last_play_stop_event_ > segment_start_event;
          });
      if (!running_.load() || !finished) {
        tts.api_result = -1;
        break;
      }
    }
    tts.state = tts.api_result == 0 ? "succeeded" : "failed";
    tts.updated_time_ms = SystemTimeMs();
    store_.UpdateTts(tts);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      tts_in_flight_ = false;
    }
  }
}

VoiceActionResult VoiceService::SetAsrEnabled(bool enabled) {
  VoiceActionResult result;
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }

  if (!enabled) {
    asr_enabled_.store(false);
    {
      std::lock_guard<std::mutex> lock(llm_mutex_);
      last_customer_wake_asr_index_ = 0;
      customer_wake_until_ = {};
    }
    store_.UpdateAsrSubscription(false, 0);
    result.accepted = true;
    result.value = 0;
    return result;
  }

  std::string error;
  if (!StartAsrSubscription(error)) {
    store_.UpdateAsrSubscription(false, -1, error);
    result.api_result = -1;
    result.error = error;
    return result;
  }
  asr_enabled_.store(true);
  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    last_customer_wake_asr_index_ = 0;
    customer_wake_until_ = {};
  }
  store_.UpdateAsrSubscription(true, 0);
  result.accepted = true;
  result.value = 1;
  return result;
}

VoiceActionResult VoiceService::SetVolume(std::int32_t volume_pct) {
  VoiceActionResult result;
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (volume_pct < 0 || volume_pct > 100) {
    result.error = "invalid_volume";
    return result;
  }

  if (mock_) {
    result.api_result = 0;
  } else {
    try {
      std::lock_guard<std::mutex> lock(voice_mutex_);
      if (!audio_client_) {
        result.error = "voice_not_ready";
        return result;
      }
      result.api_result =
          audio_client_->SetVolume(static_cast<std::uint8_t>(volume_pct));
    } catch (const std::exception& exception) {
      result.api_result = -1;
      result.error = exception.what();
    } catch (...) {
      result.api_result = -1;
      result.error = "unknown_volume_exception";
    }
  }
  store_.UpdateVolume(result.api_result, volume_pct);
  if (result.api_result != 0) {
    if (result.error.empty()) result.error = "volume_api_error";
    return result;
  }
  result.accepted = true;
  result.value = volume_pct;
  return result;
}

CustomerVoiceConfigResult VoiceService::GetCustomerVoiceConfig() {
  CustomerVoiceConfigResult result;
  std::lock_guard<std::mutex> lock(llm_mutex_);
  const std::string previous_wake = customer_wake_word_;
  const bool previous_wake_enabled = customer_wake_enabled_;
  std::string reload_error;
  if (!LoadCustomerVoiceConfig(reload_error)) {
    result.error = reload_error.empty() ? "customer_config_load_failed"
                                        : reload_error;
    return result;
  }
  if (customer_wake_word_ != previous_wake ||
      customer_wake_enabled_ != previous_wake_enabled) {
    last_customer_wake_asr_index_ = 0;
    customer_wake_until_ = {};
  }
  result.accepted = true;
  result.config.api_url = customer_api_url_;
  result.config.api_key = customer_api_key_.empty() ? "" : kSavedApiKeyMask;
  result.config.model = customer_model_;
  result.config.api_key_configured = !customer_api_key_.empty();
  result.config.preserve_api_key = !customer_api_key_.empty();
  result.config.config_path = customer_config_path_;
  result.config.role_prompt = customer_role_prompt_;
  result.config.wake_word = customer_wake_word_;
  result.config.wake_enabled = customer_wake_enabled_;
  result.config.tts_backend = customer_tts_backend_;
  result.config.qa_entries = customer_qa_entries_;
  return result;
}

CustomerVoiceConfigResult VoiceService::SetCustomerVoiceConfig(
    const CustomerVoiceConfig& config) {
  CustomerVoiceConfigResult result;
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (config.role_prompt.size() > 8192) {
    result.error = "customer_role_too_long";
    return result;
  }
  const std::string wake_word = Trim(config.wake_word);
  if (wake_word.size() > 128) {
    result.error = "customer_wake_word_too_long";
    return result;
  }
  if (config.wake_enabled && wake_word.empty()) {
    result.error = "customer_wake_word_empty";
    return result;
  }
  if (config.tts_backend != "unitree" && config.tts_backend != "kokoro") {
    result.error = "invalid_customer_tts_backend";
    return result;
  }
  // Bound pathological payloads, but deduplicate before enforcing the actual
  // 100-question library limit so a legacy file/request containing repeated
  // rows can be repaired instead of being rejected forever.
  if (config.qa_entries.size() > 2000) {
    result.error = "customer_qa_too_many_entries";
    return result;
  }

  const std::string trimmed_api_url = Trim(config.api_url);
  const std::string trimmed_model = Trim(config.model);
  std::string normalized_api_url;
  if (config.update_api_config) {
    if (!trimmed_api_url.empty() || !trimmed_model.empty()) {
      normalized_api_url = NormalizeCustomerApiUrl(trimmed_api_url);
      if (!IsValidApiUrl(normalized_api_url)) {
        result.error = "invalid_customer_api_url";
        return result;
      }
      if (trimmed_model.empty() || trimmed_model.size() > 256) {
        result.error = "invalid_customer_model";
        return result;
      }
    }
    if (config.api_key.size() > 4096) {
      result.error = "customer_api_key_too_long";
      return result;
    }
  }

  std::vector<CustomerQaEntry> normalized_entries;
  normalized_entries.reserve(config.qa_entries.size());
  std::unordered_map<std::string, std::size_t> qa_index_by_question;
  for (const auto& entry : config.qa_entries) {
    CustomerQaEntry normalized{Trim(entry.question), Trim(entry.answer)};
    if (normalized.question.empty() || normalized.answer.empty()) {
      result.error = "customer_qa_empty_entry";
      return result;
    }
    if (normalized.question.size() > 512 || normalized.answer.size() > 4096) {
      result.error = "customer_qa_entry_too_long";
      return result;
    }
    const auto existing = qa_index_by_question.find(normalized.question);
    if (existing != qa_index_by_question.end()) {
      // A fixed-answer question is a unique key. When a repeated question is
      // submitted, the last answer is the user's newest edit and replaces the
      // older value instead of appending another persistent entry.
      normalized_entries[existing->second] = std::move(normalized);
      continue;
    }
    if (normalized_entries.size() >= 100) {
      result.error = "customer_qa_too_many_entries";
      return result;
    }
    qa_index_by_question.emplace(normalized.question,
                                 normalized_entries.size());
    normalized_entries.push_back(std::move(normalized));
  }

  std::lock_guard<std::mutex> lock(llm_mutex_);
  std::string next_api_key = customer_api_key_;
  if (config.update_api_config) {
    next_api_key = config.api_key;
    const bool masked_key = next_api_key == kSavedApiKeyMask;
    if (config.preserve_api_key || masked_key) {
      if (!normalized_api_url.empty() && !customer_api_url_.empty() &&
          normalized_api_url != customer_api_url_) {
        result.error = "customer_api_key_endpoint_changed";
        return result;
      }
      next_api_key = customer_api_key_;
    }
  }

  const std::string previous_api_url = customer_api_url_;
  const std::string previous_api_key = customer_api_key_;
  const std::string previous_model = customer_model_;
  const std::string previous_role = customer_role_prompt_;
  const std::string previous_wake = customer_wake_word_;
  const bool previous_wake_enabled = customer_wake_enabled_;
  const std::string previous_tts_backend = customer_tts_backend_;
  const auto previous_qa_entries = customer_qa_entries_;

  if (config.update_api_config) {
    customer_api_url_ = normalized_api_url;
    customer_api_key_ = std::move(next_api_key);
    customer_model_ = trimmed_model;
  }
  customer_role_prompt_ = Trim(config.role_prompt);
  customer_wake_word_ = wake_word;
  customer_wake_enabled_ = config.wake_enabled;
  customer_tts_backend_ = config.tts_backend;
  customer_qa_entries_ = std::move(normalized_entries);

  std::string save_error;
  if (!SaveCustomerVoiceConfig(save_error)) {
    customer_api_url_ = previous_api_url;
    customer_api_key_ = previous_api_key;
    customer_model_ = previous_model;
    customer_role_prompt_ = previous_role;
    customer_wake_word_ = previous_wake;
    customer_wake_enabled_ = previous_wake_enabled;
    customer_tts_backend_ = previous_tts_backend;
    customer_qa_entries_ = previous_qa_entries;
    result.error = save_error.empty() ? "customer_config_save_failed" : save_error;
    return result;
  }

  if (!customer_wake_enabled_ || customer_wake_word_ != previous_wake) {
    last_customer_wake_asr_index_ = 0;
    customer_wake_until_ = {};
  }

  LlmData llm = store_.GetSnapshot().voice.llm;
  llm.customer_api_url = customer_api_url_;
  llm.customer_model = customer_model_;
  llm.customer_api_configured =
      !customer_api_url_.empty() && !customer_model_.empty();
  llm.customer_api_key_configured = !customer_api_key_.empty();
  llm.customer_role_prompt = customer_role_prompt_;
  llm.customer_wake_word = customer_wake_word_;
  llm.customer_wake_enabled = customer_wake_enabled_;
  llm.customer_qa_count =
      static_cast<std::uint32_t>(customer_qa_entries_.size());
  llm.customer_tts_backend = customer_tts_backend_;
  llm.last_error.clear();
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);

  result.accepted = true;
  result.config.api_url = customer_api_url_;
  result.config.api_key = customer_api_key_.empty() ? "" : kSavedApiKeyMask;
  result.config.model = customer_model_;
  result.config.api_key_configured = !customer_api_key_.empty();
  result.config.preserve_api_key = !customer_api_key_.empty();
  result.config.config_path = customer_config_path_;
  result.config.role_prompt = customer_role_prompt_;
  result.config.wake_word = customer_wake_word_;
  result.config.wake_enabled = customer_wake_enabled_;
  result.config.tts_backend = customer_tts_backend_;
  result.config.qa_entries = customer_qa_entries_;
  return result;
}

bool VoiceService::LoadCustomerVoiceConfig(std::string& error) {
  error.clear();
  try {
    const std::filesystem::path path(customer_config_path_);
    if (std::filesystem::exists(path)) {
      std::filesystem::permissions(
          path,
          std::filesystem::perms::owner_read |
              std::filesystem::perms::owner_write,
          std::filesystem::perm_options::replace);
    }
  } catch (const std::exception&) {
    error = "customer_config_permissions_failed";
  }
  std::ifstream stream(customer_config_path_);
  if (!stream.good()) {
    return true;
  }

  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string parse_errors;
  if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
      !root.isObject()) {
    error = "customer_config_invalid_json";
    return false;
  }

  const std::string persisted_api_url = Trim(root.get("api_url", "").asString());
  const std::string persisted_model = Trim(root.get("model", "").asString());
  const std::string persisted_api_key = root.get("api_key", "").asString();
  if (!persisted_api_url.empty() || !persisted_model.empty()) {
    const std::string normalized_url = NormalizeCustomerApiUrl(persisted_api_url);
    if (!IsValidApiUrl(normalized_url) || persisted_model.empty() ||
        persisted_model.size() > 256 || persisted_api_key.size() > 4096) {
      error = "customer_api_config_invalid";
    } else {
      customer_api_url_ = normalized_url;
      customer_model_ = persisted_model;
      customer_api_key_ = persisted_api_key;
    }
  }

  customer_role_prompt_ = Trim(root.get("role_prompt", "").asString());
  customer_wake_word_ = Trim(root.get("wake_word", "").asString());
  customer_wake_enabled_ = root.get("wake_enabled", false).asBool();
  customer_tts_backend_ = root.get("tts_backend", "kokoro").asString();
  if (customer_tts_backend_ == "melotts") {
    // Migrate the short-lived development backend name to the deployed
    // lightweight ONNX runtime without breaking an existing config file.
    customer_tts_backend_ = "kokoro";
  }
  if (customer_tts_backend_ != "unitree" &&
      customer_tts_backend_ != "kokoro") {
    customer_tts_backend_ = "kokoro";
  }
  customer_qa_entries_.clear();
  bool qa_entries_need_rewrite = false;
  const Json::Value entries = root["qa_entries"];
  if (entries.isArray()) {
    std::unordered_map<std::string, std::size_t> qa_index_by_question;
    for (Json::ArrayIndex i = 0; i < entries.size(); ++i) {
      if (!entries[i].isObject()) {
        qa_entries_need_rewrite = true;
        continue;
      }
      CustomerQaEntry entry{
          Trim(entries[i].get("question", "").asString()),
          Trim(entries[i].get("answer", "").asString())};
      if (entry.question.empty() || entry.answer.empty() ||
          entry.question.size() > 512 || entry.answer.size() > 4096) {
        qa_entries_need_rewrite = true;
        continue;
      }
      const auto existing = qa_index_by_question.find(entry.question);
      if (existing != qa_index_by_question.end()) {
        customer_qa_entries_[existing->second] = std::move(entry);
        qa_entries_need_rewrite = true;
        continue;
      }
      if (customer_qa_entries_.size() >= 100) {
        qa_entries_need_rewrite = true;
        continue;
      }
      qa_index_by_question.emplace(entry.question,
                                   customer_qa_entries_.size());
      customer_qa_entries_.push_back(std::move(entry));
    }
  } else if (!entries.isNull()) {
    qa_entries_need_rewrite = true;
  }

  // Repair legacy files that accumulated duplicate QA rows. Save writes the
  // complete JSON document via truncate+rename, so this is a true replacement
  // and never an append operation.
  if (qa_entries_need_rewrite && error.empty()) {
    std::string repair_error;
    if (!SaveCustomerVoiceConfig(repair_error)) {
      error = repair_error.empty() ? "customer_config_save_failed"
                                   : repair_error;
      return false;
    }
  }
  return true;
}

bool VoiceService::SaveCustomerVoiceConfig(std::string& error) const {
  error.clear();
  try {
    const std::filesystem::path path(customer_config_path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    Json::Value root(Json::objectValue);
    root["api_url"] = customer_api_url_;
    root["api_key"] = customer_api_key_;
    root["model"] = customer_model_;
    root["role_prompt"] = customer_role_prompt_;
    root["wake_word"] = customer_wake_word_;
    root["wake_enabled"] = customer_wake_enabled_;
    root["tts_backend"] = customer_tts_backend_;
    Json::Value entries(Json::arrayValue);
    for (const auto& entry : customer_qa_entries_) {
      Json::Value item(Json::objectValue);
      item["question"] = entry.question;
      item["answer"] = entry.answer;
      entries.append(item);
    }
    root["qa_entries"] = entries;

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "  ";
    const std::string body = Json::writeString(writer, root);
    const std::filesystem::path temp_path = path.string() + ".tmp";
    {
      std::ofstream stream(temp_path, std::ios::binary | std::ios::trunc);
      if (!stream.good()) {
        error = "customer_config_open_failed";
        return false;
      }
      std::filesystem::permissions(
          temp_path,
          std::filesystem::perms::owner_read |
              std::filesystem::perms::owner_write,
          std::filesystem::perm_options::replace);
      stream << body;
      if (!stream.good()) {
        error = "customer_config_write_failed";
        return false;
      }
    }
    std::filesystem::permissions(
        temp_path,
        std::filesystem::perms::owner_read |
            std::filesystem::perms::owner_write,
        std::filesystem::perm_options::replace);
    std::filesystem::rename(temp_path, path);
    std::filesystem::permissions(
        path,
        std::filesystem::perms::owner_read |
            std::filesystem::perms::owner_write,
        std::filesystem::perm_options::replace);
    return true;
  } catch (const std::exception&) {
    error = "customer_config_save_failed";
    return false;
  }
}

LlmModeResult VoiceService::SetLlmMode(
    const std::string& mode, const std::string& api_url,
    const std::string& api_key, const std::string& model,
    bool preserve_api_key) {
  LlmModeResult result;
  result.mode = mode;
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (mode != "builtin" && mode != "customer") {
    result.error = "invalid_llm_mode";
    return result;
  }

  const std::string requested_url = Trim(api_url);
  const std::string requested_model = Trim(model);
  const bool use_saved_customer_config =
      mode == "customer" && requested_url.empty() && requested_model.empty() &&
      api_key.empty() && !preserve_api_key;
  if (mode == "customer") {
#ifndef G1_WEB_HAS_CURL
    if (!mock_) {
      result.error = "customer_api_unavailable";
      return result;
    }
#endif
  }

  std::lock_guard<std::mutex> lock(llm_mutex_);
  std::string normalized_url = requested_url;
  std::string trimmed_model = requested_model;
  std::string next_api_key = api_key;
  if (mode == "customer") {
    if (use_saved_customer_config) {
      normalized_url = customer_api_url_;
      trimmed_model = customer_model_;
      next_api_key = customer_api_key_;
      if (normalized_url.empty() || trimmed_model.empty()) {
        result.error = "customer_api_not_configured";
        return result;
      }
    } else {
      normalized_url = NormalizeCustomerApiUrl(requested_url);
      if (!IsValidApiUrl(normalized_url)) {
        result.error = "invalid_customer_api_url";
        return result;
      }
      if (trimmed_model.empty() || trimmed_model.size() > 256) {
        result.error = "invalid_customer_model";
        return result;
      }
      if (api_key.size() > 4096) {
        result.error = "customer_api_key_too_long";
        return result;
      }
      if (preserve_api_key && api_key.empty()) {
        if (customer_api_url_.empty() || normalized_url != customer_api_url_) {
          result.error = "customer_api_key_endpoint_changed";
          return result;
        }
        next_api_key = customer_api_key_;
      }
    }
  }

  const std::string previous_mode = llm_mode_;
  const std::string previous_api_url = customer_api_url_;
  const std::string previous_api_key = customer_api_key_;
  const std::string previous_model = customer_model_;
  std::int32_t api_result = 0;
  std::int32_t status_raw = chat_go_closed_ ? 1 : 0;
  std::string switch_error;
  const bool enable_chat_go = mode == "builtin";
  if (!SwitchChatGo(enable_chat_go, api_result, status_raw,
                    switch_error)) {
    LlmData llm = store_.GetSnapshot().voice.llm;
    llm.request_state = "failed";
    llm.last_error = switch_error;
    llm.updated_time_ms = SystemTimeMs();
    PublishLlmState(llm, api_result, status_raw);
    result.api_result = api_result;
    result.error = switch_error;
    return result;
  }

  llm_mode_ = mode;
  pending_builtin_request_id_ = 0;
  if (mode != "customer") {
    last_customer_wake_asr_index_ = 0;
    customer_wake_until_ = {};
  }
  if (mode == "customer" && !use_saved_customer_config) {
    customer_api_url_ = normalized_url;
    customer_api_key_ = next_api_key;
    customer_model_ = trimmed_model;
    std::string save_error;
    if (!SaveCustomerVoiceConfig(save_error)) {
      customer_api_url_ = previous_api_url;
      customer_api_key_ = previous_api_key;
      customer_model_ = previous_model;
      llm_mode_ = previous_mode;
      std::int32_t rollback_api_result = 0;
      std::int32_t rollback_status_raw = chat_go_closed_ ? 1 : 0;
      std::string rollback_error;
      SwitchChatGo(previous_mode == "builtin", rollback_api_result,
                   rollback_status_raw, rollback_error);
      LlmData failed = store_.GetSnapshot().voice.llm;
      failed.request_state = "failed";
      failed.last_error =
          save_error.empty() ? "customer_config_save_failed" : save_error;
      failed.updated_time_ms = SystemTimeMs();
      PublishLlmState(failed, rollback_api_result, rollback_status_raw);
      result.api_result = rollback_api_result;
      result.error = failed.last_error;
      return result;
    }
  }

  LlmData llm = store_.GetSnapshot().voice.llm;
  llm.mode = mode;
#ifdef G1_WEB_HAS_CURL
  llm.customer_api_available = true;
#else
  llm.customer_api_available = mock_;
#endif
  llm.customer_api_configured = !customer_api_url_.empty() &&
                                !customer_model_.empty();
  llm.customer_api_key_configured = !customer_api_key_.empty();
  llm.customer_api_url = customer_api_url_;
  llm.customer_model = customer_model_;
  llm.customer_role_prompt = customer_role_prompt_;
  llm.customer_wake_word = customer_wake_word_;
  llm.customer_wake_enabled = customer_wake_enabled_;
  llm.customer_qa_count =
      static_cast<std::uint32_t>(customer_qa_entries_.size());
  llm.customer_tts_backend = customer_tts_backend_;
  llm.last_response_source.clear();
  llm.request_state = "idle";
  llm.request_id = 0;
  llm.response_status_code = 0;
  llm.last_error.clear();
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, api_result, status_raw);

  result.accepted = true;
  result.api_result = api_result;
  result.mode = mode;
  return result;
}

LlmChatResult VoiceService::ChatWithLlm(
    const std::string& mode, const std::string& message,
    bool auto_tts) {
  LlmChatResult result;
  result.mode = mode;
  if (mode != "builtin" && mode != "customer") {
    result.error = "invalid_llm_mode";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    if (mode != llm_mode_) {
      result.mode = llm_mode_;
      result.error = "llm_mode_mismatch";
      return result;
    }
  }

  return mode == "builtin" ? ChatWithBuiltinLlm(message)
                           : ChatWithCustomerLlm(message, auto_tts);
}

LlmChatResult VoiceService::ChatWithBuiltinLlm(
    const std::string& message) {
  LlmChatResult result;
  result.mode = "builtin";
  const std::string trimmed = Trim(message);
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (trimmed.empty()) {
    result.error = "llm_message_empty";
    return result;
  }
  if (trimmed.size() > 4096) {
    result.error = "llm_message_too_long";
    return result;
  }

  std::lock_guard<std::mutex> lock(llm_mutex_);
  if (llm_mode_ != "builtin" || chat_go_closed_) {
    result.error = "builtin_llm_not_active";
    return result;
  }

  LlmData llm = store_.GetSnapshot().voice.llm;
  if (!llm.builtin_api_available || !llm.builtin_response_subscribed ||
      (!mock_ && !builtin_llm_publisher_)) {
    result.error = "builtin_gpt_channel_unavailable";
    return result;
  }
  if (pending_builtin_request_id_ != 0 && llm.request_state == "running") {
    result.error = "builtin_llm_busy";
    return result;
  }

  const std::int64_t request_id = SteadyTimeNs();
  pending_builtin_request_id_ = request_id;
  llm.request_state = "running";
  llm.request_id = request_id;
  llm.response_status_code = 0;
  llm.last_user_message = trimmed;
  llm.last_response.clear();
  llm.last_error.clear();
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);

  result.request_id = request_id;
  if (mock_) {
    result.accepted = true;
    result.pending = false;
    result.response = "模拟笨笨同学回复：" + trimmed;
    pending_builtin_request_id_ = 0;
    llm.request_state = "succeeded";
    llm.response_status_code = 0;
    llm.last_response = result.response;
    llm.updated_time_ms = SystemTimeMs();
    PublishLlmState(llm, 0, 0);
    return result;
  }

  unitree::robot::Request request;
  request.header().identity().id(request_id);
  request.header().identity().api_id(kBuiltinGptApiId);
  request.header().lease().id(0);
  request.header().policy().priority(0);
  request.header().policy().noreply(false);
  request.parameter(trimmed);
  request.binary({});

  if (!builtin_llm_publisher_->Write(request)) {
    pending_builtin_request_id_ = 0;
    result.error = "builtin_gpt_publish_failed";
    llm.request_state = "failed";
    llm.last_error = result.error;
    llm.updated_time_ms = SystemTimeMs();
    PublishLlmState(llm, 0, 0);
    return result;
  }

  result.accepted = true;
  result.pending = true;
  return result;
}

LlmChatResult VoiceService::ChatWithCustomerLlm(
    const std::string& message, bool auto_tts) {
  LlmChatResult result;
  result.mode = "customer";
  const std::string trimmed = Trim(message);
  if (!running_.load()) {
    result.error = "voice_not_ready";
    return result;
  }
  if (trimmed.empty()) {
    result.error = "llm_message_empty";
    return result;
  }
  if (trimmed.size() > 4096) {
    result.error = "llm_message_too_long";
    return result;
  }

  std::lock_guard<std::mutex> lock(llm_mutex_);
  if (llm_mode_ != "customer") {
    result.error = "customer_llm_not_active";
    return result;
  }
  if (customer_api_url_.empty() || customer_model_.empty()) {
    result.error = "customer_api_not_configured";
    return result;
  }

  LlmData llm = store_.GetSnapshot().voice.llm;
  llm.request_state = "running";
  llm.last_user_message = trimmed;
  llm.last_response.clear();
  llm.last_response_source.clear();
  llm.last_error.clear();
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);

  const std::string qa_answer = FindCustomerQaAnswer(trimmed);
  if (!qa_answer.empty()) {
    result.accepted = true;
    result.http_status = 200;
    result.response = qa_answer;
    result.response_source = "qa";
  } else if (mock_) {
    result.accepted = true;
    result.http_status = 200;
    result.response = "模拟客户大模型回复：" + trimmed;
    result.response_source = "llm";
  } else {
#ifndef G1_WEB_HAS_CURL
    result.error = "customer_api_unavailable";
#else
    static const CURLcode curl_global_result =
        curl_global_init(CURL_GLOBAL_DEFAULT);
    if (curl_global_result != CURLE_OK) {
      result.error = "customer_api_transport_init_failed";
    } else {
      Json::Value request(Json::objectValue);
      request["model"] = customer_model_;
      request["stream"] = false;
      Json::Value messages(Json::arrayValue);
      if (!customer_role_prompt_.empty()) {
        Json::Value system_message(Json::objectValue);
        system_message["role"] = "system";
        system_message["content"] = customer_role_prompt_;
        messages.append(system_message);
      }
      Json::Value user_message(Json::objectValue);
      user_message["role"] = "user";
      user_message["content"] = trimmed;
      messages.append(user_message);
      request["messages"] = messages;
      Json::StreamWriterBuilder writer;
      writer["indentation"] = "";
      const std::string request_body = Json::writeString(writer, request);

      std::string response_body;
      char curl_error[CURL_ERROR_SIZE] = {};
      CURL* curl = curl_easy_init();
      if (!curl) {
        result.error = "customer_api_transport_init_failed";
      } else {
        curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, "Accept: application/json");
        std::string authorization;
        if (!customer_api_key_.empty()) {
          authorization = "Authorization: Bearer " + customer_api_key_;
          headers = curl_slist_append(headers, authorization.c_str());
        }
        curl_easy_setopt(curl, CURLOPT_URL, customer_api_url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                         static_cast<long>(request_body.size()));
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT,
                         "UniRoboGui/" UNI_ROBO_GUI_VERSION " customer-llm");
        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

        const CURLcode curl_result = curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &result.http_status);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (curl_result != CURLE_OK) {
          result.error = curl_error[0] != '\0'
                             ? std::string(curl_error)
                             : curl_easy_strerror(curl_result);
        } else if (result.http_status < 200 || result.http_status >= 300) {
          result.error = ExtractLlmError(response_body);
          if (result.error.empty()) result.error = "customer_api_http_error";
        } else {
          result.response = ExtractLlmResponse(response_body);
          if (result.response.empty()) {
            result.error = "customer_api_invalid_response";
          } else {
            result.accepted = true;
            result.response_source = "llm";
          }
        }
      }
    }
#endif
  }

  llm.request_state = result.accepted ? "succeeded" : "failed";
  llm.last_response = result.response;
  llm.last_response_source = result.response_source;
  llm.last_error = result.error;
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);

  if (result.accepted && auto_tts) {
    const auto tts = EnqueueTts(result.response, -1, customer_tts_backend_);
    result.tts_request_id = tts.request_id;
    result.tts_error = tts.error;
  }
  return result;
}

std::string VoiceService::NormalizeQuestionForMatch(const std::string& text) {
  std::string normalized;
  normalized.reserve(text.size());
  for (const unsigned char value : text) {
    if (value < 0x80) {
      if (std::isalnum(value) != 0) {
        normalized.push_back(static_cast<char>(std::tolower(value)));
      }
      continue;
    }
    normalized.push_back(static_cast<char>(value));
  }
  static const std::vector<std::string> punctuation = {
      "，", "。", "！", "？", "；", "：", "、", "“", "”", "‘", "’"};
  for (const auto& token : punctuation) {
    std::size_t pos = 0;
    while ((pos = normalized.find(token, pos)) != std::string::npos) {
      normalized.erase(pos, token.size());
    }
  }
  return normalized;
}

bool VoiceService::StripCustomerWakePrefix(const std::string& text,
                                           const std::string& wake_word,
                                           std::string& message) {
  const std::string normalized_wake = NormalizeQuestionForMatch(wake_word);
  const std::string trimmed = Trim(text);
  if (normalized_wake.empty() || trimmed.empty()) return false;

  for (std::size_t end = 1; end <= trimmed.size(); ++end) {
    if (end < trimmed.size() &&
        (static_cast<unsigned char>(trimmed[end]) & 0xc0) == 0x80) {
      continue;
    }
    const std::string normalized_prefix =
        NormalizeQuestionForMatch(trimmed.substr(0, end));
    if (normalized_prefix == normalized_wake) {
      message = Trim(trimmed.substr(end));
      static const std::vector<std::string> wake_separators = {
          ",", ".", ":", ";", "!", "?", "，", "。", "：", "；", "！", "？", "、"};
      bool removed_separator = true;
      while (!message.empty() && removed_separator) {
        removed_separator = false;
        for (const auto& separator : wake_separators) {
          if (message.rfind(separator, 0) == 0) {
            message = Trim(message.substr(separator.size()));
            removed_separator = true;
            break;
          }
        }
      }
      return true;
    }
    if (normalized_prefix.size() > normalized_wake.size() ||
        normalized_wake.compare(0, normalized_prefix.size(),
                                normalized_prefix) != 0) {
      return false;
    }
  }
  return false;
}

std::string VoiceService::FindCustomerQaAnswer(
    const std::string& question) const {
  const std::string normalized_question = NormalizeQuestionForMatch(question);
  if (normalized_question.empty()) return {};
  for (const auto& entry : customer_qa_entries_) {
    const std::string normalized_entry =
        NormalizeQuestionForMatch(entry.question);
    if (!normalized_entry.empty() && normalized_entry == normalized_question) {
      return entry.answer;
    }
  }
  for (const auto& entry : customer_qa_entries_) {
    const std::string normalized_entry =
        NormalizeQuestionForMatch(entry.question);
    if (normalized_entry.size() >= 9 &&
        normalized_question.find(normalized_entry) != std::string::npos) {
      return entry.answer;
    }
  }
  return {};
}

void VoiceService::QueueCustomerWakeRequest(const AsrData& asr) {
  // Current G1 firmware can emit complete ASR utterances with is_final=false.
  // The ASR index is the reliable duplicate guard for wake handling.
  if (asr.text.empty()) return;

  std::string wake_word;
  std::string tts_backend;
  bool followup_open = false;
  std::chrono::steady_clock::time_point now;
  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    now = std::chrono::steady_clock::now();
    if (llm_mode_ != "customer" || !customer_wake_enabled_ ||
        customer_wake_word_.empty()) {
      return;
    }
    if (asr.index != 0 && asr.index == last_customer_wake_asr_index_) {
      return;
    }
    wake_word = customer_wake_word_;
    tts_backend = customer_tts_backend_;
    followup_open = customer_wake_until_.time_since_epoch().count() != 0 &&
                    now <= customer_wake_until_;
  }

  std::string message;
  const bool explicit_wake =
      StripCustomerWakePrefix(asr.text, wake_word, message);
  if (!explicit_wake) {
    if (!followup_open) return;
    message = Trim(asr.text);
    if (message.empty()) return;

    const TtsData recent_tts = store_.GetSnapshot().voice.tts;
    if (recent_tts.state == "queued" || recent_tts.state == "running") return;
    const std::int64_t tts_age_ms = SystemTimeMs() - recent_tts.updated_time_ms;
    if (recent_tts.updated_time_ms > 0 && tts_age_ms >= 0 &&
        tts_age_ms <= kCustomerWakeEchoGuardMs &&
        NormalizeQuestionForMatch(message) ==
            NormalizeQuestionForMatch(recent_tts.text)) {
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    if (llm_mode_ != "customer" || !customer_wake_enabled_ ||
        customer_wake_word_ != wake_word ||
        (!explicit_wake && now > customer_wake_until_)) {
      return;
    }
    if (asr.index != 0) last_customer_wake_asr_index_ = asr.index;
    customer_wake_until_ = now + kCustomerWakeFollowupTimeout;
    tts_backend = customer_tts_backend_;
  }

  if (explicit_wake && message.empty()) {
    EnqueueTts("我在", -1, tts_backend);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (customer_wake_queue_.size() >= 2) return;
    customer_wake_queue_.push_back(std::move(message));
  }
  queue_cv_.notify_one();
}

bool VoiceService::PlayWithLocalTts(const std::string& text,
                                    std::string& error) {
#ifdef G1_WEB_HAS_CURL
  static const CURLcode curl_global_result =
      curl_global_init(CURL_GLOBAL_DEFAULT);
  if (curl_global_result != CURLE_OK) {
    error = "local_tts_transport_init_failed";
    return false;
  }
  if (!audio_client_) {
    error = "local_tts_audio_client_unavailable";
    return false;
  }

  Json::Value request(Json::objectValue);
  request["text"] = text;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  const std::string request_body = Json::writeString(writer, request);

  LocalTtsStreamContext stream_context;
  stream_context.audio_client = audio_client_.get();
  stream_context.stream_id = std::to_string(SteadyTimeNs());

  std::uint64_t play_event_baseline = 0;
  {
    std::lock_guard<std::mutex> lock(play_state_mutex_);
    play_event_baseline = play_event_sequence_;
  }

  char curl_error[CURL_ERROR_SIZE] = {};
  CURL* curl = curl_easy_init();
  if (!curl) {
    error = "local_tts_transport_init_failed";
    return false;
  }
  curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8765/tts/stream");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                   static_cast<long>(request_body.size()));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
  // Long replies are streamed and can already be playing while synthesis is
  // still running. Keep a generous transfer timeout so an active local
  // playback is never replaced by the Unitree TTS merely because CPU Kokoro
  // generation took more than the old 45/60 second whole-response limit.
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 180L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_error);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlPcmWriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream_context);
  long status = 0;
  const CURLcode curl_result = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (curl_result != CURLE_OK) {
    if (stream_context.playback_started) {
      audio_client_->PlayStop(stream_context.stream_id);
    }
    error = !stream_context.error.empty()
                ? stream_context.error
                : "local_tts_synthesis_unavailable";
    return false;
  }
  if (status < 200 || status >= 300) {
    error = "local_tts_http_error_" + std::to_string(status);
    return false;
  }
  if (stream_context.total_pcm_bytes == 0 ||
      (stream_context.total_pcm_bytes % 2) != 0) {
    error = "local_tts_invalid_pcm";
    return false;
  }
  // Unitree PlayStream can report play_state=1 for very short streams while
  // the speaker output remains inaudible. Keep the same stream alive with
  // trailing silence so sub-second Kokoro replies such as "您好" / "hello"
  // have enough buffered duration to reach the physical speaker.
  if (stream_context.total_pcm_bytes < kLocalTtsMinimumStreamBytes) {
    const std::size_t padding_bytes =
        kLocalTtsMinimumStreamBytes - stream_context.total_pcm_bytes;
    stream_context.pending_pcm.append(padding_bytes, '\0');
    stream_context.total_pcm_bytes += padding_bytes;
  }
  if (!FlushLocalTtsPcm(stream_context, true)) {
    audio_client_->PlayStop(stream_context.stream_id);
    error = stream_context.error.empty() ? "local_tts_play_stream_failed"
                                         : stream_context.error;
    return false;
  }

  std::unique_lock<std::mutex> play_lock(play_state_mutex_);
  const bool saw_playing =
      last_play_start_event_ > play_event_baseline ||
      play_state_cv_.wait_for(
          play_lock, std::chrono::seconds(4), [this, play_event_baseline] {
            return !running_.load() ||
                   last_play_start_event_ > play_event_baseline;
          });
  if (!running_.load()) {
    error = "local_tts_stopped";
    return false;
  }
  if (!saw_playing) {
    play_lock.unlock();
    audio_client_->PlayStop(stream_context.stream_id);
    error = "local_tts_play_state_missing";
    return false;
  }

  const std::uint64_t segment_start_event = last_play_start_event_;
  const auto audio_duration = std::chrono::milliseconds(
      static_cast<std::int64_t>((stream_context.total_pcm_bytes * 1000ULL) /
                                (16000ULL * 2ULL)));
  const auto finish_timeout = std::min(
      std::chrono::milliseconds(90000),
      std::max(std::chrono::milliseconds(8000),
               audio_duration + std::chrono::milliseconds(15000)));
  const bool finished =
      last_play_stop_event_ > segment_start_event ||
      play_state_cv_.wait_for(
          play_lock, finish_timeout, [this, segment_start_event] {
            return !running_.load() ||
                   last_play_stop_event_ > segment_start_event;
          });
  if (!running_.load() || !finished) {
    play_lock.unlock();
    audio_client_->PlayStop(stream_context.stream_id);
    error = "local_tts_playback_timeout";
    return false;
  }

  error.clear();
  return true;
#else
  (void)text;
  error = "local_tts_unavailable";
  return false;
#endif
}

bool VoiceService::SwitchChatGo(bool enabled, std::int32_t& api_result,
                                std::int32_t& status_raw,
                                std::string& error) {
  if (mock_) {
    api_result = 0;
    status_raw = enabled ? 0 : 1;
    chat_go_found_ = true;
    chat_go_closed_ = !enabled;
    error.clear();
    return true;
  }

  try {
    std::lock_guard<std::mutex> lock(voice_mutex_);
    if (!robot_state_client_) {
      api_result = -1;
      error = "voice_not_ready";
      return false;
    }
    api_result = robot_state_client_->ServiceSwitch(
        chat_go_service_name_, enabled ? 1 : 0, status_raw);
    const std::int32_t expected_status = enabled ? 0 : 1;
    if (api_result != 0 || status_raw != expected_status) {
      error = enabled ? "chat_go_enable_failed" : "chat_go_disable_failed";
      return false;
    }
    chat_go_found_ = true;
    chat_go_closed_ = !enabled;
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    api_result = -1;
    error = exception.what();
  } catch (...) {
    api_result = -1;
    error = "chat_go_switch_exception";
  }
  return false;
}

void VoiceService::PublishLlmState(const LlmData& llm,
                                   std::int32_t api_result,
                                   std::int32_t status_raw) {
  store_.UpdateLlmState(chat_go_found_, chat_go_service_name_,
                        chat_go_closed_, api_result, status_raw, llm);
}

bool VoiceService::IsValidApiUrl(const std::string& url) {
  if (url.size() < 10 || url.size() > 1024 ||
      (url.rfind("https://", 0) != 0 && url.rfind("http://", 0) != 0) ||
      url.find('?') != std::string::npos ||
      url.find('#') != std::string::npos) {
    return false;
  }
  return std::none_of(url.begin(), url.end(), [](char value) {
    const auto byte = static_cast<unsigned char>(value);
    return byte <= 0x20 || byte == 0x7F;
  });
}

std::string VoiceService::NormalizeCustomerApiUrl(const std::string& url) {
  std::string normalized = Trim(url);
  while (normalized.size() > 8 && normalized.back() == '/') {
    normalized.pop_back();
  }
  constexpr const char* kChatCompletionsSuffix = "/chat/completions";
  constexpr std::size_t kChatCompletionsSuffixSize = 17;
  if (normalized.size() >= kChatCompletionsSuffixSize &&
      normalized.compare(normalized.size() - kChatCompletionsSuffixSize,
                         kChatCompletionsSuffixSize,
                         kChatCompletionsSuffix) == 0) {
    return normalized;
  }
  return normalized + kChatCompletionsSuffix;
}

std::string VoiceService::ExtractLlmError(const std::string& body) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(body);
  if (Json::parseFromStream(builder, stream, &root, &errors) &&
      root.isObject()) {
    if (root["error"].isObject() && root["error"]["message"].isString()) {
      return root["error"]["message"].asString();
    }
    if (root["error"].isString()) return root["error"].asString();
    if (root["message"].isString()) return root["message"].asString();
  }
  return body.size() <= 240 ? body : body.substr(0, 240);
}

std::string VoiceService::ExtractLlmResponse(const std::string& body) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(body);
  if (!Json::parseFromStream(builder, stream, &root, &errors) ||
      !root.isObject()) {
    return {};
  }
  if (root["choices"].isArray() && !root["choices"].empty()) {
    const auto& choice = root["choices"][0];
    if (choice["message"]["content"].isString()) {
      return choice["message"]["content"].asString();
    }
    if (choice["text"].isString()) return choice["text"].asString();
  }
  if (root["message"]["content"].isString()) {
    return root["message"]["content"].asString();
  }
  if (root["output_text"].isString()) {
    return root["output_text"].asString();
  }
  if (root["response"].isString()) return root["response"].asString();
  return {};
}

std::string VoiceService::ExtractBuiltinLlmResponse(
    const std::string& body) {
  const std::string trimmed = Trim(body);
  if (trimmed.empty()) return {};

  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string errors;
  std::istringstream stream(trimmed);
  if (!Json::parseFromStream(builder, stream, &root, &errors)) {
    return trimmed;
  }
  if (root.isString()) return root.asString();
  if (!root.isObject()) return trimmed;

  const char* direct_keys[] = {
      "text", "response", "answer", "content", "final_text", "output_text"};
  for (const char* key : direct_keys) {
    if (root[key].isString() && !root[key].asString().empty()) {
      return root[key].asString();
    }
  }
  if (root["message"].isString()) return root["message"].asString();
  if (root["message"].isObject()) {
    if (root["message"]["content"].isString()) {
      return root["message"]["content"].asString();
    }
    if (root["message"]["text"].isString()) {
      return root["message"]["text"].asString();
    }
  }
  if (root["data"].isString()) return root["data"].asString();
  if (root["data"].isObject()) {
    for (const char* key : direct_keys) {
      if (root["data"][key].isString() &&
          !root["data"][key].asString().empty()) {
        return root["data"][key].asString();
      }
    }
  }
  return trimmed;
}

bool VoiceService::StartBuiltinLlmChannels(std::string& error) {
  LlmData llm = store_.GetSnapshot().voice.llm;
  llm.builtin_request_topic = kBuiltinGptRequestTopic;
  llm.builtin_response_topic = kBuiltinGptResponseTopic;

  if (mock_) {
    llm.builtin_api_available = true;
    llm.builtin_response_subscribed = true;
    PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);
    error.clear();
    return true;
  }

  try {
    auto publisher = std::make_shared<unitree::robot::ChannelPublisher<
        unitree::robot::Request>>(kBuiltinGptRequestTopic);
    publisher->InitChannel();

    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
        unitree::robot::Response>>(kBuiltinGptResponseTopic);
    subscriber->InitChannel(
        [this](const void* data) {
          const auto* response =
              static_cast<const unitree::robot::Response*>(data);
          if (running_.load() && response) {
            HandleBuiltinLlmResponse(*response);
          }
        },
        10);

    {
      std::lock_guard<std::mutex> lock(llm_mutex_);
      builtin_llm_publisher_ = std::move(publisher);
      builtin_llm_subscriber_ = std::move(subscriber);
    }
    llm.builtin_api_available = true;
    llm.builtin_response_subscribed = true;
    PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown_builtin_gpt_channel_error";
  }

  StopBuiltinLlmChannels();
  return false;
}

void VoiceService::StopBuiltinLlmChannels() {
  std::shared_ptr<unitree::robot::ChannelPublisher<
      unitree::robot::Request>> publisher;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree::robot::Response>> subscriber;
  {
    std::lock_guard<std::mutex> lock(llm_mutex_);
    publisher = std::move(builtin_llm_publisher_);
    subscriber = std::move(builtin_llm_subscriber_);
    pending_builtin_request_id_ = 0;
  }
  if (subscriber) subscriber->CloseChannel();
  if (publisher) publisher->CloseChannel();
}

void VoiceService::HandleBuiltinLlmResponse(
    const unitree::robot::Response& response) {
  const std::int64_t request_id = response.header().identity().id();
  const std::int64_t api_id = response.header().identity().api_id();
  if (api_id != kBuiltinGptApiId) return;

  std::lock_guard<std::mutex> lock(llm_mutex_);
  if (request_id == 0 || request_id != pending_builtin_request_id_) return;

  pending_builtin_request_id_ = 0;
  LlmData llm = store_.GetSnapshot().voice.llm;
  llm.request_id = request_id;
  llm.response_status_code = response.header().status().code();
  llm.last_error.clear();
  llm.last_response.clear();
  if (llm.response_status_code == 0) {
    llm.request_state = "succeeded";
    llm.last_response = ExtractBuiltinLlmResponse(response.data());
    if (llm.last_response.empty()) {
      llm.last_response =
          "已收到笨笨同学的 DDS 成功响应，但 response.data 未包含文本。";
    }
  } else {
    llm.request_state = "failed";
    llm.last_error = "builtin_gpt_response_error_" +
                     std::to_string(llm.response_status_code);
  }
  llm.updated_time_ms = SystemTimeMs();
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);
}

void VoiceService::CheckBuiltinLlmTimeout() {
  std::lock_guard<std::mutex> lock(llm_mutex_);
  if (pending_builtin_request_id_ == 0) return;

  LlmData llm = store_.GetSnapshot().voice.llm;
  const std::int64_t now_ms = SystemTimeMs();
  if (llm.request_state != "running" || llm.updated_time_ms <= 0 ||
      now_ms - llm.updated_time_ms < 45000) {
    return;
  }

  pending_builtin_request_id_ = 0;
  llm.request_state = "failed";
  llm.last_error = "builtin_gpt_response_timeout";
  llm.response_status_code = -1;
  llm.updated_time_ms = now_ms;
  PublishLlmState(llm, 0, chat_go_closed_ ? 1 : 0);
}

bool VoiceService::StartAsrSubscription(std::string& error) {
  std::lock_guard<std::mutex> lock(voice_mutex_);
  if (audio_subscriber_) {
    error.clear();
    return true;
  }
  if (mock_) {
    error.clear();
    return true;
  }
  try {
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<
        std_msgs::msg::dds_::String_>>("rt/audio_msg");
    subscriber->InitChannel(
        [this](const void* data) {
          const auto* message =
              static_cast<const std_msgs::msg::dds_::String_*>(data);
          if (running_.load()) HandleAudioMessage(message->data());
        },
        10);
    audio_subscriber_ = std::move(subscriber);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
  } catch (...) {
    error = "unknown_asr_subscription_error";
  }
  return false;
}

void VoiceService::StopAsrSubscription() {
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      std_msgs::msg::dds_::String_>> subscriber;
  {
    std::lock_guard<std::mutex> lock(voice_mutex_);
    subscriber = std::move(audio_subscriber_);
  }
  if (subscriber) subscriber->CloseChannel();
}

bool VoiceService::ResolveSpeakerId(const std::string& text,
                                    std::int32_t requested_speaker_id,
                                    std::int32_t& resolved_speaker_id,
                                    std::string& error) {
  if (requested_speaker_id != -1 && requested_speaker_id != 0 &&
      requested_speaker_id != 1) {
    error = "invalid_speaker_id";
    return false;
  }

  bool has_english = false;
  bool has_han = false;
  for (std::size_t offset = 0; offset < text.size();) {
    const auto value = static_cast<unsigned char>(text[offset]);
    if (value < 0x80) {
      has_english = has_english ||
                    std::isalpha(static_cast<unsigned char>(value)) != 0;
      ++offset;
      continue;
    }
    std::uint32_t code_point = 0;
    if (!DecodeUtf8CodePoint(text, offset, code_point)) {
      error = "invalid_utf8";
      return false;
    }
    has_han = has_han || IsHanCodePoint(code_point);
  }

  if (has_english && has_han) {
    if (requested_speaker_id != -1) {
      error = "mixed_language_requires_auto";
      return false;
    }
    resolved_speaker_id = -1;
    error.clear();
    return true;
  }
  const std::int32_t detected_speaker = has_english ? 1 : 0;
  if (requested_speaker_id != -1 &&
      (has_english || has_han) &&
      requested_speaker_id != detected_speaker) {
    error = "speaker_language_mismatch";
    return false;
  }
  resolved_speaker_id =
      requested_speaker_id == -1 ? detected_speaker : requested_speaker_id;
  error.clear();
  return true;
}

void VoiceService::HandleAudioMessage(const std::string& message) {
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string parse_errors;
  std::istringstream stream(message);
  if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
      !root.isObject()) {
    return;
  }

  if (root.isMember("play_state")) {
    const std::int32_t play_state = root["play_state"].asInt();
    {
      std::lock_guard<std::mutex> lock(play_state_mutex_);
      if (play_state != observed_play_state_) {
        observed_play_state_ = play_state;
        ++play_event_sequence_;
        if (play_state == 1) {
          last_play_start_event_ = play_event_sequence_;
        } else if (play_state == 0) {
          last_play_stop_event_ = play_event_sequence_;
        }
      }
    }
    store_.UpdatePlayState(play_state);
    play_state_cv_.notify_all();
  }
  if (!root.isMember("text") || !asr_enabled_.load()) {
    return;
  }

  AsrData asr;
  asr.received = true;
  asr.index = root.get("index", 0).asUInt64();
  asr.timestamp_raw = root.get("timestamp", 0).asUInt64();
  asr.text = root.get("text", "").asString();
  asr.angle_raw = root.get("angle", 0).asInt();
  asr.speaker_id_raw = root.get("speaker_id", 0).asInt();
  asr.sense_raw = root.get("sense", "").asString();
  asr.confidence = root.get("confidence", 0.0).asDouble();
  asr.language = root.get("language", "").asString();
  asr.is_final = root.get("is_final", false).asBool();
  asr.raw_json = message;
  asr.received_time_ms = SystemTimeMs();
  store_.UpdateAsr(asr);
  QueueCustomerWakeRequest(asr);
}

std::string VoiceService::Trim(const std::string& text) {
  const auto begin = std::find_if_not(text.begin(), text.end(), [](char value) {
    return std::isspace(static_cast<unsigned char>(value)) != 0;
  });
  const auto end = std::find_if_not(text.rbegin(), text.rend(), [](char value) {
                     return std::isspace(static_cast<unsigned char>(value)) !=
                            0;
                   }).base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

std::string VoiceService::NormalizeServiceName(const std::string& name) {
  std::string normalized;
  normalized.reserve(name.size());
  for (const char value : name) {
    if (std::isalnum(static_cast<unsigned char>(value)) != 0) {
      normalized.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(value))));
    }
  }
  return normalized;
}

}  // namespace g1_web
