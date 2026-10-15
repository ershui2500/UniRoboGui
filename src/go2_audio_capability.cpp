#include "g1_web/go2_audio_capability.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>
#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

namespace g1_web {
namespace {

constexpr std::int32_t kEnterMegaphoneApi = 4001;
constexpr std::int32_t kExitMegaphoneApi = 4002;
constexpr std::int32_t kUploadMegaphoneApi = 4003;
constexpr std::size_t kBase64ChunkChars = 32768;

class AudioHubClient final : public unitree::robot::Client {
 public:
  AudioHubClient() : unitree::robot::Client("audiohub", false) {}

  void Init() override {
    SetApiVersion("1.0.0.1");
    RegistApi(kEnterMegaphoneApi, 0);
    RegistApi(kExitMegaphoneApi, 0);
    RegistApi(kUploadMegaphoneApi, 0);
  }

  std::int32_t Request(std::int32_t api, const Json::Value& parameter,
                       std::string& response) {
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    return Call(api, Json::writeString(writer, parameter), response);
  }
};

void AppendLe16(std::vector<std::uint8_t>& output, std::uint16_t value) {
  output.push_back(static_cast<std::uint8_t>(value));
  output.push_back(static_cast<std::uint8_t>(value >> 8));
}

void AppendLe32(std::vector<std::uint8_t>& output, std::uint32_t value) {
  output.push_back(static_cast<std::uint8_t>(value));
  output.push_back(static_cast<std::uint8_t>(value >> 8));
  output.push_back(static_cast<std::uint8_t>(value >> 16));
  output.push_back(static_cast<std::uint8_t>(value >> 24));
}

std::vector<std::uint8_t> PcmToWav(const std::vector<std::uint8_t>& pcm) {
  std::vector<std::uint8_t> wav;
  wav.reserve(44 + pcm.size());
  wav.insert(wav.end(), {'R', 'I', 'F', 'F'});
  AppendLe32(wav, static_cast<std::uint32_t>(36 + pcm.size()));
  wav.insert(wav.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
  AppendLe32(wav, 16);
  AppendLe16(wav, 1);
  AppendLe16(wav, 1);
  AppendLe32(wav, 16000);
  AppendLe32(wav, 32000);
  AppendLe16(wav, 2);
  AppendLe16(wav, 16);
  wav.insert(wav.end(), {'d', 'a', 't', 'a'});
  AppendLe32(wav, static_cast<std::uint32_t>(pcm.size()));
  wav.insert(wav.end(), pcm.begin(), pcm.end());
  return wav;
}

std::string Base64(const std::vector<std::uint8_t>& input) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((input.size() + 2) / 3) * 4);
  for (std::size_t offset = 0; offset < input.size(); offset += 3) {
    const std::uint32_t a = input[offset];
    const std::uint32_t b = offset + 1 < input.size() ? input[offset + 1] : 0;
    const std::uint32_t c = offset + 2 < input.size() ? input[offset + 2] : 0;
    const std::uint32_t value = (a << 16) | (b << 8) | c;
    output.push_back(kAlphabet[(value >> 18) & 0x3f]);
    output.push_back(kAlphabet[(value >> 12) & 0x3f]);
    output.push_back(offset + 1 < input.size()
                         ? kAlphabet[(value >> 6) & 0x3f]
                         : '=');
    output.push_back(offset + 2 < input.size() ? kAlphabet[value & 0x3f]
                                               : '=');
  }
  return output;
}

class Go2AudioCapability final : public IAudioCapability {
 public:
  explicit Go2AudioCapability(bool mock, Go2AudioRequest request = {})
      : mock_(mock), test_request_(std::move(request)) {}

  AudioCapabilityFeatures Features() const override {
    return {true, false, true, false, false, false, false};
  }

  bool Start(std::string& error) override {
    if (available_) {
      error.clear();
      return true;
    }
    if (mock_) {
      available_ = true;
      error.clear();
      return true;
    }
    try {
      vui_ = std::make_unique<unitree::robot::go2::VuiClient>();
      vui_->SetTimeout(10.0F);
      vui_->Init();
      audiohub_ = std::make_unique<AudioHubClient>();
      audiohub_->SetTimeout(10.0F);
      audiohub_->Init();
      available_ = true;
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
    } catch (...) {
      error = "unknown_go2_audio_capability_init_error";
    }
    audiohub_.reset();
    vui_.reset();
    available_ = false;
    return false;
  }

  void Stop() override {
    audiohub_.reset();
    vui_.reset();
    available_ = false;
  }

  bool Available() const override { return available_; }

  std::int32_t GetVolume(std::uint8_t& volume) override {
    if (!available_) return -1;
    if (mock_) {
      volume = mock_volume_;
      return 0;
    }
    int level = 0;
    const std::int32_t result = vui_ ? vui_->GetVolume(level) : -1;
    if (result == 0) {
      volume = static_cast<std::uint8_t>(std::clamp(level, 0, 10) * 10);
    }
    return result;
  }

  std::int32_t SetVolume(std::uint8_t volume) override {
    if (!available_) return -1;
    const int level = std::clamp((static_cast<int>(volume) + 5) / 10, 0, 10);
    if (mock_) {
      mock_volume_ = static_cast<std::uint8_t>(level * 10);
      return 0;
    }
    return vui_ ? vui_->SetVolume(level) : -1;
  }

  std::int32_t TtsMaker(const std::string&, std::int32_t) override {
    return -1;
  }

  std::int32_t PlayStream(std::string, std::string,
                          std::vector<std::uint8_t> pcm_data) override {
    playback_error_.clear();
    if (!available_ || pcm_data.empty() || (pcm_data.size() % 2) != 0) {
      playback_error_ = "local_tts_invalid_pcm";
      return -1;
    }
    if (mock_ && !test_request_) return 0;
    if ((!audiohub_ && !test_request_) ||
        pcm_data.size() > 0xffffffffULL - 36ULL) return -1;

    std::size_t first_sound = 0;
    while (first_sound < pcm_data.size()) {
      const auto value = static_cast<std::uint16_t>(pcm_data[first_sound]) |
                         (static_cast<std::uint16_t>(pcm_data[first_sound + 1])
                          << 8);
      const auto sample = static_cast<std::int16_t>(value);
      if (sample > 100 || sample < -100) break;
      first_sound += 2;
    }
    if (first_sound == pcm_data.size()) {
      playback_error_ = "local_tts_silent_pcm";
      return -1;
    }
    if (first_sound > 0) {
      pcm_data.erase(pcm_data.begin(),
                     pcm_data.begin() + static_cast<std::ptrdiff_t>(first_sound));
    }

    const auto wav = PcmToWav(pcm_data);
    const std::string encoded = Base64(wav);
    const std::size_t total_blocks =
        (encoded.size() + kBase64ChunkChars - 1) / kBase64ChunkChars;
    std::string response;
    const Json::Value empty(Json::objectValue);
    const auto upload_started = std::chrono::steady_clock::now();
    std::int32_t result = Request(kEnterMegaphoneApi, empty, response);
    if (result != 0) {
      playback_error_ = "local_tts_audiohub_enter_failed";
      std::clog << "[go2-audio] enter_result=" << result << '\n';
      return result;
    }
    for (std::size_t index = 0; index < total_blocks; ++index) {
      const std::string block = encoded.substr(index * kBase64ChunkChars,
                                               kBase64ChunkChars);
      Json::Value parameter(Json::objectValue);
      parameter["current_block_size"] =
          static_cast<Json::UInt64>(block.size());
      parameter["block_content"] = block;
      parameter["current_block_index"] =
          static_cast<Json::UInt64>(index + 1);
      parameter["total_block_number"] =
          static_cast<Json::UInt64>(total_blocks);
      result = Request(kUploadMegaphoneApi, parameter, response);
      if (result != 0) {
        playback_error_ = "local_tts_audiohub_upload_failed_block_" +
                          std::to_string(index + 1);
        break;
      }
      // Each synchronous request already waits for the receiver's response.
    }
    const auto upload_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - upload_started).count();
    if (result == 0 && !mock_) {
      const auto playback_ms = (pcm_data.size() + 31) / 32;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(playback_ms + 5000));
    }
    const std::int32_t exit_result =
        Request(kExitMegaphoneApi, empty, response);
    if (result == 0 && exit_result != 0) {
      playback_error_ = "local_tts_audiohub_exit_failed";
    }
    std::clog << "[go2-audio] pcm_bytes=" << pcm_data.size()
              << " blocks=" << total_blocks << " upload_ms=" << upload_ms
              << " upload_result=" << result << " exit_result=" << exit_result
              << '\n';
    return result != 0 ? result : exit_result;
  }

  std::string PlaybackError() const override { return playback_error_; }

  std::int32_t PlayStop(std::string) override {
    if (!available_) return -1;
    if (mock_) return 0;
    return audiohub_ ? 0 : -1;
  }

 private:
  std::int32_t Request(std::int32_t api, const Json::Value& parameter,
                       std::string& response) {
    return test_request_ ? test_request_(api, parameter)
                         : audiohub_->Request(api, parameter, response);
  }

  bool mock_{false};
  Go2AudioRequest test_request_;
  bool available_{false};
  std::uint8_t mock_volume_{90};
  std::string playback_error_;
  std::unique_ptr<unitree::robot::go2::VuiClient> vui_;
  std::unique_ptr<AudioHubClient> audiohub_;
};

}  // namespace

std::unique_ptr<IAudioCapability> CreateGo2AudioCapability(bool mock) {
  return std::make_unique<Go2AudioCapability>(mock);
}

std::unique_ptr<IAudioCapability> CreateGo2AudioCapabilityForTesting(
    Go2AudioRequest request) {
  return std::make_unique<Go2AudioCapability>(true, std::move(request));
}

}  // namespace g1_web
