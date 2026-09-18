#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace g1_web {

struct AudioCapabilityFeatures {
  bool play_stream{false};
  bool tts{false};
  bool volume{false};
  bool asr{false};
};

class IAudioCapability {
 public:
  virtual ~IAudioCapability() = default;

  virtual AudioCapabilityFeatures Features() const = 0;
  virtual bool Start(std::string& error) = 0;
  virtual void Stop() = 0;
  virtual bool Available() const = 0;
  virtual std::int32_t GetVolume(std::uint8_t& volume) = 0;
  virtual std::int32_t SetVolume(std::uint8_t volume) = 0;
  virtual std::int32_t TtsMaker(const std::string& text,
                                std::int32_t speaker_id) = 0;
  virtual std::int32_t PlayStream(std::string app_name,
                                  std::string stream_id,
                                  std::vector<std::uint8_t> pcm_data) = 0;
  virtual std::int32_t PlayStop(std::string app_name) = 0;
};

}  // namespace g1_web
