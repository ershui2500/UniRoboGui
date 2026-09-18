#include "g1_web/g1_audio_capability.hpp"

#include <exception>
#include <memory>
#include <utility>

#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace g1_web {
namespace {

class G1AudioCapability final : public IAudioCapability {
 public:
  explicit G1AudioCapability(bool mock) : mock_(mock) {}

  AudioCapabilityFeatures Features() const override {
    return {true, true, true, true};
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
      client_ = std::make_unique<unitree::robot::g1::AudioClient>();
      client_->SetTimeout(10.0F);
      client_->Init();
      available_ = true;
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      client_.reset();
      available_ = false;
      error = exception.what();
      return false;
    } catch (...) {
      client_.reset();
      available_ = false;
      error = "unknown_audio_capability_init_error";
      return false;
    }
  }

  void Stop() override {
    client_.reset();
    available_ = false;
  }

  bool Available() const override { return available_; }

  std::int32_t GetVolume(std::uint8_t& volume) override {
    if (!available_) return -1;
    if (mock_) {
      volume = 82;
      return 0;
    }
    return client_ ? client_->GetVolume(volume) : -1;
  }

  std::int32_t SetVolume(std::uint8_t volume) override {
    if (!available_) return -1;
    if (mock_) return 0;
    return client_ ? client_->SetVolume(volume) : -1;
  }

  std::int32_t TtsMaker(const std::string& text,
                        std::int32_t speaker_id) override {
    if (!available_) return -1;
    if (mock_) return 0;
    return client_ ? client_->TtsMaker(text, speaker_id) : -1;
  }

  std::int32_t PlayStream(std::string app_name, std::string stream_id,
                          std::vector<std::uint8_t> pcm_data) override {
    if (!available_) return -1;
    if (mock_) return 0;
    return client_ ? client_->PlayStream(std::move(app_name),
                                         std::move(stream_id),
                                         std::move(pcm_data))
                   : -1;
  }

  std::int32_t PlayStop(std::string app_name) override {
    if (!available_) return -1;
    if (mock_) return 0;
    return client_ ? client_->PlayStop(std::move(app_name)) : -1;
  }

 private:
  bool mock_{false};
  bool available_{false};
  std::unique_ptr<unitree::robot::g1::AudioClient> client_;
};

}  // namespace

std::unique_ptr<IAudioCapability> CreateG1AudioCapability(bool mock) {
  return std::make_unique<G1AudioCapability>(mock);
}

}  // namespace g1_web
