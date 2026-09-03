#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "g1_web/control_service.hpp"
#include "g1_web/camera_service.hpp"
#include "g1_web/perception_service.hpp"
#include "g1_web/snapshot_store.hpp"
#include "g1_web/voice_service.hpp"

namespace g1_web {

class HttpServer {
 public:
  HttpServer(SnapshotStore& store, VoiceService& voice_service,
             ControlService& control_service,
             PerceptionService& perception_service,
             CameraService& camera_service,
             std::string bind_address,
             std::uint16_t port, std::string web_root,
             unsigned int publish_hz);
  ~HttpServer();

  bool Start(std::string& error);
  void Stop();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_web
